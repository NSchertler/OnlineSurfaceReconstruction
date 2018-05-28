#include "osr/gui/loaders/DavidViveScanLoader.h"

#include <iostream>
#include <boost/format.hpp>

#include "davidviveres.h"
#include "osr/meshio.h"

#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <ostream>

using namespace osr;
using namespace osr::gui;
using namespace osr::gui::loaders;

const std::string autoItPath = "D:\\Program Files (x86)\\AutoIt3\\AutoIt3.exe"; //TODO: Generalize
const std::string scanPath = "D:\\Scans\\currentScan.ply"; //TODO::Generalize

DavidViveScanLoader::DavidViveScanLoader(nse::gui::AbstractViewer* viewer)
	: trackingThread(nullptr), tracking(false), viewer(viewer)
{ }

void DavidViveScanLoader::setup(nanogui::Window*)
{
	std::cout << "Starting the DAVID/Vive scan loader .." << std::endl;

	vr::HmdError error;
	vrSystem = vr::VR_Init(&error, vr::EVRApplicationType::VRApplication_Other);

	if (vrSystem == nullptr)
	{
		std::cout << "Cannot start OpenVR." << std::endl;
		return;
	}

	std::cout << "OpenVR started successfully." << std::endl;

	//write AutoIt script to current directory
	FILE* f = fopen("TakeDavidScan.au3", "wb");
	fwrite(takedavidscan_au3, 1, takedavidscan_au3_size, f);
	fclose(f);

	f = fopen("ViveController.ply", "wb");
	fwrite(vivecontroller_ply, 1, vivecontroller_ply_size, f);
	fclose(f);

	MatrixXu F;
	Matrix3Xf V, N;
	Matrix3Xus C;
	load_ply("ViveController.ply", F, V, N, C);
	viveController = new Scan(1000 * V, N, C, F);
	viveController->initialize();
	remove("ViveController.ply");

	tracking = true;
	trackingThread = new std::thread(&DavidViveScanLoader::track, this);

	f = fopen("calibration.bin", "rb");
	if (f)
	{
		std::cout << "Reading Vive calibration from file." << std::endl;
		fread(transformScannerControllerToDavidSystem.data(), sizeof(float), 16, f);
		fclose(f);
	}
	else
	{
		std::cout << "No Vive calibration available." << std::endl;
		transformScannerControllerToDavidSystem.setIdentity();
	}

	f = fopen("calibrationTurntable.bin", "rb");
	if (f)
	{
		std::cout << "Reading turntable calibration from file." << std::endl;
		fread(axisDirection.data(), sizeof(float), 3, f);
		fread(axisCenter.data(), sizeof(float), 3, f);
		fclose(f);
	}
	else
	{
		std::cout << "No turntable calibration available." << std::endl;
		axisDirection = Vector3f::UnitZ();
		axisCenter.setZero();
	}
	boost::filesystem::path p(scanPath);
	p.remove_filename();

	boost::format fmt("ScanSession_%04d-%02d-%02d_%02d-%02d-%02d");
	auto now = boost::posix_time::second_clock::local_time();
	std::string filename = (fmt % now.date().year() % (int)now.date().month() % now.date().day() % now.time_of_day().hours() % now.time_of_day().minutes() % now.time_of_day().seconds()).str();
	sessionPath = p / boost::filesystem::path(filename);
	boost::filesystem::create_directory(sessionPath);
	
	entireALNPath = (sessionPath / boost::filesystem::path("Session.aln")).string();
	fEntireALN.open(entireALNPath);
	fEntireALN << "0       " << std::endl;
	scansInALN = 0;

#ifdef WIN32
	turntable.openConnection();
#endif

	currentAngle = 0;

	state = Normal;

	hasReference = false;
	calibrationAutoImproveMatrix.setZero();
	adaptAfterCalibration.setIdentity();

	scannerControllerMatrix.linear().setConstant(std::numeric_limits<float>::quiet_NaN());
	secondaryControllerMatrix.linear().setConstant(std::numeric_limits<float>::quiet_NaN());
}

void DavidViveScanLoader::draw(const Matrix4f & mv, const Matrix4f & proj)
{
	if (state == CalibratingShowScan || state == CalibratingShowBoth)
	{
		if (currentScan)
		{
			if (!currentScan->renderer->isInitialized())
				currentScan->initialize();
			currentScan->renderer->draw(*currentScan, mv, proj);
		}			
	}

	if (state == CalibratingShowViveController || state == CalibratingShowBoth)
	{
		if (viveController)
			viveController->renderer->draw(*viveController, mv, proj);
	}	

	if (state == Scanning)
	{
		NVGcontext* vg = viewer->nvgContext();
		float bounds[4];
		nvgTextBounds(vg, 0, 0, "Scanning ...", nullptr, bounds);

		nvgBeginPath(vg);
		nvgRoundedRect(vg, viewer->width() - 30 - bounds[2] + bounds[0], viewer->height() - 30 - bounds[3] + bounds[1], 20 + bounds[2] - bounds[0], 20 + bounds[3] - bounds[1], 4);
		nvgFillColor(vg, nvgRGBA(0, 0, 0, 128));
		nvgFill(vg);
		std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
		unsigned char color = (unsigned char)(190 + 30 * sin(ms.count() / 200.0));
		nvgFillColor(vg, nvgRGBA(color, color, color, 255));
		nvgText(vg, viewer->width() - 20 - bounds[2], viewer->height() - 20 - bounds[3], "Scanning ...", nullptr);
	}
}

bool DavidViveScanLoader::mouseButtonEvent(const Eigen::Vector2i & p, int button, bool down, int modifiers)
{
	if (state == Normal)
		return false;

	if (down && button == GLFW_MOUSE_BUTTON_1 && modifiers == GLFW_MOD_CONTROL)
	{
		float depth;
		glReadPixels(p.x(), viewer->height() - 1 - p.y(), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);

		if (depth < 1 && depth != 0)
		{
			float ndcDepth = 2 * (depth - 0.5);

			float x = 2 * ((float)p.x() / viewer->width() - 0.5f);
			float y = 2 * ((float)-p.y() / viewer->height() + 0.5f);

			Eigen::Matrix4f model, view, proj;
			viewer->camera().ComputeCameraMatrices(view, proj);

			Eigen::Matrix4f mvp = proj * view * model;
			Eigen::Matrix4f invMvp = mvp.inverse();

			Vector4f pos = invMvp * Vector4f(x, y, ndcDepth, 1);
			Vector3f pos3 = pos.block<3, 1>(0, 0) / pos.w();
			
			if (state == CalibratingShowScan)
			{
				correspondencesDavidSystem.push_back(pos3);
				state = CalibratingShowViveController;
				viveController->transform().setIdentity();
				scanCamParams = viewer->camera().saveParams();
				viewer->camera().restoreParams(controllerCamParams);				
			}
			else
			{
				correspondencesSecondaryController.push_back(pos3);
				state = CalibratingShowScan;
				controllerCamParams = viewer->camera().saveParams();
				viewer->camera().restoreParams(scanCamParams);
			}

			if (correspondencesDavidSystem.size() >= 3 && correspondencesSecondaryController.size() >= 3)
				Calibrate();
		}

		return true;
	}	

	return false;
}

void writeALNPart(std::ofstream& file, const std::string& filename, const Eigen::Affine3f& transform)
{	
	file << filename << std::endl;
	file << "#" << std::endl;
	file << transform(0, 0) << " " << transform(0, 1) << " " << transform(0, 2) << " " << transform(0, 3) << std::endl;
	file << transform(1, 0) << " " << transform(1, 1) << " " << transform(1, 2) << " " << transform(1, 3) << std::endl;
	file << transform(2, 0) << " " << transform(2, 1) << " " << transform(2, 2) << " " << transform(2, 3) << std::endl;
	file << "0.0 0.0 0.0 1.0" << std::endl;
}

int DavidViveScanLoader::FindOtherController(vr::TrackedDeviceIndex_t controller, vr::TrackedDevicePose_t* poses)
{
	int otherController = -1;
	for (vr::TrackedDeviceIndex_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i)
	{
		auto deviceClass = vrSystem->GetTrackedDeviceClass(i);
		if (i != controller && deviceClass == vr::TrackedDeviceClass::TrackedDeviceClass_Controller && poses[i].bPoseIsValid && poses[i].eTrackingResult == vr::ETrackingResult::TrackingResult_Running_OK)
		{
			otherController = i;
			break;
		}
	}	

	return otherController;
}

bool DavidViveScanLoader::ToEigenMatrix(const vr::TrackedDevicePose_t& pose, Eigen::Affine3f& m)
{
	if (!pose.bPoseIsValid || pose.eTrackingResult != vr::ETrackingResult::TrackingResult_Running_OK)
	{
		m.linear().setConstant(std::numeric_limits<float>::quiet_NaN());
		return false;
	}

	m.linear() <<
		pose.mDeviceToAbsoluteTracking.m[0][0], pose.mDeviceToAbsoluteTracking.m[0][1], pose.mDeviceToAbsoluteTracking.m[0][2],
		pose.mDeviceToAbsoluteTracking.m[1][0], pose.mDeviceToAbsoluteTracking.m[1][1], pose.mDeviceToAbsoluteTracking.m[1][2],
		pose.mDeviceToAbsoluteTracking.m[2][0], pose.mDeviceToAbsoluteTracking.m[2][1], pose.mDeviceToAbsoluteTracking.m[2][2];

	m.translation() <<
		pose.mDeviceToAbsoluteTracking.m[0][3] * 1000.0f,
		pose.mDeviceToAbsoluteTracking.m[1][3] * 1000.0f,
		pose.mDeviceToAbsoluteTracking.m[2][3] * 1000.0f;

	return true;
}

bool DavidViveScanLoader::waitUntilStill(vr::TrackedDevicePose_t* poses, vr::TrackedDeviceIndex_t device)
{
	vr::TrackedDevicePose_t posesOld[16];
	memcpy(posesOld, poses, 16 * sizeof(vr::TrackedDevicePose_t));

	int stillFrames = 0;
	int unavailableFrames = 0;
	int maxWaitFrames = 100;
	while (stillFrames < 10)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		vrSystem->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, poses, 16);

		if (!poses[device].bPoseIsValid || poses[device].eTrackingResult != vr::ETrackingResult::TrackingResult_Running_OK)
		{
			unavailableFrames++;
			if (unavailableFrames > 10)
				return false;
		}
		else
			unavailableFrames = 0;

		bool still = true;
		for(int i = 0; i < 3; ++i)
			for (int j = 0; j < 4; ++j)
			{
				if (std::abs(poses[device].mDeviceToAbsoluteTracking.m[i][j] - posesOld[device].mDeviceToAbsoluteTracking.m[i][j]) > 0.0005f)
				{
					still = false;
					break;
				}
			}

		if (still)
			stillFrames++;
		else
			stillFrames = 0;
		memcpy(posesOld, poses, 16 * sizeof(vr::TrackedDevicePose_t));
	}
	return true;
}

void DavidViveScanLoader::track()
{
	vr::TrackedDevicePose_t poses[16];

	while (tracking)
	{
		//check if the trigger has been pushed
		vr::VREvent_t e;
		while (vrSystem->PollNextEvent(&e, sizeof(e)))
		{
			if (e.eventType == vr::VREvent_ButtonUnpress && (e.data.controller.button == vr::k_EButton_SteamVR_Trigger || e.data.controller.button == vr::k_EButton_ApplicationMenu))
			{
				vrSystem->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, poses, 16);

				int primaryController = e.trackedDeviceIndex;
				int secondaryController = FindOtherController(e.trackedDeviceIndex, poses);

				if (e.data.controller.button == vr::k_EButton_ApplicationMenu)
					std::swap(primaryController, secondaryController);

				std::cout << "Waiting until primary controller stays still..." << std::endl;
				waitUntilStill(poses, primaryController);

				if (!ToEigenMatrix(poses[primaryController], scannerControllerMatrix))
				{
					std::cout << "Could not track primary controller." << std::endl;
					continue;
				}				

				if(state == Normal)
				{
					state = Scanning;
					Eigen::Affine3f transformUncalibrated = Eigen::Translation3f(axisCenter) * Eigen::AngleAxisf(-currentAngle * M_PI / 180.0f, axisDirection) * Eigen::Translation3f(-axisCenter) * scannerControllerMatrix;
					Eigen::Affine3f transformCalibrated = adaptAfterCalibration * transformUncalibrated * transformScannerControllerToDavidSystem;
					TakeScan(transformCalibrated);

					currentScan->davidViveData.transformUncalibrated = transformUncalibrated;
					currentScan->davidViveData.turntableRotation = Eigen::AngleAxisf(-currentAngle * M_PI / 180.0f, axisDirection);
					currentScan->davidViveData.davidToVive = scannerControllerMatrix * transformScannerControllerToDavidSystem;

					bool withSecondary = ToEigenMatrix(poses[secondaryController], secondaryControllerMatrix);

					std::ofstream aln("ScanAlignment.aln");
					aln << (withSecondary ? 3 : 2) << std::endl;
					writeALNPart(aln, "controller.ply", scannerControllerMatrix);
					if(withSecondary)
						writeALNPart(aln, "controller.ply", secondaryControllerMatrix);
					writeALNPart(aln, "scan.ply", transformCalibrated);
					aln.close();

					NewScan(currentScan);
					currentScan = nullptr;
					state = Normal;
				}
				else if(state == CalibratingShowScan)
				{								
					if (secondaryController == -1)
					{
						std::cout << "Secondary controller is not connected." << std::endl;
						continue;
					}

					std::cout << "Waiting until secondary controller stays still ..." << std::endl;
					waitUntilStill(poses, secondaryController);
					if (!ToEigenMatrix(poses[secondaryController], secondaryControllerMatrix))
					{
						std::cout << "Could not track secondary controller." << std::endl;
						continue;
					}		

					correspondencesDavidSystem.clear();
					correspondencesSecondaryController.clear();
					viveController->transform().setIdentity();

					TakeScan(Eigen::Affine3f::Identity());
					
					viewer->camera().FocusOnBBox(currentScan->boundingBox());
				}
			}

			if (e.eventType == vr::VREvent_ButtonUnpress && e.data.controller.button == vr::k_EButton_SteamVR_Touchpad)
			{
				vr::VRControllerState_t state;
				vrSystem->GetControllerState(e.trackedDeviceIndex, &state);
				currentAngle += turntable.move(30 * (state.rAxis[0].x > 0 ? 1 : -1));
			}

			if (e.eventType == vr::VREvent_ButtonPress && e.data.controller.button == vr::k_EButton_Grip)
			{
				if (state == Normal)
				{
					std::cout << "Entering calibration mode" << std::endl;
					state = CalibratingShowScan;

					auto currentParams = viewer->camera().saveParams();
					viewer->camera().FocusOnBBox(viveController->boundingBox());
					controllerCamParams = viewer->camera().saveParams();
					viewer->camera().restoreParams(currentParams);
				}
				else
				{
					if (state == CalibratingShowScan)
					{
						CalibrateTurntable(e.trackedDeviceIndex);

						state = Normal;
					}
					else
					{
						state = Normal;
					}
				}
			}
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	vr::VR_Shutdown();

	std::cout << "Stopped tracking ..." << std::endl;
}

void DavidViveScanLoader::TakeScan(const Eigen::Affine3f& transform)
{	
	nse::util::TimedBlock b("Scanning ..");

	if (currentScan)
		delete currentScan;	
	currentScan = nullptr;

	remove(scanPath.c_str());

	std::string command = "\"" + autoItPath + "\" TakeDavidScan.au3";
	system(command.c_str());

	while (!boost::filesystem::exists(scanPath))
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	MatrixXu F;
	Matrix3Xf V, N;
	Matrix3Xus C;
	load_ply(scanPath, F, V, N, C, true);

	if (state == Scanning)
	{
		//save scan for later use

		int i = -1;
		boost::filesystem::path savePathPLY;
		boost::format fmtPLY("DavidScan_%04d.ply");
		boost::format fmtALN("DavidScan_%04d.aln");
		boost::format fmtPNG("DavidScan_%04d.png");
		std::string saveFilename;
		do
		{
			++i;
			saveFilename = (fmtPLY % i).str();
			savePathPLY = sessionPath / boost::filesystem::path(saveFilename);
		} while (boost::filesystem::exists(savePathPLY));

		std::rename(scanPath.c_str(), savePathPLY.string().c_str());

		std::string oldPNG = (sessionPath.parent_path() / boost::filesystem::path("currentScan.png")).string();
		std::string newPNG = (sessionPath / boost::filesystem::path((fmtPNG % i).str())).string();
		std::rename(oldPNG.c_str(), newPNG.c_str());

		std::ofstream aln((sessionPath / (fmtALN % i).str()).wstring());
		aln << "1" << std::endl;
		writeALNPart(aln, saveFilename, transform);
		aln.close();

		writeALNPart(fEntireALN, saveFilename, transform);
		scansInALN++;
		fEntireALN.seekp(0, std::ios_base::beg);
		fEntireALN << scansInALN;
		fEntireALN.seekp(0, std::ios_base::end);
	}

	currentScan = new Scan(V, N, C, F, "David Scan", transform);
}

DavidViveScanLoader::~DavidViveScanLoader()
{
	tracking = false;

	if (trackingThread)
		trackingThread->join();

#ifdef WIN32
	turntable.move(-currentAngle); //reset to 0
#endif

	if(viveController)
		delete viveController;

	if (currentScan)
		delete currentScan;

	fEntireALN.close();
	if (scansInALN == 0)
		boost::filesystem::remove_all(sessionPath);
}

void DavidViveScanLoader::Calibrate()
{
	//Kabsch algorithm
	int n = std::min(correspondencesDavidSystem.size(), correspondencesSecondaryController.size());
	if (n < 3)
	{
		std::cout << "Need at least three correspondences for calibration." << std::endl;
		return;
	}


	Eigen::Vector3f X_mean, Y_mean;
	X_mean.setZero(); Y_mean.setZero();
	for (int i = 0; i < n; ++i)
	{		
		X_mean += correspondencesSecondaryController[i];
		Y_mean += correspondencesDavidSystem[i];
	}
	X_mean /= n;
	Y_mean /= n;

	/// Compute transformation
	Eigen::Matrix3f sigma; sigma.setConstant(0.0f);
	for (int i = 0; i < n; ++i)
		sigma += (correspondencesSecondaryController[i] - X_mean) * (correspondencesDavidSystem[i] - Y_mean).transpose();
	Eigen::JacobiSVD<Eigen::Matrix3f> svd(sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Affine3f transformSecondaryControllerToDavid;
	transformSecondaryControllerToDavid.setIdentity();
	transformSecondaryControllerToDavid.pretranslate(-X_mean);
	Eigen::Affine3f rotation;
	if (svd.matrixU().determinant()*svd.matrixV().determinant() < 0.0) {
		Eigen::Vector3f S = Eigen::Vector3f::Ones(); S(2) = -1.0;
		rotation.linear().noalias() = svd.matrixV()*S.asDiagonal()*svd.matrixU().transpose();
	}
	else {
		rotation.linear().noalias() = svd.matrixV()*svd.matrixU().transpose();
	}
	transformSecondaryControllerToDavid = rotation * transformSecondaryControllerToDavid;
	transformSecondaryControllerToDavid.pretranslate(Y_mean);

	float error = 0;
	for (int i = 0; i < n; ++i)
		error += (correspondencesDavidSystem[i] - transformSecondaryControllerToDavid * correspondencesSecondaryController[i]).norm();
	error /= n;

	std::cout << "Coarse calibration error: " << error << "." << std::endl;

	state = CalibratingShowBoth;

	//fine registration
	viveController->transform() = transformSecondaryControllerToDavid;
	currentScan->buildTree();
	//currentScan->closestPointRadius = 30;
	//viveController->alignTo(*currentScan, 5, 1.0);	
	currentScan->closestPointRadius = 10;
	viveController->alignTo(*currentScan, 10, 1.0);
	
	transformScannerControllerToDavidSystem = scannerControllerMatrix.inverse() * secondaryControllerMatrix * viveController->transform().inverse();

	std::cout << "Controller to Scan registration matrix:" << std::endl << viveController->transform().matrix() << std::endl;

	std::ofstream aln("Calibration.aln");
	aln << 3 << std::endl;
	writeALNPart(aln, "controller.ply", scannerControllerMatrix);
	writeALNPart(aln, "controller.ply", secondaryControllerMatrix);
	writeALNPart(aln, "scan.ply", scannerControllerMatrix * transformScannerControllerToDavidSystem);
	aln.close();

	FILE* f = fopen("calibration.bin", "wb");
	if (f)
	{
		fwrite(transformScannerControllerToDavidSystem.data(), sizeof(float), 16, f);

		fclose(f);
	}		

	correspondencesSecondaryController.clear();
	correspondencesDavidSystem.clear();
}

void DavidViveScanLoader::CalibrateTurntable(vr::TrackedDeviceIndex_t device)
{
	vr::TrackedDevicePose_t poses[16];

	//Calibrate rotation axis of turntable
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	std::vector<std::pair<double, Eigen::Affine3f>> orientations;
	int steps = 16;
	currentAngle = 0;
	for (int i = 0; i < steps; ++i)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
		vrSystem->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, poses, 16);
		Eigen::Affine3f t;
		if (ToEigenMatrix(poses[device], t))
			orientations.push_back(std::make_pair(currentAngle, t));
		currentAngle += turntable.move(720 / steps);
	}

	//Calculate the direction of the rotation axis
	axisDirection.setZero();
	int offset = 2;
	for (int i = 0; i + offset < orientations.size(); ++i)
	{
		auto& o1 = orientations[i];
		auto& o2 = orientations[i + offset];
		auto rot = o2.second.linear() * o1.second.linear().transpose();
		Vector3f localAxis(rot(2, 1) - rot(1, 2), rot(0, 2) - rot(2, 0), rot(1, 0) - rot(0, 1));
		//float angle = std::acos(std::min(1.0f, (rot(0, 0) + rot(1, 1) + rot(2, 2) - 1) / 2));
		//if (angle < 0)
		//	localAxis *= -1;
		axisDirection += localAxis.normalized();
	}
	axisDirection.normalize();
	std::cout << axisDirection.transpose() << std::endl;

	//Calculate the location of the rotation axis
	Eigen::Matrix<float, 2, 3> toLocal;
	//toLocal.row(2) = axis;
	toLocal.row(0) = (std::abs(axisDirection.x()) > std::abs(axisDirection.y()) ? Vector3f(0, 1, 0) : Vector3f(1, 0, 0));
	toLocal.row(1) = axisDirection.cross(toLocal.row(0)).normalized();
	toLocal.row(0) = toLocal.row(1).cross(axisDirection).normalized();

	Eigen::Matrix2f systemLHS; systemLHS.setZero();
	Eigen::Vector2f systemRHS; systemRHS.setZero();

	offset = 4;
	for (int i = 0; i + offset < orientations.size(); ++i)
	{
		auto p1 = toLocal * (orientations[i].second * Vector3f(0, 0, 150));
		auto p2 = toLocal * (orientations[i + offset].second * Vector3f(0, 0, 150));

		Vector2f d = (p2 - p1).normalized();
		Vector2f m = 0.5f * (p1 + p2);
		float b = m.dot(d);
		systemLHS += d * d.transpose();
		systemRHS += d * b;
	}
	auto circleCenter = systemLHS.colPivHouseholderQr().solve(systemRHS);
	axisCenter = toLocal.transpose() * circleCenter;
	std::cout << axisCenter.transpose() << std::endl;

	FILE* f = fopen("calibrationTurntable.bin", "wb");
	if (f)
	{
		fwrite(axisDirection.data(), sizeof(float), 3, f);
		fwrite(axisCenter.data(), sizeof(float), 3, f);

		fclose(f);
	}

	std::ofstream ply("turntable.ply", std::ios::binary);
	ply << "format binary_little_endian 1.0" << std::endl;
	ply << "element vertex " << orientations.size() * 4 + 3 << std::endl;
	ply << "property float32 x" << std::endl;
	ply << "property float32 y" << std::endl;
	ply << "property float32 z" << std::endl;
	ply << "element face " << 3 * orientations.size() + 1 << std::endl;
	ply << "property list int32 int32 vertex_index" << std::endl;
	ply << "end_header" << std::endl;

	float scale = 20;
	Vector3f p;
	for (auto& o : orientations)
	{		
		p = o.second * Vector3f(0, 0, 0);
		ply.write(reinterpret_cast<const char*>(p.data()), sizeof(Vector3f));
		p = o.second * Vector3f(scale, 0, 0);
		ply.write(reinterpret_cast<const char*>(p.data()), sizeof(Vector3f));
		p = o.second * Vector3f(0, scale, 0);
		ply.write(reinterpret_cast<const char*>(p.data()), sizeof(Vector3f));
		p = o.second * Vector3f(0, 0, scale);
		ply.write(reinterpret_cast<const char*>(p.data()), sizeof(Vector3f));
	}

	p = axisCenter + 200 * axisDirection;
	ply.write(reinterpret_cast<const char*>(p.data()), sizeof(Vector3f));
	ply.write(reinterpret_cast<const char*>(p.data()), sizeof(Vector3f));
	p = axisCenter - 200 * axisDirection;
	ply.write(reinterpret_cast<const char*>(p.data()), sizeof(Vector3f));

	for (int i = 0; i < orientations.size(); ++i)
	{
		int32_t data[] = {3, 4 * i, 4 * i + 1, 4 * i + 2, 3, 4 * i, 4 * i + 2, 4 * i + 3, 3, 4 * i, 4 * i + 1, 4 * i + 3};
		ply.write(reinterpret_cast<const char*>(data), sizeof(data));
	}

	int32_t axis[] = { 3, 4 * orientations.size(), 4 * orientations.size() + 1, 4 * orientations.size() + 2};
	ply.write(reinterpret_cast<const char*>(axis), sizeof(axis));

	ply.close();
}