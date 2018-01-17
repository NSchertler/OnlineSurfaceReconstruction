#include "DLLLoadPLY.h"
#include <algorithm>

#include <stdio.h>
#include "rply.h"
#include <string>
/*#include <osr/common.h>*/
#include "common.h"
#include <iostream>
#include <filesystem>
#include <Windows.h>

using namespace osr;

std::string scanPath = "D:\\Scans\\currentScan.ply"; //TODO::Generalize


extern "C" {
	void TestSort(int a[], int length) {
		std::sort(a, a + length);
	}

	void load_ply(std::string &filename, MatrixXu &F, Matrix3Xf &V,
		Matrix3Xf &N, Matrix3Xus &C, bool pointcloud)
	{
		auto message_cb = [](p_ply ply, const char *msg) { std::cerr << "rply: " << msg << std::endl; };

		//nse::util::Timer<> timer;
		std::cout << "Loading \"" << filename << "\" .. ";
		std::cout.flush();

		p_ply ply = ply_open(filename.c_str(), message_cb, 0, nullptr);
		if (!ply) {
			std::cerr << "Unable to open PLY file \"" << filename <<"\"!";
			return;
		}
			//throw std::runtime_error("Unable to open PLY file \"" + filename + "\"!");

		if (!ply_read_header(ply)) {
			ply_close(ply);
			throw std::runtime_error("Unable to open PLY header of \"" + filename + "\"!");
		}

		const float gamma = 2.2f;

		p_ply_element element = nullptr;
		uint32_t vertexCount = 0, faceCount = 0;

		p_ply_property prop = nullptr;
		const char* pname;
		e_ply_type type, length, value;

		bool hasUv = false;
		bool hasNormals = false;
		bool hasColor = false;

		/* Inspect the structure of the PLY file, load number of faces if avaliable */
		while ((element = ply_get_next_element(ply, element)) != nullptr) {
			const char *name;
			long nInstances;

			ply_get_element_info(element, &name, &nInstances);
			if (!strcmp(name, "vertex"))
			{
				vertexCount = (uint32_t)nInstances;
				while ((prop = ply_get_next_property(element, prop)) != nullptr)
				{
					ply_get_property_info(prop, &pname, &type, &length, &value);
					if (!strcmp(pname, "texture_u"))
						hasUv = true;
					else if (!strcmp(pname, "nx"))
						hasNormals = true;
					else if (!strcmp(pname, "red"))
						hasColor = true;
				}
			}
			else if (!strcmp(name, "face"))
				faceCount = (uint32_t)nInstances;
		}

		if (vertexCount == 0 && faceCount == 0)
			throw std::runtime_error("PLY file \"" + filename + "\" is invalid! No face/vertex/elements found!");
		else if (faceCount == 0)
			pointcloud = true;

		F.resize(3, faceCount);
		V.resize(3, vertexCount);
		N.resize(3, hasNormals ? vertexCount : 0);
		/*	C.resize(3, hasColor ? vertexCount : 0);*/
		/*	Eigen::Matrix<float, 2, Eigen::Dynamic> uv(2, hasUv ? vertexCount : 0);*/


		struct VertexCallbackData {
			Matrix3Xf &V;
			VertexCallbackData(Matrix3Xf &V)
				: V(V) { }
		};

		struct FaceCallbackData {
			MatrixXu &F;
			FaceCallbackData(MatrixXu &F)
				: F(F) { }
		};

		struct VertexNormalCallbackData {
			Matrix3Xf &N;
			VertexNormalCallbackData(Matrix3Xf &_N)
				: N(_N) { }
		};

		// 	struct VertexUVCallbackData {
		// 		Eigen::Matrix<float, 2, Eigen::Dynamic> &uv;
		// 		VertexUVCallbackData(Eigen::Matrix<float, 2, Eigen::Dynamic> &_uv)
		// 			: uv(_uv) { }
		// 	};
		// 
		// 	struct VertexColorCallbackData {
		// 		Eigen::Matrix<unsigned short, 3, Eigen::Dynamic> &c;
		// 		VertexColorCallbackData(Eigen::Matrix<unsigned short, 3, Eigen::Dynamic> &c)
		// 			: c(c) { }
		// 	};

		auto rply_vertex_cb = [](p_ply_argument argument) -> int {
			VertexCallbackData *data; long index, coord;
			ply_get_argument_user_data(argument, (void **)&data, &coord);
			ply_get_argument_element(argument, nullptr, &index);
			data->V(coord, index) = (Float)ply_get_argument_value(argument);
			return 1;
		};

		auto rply_vertex_normal_cb = [](p_ply_argument argument) -> int {
			VertexNormalCallbackData *data; long index, coord;
			ply_get_argument_user_data(argument, (void **)&data, &coord);
			ply_get_argument_element(argument, nullptr, &index);
			data->N(coord, index) = (Float)ply_get_argument_value(argument);
			return 1;
		};

		// 	auto rply_vertex_uv_cb = [](p_ply_argument argument) -> int
		// 	{
		// 		VertexUVCallbackData *data; long index, coord;
		// 		ply_get_argument_user_data(argument, (void **)&data, &coord);
		// 		ply_get_argument_element(argument, nullptr, &index);
		// 		data->uv(coord, index) = (Float)ply_get_argument_value(argument);
		// 		return 1;
		// 	};

		// 	auto rply_vertex_color_cb = [](p_ply_argument argument) -> int
		// 	{
		// 		VertexColorCallbackData *data; long index, coord;
		// 		ply_get_argument_user_data(argument, (void **)&data, &coord);
		// 		ply_get_argument_element(argument, nullptr, &index);
		// 		auto colorInPly = ply_get_argument_value(argument);
		// 		data->c(coord, index) = (unsigned short)(std::pow(colorInPly / 255.0, 2.2) * 65535);
		// 		return 1;
		// 	};

		auto rply_index_cb = [](p_ply_argument argument) -> int {
			FaceCallbackData *data;
			long length, value_index, index;
			ply_get_argument_property(argument, nullptr, &length, &value_index);

			if (length != 3)
				throw std::runtime_error("Only triangle faces are supported!");

			ply_get_argument_user_data(argument, (void **)&data, nullptr);
			ply_get_argument_element(argument, nullptr, &index);

			if (value_index >= 0)
				data->F(value_index, index) = (uint32_t)ply_get_argument_value(argument);

			return 1;
		};

		VertexCallbackData vcbData(V);
		FaceCallbackData fcbData(F);
		VertexNormalCallbackData vncbData(N);
		// 	VertexUVCallbackData vuvcbData(uv);
		// 	VertexColorCallbackData vcData(C);

		ply_set_read_cb(ply, "vertex", "x", rply_vertex_cb, &vcbData, 0);
		ply_set_read_cb(ply, "vertex", "y", rply_vertex_cb, &vcbData, 1);
		ply_set_read_cb(ply, "vertex", "z", rply_vertex_cb, &vcbData, 2);

		ply_set_read_cb(ply, "vertex", "nx", rply_vertex_normal_cb, &vncbData, 0);
		ply_set_read_cb(ply, "vertex", "ny", rply_vertex_normal_cb, &vncbData, 1);
		ply_set_read_cb(ply, "vertex", "nz", rply_vertex_normal_cb, &vncbData, 2);

		if (faceCount > 0)
		{
			if (!ply_set_read_cb(ply, "face", "vertex_index", rply_index_cb, &fcbData, 0) && !ply_set_read_cb(ply, "face", "vertex_indices", rply_index_cb, &fcbData, 0))
			{
				ply_close(ply);
				throw std::runtime_error("PLY file \"" + filename + "\" does not contain vertex indices!");
			}
		}

		// 	ply_set_read_cb(ply, "vertex", "texture_u", rply_vertex_uv_cb, &vuvcbData, 0);
		// 	ply_set_read_cb(ply, "vertex", "texture_v", rply_vertex_uv_cb, &vuvcbData, 1);
		// 
		// 	ply_set_read_cb(ply, "vertex", "red", rply_vertex_color_cb, &vcData, 0);
		// 	ply_set_read_cb(ply, "vertex", "green", rply_vertex_color_cb, &vcData, 1);
		// 	ply_set_read_cb(ply, "vertex", "blue", rply_vertex_color_cb, &vcData, 2);

		if (!ply_read(ply)) {
			ply_close(ply);
			throw std::runtime_error("Error while loading PLY data from \"" + filename + "\"!");
		}

		ply_close(ply);

		// 	if (hasUv && !hasColor)
		// 	{
		// 		std::string texturePath = filename;
		// 		//change extension from ply to png
		// 		texturePath[texturePath.length() - 2] = 'n';
		// 		texturePath[texturePath.length() - 1] = 'g';
		// 
		// 		if (nse::data::file_exists(texturePath))
		// 		{
		// 			std::cout << "loading texture .. ";
		// 			C.resizeLike(V);
		// 			using handleType = std::unique_ptr<uint8_t[], void(*)(void*)>;
		// 			int w, h, n;
		// 			handleType textureData(stbi_load(texturePath.c_str(), &w, &h, &n, 3), stbi_image_free);
		// 			if (textureData)
		// 			{
		// #pragma omp parallel for
		// 				for (int i = 0; i < V.cols(); ++i)
		// 				{
		// 					auto vuv = uv.col(i);
		// 					int x = (int)std::floor(w * vuv.x());
		// 					int y = (int)std::floor(h * (1 - vuv.y()));
		// 
		// 					if (x >= w)
		// 						x = w - 1;
		// 					if (y >= h)
		// 						y = h - 1;
		// 
		// 					int offset = n * (x + w * y);
		// 
		// 					//Gamma de-correction
		// 
		// 					C.col(i) <<
		// 						(uint16_t)(std::pow((float)textureData.get()[offset + 0] / 255.0f, gamma) * 65535.0f),
		// 						(uint16_t)(std::pow((float)textureData.get()[offset + 1] / 255.0f, gamma) * 65535.0f),
		// 						(uint16_t)(std::pow((float)textureData.get()[offset + 2] / 255.0f, gamma) * 65535.0f);
		// 				}
		// 			}
		// 		}
		// 	}

		// #pragma omp parallel for
		// 	for (int i = 0; i < C.cols(); ++i)
		// 	{
		// 		C.col(i) = RGBToLab(C.col(i));
		// 	}

		std::cout << "done. (V=" << vertexCount;
		if (faceCount > 0)
			std::cout << ", F=" << faceCount;
		//std::cout << ", took " << nse::util::timeString(timer.value()) << ")" << std::endl;
	}

	void LoadPLY(unsigned char * charF, unsigned char * charV, unsigned char * charN, int meshSize[])
	{
		MatrixXu F;
		Matrix3Xf V, N;
		Matrix3Xus C;
		scanPath = "D:\\Library\\rply-1.1.4\\etc\\input.ply";
		load_ply(scanPath, F, V, N, C, true);
		
		int nF = F.cols() * F.rows();
		int nN = N.cols() * N.rows();
		std::cout << "\nnF:" << nF << " nV:" << nN << "\n";
		//unsigned char * charF, *charV, *charN;
		charF = new unsigned char[F.cols() * F.rows() * sizeof(int)];
		charV = new unsigned char[V.cols() * V.rows() * sizeof(float)];
		charN = new unsigned char[N.cols() * N.rows() * sizeof(float)];

		memcpy(charF, F.data(), sizeof(F.data())*F.cols() * F.rows());
		memcpy(charV, V.data(), V.cols() * V.rows() * sizeof(float));
		memcpy(charN, N.data(), N.cols() * N.rows() * sizeof(float));
		meshSize[0] = F.cols() * F.rows();
		meshSize[1] = V.cols() * V.rows();
		meshSize[2] = N.cols() * N.rows();
	}

	void LoadPLY2(unsigned char * charF, unsigned char * charV, unsigned char * charN, int meshSize[])
	{
		MatrixXu F;
		Matrix3Xf V, N;
		Matrix3Xus C;
		scanPath = "D:\\Library\\rply-1.1.4\\etc\\input.ply";
		load_ply(scanPath, F, V, N, C, true);

		int nF = F.cols() * F.rows();
		int nN = N.cols() * N.rows();
		std::cout << "\nnF:" << nF << " nV:" << nN << "\n";
		//unsigned char * charF, *charV, *charN;
		charF = (unsigned char *)F.data();
		charV = (unsigned char *)V.data();
		charN = (unsigned char *)N.data();

		meshSize[0] = F.cols() * F.rows();
		meshSize[1] = V.cols() * V.rows();
		meshSize[2] = N.cols() * N.rows();
	}

	void LoadPLY3(const char* path, unsigned int ** pF, float ** pV, int meshSize[])
	{
		MatrixXu F;
		Matrix3Xf V, N;
		Matrix3Xus C;
		scanPath = "D:\\Library\\rply-1.1.4\\etc\\input.ply";
		scanPath = "D:\\Projects\\OnlineSurfaceReconstruction\\DavidVive\\viveController.ply";
		//char * cpath = new char(pathLen[0]);
		//strncpy(cpath, path, pathLen[0]);
		std::string spath = std::string(path);
// 		if (!std::experimental::filesystem::exists(spath))
// 			return;

		load_ply(spath, F, V, N, C, true);

// 		int nF = F.cols() * F.rows();
// 		int nN = N.cols() * N.rows();
		//std::cout << "\nnF:" << nF << " nV:" << nN << "\n";
		//OutputDebugString(std::to_string(nF).c_str());
		//unsigned char * charF, *charV, *charN;
		//pointers[0] = new unsigned char;
		pF[0] = F.data();
				//pointers[1] = new unsigned char;
		pV[0] = &V(0);//V.data();
		//pointers[2] = new unsigned char;
		//pointers[2] = (unsigned char*)N.data();
		
		meshSize[0] = F.cols() * F.rows();
		meshSize[1] = V.cols() * V.rows();
		//meshSize[2] = N.cols() * N.rows();

// 		delete cpath;
// 		cpath = NULL;
	}

	void LoadPLYDirect(int* charF, float* charV, float* charN, int meshSize[])
	{
		MatrixXu F;
		Matrix3Xf V, N;
		Matrix3Xus C;
		scanPath = "D:\\Library\\rply-1.1.4\\etc\\input.ply";
		load_ply(scanPath, F, V, N, C, true);

		int nF = F.cols() * F.rows();
		int nN = N.cols() * N.rows();
		std::cout << "\nnF:" << nF << " nV:" << nN << "\n";
		//unsigned char * charF, *charV, *charN;
		charF = new int[F.cols() * F.rows()];
		charV = new float[V.cols() * V.rows()];
		charN = new float[N.cols() * N.rows()];

		memcpy(charF, F.data(), F.cols() * F.rows());
		memcpy(charV, V.data(), V.cols() * V.rows() * sizeof(float));
		memcpy(charN, N.data(), N.cols() * N.rows() * sizeof(float));
		meshSize[0] = F.cols() * F.rows();
		meshSize[1] = V.cols() * V.rows();
		meshSize[2] = N.cols() * N.rows();
	}

}
