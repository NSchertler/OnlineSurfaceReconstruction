/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Nico Schertler
*/

#include "osr/Data.h"

using namespace osr;

template <typename Derived>
DataBase<Derived>::DataBase()
	: optimizer(meshSettings), hierarchy(optimizer, static_cast<Derived*>(this)->extractedMesh), colorScale(0)
{
	meshSettings.setScale(0.0f);
	meshSettings.setMaxRegistrationError(0.0f);
	meshSettings.rosy = std::shared_ptr<IOrientationFieldTraits>(getOrientationFieldTraits(4));
	meshSettings.posy = std::shared_ptr<IPositionFieldTraits>(getPositionFieldTraits(4));
}

template <typename Derived>
void DataBase<Derived>::AddScan(Scan* s)
{
	if(meshSettings.scanCutoff)
		for (int i = 0; i < s->V().cols(); ++i)
		{
			auto p = s->transform() * s->V().col(i);
			if (p.y() < meshSettings.scanCutoffHeight)
				s->V().col(i).setConstant(std::numeric_limits<float>::quiet_NaN());
		}
	scans.push_back(s);

	if (meshSettings.scale() == 0)
	{
		meshSettings.setScale(sqrt(s->boundingBox().diagonal().squaredNorm() / (meshSettings.scanSubsample * s->V().cols())) * 6);
		hierarchy.setMaxNeighborRadius(3 * meshSettings.scale());		
	}

	if (meshSettings.maxRegistrationError() == 0)
		meshSettings.setMaxRegistrationError(s->boundingBox().diagonal().norm() * 0.02f);

	if (colorScale == 0)
		colorScale = 10.0f / s->boundingBox().diagonal().maxCoeff();

	if (s->C().cols() == 0)
	{
		//generate procedural colors
		s->C().resizeLike(s->V());
		for (int i = 0; i < s->V().cols(); ++i)
			s->C().col(i) = generateColor(s->p(i));
	}

	ScanAdded(s);
}

template <typename Derived>
Vector3us DataBase<Derived>::generateColor(const Vector3f& p)
{
	Vector3f c;
	c(0) = 0.7f + 0.2 * sin(p(0) * colorScale);
	for (int i = 1; i < 3; ++i)
		c(i) = 0.5f + 0.5 * sin(p(i) * colorScale);	
	return (65535.0f * (Vector3f(0.1f, 0.1f, 0.1f) + 0.7f * Vector3f(c.x(), c.y(), c.z()))).cast<unsigned short>();
}

template <typename Derived>
void DataBase<Derived>::RemoveScan(Scan * scan)
{
	if (scan == lastRegistrationScan)
		lastRegistrationScan = nullptr;

	scans.erase(std::remove(scans.begin(), scans.end(), scan), scans.end());

	ScanRemoved(scan);
	delete scan;
}

template <typename Derived>
void DataBase<Derived>::RegisterScan(Scan * s)
{
	lastRegistrationScan = s;
	lastRegistration = s->transform();
	float regError = meshSettings.maxRegistrationError();

	for (int i = 0; i < 4; ++i)
	{
		s->alignTo(hierarchy, 5);
		meshSettings.setMaxRegistrationError(meshSettings.maxRegistrationError() / 2);
	}

	meshSettings.setMaxRegistrationError(regError);
}

template <typename Derived>
void DataBase<Derived>::UndoLastRegistration()
{
	if (lastRegistrationScan)
	{
		lastRegistrationScan->transform() = lastRegistration;
	}
}

template <typename Derived>
void DataBase<Derived>::Reset()
{
	hierarchy.reset();
	static_cast<Derived*>(this)->extractedMesh.reset();

	meshSettings.setScale(0);
	colorScale = 0;

	meshSettings.setMaxRegistrationError(0);

	for (auto scan : scans)
		ScanRemoved(scan);
	scans.clear();
}

template <typename Derived>
void DataBase<Derived>::IntegrateScan(Scan * scan)
{
	TimedBlock b("Integrating points ..", true);
	hierarchy.addPoints(scan->transform() * scan->V(), scan->transform().linear() * scan->N(), scan->C());

	std::cout << "Size of hierarchy: " << hierarchy.vertexCount() << " points, " << memString(hierarchy.sizeInBytes()) << std::endl;

	RemoveScan(scan);
}

template <typename Derived>
void DataBase<Derived>::saveToFile(const std::string & path) const
{
	auto f = fopen(path.c_str(), "wb");
	meshSettings.saveToFile(f);
	hierarchy.saveToFile(f);
	static_cast<const Derived*>(this)->extractedMesh.saveToFile(f);

	fwrite(&colorScale, sizeof(float), 1, f);

	fclose(f);
}

template <typename Derived>
void DataBase<Derived>::loadFromFile(const std::string & path)
{
	Reset();

	auto f = fopen(path.c_str(), "rb");
	meshSettings.loadFromFile(f);
	hierarchy.loadFromFile(f);
	static_cast<Derived*>(this)->extractedMesh.loadFromFile(f);

	fread(&colorScale, sizeof(float), 1, f);

	fclose(f);
}

Data::Data()
	: extractedMesh(meshSettings)
{ }

DataGL::DataGL()
	: extractedMesh(meshSettings)
{ }

namespace osr
{
	template class DataBase<Data>;
	template class DataBase<DataGL>;
}