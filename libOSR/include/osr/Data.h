/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Nico Schertler
*/

#pragma once

#include <vector>

#include "osr/Scan.h"
#include "osr/HierarchyDef.h"
#include "osr/Optimizer.h"
#include "osr/MeshSettings.h"
#include "osr/ExtractedMesh.h"
#include "osr/OSRLibrary.h"

#include <boost/signals2.hpp>

namespace osr
{
	//Holds all relevant data of the current session
	template <typename Derived>
	class DataBase
	{
	public:
		MeshSettings meshSettings;
		Optimizer optimizer;
		std::vector<Scan*> scans;
		THierarchy hierarchy;

		//This event is raised whenever a new scan is added to this data object.
		boost::signals2::signal<void(Scan*)> ScanAdded;

		//This event is raised whenever a scan is removed from this data object, possibly due to integration of the scan.
		boost::signals2::signal<void(Scan*)> ScanRemoved;

		DataBase();

		void AddScan(Scan*);
		void RemoveScan(Scan*);

		//Registers the scan with respect to the hierarchy by modifying the scan's transform. The hierarchy must not be empty for this to work.
		void RegisterScan(Scan*);
		//Sets the transform of the scan that has been registered last to its old state.
		void UndoLastRegistration();

		//Resets all data and produces a clean data object.
		void Reset();

		//Integrates the specified scan into the hierarchy and removes it from this data object.
		void IntegrateScan(Scan*);

		//Saves the current state of this data object to a file.
		void saveToFile(const std::string& path) const;
		//Restores the state for this data object from a file.
		void loadFromFile(const std::string& path);

	private:
		//For procedural color generation; scaling between spatial space and the domain of the color-generating function
		float colorScale;
		//Generates a procedural color for a given position. The color function is continuous over positions.
		Vector3us generateColor(const Vector3f& p);

		//Stores information that are necessary to revert a registration.
		Scan* lastRegistrationScan = nullptr;
		Eigen::Affine3f lastRegistration;
	};

	//Specialization of the DataBase class that holds an un-renderable final mesh.
	class OSR_EXPORT Data : public DataBase<Data>
	{
	public:
		ExtractedMesh extractedMesh;

		Data();
	};

	// ----------- template implementations -------------

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
		if (meshSettings.scanCutoff)
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
		nse::util::TimedBlock b("Integrating points ..", true);
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
}