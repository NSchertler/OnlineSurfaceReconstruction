#pragma once

#include <vector>

#include "Scan.h"
#include "ExtractedMeshGL.h"
#include "HierarchyDef.h"
#include "Optimizer.h"
#include "MeshSettings.h"

#include <boost/signals2.hpp>

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

	//Integrates the specified scan into the hierarchy and removed it from this data object.
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
class Data : public DataBase<Data>
{
public:
	ExtractedMesh extractedMesh;

	Data();
};

//Specialization of the DataBase class that holds a renderable final mesh.
class DataGL : public DataBase<DataGL>
{
public:
	ExtractedMeshGL extractedMesh;

	DataGL();
};