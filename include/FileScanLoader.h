#pragma once

#include "ScanLoader.h"

//Represents a scan loader that is capable of loading data from a file.
class FileScanLoader : public ScanLoader
{
public:
	void setup(nanogui::Window* window);

	void AddScan(Scan*);
private:
	void LoadData();
	void LoadDirectory();
};