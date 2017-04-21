#pragma once

#include "ScanLoader.h"

//Represents a scan loader that can generate procedural scans.
class ProceduralScanLoader : public ScanLoader
{
public:
	void setup(nanogui::Window* window);

private:
	void LoadData();
};