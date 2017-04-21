#pragma once

#include <boost/signals2.hpp>
#include "Scan.h"
#include "GUIObject.h"

#include <nanogui/window.h>

//Abstract class that is responsible from providing new scan data
class ScanLoader : public GUIObject
{
public:
	virtual ~ScanLoader() { };

	boost::signals2::signal<void(Scan*)> NewScan;

	virtual void setup(nanogui::Window* window) { }
};