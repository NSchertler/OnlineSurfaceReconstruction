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

#include "osr/Scan.h"
#include "osr/gui/GUIObject.h"

#include <nanogui/window.h>

#include <nsessentials/util/Observer.h>

namespace osr {
	namespace gui {
		namespace loaders
		{
			//Abstract class that is responsible for providing new scan data
			class ScanLoader : public GUIObject
			{
			public:
				virtual ~ScanLoader() { };

				nse::util::Observer<Scan*> NewScan;

				virtual void setup(nanogui::Window* window) { }
			};
		}
	}
}