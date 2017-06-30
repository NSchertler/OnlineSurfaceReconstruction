/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Wenzel Jakob
	@author Nico Schertler
*/

#include "osr/Timer.h"

#include <cmath>

using namespace osr;

namespace osr
{
	template class Timer<>;
}

std::string osr::timeString(double time, bool precise)
{
	if (std::isnan(time) || std::isinf(time))
		return "inf";

	std::string suffix = "ms";
#ifndef ACCURATE_TIMING
	if (time > 1000) {
		time /= 1000; suffix = "s";
		if (time > 60) {
			time /= 60; suffix = "m";
			if (time > 60) {
				time /= 60; suffix = "h";
				if (time > 24) {
					time /= 24; suffix = "d";
				}
			}
		}
	}
#endif

	std::ostringstream os;
	os << std::setprecision(precise ? 4 : 1)
		<< std::fixed << time << suffix;

	return os.str();
}
