#include "Timer.h"

#include <cmath>

template class Timer<>;

std::string timeString(double time, bool precise)
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
