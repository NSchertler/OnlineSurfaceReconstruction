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

#pragma once

#include <chrono>
#include <string>
#include <iomanip>
#include <sstream>

namespace osr
{
	template <typename TimeT = std::chrono::milliseconds>
	class Timer
	{
	public:
		Timer()
		{
			start = std::chrono::system_clock::now();
		}

		size_t value() const
		{
			auto now = std::chrono::system_clock::now();
			auto duration = std::chrono::duration_cast<TimeT>(now - start);
			return (size_t)duration.count();
		}

		size_t reset()
		{
			auto now = std::chrono::system_clock::now();
			auto duration = std::chrono::duration_cast<TimeT>(now - start);
			start = now;
			return (size_t)duration.count();
		}
	private:
		std::chrono::system_clock::time_point start;
	};

	//Converts the provided number of milliseconds to a readable string.
	//If precise=true, uses milliseconds. If precise=false, converts to 
	//an appropriate unit.
	extern ::std::string timeString(double time, bool precise = false);
}