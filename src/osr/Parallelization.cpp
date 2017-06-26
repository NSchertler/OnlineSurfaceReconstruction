/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Wenzel Jakob
*/

#include "osr/Parallelization.h"

using namespace osr;

ordered_lock::ordered_lock() : next_ticket(0), counter(0) {}
void ordered_lock::lock()
{
	std::unique_lock<std::mutex> acquire(cvar_lock);
	unsigned int ticket = next_ticket++;
	while (ticket != counter)
		cvar.wait(acquire);
}
void ordered_lock::unlock()
{
	std::unique_lock<std::mutex> acquire(cvar_lock);
	counter++;
	cvar.notify_all();
}