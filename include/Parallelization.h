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

#include <cstdint>
#include <condition_variable>
#include <mutex>
#include <vector>
#include <tbb/tbb.h>
#include <random>

#ifdef _WIN32
#include "windows.h" 
#endif

namespace osr
{
	inline bool atomicCompareAndExchange(volatile uint32_t *v, uint32_t newValue, uint32_t oldValue)
	{
	#if defined(_WIN32)
		return _InterlockedCompareExchange(
			reinterpret_cast<volatile long *>(v), (long)newValue, (long)oldValue) == (long)oldValue;
	#else
		return __sync_bool_compare_and_swap(v, oldValue, newValue);
	#endif
	}

	inline uint32_t atomicAdd(volatile uint32_t *dst, uint32_t delta)
	{
	#if defined(_MSC_VER)
		return _InterlockedExchangeAdd(reinterpret_cast<volatile long *>(dst), delta) + delta;
	#else
		return __sync_add_and_fetch(dst, delta);
	#endif
	}

	inline float atomicAdd(volatile float *dst, float delta)
	{
		union bits { float f; uint32_t i; };
		bits oldVal, newVal;
		do {
	#if defined(__i386__) || defined(__amd64__)
			__asm__ __volatile__("pause\n");
	#endif
			oldVal.f = *dst;
			newVal.f = oldVal.f + delta;
		} while (!atomicCompareAndExchange((volatile uint32_t *)dst, newVal.i, oldVal.i));
		return newVal.f;
	}

	class ordered_lock
	{
	public:
		ordered_lock();
		void lock();
		void unlock();
	protected:
		std::condition_variable  cvar;
		std::mutex               cvar_lock;
		unsigned int             next_ticket, counter;
	};

	// permutes the elements in the container
	// mutex - a vector of mutices for every element in container
	template <typename T>
	void parallel_shuffle(std::vector<T>& container, std::vector<tbb::spin_mutex>& mutex)
	{
		//Fisher-Yates shuffle
		tbb::parallel_for(tbb::blocked_range<size_t>(0u, container.size()),
			[&](const tbb::blocked_range<size_t> &range)
		{
			std::mt19937 rnd(range.begin());
			for (size_t i = range.begin(); i != range.end(); ++i)
			{
				std::uniform_int_distribution<size_t> dist(i, container.size() - 1);
				size_t k = dist(rnd);
				if (i == k)
					continue;
				tbb::spin_mutex::scoped_lock l0(mutex[i]);
				tbb::spin_mutex::scoped_lock l1(mutex[k]);
				std::swap(container[i], container[k]);
			}
		});
	}
}