#pragma once

#include <cstdint>
#include <condition_variable>
#include <mutex>
#include <vector>
#include <tbb/tbb.h>
#include <random>

extern inline bool atomicCompareAndExchange(volatile uint32_t *v, uint32_t newValue, uint32_t oldValue);

extern inline uint32_t atomicAdd(volatile uint32_t *dst, uint32_t delta);

extern inline float atomicAdd(volatile float *dst, float delta);

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