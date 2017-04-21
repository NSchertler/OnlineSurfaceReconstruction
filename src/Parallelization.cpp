#include "Parallelization.h"

#ifdef _WIN32
#include "windows.h" 
#endif

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


ordered_lock::ordered_lock()  : next_ticket(0), counter(0) {}
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