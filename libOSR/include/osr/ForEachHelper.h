#pragma once

#include <tbb/parallel_for_each.h>

template<typename TIterator>
class ForEachHelper
{
public:
	ForEachHelper(TIterator begin, TIterator end)
		: _begin(begin), _end(end)
	{ }

	TIterator begin() const { return _begin; }
	TIterator end() const { return _end; }

	template <typename TFunction>
	void processInParallel(const TFunction& f)
	{
		//Dummy implementation, should be specialized to achieve better parallel performance
		tbb::parallel_for_each(_begin, _end, f);
		//for (auto it = _begin; it != _end; ++it)
		//	f(*it);

		//override in the global namespace with:
		/*template<>
		template<typename TFunction>
		void ForEachHelper<SpecialIterator>::processInParallel(const TFunction& f) {  }*/
	}

private:
	TIterator _begin, _end;
};