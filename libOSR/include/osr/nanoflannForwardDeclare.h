#pragma once

namespace nanoflann
{
	template <typename Distance, class DatasetAdaptor, int DIM = -1, typename IndexType = size_t>
	class KDTreeSingleIndexAdaptor;

	template<class T, class DataSource, typename _DistanceType = T>
	struct L2_Adaptor;
}