#pragma once

namespace nanoflann
{
	template <typename Distance, class DatasetAdaptor, int DIM, typename IndexType>
	class KDTreeSingleIndexAdaptor;

	template<class T, class DataSource, typename _DistanceType>
	struct L2_Adaptor;
}