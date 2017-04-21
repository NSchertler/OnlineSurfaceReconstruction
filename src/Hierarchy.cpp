#include "Hierarchy.h"

template <
AbstractHierarchy::AbstractHierarchy()
	: optimizer(nullptr)
{
}

void AbstractHierarchy::set_optimizer(Optimizer * o)
{
	optimizer = o;
}
