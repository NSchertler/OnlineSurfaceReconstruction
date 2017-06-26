/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Nico Schertler
*/

#pragma once

#include <vector>
#include <deque>
#include <algorithm>

#include "osr/Serialization.h"

namespace osr
{
	struct Interval
	{
		Interval() { }
		Interval(size_t lowerInclusive, size_t upperExclusive)
			: lowerInclusive(lowerInclusive), upperExclusive(upperExclusive)
		{ }

		size_t lowerInclusive;
		size_t upperExclusive;
	};

	template <typename T, typename Allocator>
	class EntryIterator;

	//Represents a container that allows add and remove while keeping indices persistent.
	template <typename T, typename Allocator = std::allocator<T> >
	class PersistentIndexContainer
	{
	public:

		typedef EntryIterator<T, Allocator> iterator;

		PersistentIndexContainer()
			: totalEmptySlots(0)
		{ }

		size_t insert()
		{
			if (!emptySlots.empty())
			{
				auto& emptySlot = emptySlots.front();
				size_t slot = emptySlot.lowerInclusive;
				++emptySlot.lowerInclusive;
				if (emptySlot.lowerInclusive == emptySlot.upperExclusive)
					emptySlots.pop_front();
				--totalEmptySlots;
				return slot;
			}
			else
			{
				data.emplace_back();
				return data.size() - 1;
			}
		}

		T& operator[](size_t index) { return data[index]; }
		const T& operator[](size_t index) const { return data[index]; }

		void clear()
		{
			data.clear();
			emptySlots.clear();
			totalEmptySlots = 0;
		}

		void reserve(size_t additionalElements)
		{
			size_t moreSpaceNeeded = additionalElements - totalEmptySlots;
			if (moreSpaceNeeded > 0)
				data.reserve(data.size() + moreSpaceNeeded);
		}

		size_t sizeWithGaps() const { return data.size(); }
		size_t sizeNotDeleted() const { return data.size() - totalEmptySlots; }

		bool isDeleted(size_t index)
		{
			if (index >= data.size())
				return true;
			for (auto& range : emptySlots)
			{
				if (index < range.lowerInclusive)
					return false;
				if (index < range.upperExclusive)
					return true;
			}
			return false;
		}

		void erase(size_t index)
		{
			assert(!isDeleted(index));

			data[index] = T();

			//find the first empty interval whose max is greater or equal to index
			auto emptyIntervalIt = std::lower_bound(emptySlots.begin(), emptySlots.end(), index,
				[](const Interval& interval, size_t index)
			{
				return interval.upperExclusive < index;
			});

			if (emptyIntervalIt == emptySlots.end())
			{
				emptySlots.emplace_back(index, index + 1);
				++totalEmptySlots;
				return;
			}

			if (index >= emptyIntervalIt->lowerInclusive && index < emptyIntervalIt->upperExclusive)
				return; //index is already deleted

			totalEmptySlots++;

			if (index == emptyIntervalIt->upperExclusive)
			{
				//we can extend the interval by +1
				emptyIntervalIt->upperExclusive++;
				//check if we can merge with the next
				auto nextIt = emptyIntervalIt + 1;
				if (nextIt != emptySlots.end() && emptyIntervalIt->upperExclusive == nextIt->lowerInclusive)
				{
					emptyIntervalIt->upperExclusive = nextIt->upperExclusive;
					emptySlots.erase(nextIt);
				}
			}
			else if (index == emptyIntervalIt->lowerInclusive - 1)
			{
				//we can extend the interval by -1
				emptyIntervalIt->lowerInclusive--;
				//check if we can merge with the previous interval
				if (emptyIntervalIt != emptySlots.begin())
				{
					auto prev = emptyIntervalIt - 1;
					if (prev->upperExclusive == emptyIntervalIt->lowerInclusive)
					{
						emptyIntervalIt->lowerInclusive = prev->lowerInclusive;
						emptySlots.erase(prev);
					}
				}
			}
			else
			{
				//we have to generate a new interval
				emptySlots.insert(emptyIntervalIt, Interval(index, index + 1));
			}
		}

		iterator erase(iterator it)
		{
			assert(!it.deleted());

			*it = T();

			totalEmptySlots++;

			bool canExtendFromPrev = it.nextEmptyInterval != emptySlots.begin() && it.currentIndex == (it.nextEmptyInterval - 1)->upperExclusive;
			if (it.nextEmptyInterval != emptySlots.end() && it.currentIndex == it.nextEmptyInterval->lowerInclusive - 1)
			{
				if (canExtendFromPrev)
				{
					//fill a gap
					auto prevIt = it.nextEmptyInterval - 1;
					prevIt->upperExclusive = it.nextEmptyInterval->upperExclusive;
					auto afterDeleteIt = emptySlots.erase(it.nextEmptyInterval);
					return iterator(it.currentIndex, afterDeleteIt - 1, this);
				}
				else
				{
					//extend next interval by -1
					it.nextEmptyInterval->lowerInclusive--;
					it.advanceUntilValid();
					return it;
				}
			}

			if (canExtendFromPrev)
			{
				//extend previous interval by +1
				(it.nextEmptyInterval - 1)->upperExclusive++;
				it.currentIndex++;
				it.advanceUntilValid();
				return it;
			}

			auto slotIt = emptySlots.insert(it.nextEmptyInterval, Interval(it.currentIndex, it.currentIndex + 1));
			return iterator(it.currentIndex, slotIt, this);
		}

		iterator begin() { return iterator(0, emptySlots.begin(), this); }
		iterator end() { return iterator(data.size(), emptySlots.end(), this); }

		void saveToFile(FILE* f) const
		{
			osr::saveToFile(data, f);
			osr::saveToFile(emptySlots, f);
			osr::saveToFile(totalEmptySlots, f);
		}

		void loadFromFile(FILE* f)
		{
			osr::loadFromFile(data, f);
			osr::loadFromFile(emptySlots, f);
			osr::loadFromFile(totalEmptySlots, f);
		}

	private:
		std::vector<T, Allocator> data;

		std::deque<Interval> emptySlots; //sorted
		size_t totalEmptySlots;

		friend iterator;
	};

	template <typename T, typename Allocator>
	void saveToFile(const PersistentIndexContainer<T, Allocator>& object, FILE* f) { object.saveToFile(f); }
	template <typename T, typename Allocator>
	void loadFromFile(PersistentIndexContainer<T, Allocator>& object, FILE* f) { object.loadFromFile(f); }


	template <typename T, typename Allocator>
	class EntryIterator : public std::iterator<std::forward_iterator_tag, T>
	{
	public:
		EntryIterator(size_t currentIndex, std::deque<Interval>::iterator nextEmptyInterval, PersistentIndexContainer<T, Allocator>* container)
			: currentIndex(currentIndex), nextEmptyInterval(nextEmptyInterval), container(container)
		{
			advanceUntilValid();
		}

		EntryIterator<T, Allocator>& operator=(const EntryIterator<T, Allocator>& copy) = default;
		EntryIterator(const EntryIterator<T, Allocator>& copy) = default;

		EntryIterator<T, Allocator> operator++() { currentIndex++; advanceUntilValid(); return *this; }
		bool operator!=(const EntryIterator<T, Allocator>& other) const { return currentIndex != other.currentIndex; }
		bool operator==(const EntryIterator<T, Allocator>& other) const { return currentIndex == other.currentIndex; }
		T& operator*() { return container->data[currentIndex]; }
		T* operator->() { return &container->data[currentIndex]; }

		bool deleted() const
		{
			if (nextEmptyInterval == container->emptySlots.end())
				return false;
			return currentIndex >= nextEmptyInterval->lowerInclusive && currentIndex < nextEmptyInterval->upperExclusive;
		}

		size_t index() const { return currentIndex; }

	private:
		void advanceUntilValid()
		{
			if (nextEmptyInterval == container->emptySlots.end())
				return;
			if (currentIndex >= nextEmptyInterval->lowerInclusive)
			{
				currentIndex = nextEmptyInterval->upperExclusive;
				++nextEmptyInterval;
			}
		}

		size_t currentIndex;

		std::deque<Interval>::iterator nextEmptyInterval;
		PersistentIndexContainer<T, Allocator>* container;

		friend class PersistentIndexContainer<T>;
	};
}