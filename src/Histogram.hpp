#pragma once

#include <vector>
#include <iostream>

/// Basic histogram that can count items in bins and normalize this. Type T needs to support
/// operator<(T, T), operator+(T, T), operator*(T, size_t) and operator/(T, size_t)
template <typename T>
class Histogram
{
  private:
	std::vector<int> bins;
	T minVal, maxVal, binSize;
	size_t binCount;

  public:
	Histogram(T minVal, T maxVal, size_t binCount)
		: minVal(minVal), maxVal(maxVal), binSize((maxVal - minVal) / binCount), binCount(binCount),
		  bins(binCount)
	{
	}

	/// Returns the normalized histogram
	std::vector<double> Normalized()
	{
		int total = std::accumulate(bins.begin(), bins.end(), 0);
		if (!total)
			return std::vector<double>(binCount);
		std::vector<double> normalized(binCount);
		std::transform(bins.begin(), bins.end(), normalized.begin(),
					   std::bind(std::multiplies<double>(), std::placeholders::_1, 1.0 / total));
		return normalized;
	}

	/// Adds items into the histogram anything less then the minVal or greater then maxVal will be
	/// discarded
	void AddToHistogram(std::vector<T> &items)
	{
		std::sort(items.begin(), items.end());
		auto iter = items.begin();
		while (*iter < minVal)
			iter++;
		for (size_t i = 0; i < binCount; i++)
			while (*iter < minVal + binSize * (i + 1))
			{
				bins[i]++;
				if (++iter == items.end())
					return;
			}
		if (iter != items.end())
			std::cout << "[WARNING] values exceed histogram range" << std::endl;
	}
};