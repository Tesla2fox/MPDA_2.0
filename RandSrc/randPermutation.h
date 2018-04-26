#pragma once
#include "stadfx.h"

extern 	std::default_random_engine enge;

//返回一个随机排列
std::vector<size_t> randPermutation(const size_t &n);

template<typename T>
inline vector<size_t> sort_indexes(const vector<T>& v)
{
	// initialize original index locations
	vector< size_t>  idx(v.size());
	for (size_t i = 0; i != idx.size(); ++i) idx[i] = i;
	// sort indexes based on comparing values in v
	sort(idx.begin(), idx.end(),
		[&v](size_t i1, size_t i2) {return v[i1] < v[i2]; });

	return idx;
}
