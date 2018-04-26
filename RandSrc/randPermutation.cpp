//#include "randN.h"
#include "randPermutation.h"

std::default_random_engine enge(2);

std::vector<size_t> randPermutation(const size_t &n)
{
	//std::default_random_engine eng(time(nullptr));
	
	std::uniform_int_distribution<int> dis(1, 2000);

	std::vector<std::tuple<size_t, int>> vUnit;
	for (size_t i = 0; i < n; i++)
	{
		std::tuple<size_t, int> Unit(i, dis(enge));
		vUnit.push_back(Unit);
	}

	using compUnit = std::tuple<size_t, int>;
	std::function<bool(compUnit, compUnit)> cmpFunc =
		[=](compUnit A, compUnit B)
	{
		auto a2 = std::get<1>(A);
		auto b2 = std::get<1>(B);
		return a2 < b2 ? true : false;
	};

	std::sort(vUnit.begin(), vUnit.end(), cmpFunc);

	std::vector<size_t> res;

	for (auto &it : vUnit)
	{
		res.push_back(std::get<0>(it));
	}
	return res;
}
