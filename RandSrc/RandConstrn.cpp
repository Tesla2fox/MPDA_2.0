#include "RandConstrn.hpp"
#include "randPermutation.h"

namespace method {

	void RandConstrn::RandSolution()
	{
		_m_enMat.clear();
		for (size_t i = 0; i < _agentNum; i++)
		{
			auto randPerm = randPermutation(_taskNum);
			vector<int> taskPerm(randPerm.begin(),randPerm.end());
			_m_enMat.push_back(taskPerm);
		}
	}
	void RandConstrn::writeSolution()
	{
		r_debug << "__________________________" << endl;
		for (size_t i = 0; i < _agentNum; i++)
		{
			r_debug << "agent id" << i;
			for (size_t j = 0; j < _taskNum; j++)
			{
				r_debug << " " << _m_enMat[i][j];
			}
			r_debug << endl;
		}
	}

	bool EnMatEqualOrNot(EnMat const & a, EnMat const &b)
	{
		if (a.size() != b.size())
		{
			return false;
		}
		for (size_t i = 0; i < b.size(); i++)
		{
			auto &aUnit = a[i];
			auto &bUnit = b[i];
			for (size_t j = 0; j < aUnit.size(); j++)
			{
				if (aUnit[j] != bUnit[j])
				{
					return false;
				}
			}
		}
		return true;
	}

}
