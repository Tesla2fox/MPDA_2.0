#pragma once
#include "stadfx.h"
#include "TaskPnt.hpp"
#include "randPermutation.h"
namespace method {

	using namespace decode;

	using EnMat = vector<vector<int>>;
	class RandConstrn
	{
	public:
		RandConstrn(size_t agentNum, size_t taskNum):
		_agentNum(agentNum),_taskNum(taskNum)
		{
#ifdef _DEBUG
			r_debug.open("r_deg.txt", std::ios::trunc);
			r_debug.precision(15);
#endif // _DEBUG
			enge.seed(time(nullptr));
		}
		void RandSolution();
		void writeSolution();
		EnMat _m_enMat;
	private:
		size_t _agentNum;
		size_t _taskNum;

		std::ofstream r_debug;
	};
}