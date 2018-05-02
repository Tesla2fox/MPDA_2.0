#include "stadfx.h"
#include "ReadConfig.h"
#include "StaticConstrnRe.hpp"
#include "OrgEncodeMatR.h"
namespace md = method;
namespace dc = decode;
int main()
{
	
	dc::ReadConfig readCfg("D:\\VScode\\MPDA_orgDecode\\data\\MPDA_cfg.txt");
	readCfg.read();
	readCfg.getCfg();
	clock_t start, c_end;
	start = clock();
	md::StaticConstrn  static_constrn(readCfg._m_vTaskAgentPtr, readCfg._m_vTaskPntPtr, readCfg._taskDisMatPtr, readCfg._ag2taskDisMatPtr);
	//dirConstrn.initialize();
	//static_constrn.calculateSolution();
	static_constrn.minArrTimeAndMaxEmergentSolution();
	static_constrn.minArrTimeAndMinComTimeSolution();
	c_end = clock();
	double dur = (double)(c_end - start);
	cout << "dur is " << dur / CLOCKS_PER_SEC << endl;
	dc::OrgEncodeMat decodeMat(readCfg._m_vTaskAgentPtr, readCfg._m_vTaskPntPtr, readCfg._taskDisMatPtr, readCfg._ag2taskDisMatPtr);
	for (size_t i = 0; i < static_constrn._m_vEnMat.size(); i++)
	{
		vector<vector<int>> &en_mat = static_constrn._m_vEnMat.at(i);
		decodeMat.setOrgEnMat(en_mat);
		auto fitNess = decodeMat.decode();
		cout << i   << "	fitNess is " << fitNess << endl;
	}
	clock_t d_end = clock();

	double d_dur = (double)(d_end - c_end);
	cout << "平均解码时间为" << d_dur / CLOCKS_PER_SEC / 22 << endl;
	//cout << "end" << endl;
	size_t pp;
	std::cin >> pp;
	return 1;
}

 