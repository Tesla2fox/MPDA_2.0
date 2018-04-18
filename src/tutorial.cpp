#include "stadfx.h"
#include "ReadConfig.h"
#include "StaticConstrn.hpp"
#include "OrgEncodeMat.h"
namespace md = method;
namespace dc = decode;
int main()
{
	
	dc::ReadConfig readCfg("D:\\VScode\\MPDA_orgDecode\\data\\MPDA_cfg.txt");
	readCfg.read();
	readCfg.getCfg();
	md::StaticConstrn  static_constrn(readCfg._m_vTaskAgentPtr, readCfg._m_vTaskPntPtr, readCfg._taskDisMatPtr, readCfg._ag2taskDisMatPtr);
	//dirConstrn.initialize();
	//static_constrn.calculateSolution();
	static_constrn.minArrTimeAndMaxEmergentSolution();
	static_constrn.minArrTimeAndMinComTimeSolution();

	dc::OrgEncodeMat decodeMat(readCfg._m_vTaskAgentPtr, readCfg._m_vTaskPntPtr, readCfg._taskDisMatPtr, readCfg._ag2taskDisMatPtr);
	for (size_t i = 0; i < static_constrn._m_vEnMat.size(); i++)
	{
		vector<vector<int>> &en_mat = static_constrn._m_vEnMat.at(i);
		decodeMat.setOrgEnMat(en_mat);
		auto fitNess = decodeMat.decode();
		cout << i   << "	fitNess is " << fitNess << endl;
	}
	//static_constrn.minArrTimeAndMaxEmergentSolution()
	cout << "end" << endl;
	//int ppp;
	//std::cin >> ppp;
	return 1;
}

 