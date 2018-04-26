#include "stadfx.h"
#include "ReadConfig.h"
//#include "StaticConstrnRe.hpp"
#include "RandConstrn.hpp"
#include "OrgEncodeMatR.h"
namespace md = method;
namespace dc = decode;
int main(int argc, char * argv[])
{
	char * fileName = "D:\\VScode\\MPDA_orgDecode\\data\\MPDA_cfg.txt";
	std::cout << " argc = " << argc << endl;
	if (argc > 1)
	{
		std::cout << argv[1] << endl;
		fileName = argv[1];
	}
	dc::ReadConfig readCfg(fileName);
	readCfg.read();
	readCfg.getCfg();
	clock_t start, c_end;
	start = clock();
	md::RandConstrn rand_constrn(readCfg._m_vTaskAgentPtr->size(), readCfg._m_vTaskPntPtr->size());

//	md::RandConstrn rand_constrn(5,4);
	dc::OrgEncodeMat decodeMat(readCfg._m_vTaskAgentPtr, readCfg._m_vTaskPntPtr, readCfg._taskDisMatPtr, readCfg._ag2taskDisMatPtr);
	for (size_t i = 0; i < 1000; i++)
	{
		rand_constrn.RandSolution();
		vector<vector<int>> &en_mat = rand_constrn._m_enMat;
		decodeMat.setOrgEnMat(en_mat);
		auto fitNess = decodeMat.decode();
		if (fitNess > M_INFINITY)
		{
			cout << i << endl;
		}
		else {
			cout << i << "	fitNess is " << fitNess << endl;
		}
	}
	
	return 0;
}

 