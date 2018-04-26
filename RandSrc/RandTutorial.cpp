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
	md::RandConstrn rand_constrn(readCfg._m_vTaskAgentPtr->size(), readCfg._m_vTaskPntPtr->size());

//	md::RandConstrn rand_constrn(5,4);
	dc::OrgEncodeMat decodeMat(readCfg._m_vTaskAgentPtr, readCfg._m_vTaskPntPtr, readCfg._taskDisMatPtr, readCfg._ag2taskDisMatPtr);
	vector<double> vFitNess;
	clock_t start, r_end;
	start = clock();
	for (size_t i = 0; i < 1000; i++)
	{
		rand_constrn.RandSolution();
		rand_constrn.writeSolution();
		vector<vector<int>> &en_mat= rand_constrn._m_enMat;
		decodeMat.setOrgEnMat(en_mat);
		auto fitNess = decodeMat.decode();
		vFitNess.push_back(fitNess);
//		if (fitNess > M_MAX)
//		{
//		}
//		else {
//			cout << i << "	fitNess is " << fitNess << endl;
////			system("pause");
//		}
		if(i%1000==0)
		{ 
			cout << i << "	fitNess is " << fitNess << endl;
		}
	}
	r_end = clock();
	double dur = r_end - start;
	cout << "the dur is " << dur / CLOCKS_PER_SEC << endl;
	auto minIter = std::min_element(vFitNess.begin(), vFitNess.end());
	cout << "the min fitness is " << *minIter << endl;
	if(argc <= 1)
	{
		size_t pp;
		std::cin >> pp;
	}
	return 0;
}

 