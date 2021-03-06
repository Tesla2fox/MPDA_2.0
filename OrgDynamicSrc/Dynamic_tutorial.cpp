#include "stadfx.h"
#include "ReadConfig.h"
//#include "StaticConstrn.hpp"
#include "DynamicConstrn.hpp"
#include "OrgEncodeMatR.h"
namespace md = method;
namespace dc = decode;
int main(int argc, char * argv[])
{	
	char * conFileName = "D:\\VScode\\MPDA_orgDecode\\data\\MPDA_con_cfg.txt";
	std::cout << " argc = " << argc << endl;
	if (argc > 1)
	{
		std::cout << argv[1] << endl;
		conFileName = argv[1];
	}

	sscfg::ConfigFile conFile = sscfg::ConfigFile::load(conFileName);
	size_t confNum;
	conFile.get("confNum", confNum);
	std::cout << "confNum  = " << confNum << endl;
	vector<double> vMinFitness(confNum);
#ifdef _DEBUG
#else
#pragma omp parallel for num_threads(15)
#endif // _DEBUG
	for (size_t i = 0; i < confNum; i++)
	{
		string str = "cfg";
		str += std::to_string(i);
		string fileName;
		conFile.get(str, fileName);
		dc::ReadConfig readCfg(fileName.data());
		readCfg.read();
		readCfg.getCfg();
		clock_t start, c_end;
		start = clock();
		md::DynamicConstrn  dynamic_constrn(readCfg._m_vTaskAgentPtr, readCfg._m_vTaskPntPtr, readCfg._taskDisMatPtr, readCfg._ag2taskDisMatPtr);
		//dirConstrn.initialize();
		//static_constrn.calculateSolution();
		dynamic_constrn.DminArrTimeAndMaxEmergentSolution();

		//static_constrn.minArrTimeAndMaxEmergentSolution();
		//static_constrn.minArrTimeAndMinComTimeSolution();
		c_end = clock();
		double dur = (double)(c_end - start);
		cout << "dur is " << dur / CLOCKS_PER_SEC << endl;
		dc::OrgEncodeMat decodeMat(readCfg._m_vTaskAgentPtr, readCfg._m_vTaskPntPtr, readCfg._taskDisMatPtr, readCfg._ag2taskDisMatPtr);
		vector<double> vFitNess;

		for (size_t i = 0; i < dynamic_constrn._m_vEnMat.size(); i++)
		{
			vector<vector<int>> &en_mat = dynamic_constrn._m_vEnMat.at(i);
			decodeMat.setOrgEnMat(en_mat);
			auto fitNess = decodeMat.decode();
			vFitNess.push_back(fitNess);
			//cout << i   << "	fitNess is " << fitNess << endl;
		}
		clock_t d_end = clock();
		double d_dur = (double)(d_end - c_end);
		cout << "decode dur = " << d_dur / CLOCKS_PER_SEC / 22 << endl;

		auto minIter = std::min_element(vFitNess.begin(), vFitNess.end());
		cout << "the min fitness is " << *minIter << endl;

	}


	if (argc <= 1)
	{
		size_t pp;
		std::cin >> pp;
	}
	return 1;
}

 