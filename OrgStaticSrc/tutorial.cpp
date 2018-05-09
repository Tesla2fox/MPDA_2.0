#include "stadfx.h"
#include "ReadConfig.h"
#include "StaticConstrn.hpp"
#include "OrgEncodeMatR.h"
#include "ssconfig.hpp"
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

	clock_t t_start, t_end;
	t_start = clock();
#pragma omp parallel for num_threads(15)
	for (int i = 0; i < confNum; i++)
	{
		std::string str = "cfg";
		str += std::to_string(i);
//		char * fileName;
		string fileName;
		conFile.get(str, fileName);
		const char* c_fileName = fileName.data();
		dc::ReadConfig readCfg(c_fileName);
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
		vector<double> vFitNess;

		for (size_t i = 0; i < static_constrn._m_vEnMat.size(); i++)
		{
			vector<vector<int>> &en_mat = static_constrn._m_vEnMat.at(i);
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
		vMinFitness[i] = *minIter;
		cout << "_______________" << endl;
	}

	t_end = clock();
	double dur = (double)(t_end - t_start);
	std::ofstream c_debug;
	c_debug.open("c_deg_omp.txt", std::ios::trunc);
	
	for (size_t i = 0; i < confNum; i++)
	{
		c_debug << "index = " << i << " fitness  =" << vMinFitness[i] << endl;
	}
	c_debug << "total time is " << dur/CLOCKS_PER_SEC << endl;

	if (argc <= 1)
	{
		size_t pp;
		std::cin >> pp;
	}
	return 1;
}

 