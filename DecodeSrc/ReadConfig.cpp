#include "ReadConfig.h"
//#include "EncodeMat.hpp"

namespace decode
{
	ReadConfig::~ReadConfig()
	{
	}

	void decode::ReadConfig::read()
	{
		sscfg::ConfigFile co_list = sscfg::ConfigFile::load(_m_fileName);
		cout << "fileName is " << _m_fileName << endl;
		size_t taskNum;
		size_t agentNum;
		co_list.get("AgentNum", agentNum);
		co_list.get("TaskNum", taskNum);

		vector<double> vRatio, vState, vAbility;
		co_list.get("TaskIncreaseRatio", vRatio);
		co_list.get("TaskInitState", vState);
		co_list.get("AgentAbility", vAbility);

		vTaskPntPtr vTaskPtr = make_shared<decode::vTaskPnt>();
		for (size_t i = 0; i < taskNum; i++)
		{
			decode::TaskPnt pnt;
			pnt._increaseRatio = vRatio[i];
			pnt._initState = vState[i];
			vTaskPtr->push_back(pnt);
		}
		this->_m_vTaskPntPtr = vTaskPtr;

		decode::vTaskAgentPtr vAgPtr = make_shared<decode::vTaskAgent>();
		for (size_t i = 0; i < agentNum; i++)
		{
			decode::TaskAgent ag;
			ag._ability = vAbility[i];
			vAgPtr->push_back(ag);
		}
		this->_m_vTaskAgentPtr = vAgPtr;
#ifdef _DEBUG
		cout << "read the dis mat" << endl;
#endif // _DEBUG
		auto taskDisMatPtr = make_shared<decode::DisMap>();
		auto ag2taskDisMatPtr = make_shared<decode::DisMap>();

		vector<double> vag2Task;
		vector<double> vtask2Task;
		co_list.get("TaskDisMat", vtask2Task);
		co_list.get("Ag2TaskDisMat", vag2Task);
		size_t p = 0;


		for (size_t i = 0; i < taskNum; i++)
		{
			for (size_t j = (i + 1); j < taskNum; j++)
			{

				double taskInDis = vtask2Task[p++];
				decode::DisMapIndex dis_index(i, j);
				taskDisMatPtr->insert(std::pair<decode::DisMapIndex, double>(dis_index, taskInDis));
			}
		}

		p = 0;
		for (size_t i = 0; i < agentNum; i++)
		{
			for (size_t j = 0; j < taskNum; j++)
			{
				double ag2TaskDis = vag2Task[p++];
				decode::DisMapIndex dis_index(i, j);
				ag2taskDisMatPtr->insert(std::pair<decode::DisMapIndex, double>(dis_index, ag2TaskDis));
			}
		}

		this->_taskDisMatPtr = taskDisMatPtr;
		this->_ag2taskDisMatPtr = ag2taskDisMatPtr;
		
		//get the encode
		co_list.get("Encode", this->_encode);

#ifdef _DEBUG
		cout << "read configure successs" << endl;
#endif // _DEBUG
	}

	cfgTuple ReadConfig::getCfg()
	{
		tuple<vTaskAgentPtr, vTaskPntPtr, DisMapPtr, DisMapPtr> res(this->_m_vTaskAgentPtr, _m_vTaskPntPtr, _taskDisMatPtr, _ag2taskDisMatPtr);
		return res;
	}


}
