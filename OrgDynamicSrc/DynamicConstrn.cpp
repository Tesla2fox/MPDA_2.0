//#include "StaticConstrn.hpp"
#include "DynamicConstrn.hpp"

namespace method {

	DynamicConstrn::~DynamicConstrn()
	{
#ifdef _DEBUG
		std::cout << "DynamicConstrn_end" << endl;
#endif // _DEBUG
	}
	bool DynamicConstrn::initialize()
	{
		_m_arriveMat.clear();
		_m_completeMat.clear();
		_m_enMat.clear();

		//
		vector<int> taskPerm(_taskNum,-1);
		for (size_t i = 0; i < _agentNum; i++)
		{
			_m_enMat.push_back(taskPerm);
		}
#ifdef _DEBUG
		this->writeEnMat();
#endif // _DEBUG
		

		this->_vAgentState.clear();
		this->_vTaskState.clear();

		//构建智能体能力值序列
		auto  vAgAbilityPtr = make_shared<vector<double>>();
		for (size_t i = 0; i < _agentNum; i++)
		{
			vAgAbilityPtr->push_back(_vTaskAgent[i]._ability);
		}

		for (size_t i = 0; i < _taskNum; i++)
		{
			TaskState tkstate(vAgAbilityPtr,_agentNum,this->c_debug);
			tkstate._currentState = _vTaskPnt.at(i)._initState;
			tkstate._initState = _vTaskPnt.at(i)._initState;
			tkstate._currentRatio = _vTaskPnt.at(i)._increaseRatio;
			tkstate._changeRatioTime = 0;
			tkstate._id = i;
#ifdef _DEBUG
			c_debug << "task id  = " << i << endl;
#endif // _DEBUG
			for (size_t j = 0; j < _agentNum; j++)
			{
				DisMapIndex dis_index(j, i);
				tkstate._vPreArrTime.push_back(_ag2taskDisMat.at(dis_index));
				double completeTime =
					tkstate.preCalCompleteDur(tkstate._vPreArrTime.back(), _vTaskAgent[j]._ability);
				tkstate._vPreComTime.push_back(completeTime);
#ifdef _DEBUG
				c_debug << " agent id = " << j << "  arriveTime = " << _ag2taskDisMat.at(dis_index)
					<< " completeTime = " << completeTime << endl;
#endif // _DEBUG
			}			
			_vTaskState.push_back(tkstate);
		}

		for (size_t i = 0; i < _agentNum; i++)
		{
			AgentState agstate;
			agstate._taskid = -1;
			agstate._ability = _vTaskAgent.at(i)._ability;
			//c_debug << "agent id  = " << i ;
			for (size_t j = 0; j < _taskNum; j++)
			{
				DisMapIndex dis_index(i, j);
				//double arr
				agstate._vPreArrTime.push_back(_ag2taskDisMat.at(dis_index));				
				double completeTime =
					this->_vTaskState[j].preCalCompleteDur(agstate._vPreArrTime.back(), agstate._ability);
				agstate._vPreComTime.push_back(completeTime);
				//c_debug << " task id = " << j << "  arriveTime = " << _ag2taskDisMat.at(dis_index)
					//<< " completeTime = " << completeTime << endl;
				//c_debug << "  " << completeTime << endl;
			}
			_vAgentState.push_back(agstate);
		}
		c_debug << endl;
		return false;
	}
	bool DynamicConstrn::sortOnTaskDur()
	{

		return false;
	}
	void DynamicConstrn::updatePreArrAndComTime()
	{
		for (size_t i = 0; i < _taskNum; i++)
		{
			auto & tkstate = this->_vTaskState[i];
#ifdef _DEBUG
			c_debug << "task id  = " << i << endl;
#endif // _DEBUG
			for (size_t j = 0; j < _agentNum; j++)
			{
				DisMapIndex dis_index(j, i);
				tkstate._vPreArrTime.push_back(_ag2taskDisMat.at(dis_index));
				double completeTime =
					tkstate.preCalCompleteDur(tkstate._vPreArrTime.back(), _vTaskAgent[j]._ability);
				tkstate._vPreComTime.push_back(completeTime);
#ifdef _DEBUG
				c_debug << " agent id = " << j << "  arriveTime = " << _ag2taskDisMat.at(dis_index)
					<< " completeTime = " << completeTime << endl;
#endif // _DEBUG
			}
		}
	}
	void DynamicConstrn::updateEnmat(size_t const & type)
	{
		//到达时间的排序
		const map<AgTaskPair, size_t> &orderPA{ this->_orderPreFirstArrTime };
		
		//紧急程度的排序
		const map<AgTaskPair, size_t> &orderPE{ this->_orderPreEmergenceComTime };

#ifdef _DEBUG
		c_debug << " ____D_____minArrTimeAndMaxEmergentSolution___D_______" << endl;
#endif // DEBUG

		
		vector<size_t> vChsAgID;
		vector<AgTaskTimePair> vAgTaskOrderVal;
		map<AgTaskPair, double> mapAgTaskOrderVal;
		for (size_t i = 0; i < _agentNum; i++)
		{
			for (size_t j = 0; j < _taskNum; j++)
			{
				AgTaskPair indexPair(i, j);
				double orderVal = weightUnit*_orderPreFirstArrTime[indexPair] +
					(1 - weightUnit)*_orderPreEmergenceComTime[indexPair];
				vAgTaskOrderVal.push_back(AgTaskTimePair(indexPair, orderVal));
				mapAgTaskOrderVal.insert(AgTaskTimePair(indexPair, orderVal));
			}
		}
		
		auto minElementIter
		= std::min_element(vAgTaskOrderVal.begin(), vAgTaskOrderVal.end(), ComparePreFirstTime(*this));
		//minElementIter->first
		
		size_t chsAgID = (*minElementIter).first.first;
		size_t chsTskID = (*minElementIter).first.second;

		auto & tskState = _vTaskState[chsTskID];		
		calChsComPreTime(chsAgID, chsTskID);
		vChsAgID.push_back(chsAgID);

#ifdef _DEBUG
		c_debug << "find the insert chsAgID and chsTskID" << endl;
		c_debug << "chsAgID = " << chsAgID << endl;
		c_debug << "chsTskID = "<<chsTskID << endl;
		c_debug << "___" << endl;
#endif // DEBUG

		
		//compare function
		using compUnit = pair<size_t, double>;
		std::function<bool(compUnit, compUnit)> cmpFunc =
			[=](compUnit const &A, compUnit const &B)
		{
			auto a2 = A.second;
			auto b2 = B.second;
			return a2 < b2 ? true : false;
		};


		while (tskState._chsComPreTime ==M_MAX)
		{
			c_debug << "need add some agent in this task" << endl;
			vector<pair<size_t, double>> _vAgSumOrderVal;
			for (size_t i = 0; i < _agentNum; i++)
			{
				if (tskState._onTaskAgID[i] == false)
				{
					double sumOrderVal = 0;
					for (size_t j = 0; j < _taskNum; j++)
					{
						AgTaskPair indexPair(i, j);
						if (_orderPreFirstArrTime.count(indexPair) == 1)
						{
							sumOrderVal += mapAgTaskOrderVal[indexPair];
						}
						else
						{
#ifdef _DEBUG
							c_debug << "not exist" << endl;
#endif // _DEBUG
						}
					}
					_vAgSumOrderVal.push_back(pair<size_t, double>(i, sumOrderVal));
				}
			}
			auto maxAgSumOrderValIter =
				std::max_element(_vAgSumOrderVal.begin(), _vAgSumOrderVal.end(), cmpFunc);
			
			chsAgID = (*maxAgSumOrderValIter).first;
			
			//_setAllocated.insert(par)
			///updateTimeMat(chsAgID, chsTskID);
			calChsComPreTime(chsAgID, chsTskID);		
			vChsAgID.push_back(chsAgID);
		}

		//update the agent leave time
		//there may be false 
		for (auto &it : tskState._sortedArrTimeAgID)
		{
			auto &agState = _vAgentState[it];
			agState._leaveTime = tskState._chsComPreTime;
		}
#ifdef _DEBUG
		c_debug << "*******************" << endl;
		for (auto &it :vChsAgID)
		{
			c_debug << "chsAgID = " << it << endl;
		}
#endif // _DEBUG
		updateTimeMat(vChsAgID, chsTskID);
		
	}
	void DynamicConstrn::DminArrTimeAndMaxEmergentSolution()
	{
		//double weightUnit;
		for (size_t i = 0; i < _vMaxWeight.size(); i++)
		{
			//当前分配的权重大小
			weightUnit = this->_vMaxWeight[0];
#ifdef _DEBUG
			c_debug << "______weightUnit = " << weightUnit << endl;
#endif // _DEBUG

			this->initialize();
			do
			{
				this->_orderPreFirstArrTime.clear();
				this->_orderPreFirstComTime.clear();
				this->_orderPreEmergenceComTime.clear();
				sortPreFirstArrTime();
				sortPreEmergenceComTime();
				updateEnmat(1);
				updatePreArrAndComTime();
			} while (true);
		}
	}
	void DynamicConstrn::sortPreFirstArrTime()
	{
		vector<AgTaskTimePair> vPreFirstArrTime;
		for (size_t i = 0; i < _taskNum; i++)
		{
			auto & tskState = _vTaskState[i];
			for (size_t j = 0; j < _agentNum; j++)
			{
				vPreFirstArrTime.push_back(AgTaskTimePair(AgTaskPair(j, i), tskState._vPreArrTime[j]));
			}
		}
		
		std::sort(vPreFirstArrTime.begin(), vPreFirstArrTime.end(), ComparePreFirstTime(*this));

		double val = -1;
		int orderIndex = -1;
		for (size_t i = 0; i < vPreFirstArrTime.size(); i++)
		{
			if (val == vPreFirstArrTime[i].second)
			{
				this->_orderPreFirstArrTime.insert(pair<AgTaskPair, size_t>(AgTaskPair(vPreFirstArrTime[i].first), orderIndex));
			}
			else
			{
				val = vPreFirstArrTime[i].second;
				orderIndex = i;
				this->_orderPreFirstArrTime.insert(pair<AgTaskPair, size_t>(AgTaskPair(vPreFirstArrTime[i].first), orderIndex));
			}
		}		
		std::cout << "arrive time sort over" << endl;
	}
	void DynamicConstrn::sortPreEmergenceComTime()
	{
		//
		//second is the ratioBais
		//third is the arrTime
		using AgTaskTuple = tuple<AgTaskPair, double, double>;
		vector<AgTaskTuple>  vAgTaskComTuple;
		//infinite vector
		vector<AgTaskTimePair> vComTimeInfVal;
		//not infinite vector
		vector<AgTaskTimePair> vPreFirstComTime;

		double maxRatioBais = -1;
		double maxArrTime = -1;
		for (size_t i = 0; i < _taskNum; i++)
		{
			auto & tskState = _vTaskState[i];
			for (size_t j = 0; j < _agentNum; j++)
			{
				if (tskState._vPreComTime[j] == M_MAX)
				{
					double baisRatio = tskState._currentRatio - tskState._vAgAbilityPtr->at(j);
					if (maxRatioBais < baisRatio)
					{
						maxRatioBais = baisRatio;
					}
					if (maxArrTime < tskState._vPreArrTime[j])
					{
						maxArrTime = tskState._vPreArrTime[j];
					}
					vAgTaskComTuple.push_back(AgTaskTuple(AgTaskPair(j, i), baisRatio, tskState._vPreArrTime[j]));
				}
				else
				{
					vPreFirstComTime.push_back(AgTaskTimePair(AgTaskPair(j, i), tskState._vPreComTime[j]));
				}
			}
		}

		//std::cout << "over 1" << endl;
		for (size_t i = 0; i < vAgTaskComTuple.size(); i++)
		{
			double &baisRatio = get<1>(vAgTaskComTuple[i]);
			double &arrTime = get<2>(vAgTaskComTuple[i]);
			double maxVal = -1;
			for (size_t j = 0; j < _vMaxWeight.size(); j++)
			{
				auto &weightUnit = _vMaxWeight[j];
				double val = weightUnit*(baisRatio / maxRatioBais) + (1 - weightUnit)*(arrTime / maxArrTime);
				if (val > maxVal)
				{
					maxVal = val;
				}
			}

			vComTimeInfVal.push_back(AgTaskTimePair(get<0>(vAgTaskComTuple[i]), maxVal));
		}
		
		std::sort(vComTimeInfVal.begin(), vComTimeInfVal.end(), counterComparePreFirstTime(*this));

		std::sort(vPreFirstComTime.begin(), vPreFirstComTime.end(), counterComparePreFirstTime(*this));


		double val = -1;
		int orderIndex = -1;
		for (size_t i = 0; i < vComTimeInfVal.size(); i++)
		{
			if (val == vComTimeInfVal[i].second)
			{
				this->_orderPreEmergenceComTime.insert(pair<AgTaskPair, size_t>(AgTaskPair(vComTimeInfVal[i].first), orderIndex));
			}
			else
			{
				val = vComTimeInfVal[i].second;
				orderIndex = i;
				this->_orderPreEmergenceComTime.insert(pair<AgTaskPair, size_t>(AgTaskPair(vComTimeInfVal[i].first), orderIndex));
			}
		}
		size_t infoNum = vComTimeInfVal.size();
		val = -1;
		for (size_t i = infoNum; i < (vPreFirstComTime.size() + infoNum); i++)
		{
			if (val == vPreFirstComTime[i- infoNum].second)
			{
				this->_orderPreEmergenceComTime.insert(pair<AgTaskPair, size_t>(AgTaskPair(vPreFirstComTime[i- infoNum].first), orderIndex));
			}
			else
			{
				val = vPreFirstComTime[i- infoNum].second;
				orderIndex = i;
				this->_orderPreEmergenceComTime.insert(pair<AgTaskPair, size_t>(AgTaskPair(vPreFirstComTime[i - infoNum].first), orderIndex));
			}
		}

		std::cout << "_orderPreEmergenceComTime is over" << endl;
	}
	void DynamicConstrn::sortPreFirstComTime()
	{
		//
		//second is the ratioBais
		//third is the arrTime
		using AgTaskTuple = tuple<AgTaskPair, double, double>;
		vector<AgTaskTuple>  vAgTaskComTuple;
		//infinite vector
		vector<AgTaskTimePair> vComTimeInfVal;
		//not infinite vector
		vector<AgTaskTimePair> vPreFirstComTime;

		double maxRatioBais = -1;
		double maxArrTime = -1;
		for (size_t i = 0; i < _taskNum; i++)
		{
			auto & tskState = _vTaskState[i];
			for (size_t j = 0; j < _agentNum; j++)
			{
				if (tskState._vPreComTime[j] == M_MAX)
				{
					double baisRatio = tskState._currentRatio - tskState._vAgAbilityPtr->at(j);
					if (maxRatioBais < baisRatio)
					{
						maxRatioBais = baisRatio;
					}
					if (maxArrTime < tskState._vPreArrTime[j])
					{
						maxArrTime = tskState._vPreArrTime[j];
					}
					vAgTaskComTuple.push_back(AgTaskTuple(AgTaskPair(j, i), baisRatio, tskState._vPreArrTime[j]));
				}
				else
				{
					vPreFirstComTime.push_back(AgTaskTimePair(AgTaskPair(j, i), tskState._vPreComTime[j]));
				}
			}
		}

		for (size_t i = 0; i < vAgTaskComTuple.size(); i++)
		{
			double &baisRatio = get<1>(vAgTaskComTuple[i]);
			double &arrTime = get<2>(vAgTaskComTuple[i]);
			double maxVal = -1;
			for (size_t j = 0; j < _vMaxWeight.size(); j++)
			{
				auto &weightUnit = _vMaxWeight[j];
				double val = weightUnit*(baisRatio / maxRatioBais) + (1 - weightUnit)*(arrTime / maxArrTime);
				if (val > maxVal)
				{
					maxVal = val;
				}
			}
			vComTimeInfVal.push_back(AgTaskTimePair(get<0>(vAgTaskComTuple[i]), maxVal));
		}

		std::sort(vComTimeInfVal.begin(), vComTimeInfVal.end(), ComparePreFirstTime(*this));

		std::sort(vPreFirstComTime.begin(), vPreFirstComTime.end(), ComparePreFirstTime(*this));


		double val = -1;
		int orderIndex = -1;


		for (size_t i = 0; i < vPreFirstComTime.size(); i++)
		{
			if (val == vPreFirstComTime[i].second)
			{
				this->_orderPreFirstComTime.insert(pair<AgTaskPair, size_t>(AgTaskPair(vPreFirstComTime[i].first), orderIndex));
			}
			else
			{
				val = vPreFirstComTime[i].second;
				orderIndex = i;
				this->_orderPreFirstComTime.insert(pair<AgTaskPair, size_t>(AgTaskPair(vPreFirstComTime[i].first), orderIndex));
			}
		}
		size_t finSize = vPreFirstComTime.size();
		val = -1;
		for (size_t i = finSize; i < (vComTimeInfVal.size()+ finSize); i++)
		{
			if (val == vComTimeInfVal[i - finSize].second)
			{
				this->_orderPreFirstComTime.insert(pair<AgTaskPair, size_t>(AgTaskPair(vComTimeInfVal[i - finSize].first), orderIndex));
			}
			else
			{
				val = vComTimeInfVal[i - finSize].second;
				orderIndex = i;
				this->_orderPreFirstComTime.insert(pair<AgTaskPair, size_t>(AgTaskPair(vComTimeInfVal[i - finSize].first), orderIndex));
			}
		}

		std::cout << "_orderPreFirstComTime" << endl;
	}
	void DynamicConstrn::writeEnMat()
	{
		for (size_t i = 0; i < _agentNum; i++)
		{
			c_debug << "agent id" << i;
				for (size_t j = 0; j < _taskNum; j++)
				{
					c_debug << " " << _m_enMat[i][j];
				}
				c_debug << endl;
		}
	}
	void DynamicConstrn::writevTaskState()
	{
		c_debug << "___________________" << endl;
		for (size_t i = 0; i < _taskNum; i++)
		{
			c_debug << "task id = " << i << endl;
			auto &tskState = _vTaskState[i];
			for (size_t j = 0; j < _agentNum; j++)
			{
				c_debug << "agent id = " << j <<
					"	arriveTime = " << tskState._vPreArrTime[j]
					<< " completeTime = " << tskState._vPreComTime[j] << endl;
			}
		}
		c_debug << "___________________" << endl;
	}
	void DynamicConstrn::updateTimeMat(size_t const & chsAgID, size_t const & chsTskID)
	{
		//_setAllocated.insert(std::pair<size_t, size_t>(chsAgID, chsTskID));

		auto &tskState = _vTaskState[chsTskID];

		tskState.calAgArrState(chsAgID);

		double extDur = tskState.calExecuteDur();
		if (extDur == M_MAX)
		{
			tskState._chsComPreTime = M_MAX;
		}
		else
		{
			tskState._chsComPreTime = extDur + tskState._changeRatioTime;
		}		
	}
	void DynamicConstrn::infUpdateTimeMat(size_t const & chsAgID, size_t const & chsTskID)
	{
		_setAllocated.insert(std::pair<size_t, size_t>(chsAgID, chsTskID));
		auto &tskState = _vTaskState[chsTskID];
		
		tskState.calAgArrState(chsAgID);

		double extDur = tskState.calExecuteDur();
		if (extDur == M_MAX)
		{
			tskState._chsComPreTime = M_MAX;
		}
		else
		{
			tskState._chsComPreTime = extDur + tskState._changeRatioTime;
		}
	}
	void DynamicConstrn::calChsComPreTime(size_t const & chsAgID, size_t const & chsTskID)
	{
		auto &tskState = _vTaskState[chsTskID];

		tskState.calAgArrState(chsAgID);
		tskState._onTaskAgID[chsTskID] = true;
		double extDur = tskState.calExecuteDur();
		if (extDur == M_MAX)
		{
			tskState._chsComPreTime = M_MAX;
		}
		else
		{
			tskState._chsComPreTime = extDur + tskState._changeRatioTime;
		}
	}
	void DynamicConstrn::updateTimeMat(vector<size_t> const & vChsAgID, size_t const & chsTskID)
	{
		//update all choosen agent predict arrive time and complete time
		auto &chsTskState = _vTaskState[chsTskID];
		for (size_t i = 0; i < vChsAgID.size(); i++)
		{
			size_t const &chsAgID = vChsAgID[i];
			_setAllocated.insert(std::pair<size_t, size_t>(chsAgID, chsTskID));
			auto &agState = _vAgentState[chsAgID];

			for (size_t j = 0; j < _taskNum; j++)
			{
				auto &tskState = _vTaskState[j];
				if (tskState._onTaskAgID[chsAgID] == true)
				{
					//this agent has been gone this task pnt;
				}
				else
				{
					double onRoadDur = getRoadDur(j, chsTskID);
					tskState._vPreArrTime[chsAgID] = onRoadDur + agState._leaveTime;
					tskState._vPreComTime[chsAgID] = tskState.preCalCompleteDur(tskState._vPreArrTime[chsAgID], _vTaskAgent[j]._ability);
				}
			}
		}
#ifdef _DEBUG
		c_debug << "update choosen task pnt_____" << endl;
#endif // _DEBUG
		//update choosen task pnt 
		for (size_t i = 0; i < _agentNum; i++)
		{
			if (chsTskState._onTaskAgID[i] != true)
			{
				chsTskState._vPreComTime[i] =
					chsTskState.preCalCompleteDur(chsTskState._vPreArrTime[i], _vTaskAgent[i]._ability);
			}
		}

#ifdef _DEBUG
		this->writevTaskState();
		c_debug << "break" << endl;
#endif // _DEBUG

	}
	vector<size_t> DynamicConstrn::findCoordAgent(size_t const & agentid)
	{
		vector<size_t> res;
		auto &taskid = _vAgentState.at(agentid)._taskid;
		for (size_t i = 0; i < this->_agentNum; i++)
		{
			if (i == agentid) continue;
			if (_vAgentState.at(i)._stateType == AgentStateType::onRoad) continue;
			if (_vAgentState.at(i)._stopBoolean == true) continue;
			if (taskid == _vAgentState.at(i)._taskid)
			{
				res.push_back(i);
			}
		}
		return res;
	}
	// 
	void DynamicConstrn::minArrTimeAndMaxEmergentSolution()
	{
		this->_orderPreFirstArrTime.clear();
		this->_orderPreFirstComTime.clear();
		this->_orderPreEmergenceComTime.clear();
		this->initialize();
		//zhu's code
		this->sortPreFirstArrTime();
		this->sortPreEmergenceComTime();
		//this->sortPreFirstComTime();

#ifdef _DEBUG
		c_debug << " _________minArrTimeAndMaxEmergentSolution__________" << endl;
#endif // DEBUG

		for (size_t p = 0; p < _vMaxWeight.size(); p++)
		{
#ifdef _DEBUG
			c_debug << "index is " << p << endl;
#endif // _DEBUG
			vector<AgTaskTimePair> vAgTaskOrderVal;
			auto &weightUnit = this->_vMaxWeight[p];
			for (size_t i = 0; i < _agentNum; i++)
			{
				for (size_t j = 0; j < _taskNum; j++)
				{
					AgTaskPair indexPair(i, j);
					double orderVal = weightUnit*_orderPreFirstArrTime[indexPair] +
						(1 - weightUnit)*_orderPreEmergenceComTime[indexPair];
					vAgTaskOrderVal.push_back(AgTaskTimePair(indexPair, orderVal));
				}
			}
			//qsort
			sort(vAgTaskOrderVal.begin(), vAgTaskOrderVal.end(), ComparePreFirstTime(*this));

			vector<size_t> vEncodeIndex(_agentNum, 0);
			for (size_t i = 0; i < vAgTaskOrderVal.size(); i++)
			{
				auto &agTask = vAgTaskOrderVal[i].first;
				auto &agID = agTask.first;
				auto &encodeIndex = vEncodeIndex[agID];
				_m_enMat[agID][encodeIndex] = agTask.second;
				encodeIndex++;
			}
			this->_m_vEnMat.push_back(_m_enMat);
#ifdef _DEBUG
			this->writeEnMat();
			c_debug << "______________________________" << endl;
#endif // _DEBUG
		}
	}

	void DynamicConstrn::minArrTimeAndMinComTimeSolution()
	{
		this->_orderPreFirstArrTime.clear();
		this->_orderPreFirstComTime.clear();
		this->_orderPreEmergenceComTime.clear();
		this->initialize();
		//zhu's code
		this->sortPreFirstArrTime();
		this->sortPreFirstComTime();
#ifdef _DEBUG
		c_debug << " _________minArrTimeAndMinComTimeSolution__________" << endl;
#endif // _DEBUG
		for (size_t p = 0; p < _vMaxWeight.size(); p++)
		{
#ifdef _DEBUG
			c_debug << "index is " << p << endl;
#endif // !_DEBUG
			vector<AgTaskTimePair> vAgTaskOrderVal;
			auto &weightUnit = this->_vMaxWeight[p];
			for (size_t i = 0; i < _agentNum; i++)
			{
				for (size_t j = 0; j < _taskNum; j++)
				{
					AgTaskPair indexPair(i, j);
					double orderVal = weightUnit*_orderPreFirstArrTime[indexPair] +
						(1 - weightUnit)*_orderPreFirstComTime[indexPair];
					vAgTaskOrderVal.push_back(AgTaskTimePair(indexPair, orderVal));
				}
			}
			//qsort
			sort(vAgTaskOrderVal.begin(), vAgTaskOrderVal.end(), ComparePreFirstTime(*this));

			vector<size_t> vEncodeIndex(_agentNum, 0);
			for (size_t i = 0; i < vAgTaskOrderVal.size(); i++)
			{
				auto &agTask = vAgTaskOrderVal[i].first;
				auto &agID = agTask.first;
				auto &encodeIndex = vEncodeIndex[agID];
				_m_enMat[agID][encodeIndex] = agTask.second;
				encodeIndex++;
			}
			this->_m_vEnMat.push_back(_m_enMat);
#ifdef _DEBUG
			this->writeEnMat();
			c_debug << "______________________________" << endl;
#endif // _DEBUG
		}
	}

	double DynamicConstrn::TaskState::calAgArrState(size_t const & agID)
	{
		double const  &arrTime = _vPreArrTime[agID];
		double changeDur = arrTime - this->_changeRatioTime;
		if (changeDur < 0)
		{
			cout << "bug is here" << endl;
			for (auto it = _sortedArrTimeAgID.begin(); it != _sortedArrTimeAgID.end(); ++it)
			{
				if (arrTime < _vPreArrTime[*it])
				{
					_sortedArrTimeAgID.insert(it, agID);
					break;
				}
			}
#ifdef _DEBUG
			for (size_t i = 0; i < _sortedArrTimeAgID.size(); i++)
			{
				c_debug << "agID  = " << _sortedArrTimeAgID[i] << " arriTime = " << _vPreArrTime[_sortedArrTimeAgID[i]] << endl;
			}
#endif // _DEBUG
			_currentState = _initState;
			_currentRatio = _initRatio;
			_changeRatioTime = 0;
			for (auto &it: _sortedArrTimeAgID)
			{
				changeDur =  _vPreArrTime[it] -  this->_changeRatioTime;
				_currentState = _currentState*exp(changeDur*_currentRatio);
				_changeRatioTime = _vPreArrTime[it];
				_currentRatio = _currentRatio - (*_vAgAbilityPtr)[it];
			}
		}
		else
		{
			_currentState = _currentState*exp(changeDur*_currentRatio);
			this->_changeRatioTime = arrTime;
			_sortedArrTimeAgID.push_back(agID);
			_currentRatio = _currentRatio - (*_vAgAbilityPtr)[agID];
		}
		
		return 0.0;
	}

	double DynamicConstrn::TaskState::preCalExecuteDur(double const &arrTime, double const &ability)
	{
		if (ability <= this->_currentRatio){
			return std::numeric_limits<double>::max();
		}
		else{
			double changeDur = arrTime - this->_changeRatioTime;
			double preState = _currentState *(changeDur*_currentRatio);
			return log(_completedThreshold / preState) / (_currentRatio - ability);
		}
		return 0;
	}

	double DynamicConstrn::TaskState::preCalCompleteDur(double const &arrTime, double const &ability)
	{
		if (ability <= this->_currentRatio) {
			return std::numeric_limits<double>::max();
		}
		else {
			double changeDur = arrTime - this->_changeRatioTime;
			if (changeDur >= 0)
			{
				double preState = _currentState *exp(changeDur*_currentRatio);
				double excDur = log(_completedThreshold / preState) / (_currentRatio - ability);
				return excDur + arrTime;
			}
			else
			{
				
				//此处可以进行优化
				vector<size_t> _addSortedAgID(_sortedArrTimeAgID);
				bool addArrTimeBoolean = false;
				for (auto &it : _sortedArrTimeAgID)
				{
					if (arrTime < _vPreArrTime[it])
					{
						if (!addArrTimeBoolean)
						{
							vChangeTime.push_back(arrTime);
						}
					}
					else
					{
						vChangeTime.push_back(_vPreArrTime[it]);
					}
				}
#ifdef _DEBUG
				c_debug << "sortedSize  = " << _sortedArrTimeAgID.size() << "  addSize = " << vChangeTime.size() << endl;
				for (auto &it : vChangeTime){
					c_debug << "arriveTime = " << it << endl;
				}
#endif // _DEBUG
				_currentState = _initState;
				_currentRatio = _initRatio;
				_changeRatioTime = 0;
				for (auto &it : )
				{
					changeDur = _vPreArrTime[it] - this->_changeRatioTime;
					_currentState = _currentState*exp(changeDur*_currentRatio);
					_changeRatioTime = _vPreArrTime[it];
					_currentRatio = _currentRatio - (*_vAgAbilityPtr)[it];
				}
			}
		}
		return 0;
	}
	double DynamicConstrn::TaskState::preCalCompleteTime(size_t const & agID)
	{
		double const &ability = (*_vAgAbilityPtr)[agID];
		double const & arrTime = this->_vPreArrTime[agID];
		if (ability <= this->_currentRatio) {
			return std::numeric_limits<double>::max();
		}
		else {
			double changeDur = arrTime - this->_changeRatioTime;
			if (changeDur >= 0)
			{
				double preState = _currentState *exp(changeDur*_currentRatio);
				double excDur = log(_completedThreshold / preState) / (_currentRatio - ability);
				return excDur + arrTime;
			}
			else
			{
				//此处可以进行优化
				vector<size_t> _addSortedAgID(_sortedArrTimeAgID);
				bool addArrTimeBoolean = false;
				for (auto &it : _sortedArrTimeAgID)
				{
					if (arrTime < _vPreArrTime[it])
					{
						if (!addArrTimeBoolean)
						{
							vChangeTime.push_back(arrTime);
						}
					}
					else
					{
						if (ability <= this->_currentRatio) {
							return std::numeric_limits<double>::max();
						}
						else {
							double changeDur = arrTime - this->_changeRatioTime;
							if (changeDur >= 0)
							{
								double preState = _currentState *exp(changeDur*_currentRatio);
								double excDur = log(_completedThreshold / preState) / (_currentRatio - ability);
								return excDur + arrTime;
							}
							else
							{

								//此处可以进行优化
								vector<size_t> _addSortedAgID(_sortedArrTimeAgID);
								bool addArrTimeBoolean = false;
								for (auto &it : _sortedArrTimeAgID)
								{
									if (arrTime < _vPreArrTime[it])
									{
										if (!addArrTimeBoolean)
										{
											vChangeTime.push_back(arrTime);
										}
									}
									else
									{
										vChangeTime.push_back(_vPreArrTime[it]);
									}
								}
#ifdef _DEBUG
								c_debug << "sortedSize  = " << _sortedArrTimeAgID.size() << "  addSize = " << vChangeTime.size() << endl;
								for (auto &it : vChangeTime) {
									c_debug << "arriveTime = " << it << endl;
								}
#endif // _DEBUG
								_currentState = _initState;
								_currentRatio = _initRatio;
								_changeRatioTime = 0;
								for (auto &it : )
								{
									changeDur = _vPreArrTime[it] - this->_changeRatioTime;
									_currentState = _currentState*exp(changeDur*_currentRatio);
									_changeRatioTime = _vPreArrTime[it];
									_currentRatio = _currentRatio - (*_vAgAbilityPtr)[it];
								}
							}
						}
=342````````````````````````````````````````````````````````````````````````````````````````-* 000
						vChangeTime.push_back(_vPreArrTime[it]);
					}
				}
#ifdef _DEBUG
				c_debug << "sortedSize  = " << _sortedArrTimeAgID.size() << "  addSize = " << vChangeTime.size() << endl;
				for (auto &it : vChangeTime) {
					c_debug << "arriveTime = " << it << endl;
				}
#endif // _DEBUG
				_currentState = _initState;
				_currentRatio = _initRatio;
				_changeRatioTime = 0;
				for (auto &it : )
				{
					changeDur = _vPreArrTime[it] - this->_changeRatioTime;
					_currentState = _currentState*exp(changeDur*_currentRatio);
					_changeRatioTime = _vPreArrTime[it];
					_currentRatio = _currentRatio - (*_vAgAbilityPtr)[it];
				}
			}
		}
		return 0.0;
	}
	double DynamicConstrn::TaskState::getMinPreComTime()
	{
		//std::_Minmax_element
		auto minElementIter = std::min_element(_vPreComTime.begin(), _vPreComTime.end());
		if (*minElementIter == std::numeric_limits<double>::max())
		{
			//全部为无穷
			c_debug << "无穷" << endl;

			auto maxArrTimeIter = std::max_element(_vPreArrTime.begin(), _vPreArrTime.end());
			double  maxArrTime = *maxArrTimeIter;
			c_debug << "maxArrTime = " << maxArrTime << endl;

			vector<double> vRatioBais;
			for (size_t i = 0; i < _agentNum ; i++)
			{
				vRatioBais.push_back(_currentRatio - _vAgAbilityPtr->at(i));
				c_debug << "vRatioBais = " << i << " val = " << vRatioBais[i] << endl;
			}
			auto maxRatioBaisIter = std::max_element(vRatioBais.begin(), vRatioBais.end());
			double maxRatioBais = *maxRatioBaisIter;			
			c_debug << " maxRatioBais  = " << maxRatioBais << endl;

			double minVal = std::numeric_limits<double>::max();
			
			//debug mode 
			vector<double>  vVal;
			for (size_t i = 0; i < _vWeight.size(); i++)
			{
				auto &weightUnit = _vWeight[i];
				for (size_t j = 0; j < _agentNum; j++)
				{
					double val = weightUnit*(_vPreArrTime[j] / maxArrTime) + (1 - weightUnit)*(vRatioBais[j] / maxRatioBais);
					c_debug << "weight = " << weightUnit << " agentNum  = " << j << " val  = " << val << endl;
					if (val < minVal)
					{
						minVal = val;
						this->_minElementAgID = j;
					}
					vVal.push_back(val);
				}
			}
			auto MinValNum = std::count(vVal.begin(), vVal.end(), minVal);
			if (MinValNum > 1)
			{
				cout << "wtf " << endl;
			}
			_minElementRatioBais = vRatioBais[_minElementAgID];
		}
		else
		{
			_minElementAgID = minElementIter - _vPreComTime.begin();
			
		}
		_minElementArrTime = _vPreArrTime[_minElementAgID];
		return *minElementIter;
	}


	void DynamicConstrn::TaskState::calCurrentState(double const & Time)
	{
		double changeDur = Time - this->_changeRatioTime;
		//conf_debug << "_currentState is " << _currentState << endl;
		_currentState = _currentState*exp(changeDur*_currentRatio);
		this->_changeRatioTime = Time;
	}

	double DynamicConstrn::TaskState::calExecuteDur()
	{
		return 	log(_completedThreshold / _currentState) / _currentRatio;
	}

}
