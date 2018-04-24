#include "OrgEncodeMatR.h"
//#include "DirectConstrn.hpp"


namespace decode {
	OrgEncodeMat::~OrgEncodeMat()
	{
	}
	bool OrgEncodeMat::displayEnMat()
	{
		conf_debug2 << "_________" << endl;
		for (size_t i = 0; i < _agentNum; i++)
		{
			conf_debug2 << "agent id " << i << this->_enMat[i] << endl;
		}
		return false;
	}
	void OrgEncodeMat::setOrgEnMat(vector<size_t> const & perm)
	{
		this->_enMat.clear();
		size_t p = 0;
		for (size_t i = 0; i < _agentNum; i++)
		{
			OrgChrom chromUnit;
			for (size_t j = 0; j < _taskNum; j++)
			{
				//OrgEncode Unit()
				chromUnit.push_back(OrgEncode(perm[p++]));
			}
			_enMat.push_back(chromUnit);
		}
	}
	void OrgEncodeMat::setOrgEnMat(vector<vector<int>> const & enmat)
	{
		this->_enMat.clear();
		for (size_t i = 0; i < _agentNum; i++)
		{
			OrgChrom chromUnit;
			for (size_t j = 0; j < _taskNum; j++)
			{
				chromUnit.push_back(OrgEncode(enmat[i][j]));
			}
			_enMat.push_back(chromUnit);
		}
	}
	void OrgEncodeMat::writeEnMat()
	{
		for (size_t i = 0; i < _agentNum; i++)
		{
			conf_debug << "agent id" << i;
			for (size_t j = 0; j < _taskNum; j++)
			{
				conf_debug << " " << _enMat[i][j];
			}
			conf_debug << endl;
		}
	}

	double OrgEncodeMat::decode()
	{
		double fitness;
		size_t  i = 0;
		do
		{
			//conf_debug << "cal time is " << i++ << endl;
			//conf_debug << "<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
			if (i % 10000 == 0)
			{
				cout << "cal time is " << i << endl;
			}
			i++;
			//cout << "cal time =  " << i++ << endl;
			//displayEnMat();
			this->initialize();
			fitness = this->decodeProcessor();
		} while (fitness<0);
		cout << "calTimes is " << i << endl;
		return fitness;
	}

	bool OrgEncodeMat::initialize()
	{
		vtaskPntIsCompleted.clear();
		this->_vAgentState.clear();
		this->_vTaskState.clear();

		vtaskPntIsCompleted.assign(_taskNum, false);
		for (size_t i = 0; i < _agentNum; i++)
		{
			AgentState agstate;
			agstate._taskid = _enMat.at(i).front()._taskid;
			DisMapIndex dis_index(i, agstate._taskid);
			agstate._stateType = AgentStateType::onRoad;
			agstate._encodeIndex = 0;
			double arriveTime = _ag2taskDisMat.at(dis_index);
			agstate._arriveTime = arriveTime;
			agstate._leaveTime = 0;
			agstate._ability = _vTaskAgent.at(i)._ability;
			//conf_debug << "index is " << i << "  the ability is " << agstate._ability << endl;
			this->_vAgentState.push_back(agstate);
		}
		//conf_debug << endl;

		for (size_t i = 0; i < _taskNum; i++)
		{
			TaskState tkstate;
			tkstate._currentState = _vTaskPnt.at(i)._initState;
			tkstate._initState = _vTaskPnt.at(i)._initState;
			tkstate._currentRatio = _vTaskPnt.at(i)._increaseRatio;


			tkstate._changeRatioTime = 0;
			tkstate._vTimeStateAndRatio.push_back(TaskState::TimeStateAndRatio(0, tkstate._currentState, tkstate._currentRatio));
			_vTaskState.push_back(tkstate);
			//conf_debug << "task id is " << i << "	" << tkstate << endl;
		}
		this->_currentTime = 0;
		return false;
	}

	double OrgEncodeMat::decodeProcessor()
	{
		bool invalidFitness = false;
		bool backBoolean = false;
		while (allTaskPntComplete())
		{
			tuple<size_t, size_t> markType = findMotionOrLeaveId();

			switch (std::get<0>(markType))
			{
			case calType::arriveCondition:
			{
				size_t motionId = std::get<1>(markType);
				OrgChrom &agentChrom = _enMat.at(motionId);
				AgentState &agState = _vAgentState.at(motionId);
				OrgEncode &encode = agentChrom[agState._encodeIndex];
				auto &taskid = encode._taskid;
				agState._taskid = taskid;
				TaskState &tkState = _vTaskState.at(taskid);
				//conf_debug << "motionId = " << motionId << endl;
				//conf_debug << "taskid = " << taskid << endl;

				double const &arriveTime = agState._arriveTime;
				double leaveTime;
				//zhu' bug is here
				if (vtaskPntIsCompleted.at(taskid))
				{
					//this->displayEnMat();
					//conf_debug2 << "__________" << endl;
					//conf_debug2 << "agent id " << motionId << "	changePos " << agState._encodeIndex << endl;
					//conf_debug2 << "__________" << endl;
					//conf_debug2<<"task"
					//conf_debug2 << "arrive time is " << agState._arriveTime << endl;
					OrgEncode comEncode = agentChrom[agState._encodeIndex];
					for (size_t i = agState._encodeIndex; i < (_taskNum - 1); i++)
					{
						agentChrom[i] = agentChrom[i + 1];
					}
					agentChrom.back() = comEncode;
					
					double roadDur;
					if (agState._encodeIndex == 0)
					{
						DisMapIndex dis_index(motionId,encode._taskid);
						roadDur = _ag2taskDisMat.at(dis_index);
						agState._arriveTime = roadDur;
					}
					else
					{
						OrgEncode & bEncode = agentChrom[agState._encodeIndex - 1];
						roadDur = this->getRoadDur(bEncode, encode);						
						agState._arriveTime = agState._leaveTime + roadDur;
					}
					agState._stateType = AgentStateType::onRoad;
					//conf_debug2 << "arrive time is " << agState._arriveTime << endl;
					//this->displayEnMat();
				}
				else
				{
					//conf_debug << " the last current state is " << tkState._currentState << endl;
					tkState.calCurrentState(arriveTime);
					//conf_debug << "the task pnt current state is " << tkState._currentState << endl;
					tkState._currentRatio = tkState._currentRatio - agState._ability;
					//conf_debug << "the task pnt current Ratio is " << tkState._currentRatio << endl;

					//save information 
					tkState.saveAgInfo(motionId, arriveTime, tkState._currentState, calType::arriveCondition);
					tkState.saveTskInfo(arriveTime);


					auto vIn = findCoordAgent(motionId);
					if (vIn.empty())
					{
						//conf_debug << "no coordination" << endl;

						if (tkState._currentRatio >= 0)
						{
							//conf_debug << "the task can't be completed " << endl;
							//leaveTime = arriveTime + agState._executeDur;
							//agState._executeBoolean
							//conf_debug << "can't cal completed time ";
							//conf_debug << "can't move  and no leave time " << endl;
							//agState._executeBoolean = true;
							leaveTime = std::numeric_limits<double>::max();
						}
						else
						{
							//conf_debug << " the task can be completed" << endl;
							//conf_debug << "can move  and have leave time " << endl;
							agState._executeBoolean = false;
							agState._executeDur = tkState.calExecuteDur();
							leaveTime = agState._arriveTime + agState._executeDur;
						}
					}
					else
					{
						//conf_debug << "coordination condition " << endl;
						if (tkState._currentRatio >= 0)
						{
							//conf_debug << "can't move  and no leave time " << endl;
							agState._executeBoolean = true;
							//conf_debug << "the task can't be completed " << endl;
							leaveTime = std::numeric_limits<double>::max();
							for (auto &it : vIn)
							{
								//conf_debug << " coordination id " << it << endl;
							}
						}
						else
						{
							agState._executeBoolean = false;
							//agState._executeDur = tkState.calExecuteDur() * dur;
							//leaveTime = agState._arriveTime + agState._executeDur;											
							double coexecuteDur = tkState.calExecuteDur();
							double coleaveTime = agState._arriveTime + coexecuteDur;
							//conf_debug << " predict completed time is " << coleaveTime << endl;

							for (auto &it : vIn)
							{
								//conf_debug << " coordination id " << it << endl;
								auto &coagState = this->_vAgentState.at(it);
								coagState._leaveTime = coleaveTime;
								coagState._executeDur = coleaveTime - coagState._arriveTime;
							}

							leaveTime = agState._arriveTime + coexecuteDur;
							agState._executeDur = coexecuteDur;

						}
					}
					//conf_debug << "executeTime is " << agState._executeDur << endl;
					//conf_debug << "leaveTime is " << leaveTime << endl;
					agState._leaveTime = leaveTime;
					agState._stateType = AgentStateType::onTaskPnt;
					
					//
					_currentTime = agState._arriveTime;
					_currentEntType = calType::arriveCondition;
					_currentAgID = motionId;
					//conf_debug2 << " ARRIVE _currentTime is " << _currentTime << endl;
					//conf_debug2 << "_currentID is " << _currentAgID << endl;
				}
				//conf_debug << "	" << endl;
				break;
			}
			case calType::leaveCondition:
			{
				size_t leaveId = std::get<1>(markType);
				//size_t motionId = std::get<1>(markType);
				OrgChrom &agentChrom = _enMat.at(leaveId);
				AgentState &agState = _vAgentState.at(leaveId);
				OrgEncode &encode = agentChrom[agState._encodeIndex];
				//conf_debug << "the agState encodeIndex is " << agState._encodeIndex << endl;
				agState._encodeIndex++;

				auto &taskid = encode._taskid;
				//_vTaskPnt
				//conf_debug << "leave index is " << leaveId << endl;
				//conf_debug << "leave task id is " << taskid << endl;

				//debug

				TaskState &tkState = _vTaskState.at(taskid);
				if (vtaskPntIsCompleted.at(taskid))
				{
					//				conf_debug << "this task pnt has been completed by others" << endl;
					tkState.saveAgInfo(leaveId, agState._leaveTime, tkState._currentState, calType::leaveCondition);
					tkState._currentRatio = 0;
					tkState.saveTskInfo(agState._leaveTime);
				}
				else
				{
					//			conf_debug << " the last current state is " << tkState._currentState << endl;
					tkState.calCurrentState(agState._leaveTime);
					//		conf_debug << "the task current state is " << tkState._currentState << endl;
					//save 
					tkState.saveAgInfo(leaveId, agState._leaveTime, tkState._currentState, calType::leaveCondition);
					if (tkState.StateIsCompleted())
					{
						//		conf_debug << "the task has been completed" << endl;
						vtaskPntIsCompleted.at(taskid) = true;
						auto vIn = findCoordAgent(leaveId);
						//{}
						tkState._currentRatio = 0;
						tkState.saveTskInfo(agState._leaveTime);

						for (auto &it : vIn)
						{
							AgentState &coAgState = _vAgentState[it];
							OrgChrom &coAgentChrom = _enMat.at(leaveId);
							tkState.saveAgInfo(it, coAgState._leaveTime, tkState._currentState, calType::leaveCondition);
							OrgEncode &coEncode = coAgentChrom[coAgState._encodeIndex];
							coAgState._leaveTime = agState._leaveTime;
							coAgState._encodeIndex++;
							if (coAgState._encodeIndex == this->_taskNum)
							{
								coAgState._stopBoolean = true;
							}
							else
							{
								//
								auto & coTencode = coAgentChrom[coAgState._encodeIndex];
								//double roadDur = this->getEncodeDur(coEncode, coTencode);
								double roadDur = this->getRoadDur(coEncode, coTencode);
								coAgState._roadDur = roadDur;
								coAgState._arriveTime = coAgState._leaveTime + coAgState._roadDur;
								//			conf_debug << "coleave index is " << it << "  co next arrive time is " << coAgState._arriveTime << endl;
								coAgState._stateType = AgentStateType::onRoad;
							}
							//	conf_debug << "	" << endl;
						}

					}
					else
					{
						tkState._currentRatio = tkState._currentRatio + agState._ability;
						//conf_debug << " after the leave the current Ratio is " << tkState._currentRatio << endl;
						tkState.saveTskInfo(agState._leaveTime);
						//conf_debug << "the task has not been completed" << endl;
					}
				}

				if (agState._encodeIndex == this->_taskNum)
				{
					agState._stopBoolean = true;
				}
				else
				{
					//
					auto & tencode = agentChrom[agState._encodeIndex];
					double roadDur = this->getRoadDur(encode, tencode);
					agState._roadDur = roadDur;
					agState._arriveTime = agState._leaveTime + agState._roadDur;
					//conf_debug << "leave index is " << leaveId << "  next arrive time is " << agState._arriveTime << endl;
					agState._stateType = AgentStateType::onRoad;
				}
				_currentTime = agState._leaveTime;
				_currentEntType = calType::leaveCondition;
				_currentAgID = leaveId;

				//conf_debug2 << " LEAVE _currentTime is " << _currentTime << endl;
				//conf_debug2 << "_currentID is " << _currentAgID << endl;

				break;
			}
			case calType::endCondition:
			{
				for (size_t i = 0; i < vtaskPntIsCompleted.size(); i++)
				{
					if (vtaskPntIsCompleted.at(i) == true)
					{
						//conf_debug << "the task pnt index = " << i << " has been done" << endl;
					}
					else
					{
						//conf_debug << "the task pnt index = " << i << " has not been done" << endl;
					}
				}
				invalidFitness = true;
				break;
			}
			case calType::backCondition:
			{
				backBoolean = true;
				break;
			}
			default:
				break;
			}
			if (invalidFitness)
			{
				break;
			}
			if (backBoolean)
			{
				break;
			}			
		}
		if (backBoolean)
		{
			return -1;
		}
		if (!invalidFitness)
		{
			auto cmpLeaveTime = [=](AgentState const & A, AgentState const &B) {
				if (A._leaveTime < B._leaveTime)
				{
					return true;
				}
				else
				{
					return false;
				}
			};
			auto fitnessInter = std::max_element(_vAgentState.begin(), _vAgentState.end(), cmpLeaveTime);
			//conf_debug << "fitness is " << fitnessInter->_leaveTime << endl;
			return fitnessInter->_leaveTime;
		}
		//conf_debug << " fitness is wrong " <<"the value is "<<INFINITY<< endl;
		return INFINITY;
	}

	bool OrgEncodeMat::saveTaskStateData()
	{
		std::ofstream conf_TaskState;
		conf_TaskState.open("drawData.txt", std::ios::trunc);
		conf_TaskState.precision(15);
		for (size_t i = 0; i < this->_taskNum; i++)
		{
			conf_TaskState << "taskID : " << i << endl;
			auto &tkState = this->_vTaskState.at(i);
			auto &vTAS = tkState._vTimeStateAndRatio;
			for (size_t j = 0; j < (vTAS.size() - 1); j++)
			{
				conf_TaskState << "trace : ";
				conf_TaskState << "startTime " << std::get<0>(vTAS.at(j))
					<< " currentState " << std::get<1>(vTAS.at(j))
					<< " increaseRatio " << std::get<2>(vTAS.at(j))
					<< " endTime " << std::get<0>(vTAS.at(j + 1)) << endl;
			}
			if (tkState._vArriveAgIdTimeAndState.size() != tkState._vLeaveAgIdTimeAndState.size())
			{
				//cout << " the task id is " << i << endl;
				//std::cout << "bug is here" << endl;
			}
			for (auto &it : tkState._vArriveAgIdTimeAndState)
			{
				conf_TaskState << " ag : ";
				conf_TaskState << " agid " << get<0>(it)
					<< " arriveTime  " << get<1>(it)
					<< " state " << get<2>(it)
					<< endl;
			}
			for (auto &it : tkState._vLeaveAgIdTimeAndState)
			{
				conf_TaskState << " ag : ";
				conf_TaskState << " agid " << get<0>(it)
					<< " leaveTime  " << get<1>(it)
					<< " state " << get<2>(it)
					<< endl;
			}

			conf_TaskState << "EndTaskID : " << i << endl;
		}
		return false;
	}

	double OrgEncodeMat::getRoadDur(OrgEncode const & e1, OrgEncode const & e2)
	{
		auto taskid1 = e1._taskid;
		auto taskid2 = e2._taskid;
		if (taskid1 > taskid2)
		{
			std::swap(taskid1, taskid2);
		}
		DisMapIndex index(taskid1, taskid2);
		return this->_taskDisMat.at(index);
	}

	bool OrgEncodeMat::allTaskPntComplete() const
	{
		auto it = std::find(vtaskPntIsCompleted.begin(), vtaskPntIsCompleted.end(), false);
		if (it == vtaskPntIsCompleted.end())
		{
			return false;
		}
		return true;
	}

	tuple<size_t, size_t> OrgEncodeMat::findMotionOrLeaveId()
	{
		//vector<tuple<size_t, size_t, double>> vMin;
		size_t  cal_type = calType::endCondition;
		size_t agentIndex;
		double minVal = std::numeric_limits<double>::max();
		for (size_t i = 0; i < _vAgentState.size(); i++)
		{
			auto &it = _vAgentState.at(i);
			//conf_debug << "agent index = " << i << it << endl;
			if (it._stopBoolean == true)
			{
				//conf_debug << "agent index = " << i << "has been stoped" << endl;
				//continue;

			}
			else {
				if (it._stateType == AgentStateType::onRoad)
				{
					//conf_debug<<"the time is "<<it.a
					if (it._arriveTime < minVal)
					{
						minVal = it._arriveTime;
						cal_type = calType::arriveCondition;
						agentIndex = i;
					}
				}
				if (it._stateType == AgentStateType::onTaskPnt)
				{
					if (it._leaveTime < minVal)
					{
						minVal = it._leaveTime;
						cal_type = calType::leaveCondition;
						agentIndex = i;
					}
				}
			}
		}
		//conf_debug << "	" << endl;
		if (minVal < _currentTime)
		{
			//需要进行回溯
			//cout << "need cal again" << endl;
			//conf_debug2 << "current Time is " << _currentTime << endl;
			//conf_debug2 << "agentId is " << agentIndex << endl;
			//conf_debug2 << " minVal is " << minVal << endl;
			//conf_debug2 << "_EntAgID is " << _currentAgID << endl;
			//conf_debug2 << "_currentEntType is " << _currentEntType << endl;

			cal_type = calType::backCondition;
			auto back_res = std::make_tuple(cal_type, agentIndex);
		}
		
		auto res = std::make_tuple(cal_type, agentIndex);
		
		if (cal_type == calType::endCondition)
		{
		}
		else 
		{
			//当前时刻
//			_currentTime = minVal;
//			// some val for debug
//			_currentAgID = agentIndex;
//			_currentEntType = cal_type;
			
		}
		return res;
	}

	vector<size_t> OrgEncodeMat::findCoordAgent(size_t const & agentid)
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


	double OrgEncodeMat::TaskState::calExecuteDur()
	{
		return 	log(_completedThreshold / _currentState) / _currentRatio;
	}

	void OrgEncodeMat::TaskState::calCurrentState(double const & Time)
	{
		double changeDur = Time - this->_changeRatioTime;
		//conf_debug << "_currentState is " << _currentState << endl;
		_currentState = _currentState*exp(changeDur*_currentRatio);
		this->_changeRatioTime = Time;
	}

	bool OrgEncodeMat::TaskState::StateIsCompleted()
	{
		auto bais = abs(this->_currentState - this->_completedThreshold);
		if ((bais < 0.0000001) || (_currentState < _completedThreshold))
		{
			this->iscompleted = true;
			//return 
		}
		else
		{
			this->iscompleted = false;
		}
		return this->iscompleted;
	}

	
	void OrgEncodeMat::TaskState::saveAgInfo(size_t const & agID, double const & time, double const & state, size_t const &type)
	{
		if (type == calType::leaveCondition)
		{
			this->_vLeaveAgIdTimeAndState.push_back(AgIdTimeAndState(agID, time, state));
		}
		if (type == calType::arriveCondition)
		{
			this->_vArriveAgIdTimeAndState.push_back(AgIdTimeAndState(agID, time, state));
		}
	}

	void OrgEncodeMat::TaskState::saveTskInfo(double const & timeVal)
	{
		this->_vTimeStateAndRatio.push_back(TimeStateAndRatio(timeVal, _currentState, _currentRatio));
	}

}
