#pragma once
#include "stadfx.h"
#include "TaskPnt.hpp"

namespace method {

	using namespace decode;

	using  TimeMat = vector<vector<double>>;
	
	using EnMat = vector<vector<int>>;
	//the first size_t  is the agentID
	//the second size_t is the taskID
	using AgTaskPair = pair<size_t, size_t>;

	//the first is the the agentID and the taskID
	//the val of the time 
	using AgTaskTimePair = pair<AgTaskPair, double>;

	//first is the agent task id index
	//second is the comTime;
	//third is the ratioBais
	//forth is the arrTime;
	using AgTaskComTuple = std::tuple<AgTaskPair, double, double, double>;

	class 	DynamicConstrn
	{
	public:
		DynamicConstrn(vTaskAgentPtr vAgPtr, vTaskPntPtr vTaskPtr, DisMapPtr taskDisMatPtr, DisMapPtr ag2taskDisMatPtr) :
			_m_vTaskAgentPtr(vAgPtr), _m_vTaskPntPtr(vTaskPtr), _taskDisMatPtr(taskDisMatPtr), _ag2taskDisMatPtr(ag2taskDisMatPtr),
			_vTaskAgent(*vAgPtr), _vTaskPnt(*vTaskPtr), _taskDisMat(*taskDisMatPtr), _ag2taskDisMat(*ag2taskDisMatPtr),
			_agentNum(vAgPtr->size()), _taskNum(vTaskPtr->size())
		{
#ifdef _DEBUG
			c_debug.open("dy_deg.txt", std::ios::trunc);
			c_debug.precision(15);
			c_debug << "动态方法" << endl;
#endif // _DEBUG
			for (double i = 0; i < 1.01; i += 0.1)
			{
				_vMaxWeight.push_back(i);
			}
		}
		~DynamicConstrn();

		bool initialize();

		//ggq's method
		bool sortOnTaskDur();
		bool sortOnRoadDur();

		//zhu's method


		//动态排序方法开始
		//更新预估的到达时间和完成时间
		void updatePreArrAndComTime();
		//更新解
		void updateEnmat(size_t const & type);
		//最小到达时间和最大任务紧急程度复合规则
		void DminArrTimeAndMaxEmergentSolution();

		void sortPreFirstArrTime();
		std::map<AgTaskPair, size_t> _orderPreFirstArrTime;
		//从小到大
		class ComparePreFirstTime
		{
		public:
			ComparePreFirstTime(const DynamicConstrn &info) :
				m_info(info) {}
			const  DynamicConstrn &m_info;
			bool operator()(AgTaskTimePair const & p1, AgTaskTimePair const &p2)
			{
				if (p1.second < p2.second)
				{
					return true;
				}
				else
				{
					return false;
				}
			}
		};
		//从大到小
		class counterComparePreFirstTime
		{
		public:
			counterComparePreFirstTime(const DynamicConstrn &info) :
				m_info(info) {}
			const  DynamicConstrn &m_info;
			bool operator()(AgTaskTimePair const & p1, AgTaskTimePair const &p2)
			{
				if (p1.second > p2.second)
				{
					return true;
				}
				else
				{
					return false;
				}
			}
		};

		//sort from small to the large one
		void sortPreFirstComTime();
		std::map<AgTaskPair, size_t> _orderPreFirstComTime;
		//sort from the large to the small
		void sortPreEmergenceComTime();
		std::map<AgTaskPair, size_t> _orderPreEmergenceComTime;


		//最小到达时间和最大任务紧急程度复合规则
		void minArrTimeAndMaxEmergentSolution();
		//最小到达时间和最短任务完成时间
		void minArrTimeAndMinComTimeSolution();

		vector<EnMat> _m_vEnMat;

		// zhu's method end
		class AgentState
		{
		public:
			//离开任务点的时刻
			double _leaveTime;
			//到达任务点的时刻
			double _arriveTime;
			//在任务点执行任务的时间
			double _executeDur;
			//前往任务点的路程时间
			double _roadDur;
			//当前的任务点
			//当_taskid = -1 表示该移动智能体在初始位置
			int  _taskid;
			//当前初始什么状态
			size_t _stateType = AgentStateType::onTaskPnt;
			//是否已经走完全部过程
			bool _stopBoolean = false;
			//
			bool _executeBoolean = false;
			double _ability;
			//预估的到达时刻
			vector<double> _vPreArrTime;
			//预估的执行时间
			vector<double> _vPreExcDur;
			//预估的完成时刻
			vector<double> _vPreComTime;
		};

		class TaskState
		{
		public:
			TaskState(shared_ptr<vector<double>> &vDPtr, size_t const & agNum, std::ofstream &c_txt) :
				_vAgAbilityPtr(vDPtr), _agentNum(agNum), c_debug(c_txt), _onTaskAgID(agNum, false)
			{
				for (size_t i = 0; i < 11; i++)
				{
					_vWeight.push_back(double(i) / 10);
				}
			}
			double _initState;
			double _initRatio;
			double _currentState;
			double _currentRatio;
			double _changeRatioTime;
			bool iscompleted = false;
			double calExecuteDur();
			void calCurrentState(double const & arriveTime);
			bool StateIsCompleted();

			//
			double _chsComPreTime;

			//can optimal it into a list
			vector<size_t> _sortedArrTimeAgID;

			double calAgArrState(size_t const & agID);
			double preCalExecuteDur(double const &arrTime, double const &ability);
			double preCalCompleteDur(double const &arrTime, double const &ability);

			//set<size_t> _onTaskAgID;

			vector<bool> _onTaskAgID;
			size_t _id;
			//预估的到达时刻
			vector<double> _vPreArrTime;
			//预估的执行时间
			vector<double> _vPreExcDur;
			//预估的完成时刻
			vector<double> _vPreComTime;
			double getMinPreComTime();
			//
			size_t _minElementAgID;
			double _minElementArrTime;
			double _minElementRatioBais;
			//
			const shared_ptr<vector<double>> _vAgAbilityPtr;
			const size_t _agentNum;
			std::ofstream  &c_debug;
			vector<double> _vWeight;

		private:
			double _completedThreshold = 0.1;
		};

	private:


		//不可更改
		vTaskAgentPtr  const _m_vTaskAgentPtr;
		vTaskPntPtr  const _m_vTaskPntPtr;
		vector<TaskAgent> const &_vTaskAgent;
		vector<TaskPnt> const  &_vTaskPnt;

		const size_t _taskNum;
		const size_t _agentNum;

		DisMapPtr  const _taskDisMatPtr;
		DisMapPtr  const _ag2taskDisMatPtr;
		std::map<std::pair<size_t, size_t>, double> const  &_taskDisMat;
		std::map<std::pair<size_t, size_t>, double>  const &_ag2taskDisMat;

		TimeMat _m_arriveMat;
		TimeMat _m_completeMat;

		// 智能体状态和任务点状态
		vector<AgentState> _vAgentState;
		vector<TaskState> _vTaskState;


		//write txt
		std::ofstream c_debug;
		void writeEnMat();


		//max weight
		vector<double> _vMaxWeight;

		double getRoadDur(size_t const &tsk_id1, size_t const &tsk_id2) const 
		{
			if (tsk_id1 > tsk_id2) { std::swap(tsk_id1, tsk_id2); }
			DisMapIndex index(tsk_id1, tsk_id2);
			return _taskDisMat.at(index);
		}
		void updateTimeMat(size_t const &chsAgID,size_t const & chsTskID);
		void infUpdateTimeMat(size_t const &chsAgID, size_t const &chsTskID);

		void calChsComPreTime(size_t const &chsAgID, size_t const & chsTskID);
		void updateTimeMat(vector<size_t> const & vChsAgID, size_t const & chsTskID);

		double weightUnit;


		//
		vector<size_t> findCoordAgent(size_t const& agentid);

		set<pair<size_t, size_t>> _setAllocated;

		EnMat _m_enMat;
	
	};
}