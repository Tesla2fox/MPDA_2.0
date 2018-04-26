#pragma once
#include "TaskPnt.hpp"


//解码的程序
namespace decode {
	//using 

	class OrgEncode
	{
	public:
		OrgEncode(size_t const &id) :
			_taskid(id) {}
		//~OrgEncode();
		size_t _taskid;
	private:

	};

	inline std::ostream& operator<<(std::ostream &os, const OrgEncode &data)
	{
		return os << " taskPntId = " << data._taskid;
	}


	using OrgChrom = vector<OrgEncode>;
	using OrgEnMat = vector<OrgChrom>;

	inline std::ostream& operator<<(std::ostream &os, const OrgChrom & chrom)
	{
		for each (auto & var in chrom)
		{
			os << var;
		}
		return os;
	}
	
	class OrgEncodeMat
	{
	public:
		OrgEncodeMat(vTaskAgentPtr vAgPtr, vTaskPntPtr vTaskPtr, DisMapPtr taskDisMatPtr, DisMapPtr ag2taskDisMatPtr):
			_m_vTaskAgentPtr(vAgPtr), _m_vTaskPntPtr(vTaskPtr), _taskDisMatPtr(taskDisMatPtr), _ag2taskDisMatPtr(ag2taskDisMatPtr),
			_vTaskAgent(*vAgPtr), _vTaskPnt(*vTaskPtr), _taskDisMat(*taskDisMatPtr), _ag2taskDisMat(*ag2taskDisMatPtr),
			_agentNum(vAgPtr->size()),_taskNum(vTaskPtr->size())
		{
#ifdef _DEBUG
			conf_debug.open("deg.txt", std::ios::trunc);
			conf_debug.precision(15);
			conf_debug2.open("deg2.txt", std::ios::trunc);
			conf_debug2.precision(15);
#endif // _DEBUG
		};
		~OrgEncodeMat();


		bool displayEnMat();


		void setOrgEnMat(vector<size_t> const &perm);

		void setOrgEnMat(vector<vector<int>> const &enMat);

		void writeEnMat();
		class AgentState
		{
		public:
			//离开任务点的时间
			double _leaveTime;
			//到达任务点的时间
			double _arriveTime;
			//在任务点执行任务的时间
			double _executeDur;
			//在任务点执行的缩放因子
			double _dur;
			//
			//前往任务点的时间
			double _roadDur;
			//在走第几个编码过程
			size_t _encodeIndex;
			//当前的任务点
			size_t _taskid;
			//当前初始什么状态
			size_t _stateType = AgentStateType::onTaskPnt;
			//是否已经走完全部过程
			bool _stopBoolean = false;
			//
			bool _executeBoolean = false;

			double _ability;

		    friend	std::ostream& operator<<(std::ostream &os, const AgentState &data)
			{
				return os << " arriveTime = " << data._arriveTime <<" leaveTime = "<< data._leaveTime;
			}
		};

		class TaskState
		{
		public:
			double _initState;
			double _initRatio;
			double _currentState;
			double _currentRatio;
			double _changeRatioTime;
			bool iscompleted = false;
			double calExecuteDur();
			//input 时刻
			//output vaildBool
			bool calCurrentState(double const & time);
			bool StateIsCompleted();
			void saveAgInfo(size_t const &agID, double const& time, double const& state, size_t const &type);
			void saveTskInfo(double const & timeVal);
			using TimeStateAndRatio = tuple<double, double, double>;
			vector<TimeStateAndRatio> _vTimeStateAndRatio;
			using AgIdTimeAndState = tuple<size_t, double, double>;
			vector<AgIdTimeAndState> _vArriveAgIdTimeAndState;
			vector<AgIdTimeAndState> _vLeaveAgIdTimeAndState;
		private:
			double _completedThreshold = 0.5;
		};


		double decode();
		bool initialize();

		double decodeProcessor();

		bool saveTaskStateData();

	private:
		
		vTaskAgentPtr _m_vTaskAgentPtr;
		vTaskPntPtr _m_vTaskPntPtr;
		DisMapPtr _taskDisMatPtr;
		DisMapPtr _ag2taskDisMatPtr;
		//EncodeMat::AgentState wtf;

		const size_t _taskNum;
		const size_t _agentNum;


		vector<TaskAgent> &_vTaskAgent;
		vector<TaskPnt> &_vTaskPnt;

		std::map<std::pair<size_t, size_t>, double> &_taskDisMat;
		std::map<std::pair<size_t, size_t>, double>  &_ag2taskDisMat;

		vector<AgentState> _vAgentState;
		vector<TaskState> _vTaskState;

		OrgEnMat _enMat;



		//vector bool  is all completed
		vector<bool> vtaskPntIsCompleted;
		double getRoadDur(OrgEncode const& e1, OrgEncode const &e2);

		bool allTaskPntComplete() const;
		///return 0 calType: arrive or leave or end
		///  1 agent id
		tuple<size_t, size_t> findMotionOrLeaveId();

		//
		vector<size_t> findCoordAgent(size_t const& agentid);


		double _currentTime = 0;

		//write txt
		std::ofstream conf_debug;
		std::ofstream  conf_debug2;

		// some values for debug

		size_t _currentAgID;
		size_t _currentEntType;


	};
}