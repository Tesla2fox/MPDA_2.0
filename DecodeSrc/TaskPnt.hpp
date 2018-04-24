#pragma once
#include "stadfx.h"

namespace decode {
	class TaskPnt
	{
	public:
		TaskPnt();
		~TaskPnt();
		double _initState;
		double _increaseRatio;
		double _currentState;
		double _completedThreshold;
		bool iscompleted = false;
	};

	using  DisMapIndex = std::pair<size_t, size_t>;

	enum AgentStateType
	{
		onRoad,
		onTaskPnt
	};

	enum calType
	{
		arriveCondition,
		leaveCondition,
		endCondition,
		backCondition
	};

	inline std::ostream& operator<<(std::ostream &os, const TaskPnt &data)
	{
		if (!data.iscompleted)
		{
			return os << " initState = " << data._initState << "  currentState " << data._currentState << endl;
		}
		else
		{
			return os << " initState  = " << data._initState << "   task is completed " << endl;
		}
	}


	class TaskAgent
	{
	public:
		TaskAgent();
		~TaskAgent();
		double _ability;
	private:

	};
	using vTaskPnt = vector<TaskPnt>;
	using vTaskAgent = vector<TaskAgent>;
	using vTaskPntPtr = shared_ptr<vTaskPnt>;
	using vTaskAgentPtr = shared_ptr<vTaskAgent>;

	using DisMapPtr = shared_ptr<map<std::pair<size_t, size_t>, double>>;
	using DisMap = map<std::pair<size_t, size_t>, double>;
}