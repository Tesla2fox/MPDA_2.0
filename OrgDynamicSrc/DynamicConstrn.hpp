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
			c_debug << "��̬����" << endl;
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


		//��̬���򷽷���ʼ
		//����Ԥ���ĵ���ʱ������ʱ��
		void updatePreArrAndComTime();
		//���½�
		void updateEnmat(size_t const & type);
		//��С����ʱ��������������̶ȸ��Ϲ���
		void DminArrTimeAndMaxEmergentSolution();

		void sortPreFirstArrTime();
		std::map<AgTaskPair, size_t> _orderPreFirstArrTime;
		//��С����
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
		//�Ӵ�С
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


		//��С����ʱ��������������̶ȸ��Ϲ���
		void minArrTimeAndMaxEmergentSolution();
		//��С����ʱ�������������ʱ��
		void minArrTimeAndMinComTimeSolution();

		vector<EnMat> _m_vEnMat;

		// zhu's method end
		class AgentState
		{
		public:
			//�뿪������ʱ��
			double _leaveTime;
			//����������ʱ��
			double _arriveTime;
			//�������ִ�������ʱ��
			double _executeDur;
			//ǰ��������·��ʱ��
			double _roadDur;
			//��ǰ�������
			//��_taskid = -1 ��ʾ���ƶ��������ڳ�ʼλ��
			int  _taskid;
			//��ǰ��ʼʲô״̬
			size_t _stateType = AgentStateType::onTaskPnt;
			//�Ƿ��Ѿ�����ȫ������
			bool _stopBoolean = false;
			//
			bool _executeBoolean = false;
			double _ability;
			//Ԥ���ĵ���ʱ��
			vector<double> _vPreArrTime;
			//Ԥ����ִ��ʱ��
			vector<double> _vPreExcDur;
			//Ԥ�������ʱ��
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
			//Ԥ���ĵ���ʱ��
			vector<double> _vPreArrTime;
			//Ԥ����ִ��ʱ��
			vector<double> _vPreExcDur;
			//Ԥ�������ʱ��
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


		//���ɸ���
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

		// ������״̬�������״̬
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