#pragma once
#include "TaskPnt.hpp"
#include "ssconfig.hpp"
#include <tuple>
namespace decode
{
	using cfgTuple = tuple<vTaskAgentPtr, vTaskPntPtr, DisMapPtr, DisMapPtr>;
	enum cfgID
	{
		vTaskAgentPtrID,
		vTaskPntPtrID,
		taskDisMatPtrID,
		ag2taskDisMatPtrID
	};
	class ReadConfig
	{
	public:
		ReadConfig(char *filename) :
			_m_fileName(filename)
		{}
		~ReadConfig();
		void read();
		cfgTuple getCfg();
		vTaskAgentPtr _m_vTaskAgentPtr;
		vTaskPntPtr _m_vTaskPntPtr;
		DisMapPtr _taskDisMatPtr;
		DisMapPtr _ag2taskDisMatPtr;
		vector<size_t> _encode;
	private:
		char *_m_fileName;

	};
}