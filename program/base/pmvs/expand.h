#ifndef PMVS3_EXPAND_H
#define PMVS3_EXPAND_H

#include <vector>
#include <queue>
#include <list>
#include "patchOrganizerS.h"

namespace PMVS3 {
	class CfindMatch;

	class Cexpand {
	public:
		Cexpand(CfindMatch& findMatch);
		~Cexpand() {};

		void init(void);
		void run(void);

		//patch射线打在imageray平面上的距离 * csize / focallength
		float computeRadius(const Patch::Cpatch& patch);

	protected:
		//orppatch为源patch，canCoord为扩展位置。返回1表示失败。如果扩展成功，则新patch加入queue扩展队列
		int expandSub(const Patch::Ppatch& orgppatch, const int id,
			const Vec4f& canCoord);

		int updateCounts(const Patch::Cpatch& patch);

		//检测可插入的数目empty（对应全局grid为空且 或 添加次数不超过阈值） 以及不可插入数目full，若无full或者empty未达到一定数量，则可插入
		int checkCounts(Patch::Cpatch& patch);

		//找到patch平面，6块区域中空缺邻近patch的区域
		void findEmptyBlocks(const Patch::Ppatch& ppatch,
			std::vector<std::vector<Vec4f> >& canCoords);
	protected:

		std::priority_queue<Patch::Ppatch, std::vector<Patch::Ppatch>, P_compare>
			m_queue;

		CfindMatch& m_fm;

		//-----------------------------------------------------------------
		// thread related
		//-----------------------------------------------------------------  
		void expandThread(void);
		static int expandThreadTmp(void* arg);

		// Number of trials
		std::vector<int> m_ecounts;
		// Number of failures in the prep
		std::vector<int> m_fcounts0;
		// Number of failures in the post processing
		std::vector<int> m_fcounts1;
		// Number passes
		std::vector<int> m_pcounts;
	};
};

#endif // PMVS3_EXPAND_H
