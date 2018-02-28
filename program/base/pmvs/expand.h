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

		//patch���ߴ���imagerayƽ���ϵľ��� * csize / focallength
		float computeRadius(const Patch::Cpatch& patch);

	protected:
		//orppatchΪԴpatch��canCoordΪ��չλ�á�����1��ʾʧ�ܡ������չ�ɹ�������patch����queue��չ����
		int expandSub(const Patch::Ppatch& orgppatch, const int id,
			const Vec4f& canCoord);

		int updateCounts(const Patch::Cpatch& patch);

		//���ɲ������Ŀempty����Ӧȫ��gridΪ���� �� ��Ӵ�����������ֵ�� �Լ����ɲ�����Ŀfull������full����emptyδ�ﵽһ����������ɲ���
		int checkCounts(Patch::Cpatch& patch);

		//�ҵ�patchƽ�棬6�������п�ȱ�ڽ�patch������
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
