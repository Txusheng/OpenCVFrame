#ifndef PMVS3_SEED_H
#define PMVS3_SEED_H

#include <boost/shared_ptr.hpp>
#include <vector>
#include "patch.h"
#include "point.h"

#include "tinycthread.h"

namespace PMVS3 {
class CfindMatch;
typedef boost::shared_ptr<Cpoint> Ppoint;
  
class Cseed {
 public:
  Cseed(CfindMatch& findMatch);
  virtual ~Cseed() {};

  //���������㣬�������������vector
  void init(const std::vector<std::vector<Cpoint> >& points);
  //���ɵ�һ��patch	
  void run(void);
  //patch��ʼ��֮�󣬲�����Ҫ������
  void clear(void);

 protected:
  void readPoints(const std::vector<std::vector<Cpoint> >& points);

  //����Ϊ1��δ�ﵽ������ֵ��δ�ؽ�
  int canAdd(const int index, const int x, const int y);  

  //Sceed run��ʵ��ִ����ˮ�����ɵ�һ��patch
  void initialMatch(const int index, const int id);
  void collectCells(const int index0, const int index1,
                    const Cpoint& p0, std::vector<Vec2i>& cells);
  
  // make sorted array of feature points in images, that satisfy the epipolar geometry coming from point in image
  // ������ ��ǰͼƬ���ڽ�ͼƬ����ǰ�㡢&��ѡ������
  void collectCandidates(const int index, const std::vector<int>& indexes,
                         const Cpoint& point, std::vector<Ppoint>& vcp);

  int initialMatchSub(const int index0, const int index1,
                      const int id, Patch::Cpatch& patch);
  
  void unproject(const int index0, const int index1,
                 const Cpoint& p0, const Cpoint& p1,
                 Vec4f& coord) const;

  //----------------------------------------------------------------------
  CfindMatch& m_fm;
  // points in a grid. For each index, grid
  std::vector<std::vector<std::vector<Ppoint> > > m_ppoints;

  //----------------------------------------------------------------------
  // thread related
  //----------------------------------------------------------------------
  void initialMatchThread(void);
  static int initialMatchThreadTmp(void* arg);

  // Number of trials
  std::vector<int> m_scounts;
  // Number of failures in the prep
  std::vector<int> m_fcounts0;
  // Number of failures in the post processing
  std::vector<int> m_fcounts1;
  // Number passes
  std::vector<int> m_pcounts;
};
};

#endif // PMVS3_SEED_H
