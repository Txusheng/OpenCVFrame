#ifndef PMVS3_PATCH_H
#define PMVS3_PATCH_H

#include <vector>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include "../numeric/vec4.h"

namespace Patch {

class Cpatch 
{
 public:
  Cpatch(void) 
  {
    m_ncc = -1.0;
    m_timages = 0;
    m_fix = 0;
    // dflag is initialized only once. if failed in one direction, we never try that.
    m_dflag = 0;
  }
  
  //----------------------------------------------------------------------
  // saved information
  // 3D coordinates of the center of the patch
  Vec4f m_coord;
  // patch outward normal vector
  Vec4f m_normal;
  //y axis
  Vec4f m_yaxis;
  //x axis
  Vec4f m_xaxis,m_negxaxis;
  //angle
  float m_angle;
  //folder
  float m_folder;
  
  // associated image ids. first image id is the reference one. images
  // can be non-targetting image.
  std::vector<int> m_images;

  //��Ŷ�Ӧimage����grid����
  std::vector<TVec2<int> > m_grids;
  
  // visible images. m_vimages must be targetting images.
  std::vector<int> m_vimages;
  std::vector<TVec2<int> > m_vgrids;
  
  //----------------------------------------------------------------------
  inline float score(const float threshold) const{
    return (std::max)(0.0f, m_ncc - threshold) * (int)m_images.size();
  }

  // ��ncc - ��threshold
  inline float score2(const float threshold) const{
    return (std::max)(0.0f, m_ncc - threshold) * m_timages;
  }

  // average ncc ��ƽ����һ��Э���
  float m_ncc;
  // number of targetting images in m_images
  int m_timages;

  // flat for expansion
  // 0: not yet tested
  // 1: done
  int m_flag;

  // for directional flag
  unsigned char m_dflag;

  // fixed patch or not��target ��Ϊ0������Ϊ1��
  char m_fix;
  
  // id number in m_ppatches
  int m_id;

  // scaling factor corresponding to one pixel difference

  //ƽ����*vp��ͼƬÿ�ؼ����ƶ�һ�����أ���Ӧpatch�ط������ƶ��ľ���
  float m_dscale;
  //ƽ����*vp��ͼƬÿ�ؼ����ƶ�һ�����أ���Ӧ��patch��*vp�иĶ��ı�����arctanֵ
  float m_ascale;
  //�Ż�����ncc - ��threshold
  float m_tmp;
};

//Cpatch������ָ��
typedef boost::shared_ptr<Cpatch> Ppatch;

struct Spatchcmp {
  bool operator()(const Ppatch& lhs, const Ppatch& rhs) {
    if (lhs.get() < rhs.get())
      return true;
    else
      return false;
  }
};
 
std::istream& operator >>(std::istream& istr, Patch::Cpatch& rhs);
std::ostream& operator <<(std::ostream& ostr, const Patch::Cpatch& rhs);
 
};

#endif // PMVS3_PATCH_H
