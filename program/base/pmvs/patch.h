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

  //编号对应image，存grid坐标
  std::vector<TVec2<int> > m_grids;
  
  // visible images. m_vimages must be targetting images.
  std::vector<int> m_vimages;
  std::vector<TVec2<int> > m_vgrids;
  
  //----------------------------------------------------------------------
  inline float score(const float threshold) const{
    return (std::max)(0.0f, m_ncc - threshold) * (int)m_images.size();
  }

  // 总ncc - 总threshold
  inline float score2(const float threshold) const{
    return (std::max)(0.0f, m_ncc - threshold) * m_timages;
  }

  // average ncc ，平均归一化协相关
  float m_ncc;
  // number of targetting images in m_images
  int m_timages;

  // flat for expansion
  // 0: not yet tested
  // 1: done
  int m_flag;

  // for directional flag
  unsigned char m_dflag;

  // fixed patch or not（target 则为0，否则为1）
  char m_fix;
  
  // id number in m_ppatches
  int m_id;

  // scaling factor corresponding to one pixel difference

  //平均，*vp中图片每沿极线移动一个像素，对应patch沿法向量移动的距离
  float m_dscale;
  //平均，*vp中图片每沿极线移动一个像素，对应的patch在*vp中改动的比例的arctan值
  float m_ascale;
  //优化后总ncc - 总threshold
  float m_tmp;
};

//Cpatch的智能指针
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
