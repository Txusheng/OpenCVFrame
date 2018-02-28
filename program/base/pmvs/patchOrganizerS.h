#ifndef PMVS3_PATCHORGANIZERS_H
#define PMVS3_PATCHORGANIZERS_H

#include "patch.h"
#include <queue>

namespace PMVS3 {

class CfindMatch;

class P_compare {
public:
  bool operator()(const Patch::Ppatch& lhs, const Patch::Ppatch& rhs) const {
    return lhs->m_tmp < rhs->m_tmp;
  }
};
 
class CpatchOrganizerS {
 public:
  CpatchOrganizerS(CfindMatch& findMatch);

  //初始化各种pPatch数组，dpPatch全部填充某个单一废柴指针
  void init(void);

  //seed执行之后，patch指针存在grid里面。该函数将timage中的grid中patch汇总，同时更新patch的线程id
  void collectPatches(const int target = 0);

  //patch按照ncc从小到大推入queue（设置其flag为1避免重复推入）
  void collectPatches(std::priority_queue<Patch::Ppatch, std::vector<Patch::Ppatch>,
                      P_compare>& pqpatches);
  
  void collectPatches(const int index,
                      std::priority_queue<Patch::Ppatch, std::vector<Patch::Ppatch>,
                      P_compare>& pqpatches);
  void collectNonFixPatches(const int index, std::vector<Patch::Ppatch>& ppatches);
  
  void writePatches2(const std::string prefix, bool bExportPLY, bool bExportPatch, bool bExportPSet);
  
  void writePLY(const std::vector<Patch::Ppatch>& patches,
                const std::string filename);

  //将patch面输出
  void writeFace(const std::vector<Patch::Ppatch>& patches, const std::string filename);
  void writeFace2(const std::vector<Patch::Ppatch>& patches, const std::string filename);
  


  void writePLY(const std::vector<Patch::Ppatch>& patches,
                const std::string filename,
                const std::vector<Vec3i>& colors);
  
  void readPatches(void);
  
  //每一个图片每一个grid中count设置为0
  void clearCounts(void);
  //每一个patch的test flag设置为0
  void clearFlags(void);

  //按照image给patch添加m_grid 以及 m_images（复制vp以及grid）
  void setGridsImages(Patch::Cpatch& patch,
                      const std::vector<int>& images) const;

  //1. 对应image的grid（多线程互斥）添加patch指针，如果有深度检测 添加可视的grid指针，更新深度图。
  void addPatch(Patch::Ppatch& ppatch);

  // image对应的grid 和 vgrid 删除指针，但是 m_patch 还保留有备份？
  void removePatch(const Patch::Ppatch& ppatch);

  void setGrids(Patch::Ppatch& ppatch) const;

  //设置patch的grid
  void setGrids(Patch::Cpatch& patch) const;

  void setVImagesVGrids(Patch::Ppatch& ppatch);

  void setVImagesVGrids(Patch::Cpatch& patch);

  //更新dpgrid，当前patch深度小则替换，替换范围为周围4个grid
  void updateDepthMaps(Patch::Ppatch& ppatch);
  
  int isVisible(const Patch::Cpatch& patch, const int image,
                const int& ix, const int& iy,
                const float strict, const int lock);
  int isVisible0(const Patch::Cpatch& patch, const int image,
                 int& ix, int& iy,
                 const float strict, const int lock);

  //对当前patch，找其所有image（不包含vimage）中cell 周围四个cell（包含vimage）里面的 “邻近patch”
  void findNeighbors(const Patch::Cpatch& patch,
                     std::vector<Patch::Ppatch>& neighbors,
                     const int lock,
                     const float scale = 1.0f,
                     const int margin = 1,
                     const int skipvis = 0);
  
  void setScales(Patch::Cpatch& patch) const;
  
  float computeUnit(const Patch::Cpatch& patch) const;

  // change the contents of m_images from images to indexes
  void image2index(Patch::Cpatch& patch);
  // change the contents of m_images from indexes to images
  void index2image(Patch::Cpatch& patch);
  
  //----------------------------------------------------------------------
  // Widths of grids
  std::vector<int> m_gwidths;
  std::vector<int> m_gheights;
  //----------------------------------------------------------------------
  // image, grid ，三维 每张图片（网格状）智能指针patch
  std::vector<std::vector<std::vector<Patch::Ppatch> > > m_pgrids;  
  // image, grid，每张图片可视智能指针，几乎就是投影到就ok
  std::vector<std::vector<std::vector<Patch::Ppatch> > > m_vpgrids;
  // Closest patch，二维 每张图片一组patch
  std::vector<std::vector<Patch::Ppatch> > m_dpgrids;

  // all the patches in the current level of m_pgrids 
  std::vector<Patch::Ppatch> m_ppatches;

  // Check how many times patch optimization was performed for expansion，为何每张图片也是一组数字？
  std::vector<std::vector<unsigned char> > m_counts;

  static Patch::Ppatch m_MAXDEPTH;
  static Patch::Ppatch m_BACKGROUND;
  
 protected:
  CfindMatch& m_fm;
};
};

#endif //PMVS3_PATCHORGANIZERS_H
