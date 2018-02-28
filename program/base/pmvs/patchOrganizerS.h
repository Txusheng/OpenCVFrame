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

  //��ʼ������pPatch���飬dpPatchȫ�����ĳ����һ�ϲ�ָ��
  void init(void);

  //seedִ��֮��patchָ�����grid���档�ú�����timage�е�grid��patch���ܣ�ͬʱ����patch���߳�id
  void collectPatches(const int target = 0);

  //patch����ncc��С��������queue��������flagΪ1�����ظ����룩
  void collectPatches(std::priority_queue<Patch::Ppatch, std::vector<Patch::Ppatch>,
                      P_compare>& pqpatches);
  
  void collectPatches(const int index,
                      std::priority_queue<Patch::Ppatch, std::vector<Patch::Ppatch>,
                      P_compare>& pqpatches);
  void collectNonFixPatches(const int index, std::vector<Patch::Ppatch>& ppatches);
  
  void writePatches2(const std::string prefix, bool bExportPLY, bool bExportPatch, bool bExportPSet);
  
  void writePLY(const std::vector<Patch::Ppatch>& patches,
                const std::string filename);

  //��patch�����
  void writeFace(const std::vector<Patch::Ppatch>& patches, const std::string filename);
  void writeFace2(const std::vector<Patch::Ppatch>& patches, const std::string filename);
  


  void writePLY(const std::vector<Patch::Ppatch>& patches,
                const std::string filename,
                const std::vector<Vec3i>& colors);
  
  void readPatches(void);
  
  //ÿһ��ͼƬÿһ��grid��count����Ϊ0
  void clearCounts(void);
  //ÿһ��patch��test flag����Ϊ0
  void clearFlags(void);

  //����image��patch���m_grid �Լ� m_images������vp�Լ�grid��
  void setGridsImages(Patch::Cpatch& patch,
                      const std::vector<int>& images) const;

  //1. ��Ӧimage��grid�����̻߳��⣩���patchָ�룬�������ȼ�� ��ӿ��ӵ�gridָ�룬�������ͼ��
  void addPatch(Patch::Ppatch& ppatch);

  // image��Ӧ��grid �� vgrid ɾ��ָ�룬���� m_patch �������б��ݣ�
  void removePatch(const Patch::Ppatch& ppatch);

  void setGrids(Patch::Ppatch& ppatch) const;

  //����patch��grid
  void setGrids(Patch::Cpatch& patch) const;

  void setVImagesVGrids(Patch::Ppatch& ppatch);

  void setVImagesVGrids(Patch::Cpatch& patch);

  //����dpgrid����ǰpatch���С���滻���滻��ΧΪ��Χ4��grid
  void updateDepthMaps(Patch::Ppatch& ppatch);
  
  int isVisible(const Patch::Cpatch& patch, const int image,
                const int& ix, const int& iy,
                const float strict, const int lock);
  int isVisible0(const Patch::Cpatch& patch, const int image,
                 int& ix, int& iy,
                 const float strict, const int lock);

  //�Ե�ǰpatch����������image��������vimage����cell ��Χ�ĸ�cell������vimage������� ���ڽ�patch��
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
  // image, grid ����ά ÿ��ͼƬ������״������ָ��patch
  std::vector<std::vector<std::vector<Patch::Ppatch> > > m_pgrids;  
  // image, grid��ÿ��ͼƬ��������ָ�룬��������ͶӰ����ok
  std::vector<std::vector<std::vector<Patch::Ppatch> > > m_vpgrids;
  // Closest patch����ά ÿ��ͼƬһ��patch
  std::vector<std::vector<Patch::Ppatch> > m_dpgrids;

  // all the patches in the current level of m_pgrids 
  std::vector<Patch::Ppatch> m_ppatches;

  // Check how many times patch optimization was performed for expansion��Ϊ��ÿ��ͼƬҲ��һ�����֣�
  std::vector<std::vector<unsigned char> > m_counts;

  static Patch::Ppatch m_MAXDEPTH;
  static Patch::Ppatch m_BACKGROUND;
  
 protected:
  CfindMatch& m_fm;
};
};

#endif //PMVS3_PATCHORGANIZERS_H
