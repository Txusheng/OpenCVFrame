#ifndef PMVS3_OPTIM_H
#define PMVS3_OPTIM_H

#include <vector>
#include "patch.h"

using std::vector;
namespace PMVS3 {

	class CfindMatch;

	class Coptim {
	public:
		Coptim(CfindMatch& findMatch);

		//每个线程copy一份参数，textur每线程每图片每网格，weights每线程每图片
		void init(void);

		//-----------------------------------------------------------------
		// Image manipulation
		//-----------------------------------------------------------------

		// Find images with constraints m_angleThreshold, m_visdata,
		// m_sequenceThreshold, m_targets. Results are sorted by
		// CphotoSet::m_distances. 按照距离远近找到Vp数组
		void collectImages(const int index, std::vector<int>& indexes) const;

		// take into account m_edge
		void addImages(Patch::Cpatch& patch) const;
		void removeImagesEdge(Patch::Cpatch& patch) const;

		//其实就是：level放大 * patch到相机的距离 / 焦距
		float getUnit(const int index, const Vec4f& coord) const;

		//第三个参数为 level * 射线距离 / cos ，角度越偏 值越大。则该值越小越好
		void computeUnits(const Patch::Cpatch& patch,
			std::vector<int>& indexes,
			std::vector<float>& fineness,
			std::vector<Vec4f>& rays) const;

		//计算patch与每一个image的距离-角度权值，越小越贴合
		void computeUnits(const Patch::Cpatch& patch,
			std::vector<float>& fineness) const;

		//-----------------------------------------------------------------
		// Optimization
		//-----------------------------------------------------------------

		//若处理失败，则返回1。seed值为1。获取*vp和scale和一组ncc
		int preProcess(Patch::Cpatch& patch, const int id, const int seed);
		//处理失败返回1
		void refinePatch(Patch::Cpatch& patch, const int id, const int time);
		void refinePatch2(Patch::Cpatch& patch, const int id, const int time);

		bool refinePatchBFGS(Patch::Cpatch& patch, const int id, const int time,
			const int ncc);
		bool refinePatchBFGS2(Patch::Cpatch& patch, const int id, const int time);

		//更新ref和※vp，更新grids（patch）
		int postProcess(Patch::Cpatch& patch, const int id, const int seed);
		int postProcess2(Patch::Cpatch& patch, const int id, const int seed);

		//设置patch的images（毕竟第一张为ref）
		void setRefImage(Patch::Cpatch& patch, const int id);

		int check(Patch::Cpatch& patch);

		std::vector<int> m_status;
		//求当前patch对index图片上的投影点，存在text中
		int grabTex(const Vec4f& coord, const Vec4f& pxaxis, const Vec4f& pyaxis,
			const Vec4f& pzaxis, const int index, const int size,
			std::vector<float>& tex) const;
		int grabTex2(const Vec4f& coord, const Vec4f& pxaxis, const Vec4f& pnegxaxis, const Vec4f& pyaxis, const Vec4f& pzaxis, const int index, const int size, std::vector<float>& tex) const;


	protected:
		//添加完60°内后，剔除60°外的旧图片
		void filterImagesByAngle(Patch::Cpatch& patch);

		//对patch的image排序，根据是 角度越小、距离越小越好。同时剔除相似的图片
		void sortImages(Patch::Cpatch& patch) const;

		//实际上是 根据reference image获取*vp
		void constraintImages(Patch::Cpatch& patch, const float nccThreshold,
			const int id);
		void setRefConstraintImages(Patch::Cpatch& patch, const float nccThreshold,
			const int id);

		void setINCCs(const Patch::Cpatch& patch,
			std::vector<float> & nccs,
			const std::vector<int>& indexes,
			const int id, const int robust);

		void setINCCs2(const Patch::Cpatch& patch,
			std::vector<float> & nccs,
			const std::vector<int>& indexes,
			const int id, const int robust);


		//针对每一个index 求一组nccs
		void setINCCs(const Patch::Cpatch& patch,
			std::vector<std::vector<float> >& nccs,
			const std::vector<int>& indexes,
			const int id, const int robust);

		void setINCCs2(const Patch::Cpatch& patch,
			std::vector<std::vector<float> >& nccs,
			const std::vector<int>& indexes,
			const int id, const int robust);




		//判断patch在index图片上newlevel级别投影是否安全
		int grabSafe(const int index, const int size, const Vec3f& center,
			const Vec3f& dx, const Vec3f& dy, const int level) const;
		int grabSafe2(const int index, const int size, const Vec3f& center,const Vec3f& dx, const Vec3f& dnx,const Vec3f& dy, const int level) const;

		/*
		double computeINCC(const Vec4f& coord, const Vec4f& normal,
		const std::vector<int>& indexes, const int id,
		const int robust);
		*/
		double computeINCC(const Vec4f& coord, const Vec4f& normal,
			const std::vector<int>& indexes, const Vec4f& pxaxis,
			const Vec4f& pyaxis, const int id,
			const int robust);

	public:
		static void normalize(std::vector<float>& tex);
		static void normalize(std::vector<std::vector<float> >& texs, const int size);

		float dot(const std::vector<float>& tex0, const std::vector<float>& tex1) const;
		float ssd(const std::vector<float>& tex0, const std::vector<float>& tex1) const;
	protected:
		static void lfunc(double* p, double* hx, int m, int n, void* adata);
		void func(int m, int n, double* x, double* fvec, int* iflag, void* arg);

		//BFGS
		static double my_f(unsigned n, const double *x, double *grad, void *my_func_data);
		static double my_f2(unsigned n, const double *x, double *grad, void *my_func_data);

		void encode(const Vec4f& coord,
			double* const vect, const int id) const;
		//vect即为vector……
		void encode(const Vec4f& coord, const Vec4f& normal,double* const vect, const int id) const;
		void encode2(const Vec4f& coord, const Vec4f& normal,const Vec4f& xaxis,const Vec4f& yaxis, double* const vect, const int id) const;

		void decode(Vec4f& coord, Vec4f& normal,const double* const vect, const int id) const;
		void decode(Vec4f& coord, const double* const vect, const int id) const;
		void decode2(Vec4f& coord, Vec4f& normal, Vec4f& xaxis, Vec4f& negxaxis, Vec4f& yaxis, float& angle, float& folder, const double* const vect, const int id) const;
		void decode2(Vec4f& coord, Vec4f& normal, Vec4f& xaxis, Vec4f& negxaxis, Vec4f& yaxis, const double* const vect, const int id) const;

		//根据给出的两个坐标系 计算旋转角
		int getRotateAngle(const Vec4f& xaxis1, const Vec4f& yaxis1, const Vec4f& zaxis1, const Vec4f& xaxis2, const Vec4f& yaxis2, const Vec4f& zaxis2, float& anglex, float& angley, float &anglez);
		vector<vector<float>> DotMat(const vector<vector<float>>& A, const vector<vector<float>>& B);
		void getRotateMetrix(const float& anglex, const float& angley, const float &anglez, vector<vector<float>>& matR);
		void RotateWithAngle(const float& anglex, const float& angley, const float &anglez, const Vec4f& xaxis1, const Vec4f& yaxis1, const Vec4f& zaxis1, Vec4f& xaxis2, Vec4f& yaxis2, Vec4f& zaxis2);



	public:
		void setWeightsT(const Patch::Cpatch& patch, const int id);

		double computeINCC(const Vec4f& coord, const Vec4f& normal,
			const std::vector<int>& indexes, const int id,
			const int robust);
		void getPAxes(const int index, const Vec4f& coord, const Vec4f& normal,
			Vec4f& pxaxis, Vec4f& pyaxis) const;

		//incc的另外一种表达方式，incc范围 0 - 2，robust范围 0 - 0.4
		static inline float robustincc(const float rhs) {
			return rhs / (1 + 3 * rhs);
		}

		static inline float unrobustincc(const float rhs) {
			return rhs / (1 - 3 * rhs);
		}

	protected:

		void setAxesScales(void);

		static Coptim* m_one;
		CfindMatch& m_fm;

		//-----------------------------------------------------------------
		// Axes
		std::vector<Vec3f> m_xaxes;
		std::vector<Vec3f> m_yaxes;
		std::vector<Vec3f> m_zaxes;
		// Scales
		std::vector<float> m_ipscales;

		//-----------------------------------------------------------------
		// For threads
		std::vector<float> m_vect0T;
		std::vector<Vec4f> m_centersT;
		std::vector<Vec4f> m_raysT;
		std::vector<Vec4f> m_xaxisT;
		std::vector<Vec4f> m_yaxisT;
		std::vector<std::vector<int> > m_indexesT;
		std::vector<float> m_dscalesT;
		std::vector<float> m_ascalesT;//3.75°

		// stores current parameters for derivative computation
		std::vector<Vec3f> m_paramsT;

		// Grabbed texture
		std::vector<std::vector<std::vector<float> > > m_texsT; // last is 7x7x3 patch ，49个点 每个点一个rgb颜色
		// weights for refineDepthOrientationWeighed
		std::vector<std::vector<float> > m_weightsT;
		// Working array for levmar
		std::vector<std::vector<double> > m_worksT;

	};
};

#endif // PMVS3_OPTIM_H
