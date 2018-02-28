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

		//ÿ���߳�copyһ�ݲ�����texturÿ�߳�ÿͼƬÿ����weightsÿ�߳�ÿͼƬ
		void init(void);

		//-----------------------------------------------------------------
		// Image manipulation
		//-----------------------------------------------------------------

		// Find images with constraints m_angleThreshold, m_visdata,
		// m_sequenceThreshold, m_targets. Results are sorted by
		// CphotoSet::m_distances. ���վ���Զ���ҵ�Vp����
		void collectImages(const int index, std::vector<int>& indexes) const;

		// take into account m_edge
		void addImages(Patch::Cpatch& patch) const;
		void removeImagesEdge(Patch::Cpatch& patch) const;

		//��ʵ���ǣ�level�Ŵ� * patch������ľ��� / ����
		float getUnit(const int index, const Vec4f& coord) const;

		//����������Ϊ level * ���߾��� / cos ���Ƕ�Խƫ ֵԽ�����ֵԽСԽ��
		void computeUnits(const Patch::Cpatch& patch,
			std::vector<int>& indexes,
			std::vector<float>& fineness,
			std::vector<Vec4f>& rays) const;

		//����patch��ÿһ��image�ľ���-�Ƕ�Ȩֵ��ԽСԽ����
		void computeUnits(const Patch::Cpatch& patch,
			std::vector<float>& fineness) const;

		//-----------------------------------------------------------------
		// Optimization
		//-----------------------------------------------------------------

		//������ʧ�ܣ��򷵻�1��seedֵΪ1����ȡ*vp��scale��һ��ncc
		int preProcess(Patch::Cpatch& patch, const int id, const int seed);
		//����ʧ�ܷ���1
		void refinePatch(Patch::Cpatch& patch, const int id, const int time);
		void refinePatch2(Patch::Cpatch& patch, const int id, const int time);

		bool refinePatchBFGS(Patch::Cpatch& patch, const int id, const int time,
			const int ncc);
		bool refinePatchBFGS2(Patch::Cpatch& patch, const int id, const int time);

		//����ref�͡�vp������grids��patch��
		int postProcess(Patch::Cpatch& patch, const int id, const int seed);
		int postProcess2(Patch::Cpatch& patch, const int id, const int seed);

		//����patch��images���Ͼ���һ��Ϊref��
		void setRefImage(Patch::Cpatch& patch, const int id);

		int check(Patch::Cpatch& patch);

		std::vector<int> m_status;
		//��ǰpatch��indexͼƬ�ϵ�ͶӰ�㣬����text��
		int grabTex(const Vec4f& coord, const Vec4f& pxaxis, const Vec4f& pyaxis,
			const Vec4f& pzaxis, const int index, const int size,
			std::vector<float>& tex) const;
		int grabTex2(const Vec4f& coord, const Vec4f& pxaxis, const Vec4f& pnegxaxis, const Vec4f& pyaxis, const Vec4f& pzaxis, const int index, const int size, std::vector<float>& tex) const;


	protected:
		//�����60���ں��޳�60����ľ�ͼƬ
		void filterImagesByAngle(Patch::Cpatch& patch);

		//��patch��image���򣬸����� �Ƕ�ԽС������ԽСԽ�á�ͬʱ�޳����Ƶ�ͼƬ
		void sortImages(Patch::Cpatch& patch) const;

		//ʵ������ ����reference image��ȡ*vp
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


		//���ÿһ��index ��һ��nccs
		void setINCCs(const Patch::Cpatch& patch,
			std::vector<std::vector<float> >& nccs,
			const std::vector<int>& indexes,
			const int id, const int robust);

		void setINCCs2(const Patch::Cpatch& patch,
			std::vector<std::vector<float> >& nccs,
			const std::vector<int>& indexes,
			const int id, const int robust);




		//�ж�patch��indexͼƬ��newlevel����ͶӰ�Ƿ�ȫ
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
		//vect��Ϊvector����
		void encode(const Vec4f& coord, const Vec4f& normal,double* const vect, const int id) const;
		void encode2(const Vec4f& coord, const Vec4f& normal,const Vec4f& xaxis,const Vec4f& yaxis, double* const vect, const int id) const;

		void decode(Vec4f& coord, Vec4f& normal,const double* const vect, const int id) const;
		void decode(Vec4f& coord, const double* const vect, const int id) const;
		void decode2(Vec4f& coord, Vec4f& normal, Vec4f& xaxis, Vec4f& negxaxis, Vec4f& yaxis, float& angle, float& folder, const double* const vect, const int id) const;
		void decode2(Vec4f& coord, Vec4f& normal, Vec4f& xaxis, Vec4f& negxaxis, Vec4f& yaxis, const double* const vect, const int id) const;

		//���ݸ�������������ϵ ������ת��
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

		//incc������һ�ֱ�﷽ʽ��incc��Χ 0 - 2��robust��Χ 0 - 0.4
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
		std::vector<float> m_ascalesT;//3.75��

		// stores current parameters for derivative computation
		std::vector<Vec3f> m_paramsT;

		// Grabbed texture
		std::vector<std::vector<std::vector<float> > > m_texsT; // last is 7x7x3 patch ��49���� ÿ����һ��rgb��ɫ
		// weights for refineDepthOrientationWeighed
		std::vector<std::vector<float> > m_weightsT;
		// Working array for levmar
		std::vector<std::vector<double> > m_worksT;

	};
};

#endif // PMVS3_OPTIM_H
