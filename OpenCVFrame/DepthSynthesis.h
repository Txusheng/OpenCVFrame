#ifndef DEPTHSYSTHESIS_H
#define DEPTHSYSTHESIS_H

#include <array>
#include <vector>
#include <QtWidgets/QMainWindow>
#include "ui_opencvframe.h"
#include <QtWidgets/QmdiSubWindow>
#include <QtWidgets/QmdiArea>
#include <QtWidgets/QAction>
#include <QFileDialog>
#include <QString>
#include <QStringList>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <QDir>
#include <QFile>
#include <QLayout>
#include <QStackedWidget>
#include <QScrollArea>
#include <QLabel>
#include <QImage>
#include <QPixmap>
#include <QMessageBox>
#include <cmath>
#include "graph.h"
#include <list>
#include <math.h>
#include <QThread>

const float kAngleWeight = 0.9;
const bool bWarpOut = false;
const bool bDepthOut = false;

#include <Eigen/Dense>

class OpenCVFrame;

using namespace std;
using namespace cv;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class ImgInfo;
class SuperPixel;
class GridTriangle;
class DepthSynWidget;
class BinVertex;


//������������֮��ļн�
template<typename T>
float GetAngleCos(const vector<T> &a, const vector<T> &b);

//���������ľ���
template<typename T>
float GetNorm(const vector<T> &a);


//�ռ���ά�㣬���˼�¼��ά��Ϣ֮�⣬����¼��ά�� ����һ��ͼ����һ�������� ��׽
class MyPoint3D :public Point3f
{
public:
	MyPoint3D();
	~MyPoint3D();

	MyPoint3D(float x, float y, float z);

	//�� ��һ��ͼƬ ����һ�������� ��׽,[0] ��ʾ��һ��ͼ�ж�Ӧ��sp,��ʼ��Ϊ-1�����ǿ��ӵļ�Ϊ-2
	int *ConnectImgSp;
	int nSize;

	//ɾ����ά��ͳ�����֮��� ��Ӧ��ϵ
	void ClearConnect();
private:

};

class HashNode
{
public:
	HashNode(int i);
	~HashNode();

	int nIndex;
	HashNode *next;
private:

};


//ͼ�����ڼ�¼��ͬԭ��Ƭ ������֮������ӹ�ϵ
class BinGraph
{
public:
	BinGraph(const vector<ImgInfo*> &vImg);
	~BinGraph();

	//��Ԫ����ͼ�ṹ
	vector<vector<BinVertex*>> graph;


	//������ά���д�ŵ� �������ع۲���Ϣ������ͼ������֮���ͷ���ά�� �ڴ��Ӧ��
	void Initialize(vector<MyPoint3D*> vPoints);

	//���ұ��Ƿ����,�����ڣ���true �ڶ�������Ϊnull����������Ϊfalse ���ؿ��Բ���ýڵ��λ��
	pair<bool, HashNode**> FindEdge(int i1, int j1, int i2, int j2);

	//���������ڵ�
	void AddEdge(int i1, int j1, int i2, int j2);

private:


};


class BinVertex
{
public:
	BinVertex(int i, int j, int nHashSize);
	~BinVertex();

	//��Ƭ ���� �������
	int i;
	//����������
	int j;

	// 	const SuperPixel *sp;

	// 	BinVertex *next;

	//���ջ��Ǿ�����ȡ��ϣ��ķ�ʽ�洢���ӣ��������죬�Ͼ������� ����������Ƭ���ࣩ
	vector<HashNode*> hash;

private:

};


//�������
class CameraMatrix
{
public:
	CameraMatrix();
	~CameraMatrix();

	Mat matRotation;
	Mat T;
	float fFocal;
	vector<float> vK;

public:
	Point3f center;
	Point3f CalCenter();
	void NormolizeR();

private:
	bool bCenter;
};




//bundler + pmvs �����������ȫ����Ϣ
class SamplePoint
{
public:
	SamplePoint();
	~SamplePoint();

	//�ռ���ά��
	MyPoint3D *Point3D;

	//ͶӰ��ͼƬ�еĵ�
	Point2i Point2D;

	//ԭʼͶӰ�㣨��ɢ��֮ǰ��
	Point2f Point2Df;

	//ͶӰ֮������
	float fDepth;

	//�Ƿ���pmvs�ؽ������ĵ�
	bool bOrigin;

	//���ڳ����ر��
	int nSpIndex;


	//����ǲ�ֵ�õ��ĵ㣬�����������õ���ά��
	void BackProjection(Mat src, const CameraMatrix *camera);

	//����������󣬴���ά��ͶӰ����ά��
	bool Projection(Mat src, const CameraMatrix *camera);

protected:


private:

};


//����ϵͳ��Ϣ
class LinearSystem
{
public:
	LinearSystem();
	~LinearSystem();

	SuperPixel* m_spParentSP;

	vector<vector<double>> A;
	vector<double> B;

	//��δ�õ��������֮ǰ��B�洢 ����ϵͳ���ݣ��Ա����ʱֱ�ӵ���
	vector<list<pair<Point3f*, float>>> BInfo;


private:

};



//sp���棬�����εĶ���,����λ�á����������Ρ����
class TriVertex
{
public:
	TriVertex();
	~TriVertex();

	TriVertex(Point2d point, int index);

	Point2d vertex;
	list<int> parentTri;
	int nIndex;

private:

};



//��άͼ�ϵ���ȵ�
typedef struct DepthPoint
{
	Point point;
	float fDepth;
};


//ͼ�ṹ��ÿ���ڵ���ڽ�����
typedef struct Line
{
	int nEnd;
	float nWeight;
	Line* next;
};

//ͼ�ṹ�Ľڵ�
class Vertex
{
public:
	Vertex();
	~Vertex();
	int nStart;
	int nCount;
	Line* next;

	void PushBack(Line* pLine);

protected:

private:

};


//�������е�ÿһ�������νṹ
class GridTriangle
{
public:
	GridTriangle(SuperPixel *parent);
	~GridTriangle();

	//�������ĳ�����
	SuperPixel *m_spParentSP;

	//���������е�������
	vector<SamplePoint*> vpSamplePoints;

	//���������Ӧ����������
	vector<TriVertex*> vVertex;

	//

private:

};


//�����ص�������Ϣ
class SuperPixel
{

public:
	SuperPixel();
	~SuperPixel();

	//�����Լ�������ϵͳ
	void getLinearSys();
	void solveLinearSys(CameraMatrix *_camera);
	float getNovalDepth(const CameraMatrix *camera);


	ImgInfo* m_spParentImg;

	//���������sample�����Ϣ����������
	vector<float> vDepth;

	//�����ֵ
	float fMedianDepth;


	//�������е�sample
	vector<SamplePoint*> vpSamples;

	list<Point2i> listPoint;

	//��ֵ������¼��״
	Mat matSP;

	Rect boundingRect;

	bool bExtension;

	bool bHole;

	int iImgIndex;

	bool bOrigin;

	///////////////////////////////////////////////////////////////////////////////////////////////////////////
	//��չ֮��ĳ�����


	//��ֵ������¼��չ�����״
	Mat matExtSP;

	Rect ExtBounding;

	list<Point2i> listExtPoint;

	vector<TriVertex*> vVertexpoint;

	vector<vector<int>> vTriangleIndex;

	//����������������Σ���������εĻ�����Ϣ������λ�õȴ����vVertexPoint����Ϊ�����ǹ��õģ�
	vector<GridTriangle> vTriangle;

	int nVertexNum;

	int nXNum;
	int nYNum;

	float fXStep;
	float fYStep;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//warp֮��ĳ�������Ϣ

	//warp֮����������������
	vector<TriVertex*> vWarpVertex;

	//warp֮�����������Ϣ
	vector<vector<int>> vWarpTri;

	//����ϵͳ���
	LinearSystem m_LinSys;

	//���ӽǶ�Ӧ�����
	bool bNovalDepth;
	float fNovalDepth;

protected:

private:

};

//ÿһ��ͼƬ��������Ϣ(�󲿷�Ϊֱ�����ã����ǿ�¡)
class ImgInfo
{

public:
	ImgInfo();
	~ImgInfo();
	ImgInfo(DepthSynWidget* parent);

	//�������λ�ã��������ӽǵ�ͼƬ
	Mat dst;
	//���ӽ� ��Ӧ�����
	Mat dstDepth;

	//�������ӽ�ͼƬ��Ŀ��Ϊdst�� �ڴ�֮ǰ������ɳ����ص�����ϵͳ���
	void GenerateView();


	DepthSynWidget *m_spParentWidget;

	//���
	int Index;

	//ԭʼͼ
	Mat src;


	//ÿһ��ͼƬ��Ӧһ��SP
	vector<SuperPixel> vSP;

	//���ͼ,3ͨ��float����һ��Ϊlabel���ڶ���Ϊdepth(����ͶӰ���λ��)
	Mat depthMat;

	//labelͼ������
	Mat labelMat;

	//bool �ж�����ͼƬ�ǲ���jb�ö�û��
	bool bValid;

	//���λ��
	CameraMatrix* camera;

protected:

private:

};





class DepthSynWidget : public QWidget
{
	Q_OBJECT

public:
	DepthSynWidget(OpenCVFrame *parent);
	~DepthSynWidget();

	friend class depthSynThread;
	friend class warpThread;

	QMutex mutex;

	//��ǰ��Ҫ����local warp�����λ��,��ʼֵΪ��һ�����λ��
	CameraMatrix *camera;

	//�ĸ�ѡ�е�img pack
	vector<ImgInfo*> vSelectedImg;


	// ��ô˵������Ҫ����һ�����ݰ�
	vector<Mat> vSrc;
	QMutex mutexSrc;


	//label Mat�ı���,�洢��Ϊint��
	vector<Mat> vLabelMat;
	QMutex mutexLabelMat;


	OpenCVFrame *m_spOpenCVFrame;

	vector<MyPoint3D*> vPoints;

	vector<CameraMatrix*> vCamera;

	//��ȵ������ point2D �Լ� ��Ӧ�����
	vector<vector<DepthPoint>> vvDepthPoint;
	QMutex mutexDepthPoint;

	vector<vector<SamplePoint*>> vvSamplePoint;
	QMutex mutexSamplePoint;

	//ÿһ��mat����vec<float,3>�� Ԫ�ص�һ��ֵ��label���ڶ���ֵ��depth
	vector<Mat> vDepthMat;
	QMutex mutexDepthMat;

	//ÿһ��ͼƬ��ÿһ��label��Ӧ��ȵ�ĸ���
	vector<vector<int>> vvLabelPointCount;
	QMutex mutexLabelPointCount;

	//ÿһ��ͼ��ÿһ��label��Ӧ�������
	vector<map<int, vector<DepthPoint>>> vmapImgSPDepth;
	QMutex mutexSpDepth;

	//ÿһ��ͼƬ�� Ŀ��SP
	vector<vector<int>> vvTargetSPLabel;
	QMutex mutexTargetSPLabel;

	//ÿһ��ͼƬ��ԭʼ Ŀ��sp���жϳ�������ȵ��Ƿ�Ϊ��ֵ���ã�
	vector<vector<int>> vvNoneOriginSPLabel;
	QMutex mutexNoneOriginLabel;

	//ÿһ��ͼƬ ÿһ��label ��Ӧһ������ֱ��ͼ
	vector<vector<vector<float>>> vvvImgLabelHis;
	QMutex mutexImgLabelHis;

	//ÿһ��ͼ��ÿһ��label��Ӧ�ļ������ڽ�label
	vector<map<int, vector<int>>> vmapImgTargetNearLabel;
	QMutex mutexTargetNear;

	//ÿһ��ͼ��ÿһ��SP��Ӧһ���ڽ�����
	vector<vector<Vertex*>> vvImgVertexAdjoint;
	QMutex mutexVertexAdjoint;

	//ÿһ��ͼ����һ���ڽӱ�
	vector<vector<vector<float>>> vGraph;
	QMutex mutexGraph;

	vector<Mat> vTargetMaskMat;
	QMutex mutexTargetMask;

	vector<Mat> vBlendMat;
	QMutex mutexBlendMat;

	//�ж�list.txt����λ�ؽ���Ϣ�Ƿ���룬�Ƿ���Խ��к�����Ⱥϳ�
	bool bList;


	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//������������debugʹ��

	//�Ƿ�skyͼ����������ļ���
	bool bDebugSky;

	//������Ȳ�ֵ����һ�ε��� ��ִ����Ȳ�ֵ�� Ŀ��sp
	vector<vector<int>> vvImgEnableTarget;
	QMutex mutexEnableTarget;

	//�ɲ�ֵlabel�����ӦdepthHist
	vector<map<int, vector<float>>> vmapImgSpDepthHis;
	QMutex mutexSpDepthHis;

	//ÿһ��ͼƬ�� ���percentile�ֲ�
	vector<map<float, int>> vmapDepthPercentile;
	QMutex mutexDepthPercentile;




public:
	//�ṩ������õĺ���

	//��������RGBͼ�������ر��ͼ�������� ������Ŀ���飬���㲢���ض�Ӧ��ֱ��ͼ����
	vector<vector<float>> CalLabHist(const Mat &srcRGB, const Mat &matLabel, int nNumSP, const vector<int> &vSpPixelNum, int nBins = 20);

	// ��������ֱ��ͼ�Ŀ������룬����֮ǰҪ��֤�Ѿ���һ��
	float CalChiSquareDis(vector<float> vA, vector<float> vB);





private:
	void PointsProjection(int i, depthSynThread* thread);
	void TargetSPBlend(int i, depthSynThread* thread);


	//����ÿһ��ͼ��ÿһ��label��Ӧ��lab ��һ��ֱ��ͼ
	void CalLabHist(int i, depthSynThread* thread);

	//�ҵ�ÿһ��Label�����ڽ�Label
	void FindNearLabel(int i, depthSynThread* thread);

	//��ÿһ��ͼƬ �����ڽӱ�������ʽ�Լ�������ʽ��
	void CreateAdjointGraph(int i, depthSynThread* thread);

	//�õҽ�˹�����㷨�����ڽ���������cost��С��NearLabel
	void Dijskra(vector<Vertex*> vVertex, int nTargetLabel, vector<int> &vNearLabel);

	// ��ÿһ��ͼ��ÿһ������ֵ��SP���������ö�ӦSP����ȣ�������Ȳ�ֵ
	void SPInsertDepthPoint(vector<vector<int>> vvImgInsertLabel, vector<vector<int>> vImgSpDepth);

	//����vmapImgSPDepth
	void GetSPDepth(int i, depthSynThread* thread);

	//��ÿһ��targetSP���������ֱ��ͼ ���ж��Ƿ�ɲ�ֵ(vvImageEnabelTarget)
	void GenEnableSynTarget(int i, depthSynThread* thread);

	//�� ���Խ�����Ȳ�ֵ��SP�����в�ֵ �����޸���ȵ���ص�������Ϣ��depthmat depthPoint samplePoint labeldepth��
	void SetEnableDepth(int i, depthSynThread* thread);

	public slots :
	void slotDepthSynthesis();

	//��ʼ����local warp����ʼ����
	void slotPrepareLocalWarp();

	//����local warp
	Mat slotLacolWarp(bool bOutPut);

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//�������Ļ��ֺ���ȸ�ֵ���������Դ�����������ͼƬ����
public:
	//�Ƿ����skyͼ��
	bool bSkyGraphCut;

	//ÿһ��ͼƬ ������Ԫ��Ϊ0��ʾΪǰ��������Ԫ��Ϊ1��ʾΪ���
	vector<vector<int>> vvImgSkyLabel;

	//�������labelmat��0Ϊ��գ�1Ϊǰ��
	vector<Mat> vMatSky;

private:

	//��ȡ��������label�����ҽ���label��targetlabel��ɾ��
	void getSkyLabel();

	//��������������Ȳ�ֵ
	void setSkyDepth();

	void writeSkyImg();


	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//���� local warp


public:
	//�����ֱ�Ϊ ����ڲξ���һ�㱣�ֲ��䣩����ת���� �� ƫ�ƾ���������
	void viewLocalWarp(Mat matCamera, Mat R, Mat t);

	//ÿһ��imgInfo��ʼ����sp�������ֵ���Ƿ�dilate.  �����ֵ�Ĳ�ֵ ������fdeviation �� deviate
	void warpInitialize(float fDeviation);

	//Ŀ��ΪSP����ÿһ��SP 1.����չ������չ 2.��grid 3.���������ϵͳ
	void SpWarpLinearSystem(SuperPixel &sp, int iVertexDensity, int iExtPixel);

	//Ϊ�����ظ���grid
	void SpOverlayGrid(SuperPixel &sp, int iVertexDensity);

	//��ȡ��ʼ��֮���SP������ϵͳ
	void CalSPLinearSys(SuperPixel &sp);

	//���noval���ڽ���k�����λ��
	void GetKNovalNear(int k);

private:
	vector<depthSynThread*> vdepthSynThread;




public:
	//����õ�ͼƬ��Ϣ
	vector<ImgInfo*> vImgInfo;

	//����������ͼ
	BinGraph *myGraph;

	//warp�ļ�������д��warp����Ĺ�����Ϣ
	QFile *warpFile;

	//textstream������warp�ļ�
	QTextStream *warpText;

	//textMutex, ���ڻ���text����
	QMutex warpMutex;

	//depthsyn�ļ������������Ⱥϳɹ��̵�һЩ��Ϣ
	QFile* DepthFile;

	//����depthsyn�ļ���textstream
	QTextStream* DepthsynText;


private:



};

class depthSynThread :public QThread
{
public:
	depthSynThread(DepthSynWidget* parent, int index) :m_spParent(parent), i(index){};
	~depthSynThread(){};

	//������ˮ���õ������б���
	int nNumCamera;
	int nNumSp;
	vector<MyPoint3D*> vPoints;

	vector<int> vLabelPixelCount;
	map<int, vector<int>> mapTargetNearLabel;
	vector<int> vEnableTarget;
	map<int, vector<float>> mapSpDepthHis;
	map<float, int> mapDepthPercentile;
	Mat DepthMat;
	vector<DepthPoint> vDepthPoint;
	vector<SamplePoint*> vSamplePoint;
	vector<int> vTargetSPLabel;
	vector<int> vNoneOriginSPLabel;
	Mat TargetMaskMat;
	Mat BlendMat;
	vector<vector<float>> vvLabelHis;
	vector<vector<float>>	Graph;
	vector<Vertex*>	vVertexAdjoint;
	map<int, vector<DepthPoint>>	mapSPDepth;
	vector<int>	vLabelPointCount;
	Mat src;
	Mat LabelMat;
	CameraMatrix* camera;

	vector<int> vSegIndex;

	void writeImform(const QString s, bool bPrefix = true);




protected:
	void run();
private:
	int i;
	DepthSynWidget* m_spParent;

	//������ά����ͶӰ�������Ϣ���Գ����ؽ����ڷָ�
	void ReSegSp();
};

class warpThread :public QThread
{
public:
	warpThread(DepthSynWidget *parent, int index, ImgInfo* _img, vector<vector<array<Point2i, 4>>> &_pixelCorrespond, Mat _matWeight, Mat _labelMat, CameraMatrix _noval) :m_spParent(parent), i(index), img(_img), pixelCorrespond(_pixelCorrespond), matWeight(_matWeight), LabelMat(_labelMat), novalCamera(_noval){};
	~warpThread(){};
protected:
	void run();
	void writeImform(const QString s);

private:
	DepthSynWidget* m_spParent;
	int i;
	ImgInfo *img;

	vector<vector<array<Point2i, 4>>> pixelCorrespond;
	Mat matWeight;
	Mat LabelMat;

	//
	CameraMatrix novalCamera;
};



#endif

