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


//计算两个向量之间的夹角
template<typename T>
float GetAngleCos(const vector<T> &a, const vector<T> &b);

//计算向量的距离
template<typename T>
float GetNorm(const vector<T> &a);


//空间三维点，除了记录三维信息之外，还记录三维点 被那一张图的哪一个超像素 捕捉
class MyPoint3D :public Point3f
{
public:
	MyPoint3D();
	~MyPoint3D();

	MyPoint3D(float x, float y, float z);

	//被 哪一张图片 的哪一个超像素 捕捉,[0] 表示第一张图中对应的sp,初始化为-1，若是可视的即为-2
	int *ConnectImgSp;
	int nSize;

	//删除三维点和超像素之间的 对应关系
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


//图，用于记录不同原照片 超像素之间的链接关系
class BinGraph
{
public:
	BinGraph(const vector<ImgInfo*> &vImg);
	~BinGraph();

	//二元数组图结构
	vector<vector<BinVertex*>> graph;


	//根据三维点中存放的 被超像素观测信息，构建图（构建之后释放三维点 内存对应）
	void Initialize(vector<MyPoint3D*> vPoints);

	//查找边是否存在,若存在，则true 第二个变量为null，不存在则为false 返回可以插入该节点的位置
	pair<bool, HashNode**> FindEdge(int i1, int j1, int i2, int j2);

	//链接两个节点
	void AddEdge(int i1, int j1, int i2, int j2);

private:


};


class BinVertex
{
public:
	BinVertex(int i, int j, int nHashSize);
	~BinVertex();

	//照片 或者 相机索引
	int i;
	//超像素索引
	int j;

	// 	const SuperPixel *sp;

	// 	BinVertex *next;

	//最终还是决定采取哈希表的方式存储链接（索引更快，毕竟超像素 按照所属照片分类）
	vector<HashNode*> hash;

private:

};


//相机矩阵
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




//bundler + pmvs 输出的样本点全部信息
class SamplePoint
{
public:
	SamplePoint();
	~SamplePoint();

	//空间三维点
	MyPoint3D *Point3D;

	//投影在图片中的点
	Point2i Point2D;

	//原始投影点（离散化之前）
	Point2f Point2Df;

	//投影之后的深度
	float fDepth;

	//是否是pmvs重建出来的点
	bool bOrigin;

	//所在超像素编号
	int nSpIndex;


	//如果是插值得到的点，根据相机矩阵得到三维点
	void BackProjection(Mat src, const CameraMatrix *camera);

	//根据相机矩阵，从三维点投影到二维点
	bool Projection(Mat src, const CameraMatrix *camera);

protected:


private:

};


//线性系统信息
class LinearSystem
{
public:
	LinearSystem();
	~LinearSystem();

	SuperPixel* m_spParentSP;

	vector<vector<double>> A;
	vector<double> B;

	//在未得到相机矩阵之前，B存储 线性系统数据，以便计算时直接导入
	vector<list<pair<Point3f*, float>>> BInfo;


private:

};



//sp里面，三角形的顶点,包括位置、所属三角形、编号
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



//二维图上的深度点
typedef struct DepthPoint
{
	Point point;
	float fDepth;
};


//图结构，每个节点的邻接链表
typedef struct Line
{
	int nEnd;
	float nWeight;
	Line* next;
};

//图结构的节点
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


//超像素中的每一个三角形结构
class GridTriangle
{
public:
	GridTriangle(SuperPixel *parent);
	~GridTriangle();

	//所隶属的超像素
	SuperPixel *m_spParentSP;

	//三角网格中的样本点
	vector<SamplePoint*> vpSamplePoints;

	//三角网格对应的三个顶点
	vector<TriVertex*> vVertex;

	//

private:

};


//超像素的所有信息
class SuperPixel
{

public:
	SuperPixel();
	~SuperPixel();

	//计算自己的线性系统
	void getLinearSys();
	void solveLinearSys(CameraMatrix *_camera);
	float getNovalDepth(const CameraMatrix *camera);


	ImgInfo* m_spParentImg;

	//超像素里的sample深度信息，升序排列
	vector<float> vDepth;

	//深度中值
	float fMedianDepth;


	//超像素中的sample
	vector<SamplePoint*> vpSamples;

	list<Point2i> listPoint;

	//二值，仅记录形状
	Mat matSP;

	Rect boundingRect;

	bool bExtension;

	bool bHole;

	int iImgIndex;

	bool bOrigin;

	///////////////////////////////////////////////////////////////////////////////////////////////////////////
	//扩展之后的超像素


	//二值，仅记录扩展后的形状
	Mat matExtSP;

	Rect ExtBounding;

	list<Point2i> listExtPoint;

	vector<TriVertex*> vVertexpoint;

	vector<vector<int>> vTriangleIndex;

	//超像素里面的三角形，存放三角形的基本信息（顶点位置等存放在vVertexPoint，因为顶点是公用的）
	vector<GridTriangle> vTriangle;

	int nVertexNum;

	int nXNum;
	int nYNum;

	float fXStep;
	float fYStep;

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//warp之后的超像素信息

	//warp之后的三角网格点坐标
	vector<TriVertex*> vWarpVertex;

	//warp之后的三角形信息
	vector<vector<int>> vWarpTri;

	//线性系统结果
	LinearSystem m_LinSys;

	//新视角对应的深度
	bool bNovalDepth;
	float fNovalDepth;

protected:

private:

};

//每一张图片的所有信息(大部分为直接引用，并非克隆)
class ImgInfo
{

public:
	ImgInfo();
	~ImgInfo();
	ImgInfo(DepthSynWidget* parent);

	//根据相机位置，生成新视角的图片
	Mat dst;
	//新视角 对应的深度
	Mat dstDepth;

	//生成新视角图片，目标为dst。 在此之前必须完成超像素的线性系统求解
	void GenerateView();


	DepthSynWidget *m_spParentWidget;

	//编号
	int Index;

	//原始图
	Mat src;


	//每一张图片对应一组SP
	vector<SuperPixel> vSP;

	//深度图,3通道float，第一个为label，第二个为depth(存在投影点的位置)
	Mat depthMat;

	//label图，拷贝
	Mat labelMat;

	//bool 判断这张图片是不是jb用都没有
	bool bValid;

	//相机位置
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

	//当前需要进行local warp的相机位置,初始值为第一个相机位置
	CameraMatrix *camera;

	//四个选中的img pack
	vector<ImgInfo*> vSelectedImg;


	// 怎么说本还是要保留一个备份吧
	vector<Mat> vSrc;
	QMutex mutexSrc;


	//label Mat的备份,存储的为int型
	vector<Mat> vLabelMat;
	QMutex mutexLabelMat;


	OpenCVFrame *m_spOpenCVFrame;

	vector<MyPoint3D*> vPoints;

	vector<CameraMatrix*> vCamera;

	//深度点包含有 point2D 以及 对应的深度
	vector<vector<DepthPoint>> vvDepthPoint;
	QMutex mutexDepthPoint;

	vector<vector<SamplePoint*>> vvSamplePoint;
	QMutex mutexSamplePoint;

	//每一张mat都是vec<float,3>， 元素第一个值存label，第二个值存depth
	vector<Mat> vDepthMat;
	QMutex mutexDepthMat;

	//每一张图片，每一个label对应深度点的个数
	vector<vector<int>> vvLabelPointCount;
	QMutex mutexLabelPointCount;

	//每一张图，每一个label对应深度数组
	vector<map<int, vector<DepthPoint>>> vmapImgSPDepth;
	QMutex mutexSpDepth;

	//每一张图片的 目标SP
	vector<vector<int>> vvTargetSPLabel;
	QMutex mutexTargetSPLabel;

	//每一张图片的原始 目标sp（判断超像素深度点是否为插值所得）
	vector<vector<int>> vvNoneOriginSPLabel;
	QMutex mutexNoneOriginLabel;

	//每一张图片 每一个label 对应一个数组直方图
	vector<vector<vector<float>>> vvvImgLabelHis;
	QMutex mutexImgLabelHis;

	//每一张图，每一个label对应的几个最邻近label
	vector<map<int, vector<int>>> vmapImgTargetNearLabel;
	QMutex mutexTargetNear;

	//每一张图，每一个SP对应一个邻接链表
	vector<vector<Vertex*>> vvImgVertexAdjoint;
	QMutex mutexVertexAdjoint;

	//每一张图，求一个邻接表
	vector<vector<vector<float>>> vGraph;
	QMutex mutexGraph;

	vector<Mat> vTargetMaskMat;
	QMutex mutexTargetMask;

	vector<Mat> vBlendMat;
	QMutex mutexBlendMat;

	//判断list.txt等三位重建信息是否读入，是否可以进行后续深度合成
	bool bList;


	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//以下内容属于debug使用

	//是否将sky图割结果输出到文件夹
	bool bDebugSky;

	//进行深度插值，第一次迭代 可执行深度插值的 目标sp
	vector<vector<int>> vvImgEnableTarget;
	QMutex mutexEnableTarget;

	//可插值label及其对应depthHist
	vector<map<int, vector<float>>> vmapImgSpDepthHis;
	QMutex mutexSpDepthHis;

	//每一张图片的 深度percentile分布
	vector<map<float, int>> vmapDepthPercentile;
	QMutex mutexDepthPercentile;




public:
	//提供给外界用的函数

	//根据输入RGB图、超像素标记图、超像素 像素数目数组，计算并返回对应的直方图数组
	vector<vector<float>> CalLabHist(const Mat &srcRGB, const Mat &matLabel, int nNumSP, const vector<int> &vSpPixelNum, int nBins = 20);

	// 计算两个直方图的卡方距离，输入之前要保证已经归一化
	float CalChiSquareDis(vector<float> vA, vector<float> vB);





private:
	void PointsProjection(int i, depthSynThread* thread);
	void TargetSPBlend(int i, depthSynThread* thread);


	//计算每一张图，每一个label对应的lab 归一化直方图
	void CalLabHist(int i, depthSynThread* thread);

	//找到每一个Label的最邻近Label
	void FindNearLabel(int i, depthSynThread* thread);

	//对每一张图片 创建邻接表（链表形式以及数组形式）
	void CreateAdjointGraph(int i, depthSynThread* thread);

	//用狄杰斯克拉算法求最邻近的三个（cost最小）NearLabel
	void Dijskra(vector<Vertex*> vVertex, int nTargetLabel, vector<int> &vNearLabel);

	// 对每一张图，每一个待插值的SP，根据所得对应SP的深度，进行深度插值
	void SPInsertDepthPoint(vector<vector<int>> vvImgInsertLabel, vector<vector<int>> vImgSpDepth);

	//构建vmapImgSPDepth
	void GetSPDepth(int i, depthSynThread* thread);

	//对每一个targetSP，构建深度直方图 并判断是否可插值(vvImageEnabelTarget)
	void GenEnableSynTarget(int i, depthSynThread* thread);

	//对 可以进行深度插值的SP，进行插值 并且修改深度点相关的所有信息（depthmat depthPoint samplePoint labeldepth）
	void SetEnableDepth(int i, depthSynThread* thread);

	public slots :
	void slotDepthSynthesis();

	//开始进行local warp（初始化）
	void slotPrepareLocalWarp();

	//进行local warp
	Mat slotLacolWarp(bool bOutPut);

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//天空区域的划分和深度赋值，对于明显存在天空区域的图片可用
public:
	//是否进行sky图割
	bool bSkyGraphCut;

	//每一张图片 ，数组元素为0表示为前景，数组元素为1表示为天空
	vector<vector<int>> vvImgSkyLabel;

	//天空区域，labelmat，0为天空，1为前景
	vector<Mat> vMatSky;

private:

	//获取天空区域的label，并且将该label从targetlabel中删除
	void getSkyLabel();

	//对天空区域进行深度插值
	void setSkyDepth();

	void writeSkyImg();


	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//进行 local warp


public:
	//参数分别为 相机内参矩阵（一般保持不变），旋转矩阵 和 偏移矩阵（向量）
	void viewLocalWarp(Mat matCamera, Mat R, Mat t);

	//每一张imgInfo初始化，sp的深度中值、是否dilate.  深度中值的差值 若超过fdeviation 则 deviate
	void warpInitialize(float fDeviation);

	//目标为SP，对每一个SP 1.该扩展的做扩展 2.做grid 3.计算出线性系统
	void SpWarpLinearSystem(SuperPixel &sp, int iVertexDensity, int iExtPixel);

	//为超像素覆盖grid
	void SpOverlayGrid(SuperPixel &sp, int iVertexDensity);

	//获取初始化之后的SP的线性系统
	void CalSPLinearSys(SuperPixel &sp);

	//获得noval最邻近的k个相机位置
	void GetKNovalNear(int k);

private:
	vector<depthSynThread*> vdepthSynThread;




public:
	//打包好的图片信息
	vector<ImgInfo*> vImgInfo;

	//超像素连接图
	BinGraph *myGraph;

	//warp文件，用于写入warp运算的过程信息
	QFile *warpFile;

	//textstream，关联warp文件
	QTextStream *warpText;

	//textMutex, 用于互斥text变量
	QMutex warpMutex;

	//depthsyn文件，用于输出深度合成过程的一些信息
	QFile* DepthFile;

	//关联depthsyn文件的textstream
	QTextStream* DepthsynText;


private:



};

class depthSynThread :public QThread
{
public:
	depthSynThread(DepthSynWidget* parent, int index) :m_spParent(parent), i(index){};
	~depthSynThread(){};

	//整个流水线用到的所有变量
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

	//利用三维点云投影的深度信息，对超像素进行在分割
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

