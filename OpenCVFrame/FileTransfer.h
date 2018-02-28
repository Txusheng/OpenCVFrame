#ifndef FILETRANSFER_H
#define FILETRANSFER_H

#include <qwidget.h>
#include <vector>
#include <map>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <QFileDialog>

using namespace std;
using namespace cv;

class OpenCVFrame;

class sfm
{
public:
	sfm(){};
	//解析函数，将sfm对应的内存释放
	~sfm();

	//根据输入的二维点进行三维重构
	bool Reconstruction();

	//将重建的三维信息 输出到ply 以及 out 中
	void Output(bool bOut, bool bPly, QString dirPath,Mat src,Mat ref);

	vector<Point2f> vPoints1;
	vector<Point2f> vPoints2;

	vector<Point2f> vCorPoint1;
	vector<Point2f> vCorPoint2;

	vector<Point3f> v3DPoints;

	//都是double型，竖向
	Mat Cam1;
	Mat Cam2;
	Mat K;
	Mat R;
	Mat T;
	float focal;

	//char型
	Mat mask;
	Mat matStructure;


protected:
private:
};


class FT:public QWidget
{
	Q_OBJECT
public:
	FT(OpenCVFrame* parent);
	~FT();
protected:
private:
	map<int, int> NRDC2Match(string sSrc, string sRef, string sMatFile);


	public slots:

	//一对图片NRDC结果的转换
	void slotNRDC2Match();




public:
	sfm *m_spSfm;

};











#endif