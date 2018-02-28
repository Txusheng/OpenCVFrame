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
	//������������sfm��Ӧ���ڴ��ͷ�
	~sfm();

	//��������Ķ�ά�������ά�ع�
	bool Reconstruction();

	//���ؽ�����ά��Ϣ �����ply �Լ� out ��
	void Output(bool bOut, bool bPly, QString dirPath,Mat src,Mat ref);

	vector<Point2f> vPoints1;
	vector<Point2f> vPoints2;

	vector<Point2f> vCorPoint1;
	vector<Point2f> vCorPoint2;

	vector<Point3f> v3DPoints;

	//����double�ͣ�����
	Mat Cam1;
	Mat Cam2;
	Mat K;
	Mat R;
	Mat T;
	float focal;

	//char��
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

	//һ��ͼƬNRDC�����ת��
	void slotNRDC2Match();




public:
	sfm *m_spSfm;

};











#endif