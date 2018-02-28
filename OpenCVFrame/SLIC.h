#ifndef SLIC_H
#define SLIC_H

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

#include <math.h>
#include <QSpinBox>
#include <QSlider>
#include <QGroupBox>
#include <QRadioButton>
using namespace std;
using namespace cv;


typedef struct xylab
{
	float x;
	float y;
	float L;
	float a;
	float b;
};

//src represents the source image ,dst the destiny,m the number of superpixels along x or y,m for range of lab
void SLIC(Mat src, Mat &mask, int &k, vector<int> &vLabelPixCount, int m);

float distancexylab(xylab a, xylab b,float spatialrange,float labrange);

void SLICColorBlend(Mat src,Mat &dst,Mat mask,int k);

void SLICEnforceConnectivity(Mat &mask,int &k,vector<int> &vLabelPixCount);

//�µĳ������ںϷ���
void SLICmerge(const Mat &src,Mat &mask, int &k, vector<int> &vLabelPixCount);




//SLIC widget in Frame

class SLICwidget:public QWidget
{
	Q_OBJECT

public:
	QMainWindow *m_spMainWindow;
	QSlider *m_spSlider;
	QSpinBox *m_spSpinBox;
	QGroupBox *m_spBtnGroup;
	QLabel *m_spKlabel;
	QLabel *m_spMlabel;
	QRadioButton *m_spRadioSrc;
	QRadioButton *m_spRadioDst;
	
	
	QGridLayout *m_spLayout;

	vector<Mat> vLabelMat;//ʵ��������Ǵ���label��mat��label����Ϊint
	vector<Mat> vDst;

	vector<int> vNumSP;
	vector<vector<int>> vvImageLabelPixelCount;//ÿһ��ͼƬ��ÿһ��label��Ӧ��������Ŀ

	enum status
	{
		source,
		result
	}eStatus;

public slots:
	void ChangeMLabel(int m);


public:
	SLICwidget(QMainWindow *parent = 0);
	~SLICwidget();

private:


};









#endif