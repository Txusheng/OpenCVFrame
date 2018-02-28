#ifndef OPENCVFRAME_H
#define OPENCVFRAME_H

#include <iostream>
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
#include "SLIC.h"
#include <QFile>
#include <QTextStream>
#include "DepthSynthesis.h"
#include <sstream>
#include <string>
#include <QDir>
#include <QThread>
#include "mat.h"
#include "engine.h"
#include "FileTransfer.h"




using namespace std;
using namespace cv;


extern bool ebDebug;

class OpenCVFrame : public QMainWindow
{
	Q_OBJECT

//public member functions
public:
	friend class openImageThread;
	OpenCVFrame(QWidget *parent = 0);
	~OpenCVFrame();
	vector<Mat> AquireCurImages();
	int AquireCurrentIndex();
	void OutPutInf(QString sFileName,QString sHeader, Mat mat, bool bRefresh);//Mat版本的输出，暂时先用比较傻瓜的方式




public slots:
	void OpenImageFile();
	void OpenDepthSynthesisFiles();
	void ChangeViewState();// 改变视图状态
	void ViewRefresh(int i = -1);// 刷新视图
	void ChangeImgState();
	void ShowImage();
	void ShowImage(vector<Mat> vMat);


//public member variant
public:
	vector<Mat> vMat;// vec<uchar,3> ;RGB图片
	SLICwidget *m_spSLIC;
	DepthSynWidget *m_spDepthWidgt;
	vector<openImageThread*> vThreads;
	FT *m_spFT;
	


//private member functions
private:
	Ui::OpenCVFrameClass ui;
	void InitializeAll();
	void ConstructLayout();
	void InitializeActions();
	void InitializeConnections();
	void InitializeMenuBar();
	void InitializeToolBar();
	void CreatePicSubWindow();
	




//private member variants
private:
	QMenuBar *m_spMenu;
	QToolBar *m_spToolbar;
	QWidget *m_spMainWdget;

	enum ViewState
	{
		subwindows,
		Tabs
	}eViewState;

	enum CurImageState
	{
		allImages,
		curImage
	}eCurImageState;

	QStackedWidget *m_spMdiWidget;
	QMdiArea *m_spCentralArea;
	QHBoxLayout *m_spHLayout;
	QStackedWidget *m_spUserWidget;

	QTabWidget *m_spTabWdget;
	QWidget *m_spCentralWidget;
	

	vector<QMdiSubWindow*> vSubWindows; 
	vector<QLabel*> vPicLabels;
	vector<QScrollArea*> vScrollAreas;
	QStringList sPicNames;

	QAction *m_spActionOpenFile;
	QAction *m_spActSubWin;
	QAction *m_spActTabs;
	QAction *m_spActAllImg;
	QAction *m_spActSnglImg;
	QAction *m_SpActOpenDepthSynthesisFIles;
	QAction *m_spActDepthSynthesis;
	QAction *m_spActLocalWarp;

	//basic operation
	QAction *m_spActSLIC;

	QAction *m_spActNRDC2Match;
	
};

class openImageThread:public QThread
{
	Q_OBJECT
public:
	openImageThread(OpenCVFrame *frame,int i,Mat srcRGB,string path);
	~openImageThread();
	OpenCVFrame *m_frame;
	int nIndex;
	Mat src;
	string outFilePath;
	
protected:
	void run(); 
private:


};

#endif // OPENCVFRAME_H
