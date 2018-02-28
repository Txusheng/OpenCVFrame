#include "opencvframe.h"

bool ebDebug = false;


OpenCVFrame::OpenCVFrame(QWidget *parent)
: QMainWindow(parent), eCurImageState(curImage)
{
	ui.setupUi(this);
	InitializeAll();
}

OpenCVFrame::~OpenCVFrame()
{
	Point a;
}

void OpenCVFrame::InitializeActions()
{
	MATFile *a;
	a = matOpen("D:\\matlab0\\nrdc.mat","r");


	m_spActionOpenFile = new QAction(tr("Open Image Files "),this);
	m_spActionOpenFile->setShortcut(QKeySequence::Open);
	m_spActionOpenFile->setStatusTip(tr("open a image file"));
	m_spActionOpenFile->setIcon(QIcon(":/Resources/openFIle.png"));

	m_spActSubWin = new QAction(tr("SubWindows"),this);
	m_spActSubWin->setStatusTip(tr("change the view state as subwindows"));

	m_spActTabs = new QAction(tr("Tabs"),this);
	m_spActTabs->setStatusTip(tr("change the view state as Tabs"));

	m_spActAllImg = new QAction(tr("All Images"),this);
	m_spActAllImg->setStatusTip(tr("Deal with All The images in the main window"));

	m_spActSnglImg = new QAction(tr("Single Image"),this);
	m_spActSnglImg->setStatusTip(tr("Deal with current image"));

	m_SpActOpenDepthSynthesisFIles = new QAction(tr("Open list.txt, *.ply and bundle.out"),this);
	m_spActDepthSynthesis = new QAction(tr("start depth synthesis"), this);

	m_spActLocalWarp = new QAction(tr("start local warp"), this);

	m_spActNRDC2Match = new QAction(tr("transform nrdc results to matches.init.txt"), this);
}

void OpenCVFrame::InitializeAll()
{
	//set stylesheet from config file
	setWindowTitle("IBR");
	QString sStyleSheetPath = QDir::currentPath() + "/config/StyleSheet1.ini";
	QFile file(QDir::currentPath() + "/config/StyleSheet.ini");
	if (file.open(QFile::ReadOnly))
	{
// 		this->setStyleSheet(file.readAll());
	}
	file.close();

	//construct the layout of the main window
	

	ConstructLayout();
	InitializeActions();
	InitializeConnections();
	InitializeMenuBar();
	InitializeToolBar();



	
}

void OpenCVFrame::InitializeConnections()
{
	connect(m_spActionOpenFile,SIGNAL(triggered()),this,SLOT(OpenImageFile()));
	connect(m_spActSubWin,SIGNAL(triggered()),this,SLOT(ChangeViewState()));
	connect(m_spActTabs,SIGNAL(triggered()),this,SLOT(ChangeViewState()));
	connect(m_spActAllImg,SIGNAL(triggered()),this,SLOT(ChangeImgState()));
	connect(m_spActSnglImg,SIGNAL(triggered()),this,SLOT(ChangeImgState()));


	connect(m_SpActOpenDepthSynthesisFIles, SIGNAL(triggered()), this, SLOT(OpenDepthSynthesisFiles()));
	connect(m_spActDepthSynthesis, SIGNAL(triggered()), this->m_spDepthWidgt, SLOT(slotDepthSynthesis()));
	connect(m_spActLocalWarp, SIGNAL(triggered()), this->m_spDepthWidgt, SLOT(slotPrepareLocalWarp()));

	connect(m_spSLIC->m_spRadioSrc, SIGNAL(clicked()), this, SLOT(ShowImage()));
	connect(m_spSLIC->m_spRadioDst, SIGNAL(clicked()), this, SLOT(ShowImage()));

	connect(m_spActNRDC2Match, SIGNAL(triggered()), m_spFT, SLOT(slotNRDC2Match()));
	
}

void OpenCVFrame::InitializeMenuBar()
{
	m_spMenu = menuBar();
	QMenu *openMenu = m_spMenu->addMenu(tr("&File"));
	openMenu->addAction(m_spActionOpenFile);

	QMenu *viewstateMenu = m_spMenu->addMenu(tr("&ViewState"));
	viewstateMenu->addAction(m_spActSubWin);
	viewstateMenu->addAction(m_spActTabs);

	QMenu *imgStateMenu = m_spMenu->addMenu(tr("ImageState"));
	imgStateMenu->addAction(m_spActSnglImg);
	imgStateMenu->addAction(m_spActAllImg);

	QMenu *DepthSynthesisMenu = m_spMenu->addMenu(tr("DepthSynthesis"));
	DepthSynthesisMenu->addAction(m_SpActOpenDepthSynthesisFIles);
	DepthSynthesisMenu->addAction(m_spActDepthSynthesis);
	DepthSynthesisMenu->addAction(m_spActLocalWarp);

	QMenu *FTMenu = m_spMenu->addMenu(tr("FileTransform"));
	FTMenu->addAction(m_spActNRDC2Match);
	
}

void OpenCVFrame::OpenImageFile()
{
	QStringList sFilePath = QFileDialog::getOpenFileNames(this,tr("Open image files"),".",tr("Image Files(*.jpg *.png)"));
	if (sFilePath.size() == 0)
	{
		return;
	}
	else
	{
		int index = 0;
		QStringList::iterator it;

		int n = sFilePath.size();
		m_spSLIC->vNumSP.resize(n);
		this->m_spSLIC->vLabelMat.resize(n);
		this->m_spSLIC->vDst.resize(n);
		this->m_spSLIC->vvImageLabelPixelCount.resize(n);


		for (it = sFilePath.begin(); it != sFilePath.end(); it++)
		{
			Mat_<Vec<uchar, 3>> src = imread((*it).toStdString());
			Mat srcRGB;
			cvtColor(src, srcRGB, CV_BGR2RGB);
			vMat.push_back(srcRGB);

			openImageThread *myThread = new openImageThread(this, index, srcRGB, (QDir::currentPath() + "/doc/" + it->right(it->size() - it->lastIndexOf("/") - 1)).toStdString());
			vThreads.push_back(myThread);
			myThread->start();







			// 						//每次载入图片，将图片输入到SLIC当中，并进行处理
			// 						Mat dst = srcRGB.clone();
			// 						Mat mask = srcRGB.clone();
			// 
			// 						int k = m_spSLIC->m_spSpinBox->value();
			// 						vector<int> vLabelPixCount;
			// 						SLIC(srcRGB, mask, k,vLabelPixCount, m_spSLIC->m_spSlider->value());
			// 						m_spSLIC->vNumSP.push_back(k);
			// // 						m_spSLIC->m_spSpinBox->setValue(k);
			// 						SLICColorBlend(srcRGB, dst, mask, k);
			// 
			// 						this->m_spSLIC->vLabelMat.push_back(mask);
			// 						this->m_spSLIC->vDst.push_back(dst);
			// 						this->m_spSLIC->vvImageLabelPixelCount.push_back(vLabelPixCount);  
			// 
			// 						//
			// 
			QScrollArea* scrollArea = new QScrollArea();
			QLabel* picLabel = new QLabel();
			scrollArea->setWidget(picLabel);

			QImage image((const unsigned char*)(srcRGB.data),
				srcRGB.cols, srcRGB.rows,
				srcRGB.cols*srcRGB.channels(),
				QImage::Format_RGB888);
			picLabel->setPixmap(QPixmap::fromImage(image));
			picLabel->resize(picLabel->pixmap()->size());
			vPicLabels.push_back(picLabel);
			vScrollAreas.push_back(scrollArea);





			string sPicName = (*it).toStdString();
			string::size_type idx = sPicName.find_last_of("/");
			sPicName = sPicName.erase(0, idx + 1);
			sPicNames.push_back(QString(sPicName.c_str()));


			//create a subWindow for each image selected

			++index;
		}

		for (int i = 0; i < vScrollAreas.size(); i++)
		{
			if (eViewState == subwindows)
			{
				QMdiSubWindow* imageWindow = new QMdiSubWindow();
				imageWindow->setWindowTitle(sPicNames.at(i));
				// 							imageWindow->setWindowTitle(QString::number(index, 10));
				imageWindow->setAttribute(Qt::WA_DeleteOnClose);
				imageWindow->setWidget(vScrollAreas[i]);
				m_spCentralArea->addSubWindow(imageWindow);
				vSubWindows.push_back(imageWindow);

				vScrollAreas[i]->show();
				vPicLabels[i]->show();
				imageWindow->show();

			}
			//create a tab widget for each image
			else if (eViewState == Tabs)
			{
				m_spTabWdget->addTab(vScrollAreas[i], sPicNames.at(i));
			}

		}

		m_spCentralArea->tileSubWindows();

		for (int i = 0; i < vThreads.size(); i++)
		{
			vThreads[i]->wait();
		}





// 		int i = 0;
// 		
// 		QStringList::iterator it;
// 		for (it = sFilePath.begin();it != sFilePath.end() ;it++)
// 		{
// 			Mat_<Vec<uchar,3>> src = imread((*it).toStdString());
// 			Mat srcRGB;
// 			cvtColor(src,srcRGB,CV_BGR2RGB);
// 			vMat.push_back(srcRGB);
// 
// 			//每次载入图片，将图片输入到SLIC当中，并进行处理
// 			Mat dst = srcRGB.clone();
// 			Mat mask = srcRGB.clone();
// 
// 			int k = m_spSLIC->m_spSpinBox->value();
// 			vector<int> vLabelPixCount(1,0);
// 			SLIC(srcRGB, mask, k, vLabelPixCount, m_spSLIC->m_spSlider->value());
// 			m_spSLIC->vNumSP.push_back(k);
// // 			m_spSLIC->m_spSpinBox->setValue(k);
// 			SLICColorBlend(srcRGB, dst, mask, k);
// 			Mat dstBGR;
// 			cvtColor(dst, dstBGR, CV_RGB2BGR);
// 			imwrite((QDir::currentPath() + "/doc/slic" + it->right(it->size() - it->lastIndexOf("/") - 1)).toStdString(), dstBGR);
// 
// 			this->m_spSLIC->vLabelMat.push_back(mask);
// 			this->m_spSLIC->vDst.push_back(dst);
// 			this->m_spSLIC->vvImageLabelPixelCount.push_back(vLabelPixCount);
// 
// 			//
// 
// 			QScrollArea* scrollArea = new QScrollArea();
// 			QLabel* picLabel = new QLabel();
// 			scrollArea->setWidget(picLabel);
// 
// 			QImage image((const unsigned char*)(srcRGB.data),
// 				srcRGB.cols,srcRGB.rows,
// 				srcRGB.cols*srcRGB.channels(),
// 				QImage::Format_RGB888);
// 			picLabel->setPixmap(QPixmap::fromImage(image));
// 			picLabel->resize(picLabel->pixmap()->size());
// 			vPicLabels.push_back(picLabel);
// 			vScrollAreas.push_back(scrollArea);
// 
// 			string sPicName = (*it).toStdString();
// 			string::size_type idx = sPicName.find_last_of("/");
// 			sPicName = sPicName.erase(0,idx + 1);
// 			sPicNames.push_back(QString(sPicName.c_str()));
// 
// 
// 			//create a subWindow for each image selected
// 			if (eViewState == subwindows)
// 			{
// 				QMdiSubWindow* imageWindow = new QMdiSubWindow();
// 				imageWindow->setWindowTitle(QString(sPicName.c_str()));
// 				imageWindow->setAttribute(Qt::WA_DeleteOnClose);
// 				imageWindow->setWidget(scrollArea);
// 				m_spCentralArea->addSubWindow(imageWindow);
// 				vSubWindows.push_back(imageWindow);
// 
// 				scrollArea->show();
// 				picLabel->show(); 
// 				imageWindow->show();
// 
// 			}
// 			//create a tab widget for each image
// 			else if (eViewState == Tabs)
// 			{
// 				m_spTabWdget->addTab(scrollArea,QString(sPicName.c_str()));
// 			}
// 		}
// 
// 		m_spCentralArea->tileSubWindows();
	}
}


void OpenCVFrame::OpenDepthSynthesisFiles()
{
// 	
// 	QStringList sFIlePaths = QFileDialog::getOpenFileNames(this, tr("Open image files"), "D:/Bundler4/result/pmvs", tr("Out files and Text files (*.out *.dat *.txt);;Patch files (*.patch)"));

// 	必须保证ske文件在最前面
 	QStringList sFIlePaths;
 	sFIlePaths << "D:/Bundler4/resulttree/pmvs/ske.dat"
 		<< "D:/Bundler4/resulttree/pmvs/bundle.out"//原图党
 		<<"D:/Bundler4/resulttree/pmvs/list.rd.txt"
 		<< "D:/Bundler4/resulttree/pmvs/models/option-0000.patch";


	if (sFIlePaths.size() == 0)
	{
		return;
	}
	else
	{
		//
		vector<int> vClustList;

		QStringList::iterator it;
		for (it = sFIlePaths.begin(); it != sFIlePaths.end();it++)
		{
			QFile file(*it);
			QString sDirectory = it->left(it->lastIndexOf("/") + 1);
			
			if (file.open(QIODevice::ReadOnly))
			{
				QStringList ssLines;
				QTextStream textStream(&file);
				while (!textStream.atEnd())
				{
					ssLines += textStream.readLine();
				}

				if (it->endsWith("ske.dat"))
				{
					QStringList clustList = ssLines.at(3).split(" ",QString::SkipEmptyParts);
					for (auto it = clustList.begin(); it != clustList.end();it++)
					{
						vClustList.push_back(it->toInt());
					}

					int n = vClustList.size();
					//容器大小设置好，为多线程做准备
					m_spSLIC->vNumSP.resize(n);
					this->m_spSLIC->vLabelMat.resize(n);
					this->m_spSLIC->vDst.resize(n);
					this->m_spSLIC->vvImageLabelPixelCount.resize(n);
					
					
				}


				if (it->endsWith("list.rd.txt"))
				{
					this->m_spDepthWidgt->bList = true;
					//list.txt 中所罗列的图片并打开
					QStringList sIamgeFilePath;
					QStringList::iterator itLine;

					for (int i = 0; i < vClustList.size();i++)
					{
						QString s = ssLines.at(vClustList[i]);
						QString sImageName = s.right(s.size() - s.lastIndexOf("/") - 1);
// 						sImageName.insert(sImageName.indexOf(".") + 1, "rd.");
						sIamgeFilePath.push_back(sDirectory + sImageName); 
					}

					//打开图片
					int index = 0;
					QStringList::iterator it;
					for (it = sIamgeFilePath.begin(); it != sIamgeFilePath.end(); it++)
					{
// 						if (index != 3)
// 						{
// 							++index;
// 							continue;
// 						}

						Mat_<Vec<uchar, 3>> src = imread((*it).toStdString());
						Mat srcRGB;
						cvtColor(src, srcRGB, CV_BGR2RGB);
						vMat.push_back(srcRGB);



						openImageThread *myThread = new openImageThread(this, index, srcRGB, (QDir::currentPath() + "/doc/slic" + it->right(it->size() - it->lastIndexOf("/") - 1)).toStdString());
						vThreads.push_back(myThread);
						myThread->start();







// 						//每次载入图片，将图片输入到SLIC当中，并进行处理
// 						Mat dst = srcRGB.clone();
// 						Mat mask = srcRGB.clone();
// 
// 						int k = m_spSLIC->m_spSpinBox->value();
// 						vector<int> vLabelPixCount;
// 						SLIC(srcRGB, mask, k,vLabelPixCount, m_spSLIC->m_spSlider->value());
// 						m_spSLIC->vNumSP.push_back(k);
// // 						m_spSLIC->m_spSpinBox->setValue(k);
// 						SLICColorBlend(srcRGB, dst, mask, k);
// 
// 						this->m_spSLIC->vLabelMat.push_back(mask);
// 						this->m_spSLIC->vDst.push_back(dst);
// 						this->m_spSLIC->vvImageLabelPixelCount.push_back(vLabelPixCount);  
// 
// 						//
// 
						QScrollArea* scrollArea = new QScrollArea();
						QLabel* picLabel = new QLabel();
						scrollArea->setWidget(picLabel);

						QImage image((const unsigned char*)(srcRGB.data),
							srcRGB.cols, srcRGB.rows,
							srcRGB.cols*srcRGB.channels(),
							QImage::Format_RGB888);
						picLabel->setPixmap(QPixmap::fromImage(image));
						picLabel->resize(picLabel->pixmap()->size());
						vPicLabels.push_back(picLabel);
						vScrollAreas.push_back(scrollArea);





						string sPicName = (*it).toStdString();
						string::size_type idx = sPicName.find_last_of("/");
						sPicName = sPicName.erase(0, idx + 1);
						sPicNames.push_back(QString(sPicName.c_str()));


						//create a subWindow for each image selected

						++index;
					}

					for (int i = 0; i < vScrollAreas.size();i++)
					{
						if (eViewState == subwindows)
						{
							QMdiSubWindow* imageWindow = new QMdiSubWindow();
							imageWindow->setWindowTitle(sPicNames.at(i));
							// 							imageWindow->setWindowTitle(QString::number(index, 10));
							imageWindow->setAttribute(Qt::WA_DeleteOnClose);
							imageWindow->setWidget(vScrollAreas[i]);
							m_spCentralArea->addSubWindow(imageWindow);
							vSubWindows.push_back(imageWindow);

							vScrollAreas[i]->show();
							vPicLabels[i]->show();
							imageWindow->show();

						}
						//create a tab widget for each image
						else if (eViewState == Tabs)
						{
							m_spTabWdget->addTab(vScrollAreas[i], sPicNames.at(i));
						}

					}

					m_spCentralArea->tileSubWindows();
					
				}
				else if (it->endsWith("bundle.out"))
				{
					int iNumCamera = ssLines.at(1).left(ssLines.at(1).indexOf(" ")).toInt();

					//ske簇 list的索引
// 					int index = 0;
// 					for (int m = 0; m < iNumCamera;m++)
// 					{
// 						if (m == vClustList[index] - 1)
// 						{
// 							++index;
// 							CameraMatrix *camera = new CameraMatrix;
// 							QStringList sFKK = ssLines.at(m * 5 + 0 + 2).split(" ");
// 							QStringList R1 = ssLines.at(m * 5 + 1 + 2).split(" ");
// 							QStringList R2 = ssLines.at(m * 5 + 2 + 2).split(" ");
// 							QStringList R3 = ssLines.at(m * 5 + 3 + 2).split(" ");
// 							QStringList T = ssLines.at(m * 5 + 4 + 2).split(" ");
// 
// 							camera->fFocal = sFKK.at(0).toFloat();
// 							camera->vK.push_back(sFKK.at(1).toFloat());
// 							camera->vK.push_back(sFKK.at(2).toFloat());
// 
// 
// 							camera->T = (Mat_<float>(3, 1) << T.at(0).toFloat(), T.at(1).toFloat(), T.at(2).toFloat());
// 
// 							camera->matRotation = (Mat_<float>(3, 3) << R1.at(0).toFloat(), R1.at(1).toFloat(), R1.at(2).toFloat(),
// 								R2.at(0).toFloat(), R2.at(1).toFloat(), R2.at(2).toFloat(),
// 								R3.at(0).toFloat(), R3.at(1).toFloat(), R3.at(2).toFloat());
// 
// 							camera->CalCenter();
// 
// 							this->m_spDepthWidgt->vCamera.push_back(camera);
// 
// 						}
// 
// 					}

					for (int index = 0; index < vClustList.size();index++)
					{
						int m = vClustList[index];
						CameraMatrix *camera = new CameraMatrix;
						QStringList sFKK = ssLines.at(m * 5 + 0 + 2).split(" ");
						QStringList R1 = ssLines.at(m * 5 + 1 + 2).split(" ");
						QStringList R2 = ssLines.at(m * 5 + 2 + 2).split(" ");
						QStringList R3 = ssLines.at(m * 5 + 3 + 2).split(" ");
						QStringList T = ssLines.at(m * 5 + 4 + 2).split(" ");

						camera->fFocal = sFKK.at(0).toFloat();
						camera->vK.push_back(sFKK.at(1).toFloat());
						camera->vK.push_back(sFKK.at(2).toFloat());


						camera->T = (Mat_<float>(3, 1) << T.at(0).toFloat(), T.at(1).toFloat(), T.at(2).toFloat());

						camera->matRotation = (Mat_<float>(3, 3) << R1.at(0).toFloat(), R1.at(1).toFloat(), R1.at(2).toFloat(),
							R2.at(0).toFloat(), R2.at(1).toFloat(), R2.at(2).toFloat(),
							R3.at(0).toFloat(), R3.at(1).toFloat(), R3.at(2).toFloat());

						camera->CalCenter();

						this->m_spDepthWidgt->vCamera.push_back(camera);

					}

					
					//当前相机位置初始化为 第一个相机位置,主法线进行修正
					this->m_spDepthWidgt->camera = new CameraMatrix();
					this->m_spDepthWidgt->camera->fFocal = this->m_spDepthWidgt->vCamera[0]->fFocal;
					this->m_spDepthWidgt->camera->matRotation = this->m_spDepthWidgt->vCamera[0]->matRotation.clone();
					this->m_spDepthWidgt->camera->T = this->m_spDepthWidgt->vCamera[0]->T.clone();
					this->m_spDepthWidgt->camera->vK = this->m_spDepthWidgt->vCamera[0]->vK;


					// 					for (int k = 0; k < 3; k++)
// 					{
// 						this->m_spDepthWidgt->camera->matRotation.at<float>(2, k) = -this->m_spDepthWidgt->camera->matRotation.at<float>(2, k);
// 					}
					

				}
				else if (it->endsWith("option-0000.patch"))
				{
// 					bool flag(false);
// 					for (int i = 0; i < ssLines.size();i++)
// 					{
// 						if (!flag)
// 						{
// 							if (ssLines.at(i).contains("end_header"))
// 							{
// 								flag = true;
// 							}
// 						}
// 						else
// 						{
// 							QStringList slCoordinates;
// 							slCoordinates = ssLines.at(i).split(" ", QString::SkipEmptyParts);
// 							MyPoint3D* point = new MyPoint3D(slCoordinates.at(0).toFloat(),slCoordinates.at(1).toFloat(),slCoordinates.at(2).toFloat());
// 							
// 							this->m_spDepthWidgt->vPoints.push_back(point);
// 						}
// 
// 					}

					for (int i = 0; i < ssLines.size();i++)
					{
						//不管是否纹理一致性，都录入
						if (ssLines.at(i) == "PATCHS")
						{
							QStringList slCoordinates;
							slCoordinates = ssLines.at(i + 1).split(" ", QString::SkipEmptyParts);
							MyPoint3D* point = new MyPoint3D(slCoordinates.at(0).toFloat(), slCoordinates.at(1).toFloat(), slCoordinates.at(2).toFloat());
							point->ConnectImgSp = new int[vClustList.size()];
							point->nSize = vClustList.size();
							for (int k = 0; k < vClustList.size(); k++)
							{
								point->ConnectImgSp[k] = -1;
							}

							if (ssLines.at(i + 4).toInt() != 0)
							{
								QStringList sl = ssLines.at(i + 5).split(" ", QString::KeepEmptyParts);
								for (auto it = sl.begin(); it != sl.end();it++)
								{
									for (int k = 0; k < vClustList.size();k++)
									{
										if (vClustList[k] == it->toInt())
										{
											point->ConnectImgSp[k] = -2;
										}
									}
									
								}
							}

							if (ssLines.at(i + 6).toInt() != 0)
							{
								QStringList sl = ssLines.at(i + 7).split(" ", QString::KeepEmptyParts);
								for (auto it = sl.begin(); it != sl.end(); it++)
								{
									for (int k = 0; k < vClustList.size(); k++)
									{
										if (vClustList[k] == it->toInt())
										{
											point->ConnectImgSp[k] = -2;
										}
									}
								}
							}


							this->m_spDepthWidgt->vPoints.push_back(point);

						}
					}
				}
 			}

		}
	}

	for (int i = 0; i < vThreads.size();i++)
	{
		vThreads[i]->wait();
	}
	for (int i = 0; i < vThreads.size(); i++)
	{
		delete vThreads[i];
	}
	vThreads.clear();
	 
}

void OpenCVFrame::ChangeViewState()
{
	if (sender() == m_spActSubWin)
	{
		if (eViewState == subwindows)
		{
			return;
		}
		eViewState = subwindows;
		emit(ViewRefresh());
	}
	if (sender() == m_spActTabs)
	{
		if (eViewState == Tabs)
		{
			return;
		}
		eViewState = Tabs;
		emit(ViewRefresh());

	}
}

void OpenCVFrame::InitializeToolBar()
{
	m_spToolbar = addToolBar(tr("File"));
	m_spToolbar->addAction(m_spActionOpenFile);
}

void OpenCVFrame::CreatePicSubWindow()
{

}

void OpenCVFrame::ConstructLayout()
{
	m_spHLayout = new QHBoxLayout();
	m_spMainWdget = new QWidget();

	m_spMdiWidget = new QStackedWidget();
	m_spCentralArea = new QMdiArea();
	m_spTabWdget = new QTabWidget();
	m_spTabWdget->setTabsClosable(true);
	m_spTabWdget->setTabShape(QTabWidget::Rounded);
	m_spMdiWidget->addWidget(m_spCentralArea);
	m_spMdiWidget->addWidget(m_spTabWdget);
	m_spMdiWidget->setCurrentWidget(m_spCentralArea);
	eViewState = subwindows;

	m_spUserWidget = new QStackedWidget();
	m_spSLIC = new SLICwidget(this);
	m_spDepthWidgt = new DepthSynWidget(this);
	m_spFT = new FT(this);
	m_spUserWidget->addWidget(m_spSLIC);
	m_spUserWidget->addWidget(m_spDepthWidgt);
	m_spUserWidget->setCurrentWidget(m_spSLIC);

	m_spHLayout->addWidget(m_spMdiWidget,10);
	m_spHLayout->addWidget(m_spUserWidget,2);

	m_spMainWdget->setLayout(m_spHLayout);

	this->setCentralWidget(m_spMainWdget);
}



void OpenCVFrame::ViewRefresh(int i)
{
	if (eViewState == subwindows)
	{
		m_spMdiWidget->setCurrentWidget(m_spCentralArea);
		m_spCentralArea->closeAllSubWindows();
		m_spTabWdget->clear();
		QStringList::iterator sit;
		vector<QScrollArea*>::iterator vit;
		for (sit = sPicNames.begin(),vit = vScrollAreas.begin();sit != sPicNames.end() && vit != vScrollAreas.end();sit++,vit++)
		{
			QMdiSubWindow* curSubWindow = new QMdiSubWindow();
			curSubWindow->setWindowTitle(*sit);
			curSubWindow->setAttribute(Qt::WA_DeleteOnClose);
			curSubWindow->setWidget(*vit);
			m_spCentralArea->addSubWindow(curSubWindow);

			(*vit)->show();
			curSubWindow->show();
		}
		m_spCentralArea->tileSubWindows();
		
		if (i == -1)
		{
			return;
		}
		
		m_spCentralArea->setActiveSubWindow(m_spCentralArea->subWindowList(QMdiArea::CreationOrder).at(i));

	}
	else if (eViewState == Tabs)
	{
		m_spMdiWidget->setCurrentWidget(m_spTabWdget);
		m_spTabWdget->clear();
		QStringList::iterator sit;
		vector<QScrollArea*>::iterator vit;
		for (sit = sPicNames.begin(),vit = vScrollAreas.begin();sit != sPicNames.end() && vit != vScrollAreas.end();sit++,vit++)
		{
			m_spTabWdget->addTab(*vit,*sit);
		}

		if (i == -1)
		{
			return;
		}

		m_spTabWdget->setCurrentIndex(i);
		
	}
}

vector<Mat> OpenCVFrame::AquireCurImages()
{
	if (eCurImageState == allImages)
	{
		return vMat;
	}
	if (eCurImageState == curImage)
	{
		if (eViewState == subwindows)
		{
			QString sTitle;
			sTitle = m_spCentralArea->currentSubWindow()->windowTitle();
			int index = sPicNames.indexOf(sTitle);
			return vMat.at(index);
		}
		else if (eViewState == subwindows)
		{
			return vMat.at(m_spTabWdget->currentIndex());
		}
	}
}

int OpenCVFrame::AquireCurrentIndex()
{
	if (eViewState == subwindows)
	{
		QString sTitle;
		sTitle = m_spCentralArea->currentSubWindow()->windowTitle();
		int index = sPicNames.indexOf(sTitle);
		return index;
	}
	else if (eViewState == Tabs)
	{
		return m_spTabWdget->currentIndex();
	}
}

void OpenCVFrame::OutPutInf(QString sFileName,QString sHeader, Mat mat, bool bRefresh)
{
	sFileName = QDir::currentPath() + "/doc/" + sFileName;
	QFile file(sFileName);
	
	if (bRefresh)
	{
		if (!file.open(QIODevice::WriteOnly))
		{
			QMessageBox::information(this, "alarm  " + sHeader, file.errorString() + sFileName);

		}
	}
	else
	{
		if (!file.open(QIODevice::WriteOnly | QIODevice::Append))
		{
			QMessageBox::information(this, "alarm  " + sHeader, file.errorString() + sFileName);

		}
	}

	

	QTextStream out(&file);

	out << sHeader << endl;

	switch (mat.depth())
	{
	case 0:
		for (int i = 0; i < mat.rows; i++)
		{
			for (int j = 0; j < mat.cols; j++)
			{
				out << "(";
				for (int m = 0; m < mat.channels(); m++)
				{
					out << mat.at<uchar>(i, j * 3 + m) << " ";

				}
				out << ")";
			}
			out << endl;
		}

		break;
	case 1:
		break;
	case 2:
		break;
	case 3:
		break;
	case 4:
		break;
	case 5:
		for (int i = 0; i < mat.rows; i++)
		{
			for (int j = 0; j < mat.cols; j++)
			{
				out << "(";
				for (int m = 0; m < mat.channels(); m++)
				{
					out << mat.at<float>(i, j * 3 + m) << " ";

				}
				out << ")";
			}
			out << endl;
		}
		break;
	case 6:
		break;


	default:
		break;
	}
	
	file.close();


}

void OpenCVFrame::ChangeImgState()
{
	if (sender() == m_spActAllImg)
	{
		eCurImageState = allImages;
	}
	else if (sender() == m_spActSnglImg)
	{
		eCurImageState = curImage;
	}
	
} 

void OpenCVFrame::ShowImage()
{
	if (sender() == m_spSLIC->m_spRadioDst)
	{
		if (m_spSLIC->eStatus == m_spSLIC->result)
		{
			return;
		}
		else
		{
			m_spSLIC->eStatus = m_spSLIC->result;
			if (eCurImageState == allImages)
			{
				vector<QScrollArea*>::iterator sit = vScrollAreas.begin();
				vector<QLabel*>::iterator lit = vPicLabels.begin();
				
				for (int i = 0; lit != vPicLabels.end() && sit != vScrollAreas.end();lit++,sit++,i++)
				{
					QImage image((const unsigned char*)(m_spSLIC->vDst.at(i).data),
						m_spSLIC->vDst.at(i).cols, m_spSLIC->vDst.at(i).rows,
						m_spSLIC->vDst.at(i).cols*m_spSLIC->vDst.at(i).channels(),
						QImage::Format_RGB888);
					(*lit)->setPixmap(QPixmap::fromImage(image));
					(*lit)->resize((*lit)->pixmap()->size());
					
				}
				ViewRefresh();
			}
			else if (eCurImageState == curImage)
			{
				int i = AquireCurrentIndex();
				QImage image((const unsigned char*)(m_spSLIC->vDst.at(i).data),
					m_spSLIC->vDst.at(i).cols, m_spSLIC->vDst.at(i).rows,
					m_spSLIC->vDst.at(i).cols*m_spSLIC->vDst.at(i).channels(),
					QImage::Format_RGB888);
				vPicLabels.at(i)->setPixmap(QPixmap::fromImage(image));
				vPicLabels.at(i)->resize(vPicLabels.at(i)->pixmap()->size());
				ViewRefresh(i);
			}
		}
	}
	if (sender() == m_spSLIC->m_spRadioSrc)
	{
		if (m_spSLIC->eStatus == m_spSLIC->source)
		{
			return;
		}
		else
		{
			m_spSLIC->eStatus = m_spSLIC->source;
			if (eCurImageState == allImages)
			{
				vector<QScrollArea*>::iterator sit = vScrollAreas.begin();
				vector<QLabel*>::iterator lit = vPicLabels.begin();

				for (int i = 0; lit != vPicLabels.end() && sit != vScrollAreas.end(); lit++, sit++, i++)
				{
					QImage image((const unsigned char*)(vMat.at(i).data),
						vMat.at(i).cols, vMat.at(i).rows,
						vMat.at(i).cols*vMat.at(i).channels(),
						QImage::Format_RGB888);
					(*lit)->setPixmap(QPixmap::fromImage(image));
					(*lit)->resize((*lit)->pixmap()->size());

				}
				ViewRefresh();
			}
			else if (eCurImageState == curImage)
			{
				int i = AquireCurrentIndex();
				QImage image((const unsigned char*)(vMat.at(i).data),
					vMat.at(i).cols, vMat.at(i).rows,
					vMat.at(i).cols*vMat.at(i).channels(),
					QImage::Format_RGB888);
				vPicLabels.at(i)->setPixmap(QPixmap::fromImage(image));
				vPicLabels.at(i)->resize(vPicLabels.at(i)->pixmap()->size());
				ViewRefresh(i);
			}
		}

	}
}

void OpenCVFrame::ShowImage(vector<Mat> vMat)
{
	vector<QScrollArea*>::iterator sit = vScrollAreas.begin();
	vector<QLabel*>::iterator lit = vPicLabels.begin();

	for (int i = 0; lit != vPicLabels.end() && sit != vScrollAreas.end(); lit++, sit++, i++)
	{
		QImage image((const unsigned char*)(vMat.at(i).data),
			vMat.at(i).cols, vMat.at(i).rows,
			vMat.at(i).cols*vMat.at(i).channels(),
			QImage::Format_RGB888);
		(*lit)->setPixmap(QPixmap::fromImage(image));
		(*lit)->resize((*lit)->pixmap()->size());
	}
	ViewRefresh();


}

openImageThread::openImageThread(OpenCVFrame *frame, int i, Mat srcRGB, string path) :m_frame(frame), nIndex(i), src(srcRGB), outFilePath(path)
{
	
}

openImageThread::~openImageThread()
{

}

void openImageThread::run()
{
	//这里可以做图像批量处理
	Mat dst = src.clone();
	resize(src, dst, Size(), 0.5, 0.5, INTER_LINEAR);
	cvtColor(dst, dst, CV_RGB2BGR);
	imwrite(outFilePath, dst);


	//一下为超像素分割正文
// 	Mat dst = src.clone();
// 	Mat mask = src.clone();
// 	int k = m_frame->m_spSLIC->m_spSpinBox->value();
// 	vector<int> vLabelPixCount;
// 	SLIC(src, mask, k, vLabelPixCount, m_frame->m_spSLIC->m_spSlider->value());
// 	m_frame->m_spSLIC->vNumSP.at(nIndex) = k;
// 	SLICColorBlend(src, dst, mask, k);
// 	Mat dstBGR;
// 	cvtColor(dst, dstBGR, CV_RGB2BGR);
// 	imwrite(outFilePath, dstBGR);
// 
// 	m_frame->m_spSLIC->vLabelMat.at(nIndex) = (mask);
// 	m_frame->m_spSLIC->vDst.at(nIndex) = (dst);
// 	m_frame->m_spSLIC->vvImageLabelPixelCount.at(nIndex) = (vLabelPixCount);

}
