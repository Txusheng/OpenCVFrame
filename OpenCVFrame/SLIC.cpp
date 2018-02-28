#include "SLIC.h"
#include "opencvframe.h"
#include "DepthSynthesis.h"
#include "MyAlgorithms.h"


void SLIC(Mat src, Mat &mask, int &k, vector<int> &vLabelPixCount, int m = 10)
{
	m = 30;
	Mat srcLab,srcXylab;
	Mat srclapcian;
	
	Mat kern = (Mat_<char>(3,3) <<  0, -1,  0,
		-1,  4, -1,
		0, -1,  0);
	filter2D(src,srclapcian,src.depth(),kern);
	cvtColor(src,srcLab,CV_RGB2Lab);

	int iNumPix = src.cols * src.rows;

	float fSPWidth = sqrtf((iNumPix)/float(k));
	float fSPHeight = sqrtf((iNumPix)/float(k));



	//define the centers of clusters
	vector<xylab> vC;
	
	int iXNumSP = ceil(src.cols/fSPWidth);
	int iYNumSP = ceil(src.rows/fSPHeight);
	for (int i = 0; i < iXNumSP ; i++)
	{
		for (int j = 0; j < iYNumSP ; j++)
		{
			xylab CC;
			if (i != iXNumSP - 1)
			{
				CC.x = int((i + 0.5) * fSPWidth);
			}
			else
			{
				CC.x = int((i * fSPWidth + src.cols - 1)/2);
			}
			if ( j != iYNumSP - 1)
			{
				CC.y = int((j + 0.5) * fSPHeight);
			}
			else
			{
				CC.y = int((j * fSPHeight + src.rows - 1)/2);
			}

			int x = 0;
			int y = 0;

			//choose the one nearby with the least gradient
			int iGrad = 10000;
			for (int n = CC.x - 1 ; n < CC.x + 1;n++)
			{
				for (int m = CC.y - 1;m < CC.y + 1;m++)
				{
					if (n < 0 || n > src.cols - 1 || m < 0 || m > src.rows - 1 )
					{
						continue;
					}
					int iTotal = abs(srclapcian.at<Vec<uchar, 3>>(m, n)[0]) + abs(srclapcian.at<Vec<uchar, 3>>(m, n)[1]) + abs(srclapcian.at<Vec<uchar, 3>>(m, n)[2]);
					if (iTotal < iGrad)
					{
						iGrad = iTotal;
						x = n;
						y = m;
					}					
				}
			}
			CC.x = x;
			CC.y = y;
			CC.L = srcLab.at<Vec<uchar,3>>(CC.y,CC.x)[0];
			CC.a = srcLab.at<Vec<uchar,3>>(CC.y,CC.x)[1];
			CC.b = srcLab.at<Vec<uchar,3>>(CC.y,CC.x)[2];
			vC.push_back(CC);

		}
	}
	k = vC.size();


	//iterate!!!
	Mat mIndex = Mat_<int>(src.rows,src.cols,-1);
	Mat mDistance = Mat_<float>(src.rows,src.cols,-1);
	float fResidualError = 10000;



	while (fResidualError >= vC.size()*0.1)
	{
		fResidualError = 0;
		vector<xylab> vC1;
		vC1.resize(vC.size());
		vector<int> vClusterNum;
		vClusterNum.assign(vC.size(), 0);

		vector<xylab>::iterator it;
		int iIndex;
		for (it = vC.begin() ,iIndex = 0; it != vC.end(); it ++ ,iIndex++)
		{
			xylab CC = *it;
			int x,y;
			for (x = int(CC.x - fSPWidth);x <= int(CC.x + fSPWidth);x++ )
			{
				for (y = int(CC.y - fSPHeight); y <= int(CC.y + fSPHeight);y++)
				{
					if (x < 0 || y < 0 || x > src.cols - 1 || y > src.rows - 1)
					{
						continue;
					}
					Vec3b vlab = srcLab.at<Vec<uchar, 3>>(y, x);
					xylab curXylab = { x, y, vlab[0], vlab[1], vlab[2] };
					if (mIndex.at<int>(y, x) == -1)
					{
						mDistance.at<float>(y, x) = distancexylab(curXylab, CC, fSPHeight, m);
						mIndex.at<int>(y, x) = iIndex;
						continue;
					}
					if (distancexylab(curXylab, CC, fSPHeight, m) < mDistance.at<float>(y, x))
					{
						mDistance.at<float>(y, x) = distancexylab(curXylab, CC, fSPHeight, m);
						mIndex.at<int>(y, x) = iIndex;
					}

				}
			}
		}
		for (int i = 0; i < src.cols; i++)
		{
			for (int j = 0; j < src.rows; j++)
			{
				int iCurIndex = mIndex.at<int>(j,i);
				vC1.at(iCurIndex).x += i;
				vC1.at(iCurIndex).y += j;
				vC1.at(iCurIndex).L += srcLab.at<Vec<uchar,3>>(j,i)[0];
				vC1.at(iCurIndex).a += srcLab.at<Vec<uchar,3>>(j,i)[1];
				vC1.at(iCurIndex).b += srcLab.at<Vec<uchar,3>>(j,i)[2];
				vClusterNum.at(iCurIndex)++;
			}
		}
		for (int i = 0;i < vC1.size();i++)
		{
			vC1.at(i).x /= vClusterNum.at(i);
			vC1.at(i).y /= vClusterNum.at(i);
			vC1.at(i).L /= vClusterNum.at(i);
			vC1.at(i).a /= vClusterNum.at(i);
			vC1.at(i).b /= vClusterNum.at(i);
			fResidualError += pow(vC1.at(i).x - vC.at(i).x,2) + pow(vC1.at(i).y - vC.at(i).y,2);
		}
		vC = vC1;
	}
	mask = mIndex.clone();
// 	SLICEnforceConnectivity(mask, k,vLabelPixCount);
	SLICmerge(src, mask, k, vLabelPixCount);

}

float distancexylab( xylab a, xylab b,float spatialrange,float labrange )
{
	float fSpatial;
	float fLab;
	fSpatial = pow(a.x - b.x,2) + pow(a.y - b.y,2);
	fLab = pow(a.L - b.L,2) + pow(a.a - b.a,2) + pow(a.b - b.b,2);

	return fSpatial / pow(spatialrange,2) + fLab / pow(labrange,2);

}

void SLICColorBlend(Mat src,Mat &dst,Mat mask,int k )
{
// 	RNG rng(0xFFFFFFFF);
// 	vector<Vec<uchar,3>> vRandomColor(k);
// 	for (int i = 0;i < k;i++)
// 	{
// 		Vec<uchar, 3> vCur;
// 		vCur[0] = rng.uniform(0, 256);
// 		vCur[1] = rng.uniform(0, 256);
// 		vCur[2] = rng.uniform(0, 256);
// 		vRandomColor.at(i) = vCur;
// 	}
// 	Mat mRandom = src.clone();
// 	for (int x = 0; x < mask.cols;x++)
// 	{
// 		for (int y = 0; y < mask.rows;y++)
// 		{
// // 			int p = mask.at<int>(y, x);
// // 			Vec<uchar, 3> q = vRandomColor.at(p);
// // 			mRandom.at<Vec<uchar, 3>>(y,x) = q;
// 			mRandom.at<Vec<uchar,3>>(y,x) = vRandomColor.at(mask.at<int>(y,x));
// // 			mRandom.at<Vec<uchar, 3>>(y, x)[1] = vRandomColor.at(mask.at<int>(y, x))[1];
// // 			mRandom.at<Vec<uchar, 3>>(y, x)[2] = vRandomColor.at(mask.at<int>(y, x))[2];
// 		}
// 	}
// 	addWeighted(dst,0.8,mRandom,0.2,0.0,dst);


	//采用白线划分的方式
	dst = src.clone();
	int a[4] = { 0, -1, 0, 1 };
	int b[4] = { 1, 0, -1, 0 };

	for (int m = 1; m < src.rows - 1;m++)
	{
		for (int n = 1; n < src.cols - 1;n++)
		{
			int id = mask.at<int>(m, n);
			for (int i = 0; i < 4; i++)
			{
				if (mask.at<int>(m + a[i],n + b[i]) != id)
				{
					dst.at<Vec3b>(m, n) = Vec3b(255,255,255);
					break;
				}
			}
		}
	}

	
}

void SLICEnforceConnectivity(Mat &mask, int &k, vector<int> &vLabelPixCount)
{
	Mat curMask = Mat_<int>(mask.rows, mask.cols, -1);
	int label(0);
	int adjlabel(0);
	int dx4[4] = { -1, 0, 1, 0 };
	int dy4[4] = { 0, -1, 0, 1 };
	int SPsize = mask.cols * mask.rows / float(k);
	

	for (int y = 0; y < mask.rows;y++)
	{
		for (int x = 0; x < mask.cols; x++)
		{
			if (curMask.at<int>(y,x) < 0)
			{
				curMask.at<int>(y, x) = label;
				for (int i = 0; i < 4;i++)
				{
					int nx = x + dx4[i];
					int ny = y + dy4[i];
					if (nx > -1 && nx <mask.cols && ny > -1 && ny <mask.rows )
					{
						if (curMask.at<int>(ny,nx) >= 0)
						{
							adjlabel = curMask.at<int>(ny, nx);
						}
					}
 				}
				int count(1);
				vector<Point2i> vClustPoints;
				vClustPoints.push_back(Point2i(x, y));
				for (int m = 0; m < count; m++)
				{
					for (int i = 0; i < 4; i++)
					{
						int nx = vClustPoints.at(m).x + dx4[i];
						int ny = vClustPoints.at(m).y + dy4[i];
						if (nx > -1 && nx <mask.cols && ny > -1 && ny < mask.rows)
						{
							if (curMask.at<int>(ny, nx) < 0 && mask.at<int>(y, x) == mask.at<int>(ny, nx))
							{
								curMask.at<int>(ny, nx) = curMask.at<int>(y, x);
								count++;
								vClustPoints.push_back(Point2i(nx, ny));
							}
						}
					}
				}

				if (count <= SPsize / 4)
				{


					for (int m = 0; m < count; m++)
					{
						Point curPoint = vClustPoints.at(m);
						curMask.at<int>(curPoint.y, curPoint.x) = adjlabel;
					}
					if (!vLabelPixCount.size())
					{
						vLabelPixCount.push_back(count);
						++label;
					}
					else
					{
						vLabelPixCount.at(adjlabel) += count;
					}
					
				}
				else
				{
					vLabelPixCount.push_back(count);
					++label;
				}

				

			
			}

		}
	}
	mask = curMask.clone();
	k = vLabelPixCount.size();
}

void SLICmerge(const Mat &src, Mat &mask, int &k, vector<int> &vLabelPixCount)
{
	float tresh = 0.1;
	int SPsize = mask.cols * mask.rows / float(k);
	Mat curLabel = Mat_<int>(mask.rows, mask.cols, -1);

	Mat matLab;
	cvtColor(src, matLab, CV_RGB2Lab);
	float fStep =float(255) / 20;

	int index(0);
	vector<int> vSmallSp;
	vector<vector<Point2i>> vvSpPoints;

	//每一个超像素，对应一个LAb直方图
	vector<vector<float>> vvSPHist;
	

	//四个方向
	int dm[4] = { -1, 0, 1, 0 };
	int dn[4] = { 0, -1, 0, 1 };

	MyGraph<MyNode> graph(0);

	for (int m = 0; m < curLabel.rows;m++)
	{
		for (int n = 0; n < curLabel.cols;n++)
		{
			//新像素，进行种子填充、添加邻接图节点、添加小超像素数组
			if (curLabel.at<int>(m, n) < 0)
			{
				int nCount(1);
				graph.addNode();
				vector<Point2i> vPoints;
				vPoints.push_back(Point2i(n, m));
				vector<float> vHist(60,0);
				float fValue;
				curLabel.at<int>(m, n) = index;

				for (int k = 0; k < nCount;k++)
				{
					for (int i = 0; i < 4; i++)
					{

						int Nm = vPoints[k].y + dm[i];
						int Nn = vPoints[k].x + dn[i];

						if (Nm >= 0 && Nn >= 0 && Nm < curLabel.rows && Nn < curLabel.cols)
						{
							if (curLabel.at<int>(Nm, Nn) >= 0)
							{
								graph.AddEdge(index, curLabel.at<int>(Nm, Nn));
								graph.AddEdge(curLabel.at<int>(Nm, Nn), index);
							}
							else if (mask.at<int>(Nm,Nn) == mask.at<int>(vPoints[k]))
							{
								nCount++;
								vPoints.push_back(Point2i(Nn, Nm));
								curLabel.at<int>(Nm, Nn) = index;
							}
						}
					}

				}
				vvSpPoints.push_back(vPoints);



				//开始构建该区域的lab直方图
				fValue = 1 / (float(nCount) * 3);
				
				for (int k = 0; k < vPoints.size();k++)
				{
					vHist[(matLab.at<Vec3b>(vPoints[k])[0] == 255 ? 254.9 : matLab.at<Vec3b>(vPoints[k])[0]) / fStep] += fValue;
					vHist[(matLab.at<Vec3b>(vPoints[k])[1] == 255 ? 254.9 : matLab.at<Vec3b>(vPoints[k])[1]) / fStep + 20] += fValue;
					vHist[(matLab.at<Vec3b>(vPoints[k])[2] == 255 ? 254.9 : matLab.at<Vec3b>(vPoints[k])[2]) / fStep + 40] += fValue;
				}
				vvSPHist.push_back(vHist);


				//若数目较小，则标记为需要融合
				if (nCount < (SPsize / 4))
				{
					vSmallSp.push_back(index);
				}

				index++;

			}
		}
	}

	//开始进行融合！
	int nSmallSpNum;
	do 
	{
		nSmallSpNum = vSmallSp.size();
		for (int i = 0; i < vSmallSp.size(); i++)
		{
			float fMinDis = 100;
			int nMergeIndex;

			int nCurIndex = vSmallSp[i]; 
			MyNode *ConNode = graph.nodes[nCurIndex]->next;

			//index1表示较大的超像素，index2表示较小的
			int index1, index2;
			
			//如果说当前小像素 非常小，则直接和左上方向融合
			if (vvSpPoints[vSmallSp[i]].size() < SPsize * k / 7500)
			{
				if (ConNode->next)
				{
					index1 = ConNode->next->Index;
				}
				else
				{
					index1 = ConNode->Index;
				}			
				index2 = nCurIndex;
			}
			else
			{
				while (ConNode != NULL)
				{
					float dis = CalChiSquareDis(vvSPHist[nCurIndex], vvSPHist[ConNode->Index]);
					if (dis < fMinDis)
					{
						fMinDis = dis;
						nMergeIndex = ConNode->Index;
					}
					ConNode = ConNode->next;
				}

				//当距离比较小的时候，进行融合
				if (fMinDis > tresh)
				{
					continue;
				}
				else
				{
					//开始融合,更新过程服从：先大后小



					if (vvSpPoints[nCurIndex].size() < vvSpPoints[nMergeIndex].size())
					{
						index1 = nMergeIndex;
						index2 = nCurIndex;
					}
					else
					{
						index1 = nCurIndex;
						index2 = nMergeIndex;
					}

				}


			}
			//开始更新vvhist（lab直方图）
			float fStep = 255 / float(20);
			float fCount = vvSpPoints[index1].size() + vvSpPoints[index2].size();
			float fValue = 1 / (fCount * 3);
			bool bSmall = fCount < (SPsize / 4);

			vvSPHist[index1].resize(60, 0);
			for (int k = 0; k < vvSpPoints[index1].size(); k++)
			{
				Vec3b vLab = matLab.at<Vec3b>(vvSpPoints[index1][k]);
				vvSPHist[index1][(vLab[0] == 255 ? 254.9 : vLab[0]) / fStep] += fValue;
				vvSPHist[index1][(vLab[1] == 255 ? 254.9 : vLab[1]) / fStep + 20] += fValue;
				vvSPHist[index1][(vLab[2] == 255 ? 254.9 : vLab[2]) / fStep + 40] += fValue;
			}
			for (int k = 0; k < vvSpPoints[index2].size(); k++)
			{
				Vec3b vLab = matLab.at<Vec3b>(vvSpPoints[index2][k]);
				vvSPHist[index1][(vLab[0] == 255 ? 254.9 : vLab[0]) / fStep] += fValue;
				vvSPHist[index1][(vLab[1] == 255 ? 254.9 : vLab[1]) / fStep + 20] += fValue;
				vvSPHist[index1][(vLab[2] == 255 ? 254.9 : vLab[2]) / fStep + 40] += fValue;
			}
			vvSPHist[index2].clear();

			//更新邻接关系
			MyNode* curNode = graph.nodes[index2]->next;
			while (curNode)
			{
				graph.AddEdge(index1, curNode->Index);
				graph.AddEdge(curNode->Index, index1);
				graph.DelEdge(curNode->Index, index2);
				curNode = curNode->next;
			}
			graph.nodes[index2] = NULL;

			//融合点数据
			for (int k = 0; k < vvSpPoints[index2].size(); k++)
			{
				vvSpPoints[index1].push_back(vvSpPoints[index2][k]);
				curLabel.at<int>(vvSpPoints[index2][k]) = index1;
			}
			vvSpPoints[index2].clear();


			//更新smallSp信息！根据融合之后的超像素大小
			if (bSmall)
			{
				for (auto it = vSmallSp.begin(); it != vSmallSp.end(); it++)
				{
					if (*it == index2)
					{
						it = vSmallSp.erase(it);
						break;
					}
				}
			}
			else
			{
				int a = 2;
				for (auto it = vSmallSp.begin(); it != vSmallSp.end();)
				{
					if (*it == index2 || *it == index1)
					{
						it = vSmallSp.erase(it);
						if (--a == 0)
						{
							break;
						}
					}
					else
					{
						it++;
					}
				}

			}



		}
	} 
	while (vSmallSp.size() != nSmallSpNum);


	//融合过程结束之后，调整输出值
	vLabelPixCount.clear();
	for (int m = 0; m < mask.rows; m++)
	{
		for (int n = 0; n < mask.cols;n++)
		{
			mask.at<int>(m, n) = -1;
		}
	}


	index = 0;
	for (int m = 0; m < mask.rows;m++)
	{
		for (int n = 0; n < mask.cols;n++)
		{
			if (mask.at<int>(m,n) < 0)
			{
				vector<Point2i> vPoints;
				vPoints.push_back(Point2i(n, m));
				int count = 1;
				mask.at<int>(m, n) = index;
				for (int i = 0; i < count;i++)
				{
					for (int k = 0; k < 4;k++)
					{
						int cm = vPoints[i].y + dm[k];
						int cn = vPoints[i].x + dn[k];
						if (cm >= 0 && cm < mask.rows &&cn >= 0 && cn < mask.cols)
						{
							if (mask.at<int>(cm,cn) >=0 )
							{
								continue;
							}
							else if (curLabel.at<int>(m, n) == curLabel.at<int>(cm, cn))
							{
								++count;
								mask.at<int>(cm, cn) = index;
								vPoints.push_back(Point2i(cn, cm));
							}
							
						}
					}
				}
				vLabelPixCount.push_back(count);
				++index;
			}
		}
	}
	k = vLabelPixCount.size();

	int residualSP = 0;
	for (int i = 0; i < vvSpPoints.size();i++)
	{
		if (vvSpPoints[i].size() > 0)
		{
			residualSP++;
		}
	}

	bool ba = (k == residualSP);
}

void SLICwidget::ChangeMLabel(int m)
{
	m_spMlabel->setText("set M value : " + QString::number(m,10));
}

SLICwidget::SLICwidget(QMainWindow *parent) :QWidget(parent), eStatus(source)
{
	m_spMainWindow = parent;
	m_spLayout = new QGridLayout(this);

	m_spSlider = new QSlider();
	m_spSlider->setRange(1,100);
	m_spSlider->setValue(10);

	m_spSpinBox = new QSpinBox();
	m_spSpinBox->setRange(2,100000);
	m_spSpinBox->setValue(800);
		
	m_spKlabel = new QLabel(tr("set K value"));
	m_spMlabel = new QLabel(tr("set M value : 10"));

	m_spBtnGroup = new QGroupBox(tr("Just Do It!"),this);
	m_spRadioSrc = new QRadioButton(tr("View Source Image"),this);
	m_spRadioSrc->setChecked(true);
	m_spRadioDst = new QRadioButton(tr("View Result Image"),this);
	QVBoxLayout *vlayout = new QVBoxLayout();
	vlayout->addWidget(m_spRadioSrc);
	vlayout->addWidget(m_spRadioDst);
	m_spBtnGroup->setLayout(vlayout);

	m_spLayout->addWidget(m_spKlabel,0,0,1,1);
	m_spLayout->addWidget(m_spSpinBox,0,1,1,1);
	m_spLayout->addWidget(m_spMlabel,2,0,1,1);
	m_spLayout->addWidget(m_spSlider,2,1,1,1);
	m_spLayout->addWidget(m_spBtnGroup,4,0,2,2);


	connect(m_spSlider, SIGNAL(valueChanged(int)), this, SLOT(ChangeMLabel(int)));
}

SLICwidget::~SLICwidget()
{

}
