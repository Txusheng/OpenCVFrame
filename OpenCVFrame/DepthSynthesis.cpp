#include "DepthSynthesis.h"
#include "opencvframe.h"

const QString sFileDepthMat = "DepthMat.txt";
QMutex qWarpMutex;
QMutex depthMutex;

bool IntAscendSort(const int &a, const int &b)
{
	return a > b;
}

bool IntDescend(const int &a, const int &b)
{
	return a < b;
}

//命名反了，不过无所谓
bool pairIFDescenf(const pair<int, float> &a, const pair<int, float> &b)
{
	return a.second < b.second;
}

//其实我命名反了，不过无所谓
bool pairIFAcsend(const pair<int, float> &a, const pair<int, float> &b)
{
	return a.second > b.second;
}


bool depthPointAscend(const DepthPoint &a, const DepthPoint &b)
{
	return a.fDepth <= b.fDepth;
}


template<typename T>
float GetAngleCos(const vector<T> &a, const vector<T> &b)
{
	if (a.size() != b.size())
	{
		//超出夹角范围
		return 1.1;
	}


	float avalue = 0;
	float bvalue = 0;
	float dot = 0;

	for (int i = 0; i < a.size(); i++)
	{
		avalue += (a[i] * a[i]);
		bvalue += (b[i] * b[i]);

		dot += a[i] * b[i];
	}

	dot /= sqrtf(avalue) * sqrtf(bvalue);

	//防止结果 因为精度问题 超出范围 -1~1
	if (dot < -1)
	{
		return -1;
	}
	if (dot > 1)
	{
		return 1;
	}

	return dot;


}

template<typename T>
float GetNorm(const vector<T> &a)
{
	float result = 0;
	for (int i = 0; i < a.size(); i++)
	{
		result += a[i] * a[i];
	}

	result = sqrtf(result);
	return result;

}


DepthSynWidget::DepthSynWidget(OpenCVFrame* parent) :QWidget(parent), bList(false), bSkyGraphCut(true), bDebugSky(true)
{
	m_spOpenCVFrame = parent;

	
	warpFile = new QFile(QDir::currentPath() + "/warpInform.txt");
	if (!warpFile->open(QIODevice::WriteOnly))
	{
		QMessageBox::information(this, "error", "open warpInform.txt failed!");
	}
	warpText = new QTextStream(warpFile);

	DepthFile = new QFile(QDir::currentPath() + "/depthInform.txt");
	if (!DepthFile->open(QIODevice::WriteOnly))
	{
		QMessageBox::information(this, "error", "open depthInform.txt failed!");
	}
	DepthsynText = new QTextStream(DepthFile);

}

DepthSynWidget::~DepthSynWidget()
{

}

void DepthSynWidget::PointsProjection(int i, depthSynThread* thread)
{


	Mat_<Vec<float, 3>> matDepthMap(thread->LabelMat.rows, thread->LabelMat.cols, Vec3f(-1.0, 0.0, 0.0));

	if (ebDebug)
	{
		m_spOpenCVFrame->OutPutInf(QString::number(i, 10) + sFileDepthMat, "Initial", matDepthMap, true);
	}


	vector<DepthPoint> vDepthPoint;
	vector<SamplePoint*> vSamplePoint;
	vector<int> vLabelPointCount(thread->nNumSp, 0);

	for (int j = 0; j < thread->vPoints.size(); j++)
	{
		if (thread->vPoints.at(j)->ConnectImgSp[i] == -1)
		{
			continue;
		}

		Mat_<float> X(3, 1);
		Mat_<float> x(3, 1, 0.0);

		X.at<float>(0, 0) = thread->vPoints.at(j)->x;
		X.at<float>(1, 0) = thread->vPoints.at(j)->y;
		X.at<float>(2, 0) = thread->vPoints.at(j)->z;
		// 			X.at<float>(3, 0) = 1;


		//二维空间x坐标的计算
		// 			x = camera.matRotation * X;
		// 			x = x + camera.T;
		// 			float fxValue = powf(x.at<float>(0, 0), 2) + powf(x.at<float>(1, 0), 2) + powf(x.at<float>(2, 0), 2);
		// 			x = x * camera.fFocal * (1 + camera.vK.at(0) * fxValue + camera.vK.at(1) * powf(fxValue, 2));
		// 			x.at<float>(2, 0) /= camera.fFocal;
		// 
		// 			//以上是原生结果，实际结果需要乘以 -1
		// 			x.at<float>(2, 0) *= -1;

		////////////////////////////////////////////////////////
		x = thread->camera->matRotation * X;
		x = x + thread->camera->T;
		x.at<float>(0, 0) /= (-x.at<float>(2, 0));
		x.at<float>(1, 0) /= (-x.at<float>(2, 0));
		float fxValue = powf(x.at<float>(0, 0), 2) + powf(x.at<float>(1, 0), 2);
		x.at<float>(0, 0) = x.at<float>(0, 0) * thread->camera->fFocal * (1 + thread->camera->vK.at(0) * fxValue + thread->camera->vK.at(1) * powf(fxValue, 2));
		x.at<float>(1, 0) = x.at<float>(1, 0) * thread->camera->fFocal * (1 + thread->camera->vK.at(0) * fxValue + thread->camera->vK.at(1) * powf(fxValue, 2));

		//一直到这里，x的第三个坐标依然是w
		float fDepth;
		float fM3 = sqrtf(powf(thread->camera->matRotation.at<float>(2, 0), 2) + powf(thread->camera->matRotation.at<float>(2, 1), 2) + powf(thread->camera->matRotation.at<float>(2, 2), 2));
		fDepth = -x.at<float>(2, 0) * (determinant(thread->camera->matRotation) < 0 ? -1 : 1) / fM3;

		Point point;

		//bundler的结果，坐标原点是图片正中心，需要将图片坐标转化到opencv坐标,如果转化的坐标超出了图片范围，则跳出循环

		point.x = (x.at<float>(0, 0) + float(thread->src.cols + 1) / 2);
		point.y = (x.at<float>(1, 0) + float(thread->src.rows + 1) / 2);

		point.y = thread->src.rows - point.y;

		if (point.x < 0 || point.x >= matDepthMap.cols || point.y < 0 || point.y >= matDepthMap.rows)
		{
			continue;
		}

		DepthPoint depthpoint;
		depthpoint.point = point;
		depthpoint.fDepth = fDepth;

		vDepthPoint.push_back(depthpoint);

		//samplepoint 的 推入
		SamplePoint* samplePoint = new SamplePoint;
		samplePoint->Point3D = thread->vPoints.at(j);
		//初始化链接，不要if也行
		if (samplePoint->Point3D->ConnectImgSp == NULL)
		{
			samplePoint->Point3D->ConnectImgSp = new int[thread->nNumCamera];
			for (int i = 0; i < thread->nNumCamera; i++)
			{
				samplePoint->Point3D->ConnectImgSp[i] = -1;
			}
			samplePoint->Point3D->nSize = thread->nNumCamera;
		}
		samplePoint->Point2D = point;
		samplePoint->Point2Df = Point2f(x.at<float>(0, 0) + float(thread->src.cols + 1) / 2, thread->src.rows - (x.at<float>(1, 0) + float(thread->src.rows + 1) / 2));
		samplePoint->fDepth = fDepth;
		samplePoint->bOrigin = true;
		samplePoint->nSpIndex = thread->LabelMat.at<int>(point);
		// 			samplePoint->BackProjection(vSrc[i], vCamera[i]);
		vSamplePoint.push_back(samplePoint);


		int label;
		label = thread->LabelMat.at<int>(int(point.y + 0.5), int(point.x + 0.5));

		matDepthMap.at<float>(point.y, point.x * 3 + 0) = label;

		matDepthMap.at<Vec<float, 3>>(int(point.y + 0.5), int(point.x + 0.5))[0] = thread->LabelMat.at<int>(int(point.y + 0.5), int(point.x + 0.5));

		vLabelPointCount.at(label)++;
		matDepthMap.at<Vec<float, 3>>(int(point.y + 0.5), int(point.x + 0.5))[1] = fDepth;

		if (ebDebug)
		{
			m_spOpenCVFrame->OutPutInf(QString::number(i, 10) + sFileDepthMat, "Point" + QString::number(j, 10), matDepthMap, true);
		}
	}

	thread->DepthMat = (matDepthMap);
	thread->vDepthPoint = (vDepthPoint);
	thread->vSamplePoint = (vSamplePoint);
	thread->vLabelPointCount = vLabelPointCount;


}

void DepthSynWidget::TargetSPBlend(int i, depthSynThread* thread)
{
	RNG rng(0xFFFFFFFF);


	Mat src = thread->src;
	Mat slicMask = thread->LabelMat;

	Mat_<Vec<uchar, 3>> TargetSPMask;
	TargetSPMask.create(src.rows, src.cols);
	Mat BlendMat = TargetSPMask.clone();
	vector<int> vTargetSPLabel(thread->vTargetSPLabel);
	vector<int> vSegIndex(thread->vSegIndex);

	vector<Vec<uchar, 3>> vRandomLabelColor;


	int n = 0;//表示vTargetSPLabel的下标
	int p = 0;//表示vSegIndex下标
	for (int m = 0; m < thread->nNumSp; m++)
	{
		Vec<uchar, 3> r1;
		//深度缺失为蓝色
		if (m == vTargetSPLabel.at(n))
		{
			r1[0] = 255;
			r1[1] = 0;
			r1[2] = rng.uniform(0, 256);
			if (n < vTargetSPLabel.size() - 1)
			{
				n++;
			}

		}
		else if (m == vSegIndex[p])
		{
			r1[1] = r1[2] = r1[0] = rng.uniform(128, 256);
			if (p < vSegIndex.size() - 1)
			{
				p++;
			}

		}
		//深度充足为绿色
		else
		{
			r1[1] = 255;
			r1[0] = 0;
			r1[2] = rng.uniform(0, 256);
		}
		vRandomLabelColor.push_back(r1);
	}


	for (int x = 0; x < slicMask.cols; x++)
	{
		for (int y = 0; y < slicMask.rows; y++)
		{

			int iLabel = slicMask.at<int>(y, x);
			TargetSPMask.at<Vec<uchar, 3>>(y, x) = vRandomLabelColor.at(iLabel);
		}
	}
	addWeighted(src, 0.6, TargetSPMask, 0.4, 0.0, BlendMat);


	/////////////////////////////////


	//在每一个超像素边缘画白线，并在超像素上标记超像素的编号

	vector<int> vMarkSP(thread->nNumSp, 0);

	int a[4] = { 0, -1, 0, 1 };
	int b[4] = { 1, 0, -1, 0 };

	for (int m = 1; m < src.rows - 1; m++)
	{
		for (int n = 1; n < src.cols - 1; n++)
		{
			int id = slicMask.at<int>(m, n);
			for (int i = 0; i < 4; i++)
			{
				if (slicMask.at<int>(m + a[i], n + b[i]) != id)
				{
					BlendMat.at<Vec3b>(m, n) = Vec3b(255, 255, 255);
					break;
				}
			}
		}
	}

	for (int m = 0; m < thread->vDepthPoint.size(); m++)
	{
		Point2d point = Point2d(thread->vDepthPoint[m].point);

		circle(BlendMat, point, 1, Scalar(0, 0, 255), -1, 8);
	}


	for (int m = 0; m < slicMask.rows; m++)
	{
		for (int n = 0; n < slicMask.cols; n++)
		{
			int id = slicMask.at<int>(m, n);
			if (++vMarkSP[id] > thread->vLabelPixelCount[id] / 2)
			{
				putText(BlendMat, (QString::number(id)).toStdString(), Point2i(n, m), CV_FONT_NORMAL, 0.25, Scalar(0, 0, 0), 1, 8);
				vMarkSP[id] = -100;
			}
		}
	}


	thread->TargetMaskMat = TargetSPMask;
	thread->BlendMat = BlendMat;

	//将混合图保存下来
	QString path = QDir::currentPath() + "/doc/";
	QString cur = path + "blend" + QString::number(i) + "x.bmp";
	imwrite(cur.toStdString(), BlendMat);


}

float DepthSynWidget::CalChiSquareDis(vector<float> vA, vector<float> vB)
{
	float sum = 0;

	if (vA.size() != vB.size())
	{
		return -1;
	}

	for (int i = 0; i < vA.size(); i++)
	{
		if (vA[i] == 0 && vB[i] == 0)
		{
			continue;
		}
		else
		{
			sum += powf(vA[i] - vB[i], 2) / (vA[i] + vB[i]);
		}
	}
	return sum;
}

void DepthSynWidget::CalLabHist(int i, depthSynThread* thread)
{
	//步长设置为12.8 ，即为 256/20


	Mat matLab;
	cvtColor(thread->src, matLab, CV_RGB2Lab);
	if (ebDebug)
	{
		m_spOpenCVFrame->OutPutInf("matLab.txt", " ", matLab, true);
	}
	vector<vector<float>> vvLabelHist;
	for (int j = 0; j < thread->nNumSp; j++)
	{
		vector<float> vHist(60, 0);
		vvLabelHist.push_back(vHist);
	}

	//对图片像素进行循环，将histgram统计出来
	for (int m = 0; m < thread->src.rows; m++)
	{
		for (int n = 0; n < thread->src.cols; n++)
		{
			int nLabel = thread->LabelMat.at<int>(m, n);
			int nLabelCount = thread->vLabelPixelCount[nLabel];
			float fValue = 1 / (3 * float(nLabelCount));




			int nLBin = int((matLab.at<Vec<uchar, 3>>(m, n)[0]) / 12.8);
			int nABin = int((matLab.at<Vec<uchar, 3>>(m, n)[1]) / 12.8) + 20;
			int nBBin = int((matLab.at<Vec<uchar, 3>>(m, n)[2]) / 12.8) + 40;
			vvLabelHist.at(nLabel)[nLBin] += fValue;
			vvLabelHist.at(nLabel)[nABin] += fValue;
			vvLabelHist.at(nLabel)[nBBin] += fValue;

		}
	}

	thread->vvLabelHis = (vvLabelHist);

}


vector<vector<float>> DepthSynWidget::CalLabHist(const Mat &srcRGB, const Mat &matLabel, int nNumSP, const vector<int> &vSpPixelNum, int nBins /*= 20*/)
{
	//根据通道数 ，计算步长，后期要注意正好为255的值应当特殊处理
	float fStep = 255 / float(nBins);

	Mat matLab;
	cvtColor(srcRGB, matLab, CV_RGB2Lab);

	vector<vector<float>> vvLabHist(nNumSP, vector<float>(3 * nBins, 0));

	for (int m = 0; m < matLab.rows; m++)
	{
		for (int n = 0; n < matLab.cols; n++)
		{
			int nLabel = matLabel.at<int>(m, n);
			int nLabelCount = vSpPixelNum[nLabel];
			float fValue = 1 / (3 * float(nLabelCount));

			vvLabHist[nLabel][(matLab.at<Vec3b>(m, n)[0] == 255 ? 254.99 : matLab.at<Vec3b>(m, n)[0]) / fStep] += fValue;
			vvLabHist[nLabel][(matLab.at<Vec3b>(m, n)[1] == 255 ? 254.99 : matLab.at<Vec3b>(m, n)[1]) / fStep + 20] += fValue;
			vvLabHist[nLabel][(matLab.at<Vec3b>(m, n)[2] == 255 ? 254.99 : matLab.at<Vec3b>(m, n)[2]) / fStep + 40] += fValue;
		}
	}

	return vvLabHist;
}

void DepthSynWidget::FindNearLabel(int i, depthSynThread* thread)
{
	//首先对每一个target Label 找到 直方图卡方差距最小的 label 40 个

	vector<int> vTargetLabel = thread->vTargetSPLabel;
	vector<vector<float>> vvLabelHist = thread->vvLabelHis;
	map<int, vector<int>> mapTargetNearLabel;
	for (int m = 0; m < vTargetLabel.size(); m++)
	{
		int n = 0;// n表示再循环label的过程中，当前target Label 的所在位置
		vector<int> vNearLabel;
		//计算所有非target label 与 target label 对应的 卡方距离


		vector<pair<int, float>> vLabelDis;//用结构体 按照dis排序

		for (int j = 0; j < thread->nNumSp; j++)
		{
			if (j == vTargetLabel[n])
			{
				if (n < vTargetLabel.size() - 1)
				{
					n++;
				}
				continue;
			}
			else
			{
				vLabelDis.push_back(pair<int, float>(j, CalChiSquareDis(vvLabelHist.at(vTargetLabel[m]), vvLabelHist[j])));
			}

		}

		qSort(vLabelDis.begin(), vLabelDis.end(), pairIFDescenf);
		if (vLabelDis.size() >= 40)
		{
			vLabelDis.erase(vLabelDis.begin() + 40, vLabelDis.end());

		}
		for (int i = 0; i < vLabelDis.size(); i++)
		{
			vNearLabel.push_back(vLabelDis[i].first);
		}




		//由小到大快排，并截取前四十个
		mapTargetNearLabel[vTargetLabel[m]] = vNearLabel;
	}

	thread->mapTargetNearLabel = mapTargetNearLabel;

}

void DepthSynWidget::CreateAdjointGraph(int i, depthSynThread* thread)
{
	int nLabelCount = thread->nNumSp;

	vector<vector<float>> Graph;
	vector<Vertex*> vVertex(nLabelCount);

	for (int k = 0; k < nLabelCount; k++)
	{
		vector<float> vRow(nLabelCount, -1);
		vRow.at(k) = 0;
		Graph.push_back(vRow);

		vVertex[k] = (new Vertex);
		vVertex[k]->nStart = k;

	}




	//只是为了求邻接，没必要把所有像素遍历到
	int dRows[4] = { 0, -1, 0, 1 };
	int dCols[4] = { -1, 0, 1, 0 };

	for (int m = 0; m < thread->src.rows; m++)
	{
		for (int n = 0; n < thread->src.cols; n++)
		{
			int label1, label2;
			label1 = thread->LabelMat.at<int>(m, n);

			for (int k = 0; k < 4; k++)
			{
				if (m + dRows[k] < 0 || m + dRows[k] >= thread->src.rows || n + dCols[k] < 0 || n + dCols[k] >= thread->src.cols)
				{
					continue;
				}

				label2 = thread->LabelMat.at<int>(m + dRows[k], n + dCols[k]);
				if (label1 != label2)
				{
					if (Graph[label1][label2] == -1)
					{
						float fWeight = CalChiSquareDis(thread->vvLabelHis[label1], thread->vvLabelHis[label2]);
						Graph[label1][label2] = fWeight;

						Line* line;
						line = new Line;
						line->nEnd = thread->LabelMat.at<int>(m + dRows[k], n + dCols[k]);
						line->next = NULL;
						line->nWeight = fWeight;
						vVertex[label1]->nCount++;
						vVertex[label1]->PushBack(line);

					}


				}
			}
		}
	}

	thread->Graph = (Graph);
	thread->vVertexAdjoint = (vVertex);

}

void DepthSynWidget::Dijskra(vector<Vertex*> vVertex, int nTargetLabel, vector<int> &vNearLabel)
{
	//是否被加入到最短顶点集合
	int *set;
	set = new int[vVertex.size()];

	//当前顶点路径的上一个顶点
	int *path;
	path = new int[vVertex.size()];

	//路径开销
	float *cost;
	cost = new float[vVertex.size()];


	//初始化
	for (int i = 0; i < vVertex.size(); i++)
	{
		set[i] = 0;
		path[i] = -1;
		cost[i] = -1;
	}

	Line *pLine = vVertex[nTargetLabel]->next;
	while (pLine != NULL)
	{
		int n = pLine->nEnd;
		path[n] = nTargetLabel;
		cost[n] = pLine->nWeight;
		pLine = pLine->next;
	}

	set[nTargetLabel] = 1;
	cost[nTargetLabel] = 0;


	//开始循环加入最短路径点

	for (int i = 0; i < vVertex.size() - 1; i++)
	{
		float fMinCost = 62500.0f;
		int nV;//此次循环新添的顶点

		//循环选取要加入最邻近距离的点
		for (int j = 0; j < vVertex.size(); j++)
		{
			if (set[j] == 0 && cost[j] >= 0 && cost[j] < fMinCost)
			{
				fMinCost = cost[j];
				nV = j;
			}
		}
		set[nV] = 1;


		//更新链接的顶点距离
		pLine = vVertex[nV]->next;
		while (pLine != NULL)
		{
			if (set[pLine->nEnd] == 0)
			{
				if ((cost[pLine->nEnd]<0) || (cost[pLine->nEnd]> cost[nV] + pLine->nWeight))
				{
					cost[pLine->nEnd] = cost[nV] + pLine->nWeight;
					path[pLine->nEnd] = nV;
				}

			}
			pLine = pLine->next;

		}

	}


	//找到了所有的cost，现在开始从NearLabel中筛选

	vector<pair<int, float>> vPairLabelDis;
	for (int i = 0; i < vNearLabel.size(); i++)
	{
		vPairLabelDis.push_back(pair<int, float>(vNearLabel[i], cost[vNearLabel[i]]));
	}
	qSort(vPairLabelDis.begin(), vPairLabelDis.end(), pairIFDescenf);


	vNearLabel.clear();

	vector<pair<int, float>>::iterator it;
	int nCount3;
	for (it = vPairLabelDis.begin(), nCount3 = 0; (it != vPairLabelDis.end() && nCount3 < 3); it++, nCount3++)
	{
		vNearLabel.push_back(it->first);
	}

}

void DepthSynWidget::SPInsertDepthPoint(vector<vector<int>> vvImgInsertLabel, vector<vector<int>> vImgSpDepth)
{
	for (int i = 0; i < vSrc.size(); i++)
	{
		RNG rng;
		map<int, vector<int>> mapSkyLabelHash;
		map<int, int> mapSkyLabelIndex;
		for (int m = 0; m < vvImgInsertLabel[i].size(); m++)
		{
			if (vvImgInsertLabel[i][m] == 1)
			{
				vector<int> vHash(m_spOpenCVFrame->m_spSLIC->vvImageLabelPixelCount[i][m], 0);
				mapSkyLabelHash.insert(pair<int, vector<int>>(m, vHash));
				mapSkyLabelIndex.insert(pair<int, int>(m, 0));

				//开始随机选取像素进行赋值
				assert(vHash.size() > 15);
				int  nCountDown = (vHash.size() / 200 > 15) ? (vHash.size() / 200) : 15;//剩余需要赋值的像素数目

				int nCount = 0;//已经赋值的像素数目

				while (nCountDown)
				{
					int index;	//要赋值的像素下标
					index = rng.uniform(0, vHash.size() - nCount);
					if (mapSkyLabelHash[m][index])
					{
						continue;
					}
					else
					{
						mapSkyLabelHash[m][index] = 1;
						++nCount;
						--nCountDown;
					}
				}
			}
		}

		map<int, vector<int>>::iterator it;
		for (int m = 0; m < vLabelMat[i].rows; m++)
		{
			for (int n = 0; n < vLabelMat[i].cols; n++)
			{
				it = mapSkyLabelHash.find(vLabelMat[i].at<int>(m, n));

				if (it != mapSkyLabelHash.end())
				{
					if ((it->second)[mapSkyLabelIndex[it->first]])
					{
						DepthPoint curDepthPoint;
						curDepthPoint.point = Point(n, m);
						curDepthPoint.fDepth = vImgSpDepth[i][vLabelMat[i].at<int>(m, n)];
						vvDepthPoint[i].push_back(curDepthPoint);

						vDepthMat[i].at<Vec<float, 3>>(m, n) = { (float)vLabelMat[i].at<int>(m, n), curDepthPoint.fDepth, 0 };

						(it->second)[mapSkyLabelIndex[it->first]++] = 0;

					}
				}
			}
		}
	}

}


void DepthSynWidget::GetSPDepth(int i, depthSynThread* thread)
{
	vector<DepthPoint> vDepthPoint(thread->vDepthPoint);


	//每个label 对应一组深度值
	map<int, vector<DepthPoint>> mapLabelDepth;
	map<int, vector<DepthPoint>>::iterator itLabelDepth;

	Mat matDepthMat = thread->DepthMat;

	//如果没有足够深度点，根本没必要玩了,否则获得该图片的深度值范围
	if (vDepthPoint.size() <= 10)
	{
		vmapImgSPDepth[i] = (mapLabelDepth);
		return;
	}

	//对VdepthPoint 进行快排，然后确定百分位，将百分位信息输入到map中
	qSort(vDepthPoint.begin(), vDepthPoint.end(), depthPointAscend);
	thread->vDepthPoint = vDepthPoint;

	//每一个label 输入一组 深度值
	for (int m = 0; m < vDepthPoint.size(); m++)
	{
		int nLabel;
		nLabel = matDepthMat.at<Vec<float, 3>>(vDepthPoint[m].point.y, vDepthPoint[m].point.x)[0];
		itLabelDepth = mapLabelDepth.find(nLabel);
		if (itLabelDepth == mapLabelDepth.end())
		{
			vector<DepthPoint> vDepth;
			vDepth.push_back(vDepthPoint[m]);
			mapLabelDepth.insert(pair<int, vector<DepthPoint>>(nLabel, vDepth));

		}
		else
		{
			itLabelDepth->second.push_back(vDepthPoint[m]);
		}
	}
	//

	thread->mapSPDepth = mapLabelDepth;


}

void DepthSynWidget::GenEnableSynTarget(int i, depthSynThread* thread)
{
	int nNumberPass = thread->vTargetSPLabel.size();
	vector<int> vEnableTarget(thread->nNumSp, 0);

	vector<DepthPoint> vDepthPoint(thread->vDepthPoint);

	qSort(vDepthPoint.begin(), vDepthPoint.end(), depthPointAscend);
	map<float, int> mapDepthPercentile;
	float fStep = float(vDepthPoint.size()) / 100;
	for (int i = 0; i < vDepthPoint.size(); i++)
	{
		int nPercentile = int(i / fStep);
		mapDepthPercentile.insert(pair<float, int>(vDepthPoint[i].fDepth, nPercentile));
	}


	map<int, vector<float>> mapSpDepthHist;

	for (auto itNearLabel = thread->mapTargetNearLabel.begin(); itNearLabel != thread->mapTargetNearLabel.end(); itNearLabel++)
	{
		int nPeak = 0;
		int nLastPeakPos = -1;
		int nStart;

		vector<float> vHisDepth(100, 0);
		bool bEnableDepth = true;
		bool bFlag = false;


		//每一个targetLabel对应的深度点的数目
		int nDepthCount = 0;

		for (int m = 0; m < itNearLabel->second.size(); m++)
		{


			//获取该label对应的深度数组
			vector<DepthPoint> vDepth(thread->mapSPDepth[itNearLabel->second[m]]);

			nDepthCount += vDepth.size();

			for (int n = 0; n < vDepth.size(); n++)
			{
				vHisDepth[mapDepthPercentile[vDepth[n].fDepth]]++;
			}
		}


		//获取深度分布 的 概率密度图,同时获取峰值位置和数目

		for (int m = 0; m < vHisDepth.size(); m++)
		{
			vHisDepth[m] /= nDepthCount;
			if (!bFlag && vHisDepth[m])
			{
				bFlag = true;
				nStart = m;
			}
			else if (bFlag && (!vHisDepth[m] || m == (vHisDepth.size() - 1)))
			{
				bFlag = false;
				nPeak++;
				if (nLastPeakPos < 0)
				{
					nLastPeakPos = m;
				}
				else
				{
					if (nPeak > 2)
					{
						if (bEnableDepth)
						{
							--nNumberPass;
						}
						bEnableDepth = false;
					}
					else
					{
						int nDis = nStart - nLastPeakPos;
						if (nDis >= 5)
						{

							if (bEnableDepth)
							{
								--nNumberPass;
							}
							bEnableDepth = false;
						}

					}
					nLastPeakPos = m;
				}

			}



		}

		if (bEnableDepth)
		{
			vEnableTarget[itNearLabel->first] = 1;
			mapSpDepthHist.insert(pair<int, vector<float>>(itNearLabel->first, vHisDepth));
		}

	}

	thread->vEnableTarget = (vEnableTarget);
	thread->mapSpDepthHis = (mapSpDepthHist);
	thread->mapDepthPercentile = (mapDepthPercentile);


}

void DepthSynWidget::SetEnableDepth(int i, depthSynThread* thread)
{
	//输入参数





	vector<int> vEnableTarget(thread->vEnableTarget);
	map<int, vector<float>> mapSpDepthHist(thread->mapSpDepthHis);
	map<int, vector<int>> mapTargetNearLabel(thread->mapTargetNearLabel);
	map<int, vector<DepthPoint>> mapSpDepth(thread->mapSPDepth);
	map<float, int> mapDepthPercentile(thread->mapDepthPercentile);
	Mat src = thread->src;
	Mat LabelMat = thread->LabelMat;
	//输出参数（修改参数）


	map<int, int> mapSpPos;

	for (int m = 0; m < src.rows; m++)
	{
		for (int n = 0; n < src.cols; n++)
		{
			int nLabel = LabelMat.at<int>(m, n);
			if (vEnableTarget[nLabel])
			{
				auto it = mapSpPos.find(nLabel);
				if (it == mapSpPos.end())
				{
					mapSpPos.insert(pair<int, int>(nLabel, 0));

					vector<DepthPoint> vCurDepthPoint;
					thread->mapSPDepth.insert(pair<int, vector<DepthPoint>>(nLabel, vCurDepthPoint));

					it = mapSpPos.find(nLabel);
				}
				if (!(it->second % 150))
				{
					float fTotalDis = 0;
					float fTotal = 0;
					auto itNear = mapTargetNearLabel.find(nLabel);
					for (int k = 0; k < itNear->second.size(); k++)
					{
						auto itDepth = mapSpDepth.find(itNear->second[k]);
						for (int p = 0; p < itDepth->second.size(); p++)
						{
							DepthPoint yPoint = itDepth->second[p];
							float fDis = (yPoint.point.x - n) * (yPoint.point.x - n) + (yPoint.point.y - m) * (yPoint.point.y - m);
							fTotalDis += mapSpDepthHist[nLabel][mapDepthPercentile[yPoint.fDepth]] * fDis;
							fTotal += (mapSpDepthHist[nLabel][mapDepthPercentile[yPoint.fDepth]] * fDis / yPoint.fDepth);

						}
					}
					float fDepth = fTotal / fTotalDis;
					fDepth = 1 / fDepth;

					DepthPoint curDepthPoint;
					curDepthPoint.point = Point(n, m);
					curDepthPoint.fDepth = fDepth;

					thread->vDepthPoint.push_back(curDepthPoint);

					//新的数据结构 sample point
					SamplePoint *sample = new SamplePoint();
					sample->Point2D = Point2i(n, m);
					sample->fDepth = fDepth;
					sample->bOrigin = false;
					sample->BackProjection(src, thread->camera);
					thread->vSamplePoint.push_back(sample);


					thread->DepthMat.at<Vec<float, 3>>(m, n) = { (float)thread->LabelMat.at<int>(m, n), fDepth, 0 };



					thread->mapSPDepth[nLabel].push_back(curDepthPoint);


					thread->vLabelPointCount[nLabel]++;

					mapSpPos[nLabel]++;
				}
				++it->second;
			}
		}
	}

	//去掉其target身份
	for (int j = 0; j < vEnableTarget.size(); j++)
	{
		if (vEnableTarget[j])
		{
			thread->mapTargetNearLabel.erase(vEnableTarget[j]);
			for (auto it = thread->vTargetSPLabel.begin(); it != thread->vTargetSPLabel.end();)
			{
				if (*it == j)
				{
					it = thread->vTargetSPLabel.erase(it);
				}
				else
				{
					it++;
				}
			}
		}

	}

}

void DepthSynWidget::slotDepthSynthesis()
{

	if (!bList || vCamera.size() == 0 || vPoints.size() == 0)
	{
		QMessageBox::warning(this, "cao!!!", "haven't input enough information");
		return;
	}

	vmapImgTargetNearLabel.resize(vCamera.size());
	vvImgEnableTarget.resize(vCamera.size());
	vmapImgSpDepthHis.resize(vCamera.size());
	vmapDepthPercentile.resize(vCamera.size());
	vSrc.resize(vCamera.size());
	vLabelMat.resize(vCamera.size());
	vDepthMat.resize(vCamera.size());
	vvDepthPoint.resize(vCamera.size());
	vvSamplePoint.resize(vCamera.size());
	vvTargetSPLabel.resize(vCamera.size());
	vvNoneOriginSPLabel.resize(vCamera.size());
	vTargetMaskMat.resize(vCamera.size());
	vBlendMat.resize(vCamera.size());
	vvvImgLabelHis.resize(vCamera.size());
	vGraph.resize(vCamera.size());
	vvImgVertexAdjoint.resize(vCamera.size());
	vmapImgSPDepth.resize(vCamera.size());
	vvLabelPointCount.resize(vCamera.size());

	for (int i = 0; i < m_spOpenCVFrame->vMat.size(); i++)
	{
		//调试用，仅仅观察一个线程
// 		if (i != 3 )
// 		{
// 			continue;
// 		}
// 
		depthSynThread* pDepthThread = new depthSynThread(this, i);
		vdepthSynThread.push_back(pDepthThread);
		pDepthThread->start();
	}
	for (int i = 0; i < m_spOpenCVFrame->vMat.size(); i++)
	{
		vdepthSynThread[i]->wait();
	}
	for (int i = 0; i < vdepthSynThread.size();i++)
	{
		delete vdepthSynThread[i];
	}

	//测试程序，用于显示同一点对应不同视角的位置
	for (int i = 0; i < vPoints.size(); i++)
	{
		int nCount = 0;
		for (int m = 0; m < vPoints[i]->nSize; m++)
		{
			if (vPoints[i]->ConnectImgSp[m] != -1)
			{
				++nCount;
			}
		}

		if (nCount >= vPoints[i]->nSize - 1)
		{
			SamplePoint curSample;
			curSample.Point3D = vPoints[i];
			for (int k = 0; k < vPoints[i]->nSize; k++)
			{
				if (curSample.Projection(vBlendMat[k], vCamera[k]))
				{
					circle(vBlendMat[k], curSample.Point2D, 20, Scalar(255, 255, 255), -1, 8);
				}

// 				if (vPoints[i]->ConnectImgSp[k] != -1)
// 				{
// 					SamplePoint* sample = new SamplePoint();
// 					sample->Point3D = vPoints[i];
// 					if (sample->Projection(vBlendMat[k], vCamera[k]))
// 					{
// 						circle(vBlendMat[k], sample->Point2D, 20, Scalar(255, 255, 255), -1, 8);
// 					}
// 
// 
// 
// 				}
			}
			break;
		}
	}



	//接下来的内容可以通过action触发，我现在懒得写了 
	m_spOpenCVFrame->ShowImage(vBlendMat);


}

void DepthSynWidget::slotPrepareLocalWarp()
{
	//完成信息的转录
	warpInitialize(0.1);

	//构建 共享三维点的超像素 的连接图
	myGraph = new BinGraph(vImgInfo);
	myGraph->Initialize(vPoints);

	//获取每一个像素的warp结构以及线性系统
	for (int i = 0; i < vImgInfo.size(); i++)
	{
		if (!vImgInfo[i]->bValid)
		{
			continue;
		}
		for (int j = 0; j < vImgInfo[i]->vSP.size(); j++)
		{
			if (vImgInfo[i]->vSP[j].bHole)
			{
				continue;
			}
			SpWarpLinearSystem(vImgInfo[i]->vSP[j], 30, 4);
		}
	}


	//实验，给特定角度摄像头解线性方程组
// 	for (int i = 0; i < vImgInfo.size(); i++)
// 	{
// 		if (!vImgInfo[i]->bValid)
// 		{
// 			continue;
// 		}
// 		for (int j = 0; j < vImgInfo[i]->vSP.size(); j++)
// 		{
// 			if (vImgInfo[i]->vSP[j].bHole)
// 			{
// 				continue;
// 			}
// 			vImgInfo[i]->vSP[j].solveLinearSys();
// 		}
// 	}
// 


// 	float fWeight = float(18) / 30;
// 	camera->matRotation = (1 - fWeight) * vCamera[2]->matRotation + fWeight * vCamera[2 + 1]->matRotation;
// 	camera->T = (1 - fWeight) *  vCamera[2]->T + fWeight * vCamera[2 + 1]->T;
// 	camera->NormolizeR();
// 	slotLacolWarp(true);

	VideoWriter video;
	video.open((QDir::currentPath() + "/video.avi").toStdString(), CV_FOURCC('P', 'I', 'M', '1'), 30, Size(vSrc[0].cols, vSrc[0].rows), true);
	if (video.isOpened())
	{
		for (int i = 0; i < vCamera.size() - 1; i++)
		{
			for (int j = 0; j < 60; j++)
			{
				float fWeight = float(j) / 60;
				camera->matRotation = (1 - fWeight) * vCamera[i]->matRotation + fWeight * vCamera[i + 1]->matRotation;
				camera->T = (1 - fWeight) *  vCamera[i]->T + fWeight * vCamera[i + 1]->T;
				camera->NormolizeR();

				if (bWarpOut)
				{
					(*warpText) << "\n\n\n\n\n" << "NovalView" << QString::number(i * 30 + j)<<endl;
				}

				Mat frame = slotLacolWarp(false);

				imwrite((QDir::currentPath() + "/" + QString::number(j + 60 * i) + ".jpg").toStdString(), frame);
				video << frame;
				frame.release();
			}
		
		}

 	}

}

Mat DepthSynWidget::slotLacolWarp(bool bOutPut)
{
	//相机矩阵由 交互窗口 重新计算过


	//挑选四个角度最相近的相机
	GetKNovalNear(4);

	//权值图，novalview 每一个像素 对应到四张图 的权值
	Mat matWeight = Mat(vSrc[0].rows, vSrc[0].cols, CV_32FC4, Scalar(-1, -1, -1, -1));

	//权值图的中间过程
	Mat matMWeight = Mat(vSrc[0].rows, vSrc[0].cols, CV_32FC4, Scalar(0, 0, 0, 0));

	//novalview 每一个像素 对应到 原图的位置
	vector<vector<array<Point2i, 4>>> pixelCorrespond(vSrc[0].rows, vector<array<Point2i, 4>>(vSrc[0].cols));

	//多线程数组
	vector<warpThread*> vThreads(4);

	if (bWarpOut)
	{
		(*warpText) << tr("warp initialized") << endl;
	}

	//对四个相机 对应的图片 进行线性方程组的解 以及 投影（图片）
	for (int i = 0; i < vSelectedImg.size(); i++)
	{


		warpThread* thread = new warpThread(this, i, vSelectedImg[i], pixelCorrespond, matWeight, vLabelMat[vSelectedImg[i]->Index],*(camera));
		vThreads[i] = thread;

		if (bWarpOut)
		{
			(*warpText) << QString::number(i) << " thread has initialized" << endl;
		}

// 
// 		if (i != 0)
// 		{
// 			continue;
// 		}
// 		thread->start();

// 		ImgInfo* img = vSelectedImg[i];
// 		img->dst = Mat(img->src.rows, img->src.cols, CV_8UC3, Scalar(0,0,0));
// 		img->dstDepth = Mat(img->src.rows, img->src.cols, CV_32FC1, 0.0);
// 
// 		for (int j = 0; j < img->vSP.size(); j++)
// 		{
// 			SuperPixel* sp = &img->vSP[j];
// 
// 			//对于hole超像素，直接pass
// 			if (sp->bHole)
// 			{
// 				continue;
// 			}
// 			sp->solveLinearSys();
// 
// 			for (int k = 0; k < sp->vTriangleIndex.size(); k++)
// 			{
// 
// 
// 				Point2d *p1 = &(sp->vWarpVertex[sp->vTriangleIndex[k][0]]->vertex);
// 				Point2d *p2 = &(sp->vWarpVertex[sp->vTriangleIndex[k][1]]->vertex);
// 				Point2d *p3 = &(sp->vWarpVertex[sp->vTriangleIndex[k][2]]->vertex);
// 
// 
// 
// 				//三角网外包矩形
// 				int xLeft, xRight, yTop, yBottom;
// 				xLeft = p1->x < p2->x ? (p1->x < p3->x ? p1->x : p3->x) : (p2->x < p3->x ? p2->x : p3->x);
// 				xRight = p1->x > p2->x ? (p1->x > p3->x ? p1->x : p3->x) : (p2->x > p3->x ? p2->x : p3->x);
// 				yBottom = p1->y < p2->y ? (p1->y < p3->y ? p1->y : p3->y) : (p2->y < p3->y ? p2->y : p3->y);
// 				yTop = p1->y > p2->y ? (p1->y > p3->y ? p1->y : p3->y) : (p2->y > p3->y ? p2->y : p3->y);
// 
// 				xLeft = xLeft < 0 ? 0 : xLeft;
// 				xRight = xRight > img->src.cols - 1 ? img->src.cols - 1 : xRight;
// 				yBottom = yBottom < 0 ? 0 : yBottom;
// 				yTop = yTop > img->src.rows - 1 ? img->src.rows - 1 : yTop;
// 
// 				for (int y = yBottom; y <= yTop; y++)
// 				{
// 					for (int x = xLeft; x <= xRight; x++)
// 					{
// 
// 						//获取重心坐标
// 						float alpha = float((p2->y - p3->y) * (x - p3->x) + (p3->x - p2->x) * (y - p3->y)) / float((p2->y - p3->y)*(p1->x - p3->x) + (p3->x - p2->x) * (p1->y - p3->y));
// 
// 						float beta = float((p3->y - p1->y) * (x - p3->x) + (p1->x - p3->x) * (y - p3->y)) / float((p2->y - p3->y)*(p1->x - p3->x) + (p3->x - p2->x) * (p1->y - p3->y));
// 
// 						float gamma = 1 - alpha - beta;
// 
// 						//该点在三角形内，计算对应原图坐标
// 						if (alpha >= 0 && beta >= 0 && gamma >= 0)
// 						{
// 							//浮点型 对应原图点位置
// 							Point2d srcPoint;
// 
// 							//离散型（最邻近） 对应原图点位置
// 							Point2i iSrcPoint;
// 
// 
// 							Point2d *sp1 = &(sp->vVertexpoint[sp->vTriangleIndex[k][0]]->vertex);
// 							Point2d *sp2 = &(sp->vVertexpoint[sp->vTriangleIndex[k][1]]->vertex);
// 							Point2d *sp3 = &(sp->vVertexpoint[sp->vTriangleIndex[k][2]]->vertex);
// 
// 							srcPoint.x = alpha * sp1->x + beta * sp2->x + gamma * sp3->x;
// 							srcPoint.y = alpha * sp1->y + beta * sp2->y + gamma * sp3->y;
// 
// 							iSrcPoint.x = int(srcPoint.x + 0.5);
// 							iSrcPoint.y = int(srcPoint.y + 0.5);
// 
// 							//判断重投影点是否在超像素内，如果是 则 双线性插值 + 创立链接
// 							if ((vLabelMat[img->Index].at<int>(iSrcPoint) == j) || (sp->bExtension && sp->matExtSP.at<uchar>(iSrcPoint.y - sp->ExtBounding.y, iSrcPoint.x - sp->ExtBounding.x)))
// 							{
// 								//建立连接，同时计算权重
// 								pixelCorrespond[y][x].at(i) = iSrcPoint;
// 
// 								SamplePoint curPoint;
// 								curPoint.Point2D = iSrcPoint;
// 								curPoint.bOrigin = false;
// 								curPoint.fDepth = sp->fMedianDepth;
// 								curPoint.BackProjection(img->src, img->camera);
// 
// 								//点到原始相机的射线
// 								Point3f pRay = (*(dynamic_cast<Point3f*>(curPoint.Point3D)) - img->camera->CalCenter());
// 								vector<float> ray;
// 								ray.push_back(pRay.x);
// 								ray.push_back(pRay.y);
// 								ray.push_back(pRay.z);
// 
// 								//点到novalview的射线
// 								Point3f cpRay = (*(dynamic_cast<Point3f*>(curPoint.Point3D)) - this->camera->CalCenter());
// 								vector<float> cray;
// 								cray.push_back(cpRay.x);
// 								cray.push_back(cpRay.y);
// 								cray.push_back(cpRay.z);
// 
// 								float fCos = GetAngleCos(ray, cray);
// 								float fAngleWeight = acos(GetAngleCos(ray, cray));
// 
// 								float fDisWeigh;
// 								fDisWeigh = fabs(GetNorm(ray) - GetNorm(cray));
// 
// 								//
// 								if (img->dstDepth.at<float>(y,x) != 0 && GetNorm(cray) > img->dstDepth.at<float>(y,x))
// 								{
// 									continue;
// 								}
// 
// 
// 								matWeight.at<Vec4f>(y, x)[i] = kAngleWeight * fAngleWeight + (1 - kAngleWeight) * fDisWeigh;
// 
// 								//本来应该是双线性插值 ,目前先用最邻近法
// 								img->dst.at<Vec3b>(y, x) = img->src.at<Vec3b>(iSrcPoint);
// 								img->dstDepth.at<float>(y, x) = GetNorm(cray);
// 
// 							}
// 
// 						}
// 					}
// 				}
// 			}
// 
// 		}

	}
	for (int k = 0; k < vThreads.size(); k++)
	{
		vThreads[k]->start();
		if (bWarpOut)
		{
			*warpText << QString::number(k) << " start" << endl;
		}
	}
	for (int k = 0; k < vThreads.size();k++)
	{
		vThreads[k]->wait();
		if (bWarpOut)
		{
			*warpText << QString::number(k) << " end" << endl;
		}
	}
	for (int k = 0; k < vThreads.size(); k++)
	{
		delete vThreads[k];
	}

	//目前权值图其实是cost图，需要改造成权值图
	for (int m = 0; m < matWeight.rows; m++)
	{
		for (int n = 0; n < matWeight.cols; n++)
		{
			Vec4f vec = matWeight.at<Vec4f>(m, n);


			//数组，int表示id，float表示cost
			vector<pair<int, float>> vIF;


			for (int i = 0; i < 4; i++)
			{
				if (vec[i] == -1)
				{
					matWeight.at<Vec4f>(m, n)[i] = 0;
				}
				else
				{
					vIF.push_back(pair<int, float>(i, vec[i]));
				}
			}
			if (vIF.size() != 0)
			{
				qSort(vIF.begin(), vIF.end(), pairIFDescenf);
				vector<pair<int, float>> vIFWeight = vIF;

				//根据超像素连接图，判断权值调整关系
				array<float, 2> aFactor = { 1.0, 1.0 };
				if (vIF.size() > 1)
				{
					//数组第一个数为img编号，第二个为sp编号
					int i1, j1, i2, j2;
					i1 = vSelectedImg[vIF[0].first]->Index;
					i2 = vSelectedImg[vIF[1].first]->Index;
					j1 = vSelectedImg[vIF[0].first]->labelMat.at<int>(pixelCorrespond[m][n].at(vIF[0].first));
					j2 = vSelectedImg[vIF[1].first]->labelMat.at<int>(pixelCorrespond[m][n].at(vIF[1].first));

					if ((myGraph->FindEdge(i1, j1, i2, j2)).first)
					{
						;
					}
					else
					{
						//深度较深的超像素加权
						if (vImgInfo[i1]->vSP[j1].getNovalDepth(camera) < vImgInfo[i2]->vSP[j2].getNovalDepth(camera))
						{
							aFactor[0] *= 2;
						}
						else
						{
							aFactor[1] *= 2;
						}

						//非插值超像素 加权
						if (vImgInfo[i1]->vSP[j1].bOrigin)
						{
							aFactor[0] *= 2;
						}
						if (vImgInfo[i2]->vSP[j2].bOrigin)
						{
							aFactor[1] *= 2;
						}

					}

				}


				//其实这里可以用switch，算了
				if (vIF.size() == 4)
				{
					vIFWeight[0].second = 1 - vIF[0].second / vIF[2].second;
					vIFWeight[1].second = 1 - vIF[1].second / vIF[2].second;

					vIFWeight[0].second *= aFactor[0];
					vIFWeight[0].second *= aFactor[1];

					float total = vIFWeight[0].second + vIFWeight[1].second;

					vIFWeight[0].second /= total;
					vIFWeight[1].second /= total;

					matWeight.at<Vec4f>(m, n)[vIF[0].first] = vIFWeight[0].second;
					matWeight.at<Vec4f>(m, n)[vIF[1].first] = vIFWeight[1].second;

					matWeight.at<Vec4f>(m, n)[vIF[2].first] = 0;
					matWeight.at<Vec4f>(m, n)[vIF[3].first] = 0;
				}
				if (vIF.size() == 3)
				{
					vIFWeight[0].second = 1 - vIF[0].second / vIF[2].second;
					vIFWeight[1].second = 1 - vIF[1].second / vIF[2].second;

					vIFWeight[0].second *= aFactor[0];
					vIFWeight[0].second *= aFactor[1];

					float total = vIFWeight[0].second + vIFWeight[1].second;

					vIFWeight[0].second /= total;
					vIFWeight[1].second /= total;

					matWeight.at<Vec4f>(m, n)[vIF[0].first] = vIFWeight[0].second;
					matWeight.at<Vec4f>(m, n)[vIF[1].first] = vIFWeight[1].second;

					matWeight.at<Vec4f>(m, n)[vIF[2].first] = 0;
				}
				if (vIF.size() == 2)
				{
					vIFWeight[0].second = 1 - vIF[0].second / (vIF[1].second + vIF[0].second);
					vIFWeight[1].second = 1 - vIF[1].second / (vIF[1].second + vIF[0].second);

					vIFWeight[0].second *= aFactor[0];
					vIFWeight[0].second *= aFactor[1];

					float total = vIFWeight[0].second + vIFWeight[1].second;

					vIFWeight[0].second /= total;
					vIFWeight[1].second /= total;


					matWeight.at<Vec4f>(m, n)[vIF[0].first] = vIFWeight[0].second;
					matWeight.at<Vec4f>(m, n)[vIF[1].first] = vIFWeight[1].second;
				}
				if (vIF.size() == 1)
				{
					matWeight.at<Vec4f>(m, n)[vIF[0].first] = 1;
				}
			}



			// 			map<float, int> curMap;
			// 
			// 			int nCount = 4;
			// 			for (int i = 0; i < 4; i++)
			// 			{
			// 				vIF.push_back(pair<int, float>(i, vec[i]));
			// 			}
			// 			qSort(vIF.begin(), vIF.end(), pairIFAcsend);
			// 
			// 			for (int i = 0; i < 4; i++)
			// 			{
			// 				if (vIF[i].second == 0)
			// 				{
			// 					nCount = i;
			// 					break;
			// 				}
			// 			}
			// 			if (nCount >= 3)
			// 			{
			// 				vIF[0].second = 1 - vIF[0].second / vIF[2].second;
			// 				vIF[1].second = 1 - vIF[1].second / vIF[2].second;
			// 
			// 				vIF[0].second /= vIF[0].second + vIF[1].second;
			// 				vIF[1].second /= vIF[0].second + vIF[1].second;
			// 
			// 				matWeight.at<Vec4f>(m, n)[vIF[0].first] = vIF[0].second;
			// 				matWeight.at<Vec4f>(m, n)[vIF[1].first] = vIF[1].second;
			// 
			// 				matWeight.at<Vec4f>(m, n)[vIF[2].first] = 0;
			// 				matWeight.at<Vec4f>(m, n)[vIF[3].first] = 0;
			// 			}
			// 			if (nCount == 2)
			// 			{
			// 				vIF[0].second = 1 - vIF[0].second / vIF[2].second;
			// 				vIF[1].second = 1 - vIF[1].second / vIF[2].second;
			// 
			// 				vIF[0].second /= vIF[0].second + vIF[1].second;
			// 				vIF[1].second /= vIF[0].second + vIF[1].second;
			// 
			// 				matWeight.at<Vec4f>(m, n)[vIF[0].first] = vIF[0].second;
			// 				matWeight.at<Vec4f>(m, n)[vIF[1].first] = vIF[1].second;
			// 
			// 				matWeight.at<Vec4f>(m, n)[vIF[2].first] = 0;
			// 				matWeight.at<Vec4f>(m, n)[vIF[3].first] = 0;
			// 
			// 			}
			// 
			// 			auto it = curMap.rbegin();
			// 
			// 			float fThres = (++it)->first;
			// 
			// 			float fWeight1 = 1 - (++it)->first / fThres;
			// 			float fWeight2 = 1 - (++it)->first / fThres;
			// 
			// 			fWeight1 /= fWeight1 + fWeight2;
			// 			fWeight2 /= fWeight1 + fWeight2;
			// 
			// 			matWeight.at<Vec4f>(m, n)[it->second] = fWeight1;
			// 			matWeight.at<Vec4f>(m, n)[(--it)->second] = fWeight2;
			// 			matWeight.at<Vec4f>(m, n)[(--it)->second] = 0;
			// 			matWeight.at<Vec4f>(m, n)[(--it)->second] = 0;
			// 
		}
	}

	//合成新视角！！！
	Mat matNoval = Mat(vSrc[0].rows, vSrc[0].cols, CV_8UC3, Scalar(0, 0, 0));

	for (int m = 0; m < matNoval.rows; m++)
	{
		for (int n = 0; n < matNoval.cols; n++)
		{
			Vec3b vec(0, 0, 0);
			for (int i = 0; i < vSelectedImg.size(); i++)
			{
				vec += vSelectedImg[i]->dst.at<Vec3b>(m, n) * matWeight.at<Vec4f>(m, n)[i];
			}

			matNoval.at<Vec3b>(m, n) = vec;
		}
	}

	cvtColor(matNoval, matNoval, CV_RGB2BGR);

	if (bOutPut)
	{
		imshow("NovalView!", matNoval);

		QString sFilePath = QDir::currentPath() + "/doc/" + "NovalView.jpg";
		imwrite(sFilePath.toStdString(), matNoval);

		//调试用
		for (int i = 0; i < vSelectedImg.size(); i++)
		{
			stringstream ss;
			ss << vSelectedImg[i]->Index;
			string s;
			ss >> s;

			cvtColor(vSelectedImg[i]->dst, vSelectedImg[i]->dst, CV_RGB2BGR);
			imshow(s, vSelectedImg[i]->dst);

			QString sfilePath = QDir::currentPath() + "/doc/" + "Noval" + QString::fromStdString(s) + ".jpg";
			imwrite(sfilePath.toStdString(), vSelectedImg[i]->dst);
		}

	}

	return matNoval;

}

void DepthSynWidget::getSkyLabel()
{
	for (int i = 0; i < vSrc.size(); i++)
	{
		map<int, char> mLabelType;// A为图片上端，没有深度信息；B为没有深度信息；C为有深度信息
		Mat LabelMat = vLabelMat[i].clone();

		typedef Graph<int, int, int> GraphType;
		GraphType *myGraph = new GraphType(LabelMat.cols * LabelMat.rows, 2 * LabelMat.cols * LabelMat.rows - LabelMat.cols - LabelMat.rows);
		int nFlow;	//最大流，然而这里没有用

		for (int m = 0; m < LabelMat.rows; m++)
		{
			for (int n = 0; n < LabelMat.cols; n++)
			{
				map<int, char>::iterator it;
				it = mLabelType.find(LabelMat.at<int>(m, n));
				char cCurType;

				if (it == mLabelType.end())
				{
					if (vvLabelPointCount[i].at(LabelMat.at<int>(m, n)) != 0)
					{
						mLabelType[LabelMat.at<int>(m, n)] = 'C';
						cCurType = 'C';
					}
					else if (m == 0)
					{
						mLabelType[LabelMat.at<int>(m, n)] = 'A';
						cCurType = 'A';
					}
					else
					{
						mLabelType[LabelMat.at<int>(m, n)] = 'B';
						cCurType = 'B';
					}

				}
				else
				{
					cCurType = it->second;
				}

				//得到当前像素点对应类型，开始构建节点和权值（为了避免重复，每一个像素点都和左、上 像素构建边权值）
				myGraph->add_node();

				//构建T link
				switch (cCurType)
				{
				case 'A':
					myGraph->add_tweights(m * LabelMat.cols + n, 0, 9000000);
					break;

				case 'B':
					myGraph->add_tweights(m * LabelMat.cols + n, 1, 0);
					break;

				case 'C':
					myGraph->add_tweights(m * LabelMat.cols + n, 9000000, 1);
					break;

				default:
					break;
				}

				//构建 N link
				if (n != 0)//和左边的像素构建边
				{
					if (LabelMat.at<int>(m, n) == LabelMat.at<int>(m, n - 1))
					{
						myGraph->add_edge(m * LabelMat.cols + n, m * LabelMat.cols + n - 1, 1000000, 1000000);
					}
					else
					{
						myGraph->add_edge(m * LabelMat.cols + n, m * LabelMat.cols + n - 1, 100, 100);
					}
				}

				if (m != 0)//和右边的像素构建边
				{
					if (LabelMat.at<int>(m, n) == LabelMat.at<int>(m - 1, n))
					{
						myGraph->add_edge(m * LabelMat.cols + n, (m - 1)* LabelMat.cols + n, 1000000, 1000000);
					}
					else
					{
						myGraph->add_edge(m * LabelMat.cols + n, (m - 1) * LabelMat.cols + n, 100, 100);
					}

				}

			}
		}

		nFlow = myGraph->maxflow();

		//构建天空划分label图，构建判断labelmat中label是否为天空的mat，最后推入vector中
		Mat_<int> matSky(LabelMat.rows, LabelMat.cols);
		vector<int> vSky(m_spOpenCVFrame->m_spSLIC->vNumSP[i]);

		for (int m = 0; m < LabelMat.rows; m++)
		{
			for (int n = 0; n < LabelMat.cols; n++)
			{
				int nLabel = myGraph->what_segment(m * LabelMat.cols + n);
				matSky.at<int>(m, n) = nLabel;

				//有没有可能，一个SP里面 部分是天空，部分没有认定是天空？
				vSky[LabelMat.at<int>(m, n)] |= nLabel;
			}
		}

		for (int j = 0; j < vSky.size(); j++)
		{
			if (vSky[j])
			{
				vector<int>::iterator it1;
				for (it1 = vvTargetSPLabel[i].begin(); it1 != vvTargetSPLabel[i].end(); it1++)
				{
					if ((*it1) == j)
					{
						vvTargetSPLabel[i].erase(it1);
						break;
					}
				}

				map<int, vector<int>>::iterator it2;
				it2 = vmapImgTargetNearLabel[i].find(j);
				vmapImgTargetNearLabel[i].erase(it2);
			}
		}

		vvImgSkyLabel.push_back(vSky);
		vMatSky.push_back(matSky);

		delete myGraph;
	}
}

void DepthSynWidget::setSkyDepth()
{
	for (int i = 0; i < vSrc.size(); i++)
	{
		//找到目前图片中的最大深度，然后用最大深度乘以1000 作为天空深度（非常远）
		vector<DepthPoint> vDepthPoint(vvDepthPoint[i]);

		//如果这张图一个深度点都没有，那就不用玩了！
		if (vDepthPoint.size() == 0)
		{
			continue;
		}


		float fMatDepth = vDepthPoint.at(0).fDepth;
		for (int m = 0; m < vDepthPoint.size(); m++)
		{
			if (vDepthPoint[m].fDepth > fMatDepth)
			{
				fMatDepth = vDepthPoint[m].fDepth;
			}
		}

		//如果最深深度为 正，则直接乘以1000，如果为负，则直接加上1000
		if (fMatDepth > 0)
		{
			fMatDepth *= 1000;
		}
		else
		{
			fMatDepth += 1000;
		}



		//开始对天空SP中的随机像素进行深度赋值
		RNG rng;
		map<int, vector<int>> mapSkyLabelHash;
		map<int, int> mapSkyLabelIndex;
		for (int m = 0; m < vvImgSkyLabel[i].size(); m++)
		{
			if (vvImgSkyLabel[i][m] == 1)
			{
				vector<int> vHash(m_spOpenCVFrame->m_spSLIC->vvImageLabelPixelCount[i][m], 0);
				mapSkyLabelHash.insert(pair<int, vector<int>>(m, vHash));
				mapSkyLabelIndex.insert(pair<int, int>(m, 0));

				//开始随机选取像素进行赋值
				assert(vHash.size() > 15);
				int  nCountDown = (vHash.size() / 200 > 15) ? (vHash.size() / 200) : 15;//剩余需要赋值的像素数目

				int nCount = 0;//已经赋值的像素数目

				while (nCountDown)
				{
					int index;	//要赋值的像素下标
					index = rng.uniform(0, vHash.size());
					if (mapSkyLabelHash[m][index])
					{
						continue;
					}
					else
					{
						mapSkyLabelHash[m][index] = 1;
						++nCount;
						--nCountDown;
					}
				}


			}
		}

		map<int, vector<int>>::iterator it;
		for (int m = 0; m < vLabelMat[i].rows; m++)
		{
			for (int n = 0; n < vLabelMat[i].cols; n++)
			{
				it = mapSkyLabelHash.find(vLabelMat[i].at<int>(m, n));

				if (it != mapSkyLabelHash.end())
				{
					if ((it->second)[mapSkyLabelIndex[it->first]])
					{
						DepthPoint curDepthPoint;
						curDepthPoint.point = Point(n, m);
						curDepthPoint.fDepth = fMatDepth;
						vvDepthPoint[i].push_back(curDepthPoint);

						//新的数据结构 sample point
						SamplePoint *sample = new SamplePoint();
						sample->Point2D = Point2i(n, m);
						sample->fDepth = fMatDepth;
						sample->bOrigin = false;
						sample->BackProjection(vSrc[i], vCamera[i]);
						vvSamplePoint[i].push_back(sample);




						vDepthMat[i].at<Vec<float, 3>>(m, n) = { (float)vLabelMat[i].at<int>(m, n), fMatDepth, 0 };


					}
					mapSkyLabelIndex[it->first]++;

				}
			}
		}


	}
}

void DepthSynWidget::writeSkyImg()
{
	QString sPath = QDir::currentPath() + "/doc/";
	for (int i = 0; i < vMatSky.size(); i++)
	{
		Mat matBlendMask = vSrc.at(i).clone();
		for (int m = 0; m < matBlendMask.rows; m++)
		{
			for (int n = 0; n < matBlendMask.cols; n++)
			{
				if (vMatSky.at(i).at<int>(m, n))
				{
					matBlendMask.at<Vec<uchar, 3>>(m, n) = { 0, 0, 0 };
				}
				else
				{
					matBlendMask.at<Vec<uchar, 3>>(m, n) = { 0, 0, 255 };
				}

			}
		}
		addWeighted(vSrc[i], 0.6, matBlendMask, 0.4, 0.0, matBlendMask);
		QString sFilePath = sPath + "SkyRegion" + QString::number(i) + ".jpg";
		imwrite(sFilePath.toStdString(), matBlendMask);
	}

}

void DepthSynWidget::viewLocalWarp(Mat matCamera, Mat R, Mat t)
{

}

void DepthSynWidget::warpInitialize(float fDeviation)
{
	//初始化权值图（4）,默认noval view的视窗 和 第一张原图大小相同

	for (int i = 0; i < vSrc.size(); i++)
	{
		ImgInfo *img = new ImgInfo(this);
		img->vSP.resize(vvLabelPointCount[i].size());
		img->depthMat = vDepthMat[i];
		img->labelMat = vLabelMat[i];
		img->src = vSrc[i];
		img->camera = vCamera[i];
		img->Index = i;
		img->dst = Mat(img->src.rows, img->src.cols, CV_8UC3, Scalar(0, 0, 0));
		img->dstDepth = Mat(img->src.rows, img->src.cols, CV_32FC1, 0.0);


		//深度点都很少的，就没必要玩
		if (vvSamplePoint[i].size() < 100)
		{
			img->bValid = false;
			vImgInfo.push_back(img);
			continue;
		}

		//在图片的左上边缘 一个像素宽度，应该不会出现这么细的超像素吧

		for (int m = 0; m < vDepthMat[i].rows; m++)
		{
			for (int n = 0; n < vDepthMat[i].cols; n++)
			{
				int nLabel = img->labelMat.at<int>(m, n);

				//SP的第一个像素，初始化SP的boundingbox 以及 SP的fMedianDepth
				if (img->vSP[nLabel].listPoint.size() == 0)
				{
					img->vSP[nLabel].iImgIndex = i;
					img->vSP[nLabel].m_spParentImg = img;
					img->vSP[nLabel].boundingRect.x = n;
					img->vSP[nLabel].boundingRect.y = m;
					img->vSP[nLabel].boundingRect.width = 1;
					img->vSP[nLabel].boundingRect.height = 1;

					if (vmapImgSPDepth[i].find(nLabel) != vmapImgSPDepth[i].end())
					{
						for (int k = 0; k < vmapImgSPDepth[i][nLabel].size(); k++)
						{
							img->vSP[nLabel].vDepth.push_back(vmapImgSPDepth[i][nLabel][k].fDepth);
						}
						img->vSP[nLabel].fMedianDepth = vmapImgSPDepth[i][nLabel][vmapImgSPDepth[i][nLabel].size() / 2].fDepth;

					}
					else
					{
						img->vSP[nLabel].bHole = true;
					}
				}
				else
				{
					//boundingbox 修正
					if (n < img->vSP[nLabel].boundingRect.x)
					{
						img->vSP[nLabel].boundingRect.width += (img->vSP[nLabel].boundingRect.x - n);
						img->vSP[nLabel].boundingRect.x = n;

					}
					else if (n >= img->vSP[nLabel].boundingRect.x + img->vSP[nLabel].boundingRect.width)
					{
						img->vSP[nLabel].boundingRect.width = n - img->vSP[nLabel].boundingRect.x + 1;
					}

					if (m < img->vSP[nLabel].boundingRect.y)
					{
						img->vSP[nLabel].boundingRect.height += (img->vSP[nLabel].boundingRect.y - m);
						img->vSP[nLabel].boundingRect.y = m;
					}
					else if (m >= img->vSP[nLabel].boundingRect.y + img->vSP[nLabel].boundingRect.height)
					{
						img->vSP[nLabel].boundingRect.height = m - img->vSP[nLabel].boundingRect.y + 1;
					}
				}

				//将像素信息推入到list当中
				img->vSP[nLabel].listPoint.push_back(Point2i(n, m));
// 				if (m == 0 || n == 0 || m == img->src.rows - 1 || n == img->src.cols - 1)
// 				{
// 					img->vSP[nLabel].bHole = true;
// 				}

				if (m != 0 && n != 0)
				{
					int nLeftLabel = img->labelMat.at<int>(m, n - 1);
					int nTopLabel = img->labelMat.at<int>(m - 1, n);

					if (nLeftLabel != nLabel && img->vSP[nLeftLabel].fMedianDepth != -1)
					{
						if (fabs((img->vSP[nLabel].fMedianDepth - img->vSP[nLeftLabel].fMedianDepth) / (img->vSP[nLabel].fMedianDepth)) <= fDeviation)
						{
							img->vSP[nLabel].bExtension = true;
							img->vSP[nLeftLabel].bExtension = true;
						}
					}

					if (nTopLabel != nLabel && img->vSP[nTopLabel].fMedianDepth != -1)
					{
						if (fabs((img->vSP[nLabel].fMedianDepth - img->vSP[nTopLabel].fMedianDepth) / (img->vSP[nLabel].fMedianDepth)) <= fDeviation)
						{
							img->vSP[nLabel].bExtension = true;
							img->vSP[nTopLabel].bExtension = true;
						}
					}



				}



			}
		}
		for (int k = 0; k < vvSamplePoint[i].size(); k++)
		{
			img->vSP[img->labelMat.at<int>(vvSamplePoint[i][k]->Point2D.y, vvSamplePoint[i][k]->Point2D.x)].vpSamples.push_back(vvSamplePoint[i][k]);
			if (vvSamplePoint[i][k]->bOrigin)
			{
				vvSamplePoint[i][k]->Point3D->ConnectImgSp[i] = img->labelMat.at<int>(vvSamplePoint[i][k]->Point2D.y, vvSamplePoint[i][k]->Point2D.x);
			}
		}

		//对于深度插值之后仍然是target的sp，标记为hole
		for (int j = 0; j < vvTargetSPLabel[i].size(); j++)
		{
			img->vSP[vvTargetSPLabel[i][j]].bHole = true;
		}

		//对于原本就缺乏深度信息的sp，标记为非origin
		for (int j = 0; j < vvNoneOriginSPLabel[i].size(); j++)
		{
			img->vSP[vvNoneOriginSPLabel[i][j]].bOrigin = false;
		}

		vImgInfo.push_back(img);
	}



	//构建超像素之间的链接关系（图）


}

void DepthSynWidget::SpWarpLinearSystem(SuperPixel &sp, int iVertexDensity, int iExtPixel)
{
	//预处理
	//构建每一SP对应的小mask mat 以及其 sample？
	sp.matSP = Mat(sp.boundingRect.height, sp.boundingRect.width, CV_8UC1, Scalar(0));
	list<Point2i>::iterator it;
	for (it = sp.listPoint.begin(); it != sp.listPoint.end(); it++)
	{
		sp.matSP.at<uchar>(it->y - sp.boundingRect.y, it->x - sp.boundingRect.x) = 255;
	}



	if (sp.bExtension)
	{
		sp.ExtBounding.x = (sp.boundingRect.x - iExtPixel >= 0) ? (sp.boundingRect.x - iExtPixel) : 0;
		sp.ExtBounding.y = (sp.boundingRect.y - iExtPixel >= 0) ? (sp.boundingRect.y - iExtPixel) : 0;
		sp.ExtBounding.width = (sp.boundingRect.width + 4 + sp.boundingRect.x - 1 < sp.m_spParentImg->depthMat.cols) ? (sp.boundingRect.width + 4 + sp.boundingRect.x - sp.ExtBounding.x) : (sp.m_spParentImg->depthMat.cols - sp.boundingRect.x + sp.boundingRect.x - sp.ExtBounding.x);
		sp.ExtBounding.height = (sp.boundingRect.height + 4 + sp.boundingRect.y - 1 < sp.m_spParentImg->depthMat.rows) ? (sp.boundingRect.height + 4 + sp.boundingRect.y - sp.ExtBounding.y) : (sp.m_spParentImg->depthMat.rows - sp.boundingRect.y + sp.boundingRect.y - sp.ExtBounding.y);

		sp.matExtSP = Mat(sp.ExtBounding.height, sp.ExtBounding.width, CV_8UC1, Scalar(0));

		Mat Area = sp.matExtSP.colRange(sp.boundingRect.x - sp.ExtBounding.x, sp.boundingRect.width + sp.boundingRect.x - sp.ExtBounding.x).rowRange(sp.boundingRect.y - sp.ExtBounding.y, sp.boundingRect.height + sp.boundingRect.y - sp.ExtBounding.y);

		for (int m = 0; m < sp.matSP.rows; m++)
		{
			for (int n = 0; n < sp.matSP.cols; n++)
			{
				Area.at<uchar>(m, n) = sp.matSP.at<uchar>(m, n);
			}
		}



		//调试用
		// 		QString sPath = QDir::currentPath() + "/doc/";
		// 		QString sArea = sPath + "Area.jpg";
		// 		QString sExt = sPath + "Ext.jpg";
		// 		QString sExtDilate = sPath + "dilate.jpg";
		// 		imwrite(sArea.toStdString(), Area);
		// 		imwrite(sExt.toStdString(), sp.matExtSP);
		// 

		dilate(sp.matExtSP, sp.matExtSP, Mat(CV_8UC1, 2 * iExtPixel + 1, 2 * iExtPixel + 1), Point2i(-1, -1));


		// 		imwrite(sExtDilate.toStdString(), sp.matExtSP);

		for (int m = 0; m < sp.matExtSP.rows; m++)
		{
			for (int n = 0; n < sp.matExtSP.cols; n++)
			{
				if (sp.matExtSP.at<uchar>(m, n) > 0)
				{
					sp.listExtPoint.push_back(Point2i(n + sp.ExtBounding.x, m + sp.ExtBounding.y));
				}
			}
		}
	}

	//给sp覆盖上grid，同时计算每一个三角中的sample point
	SpOverlayGrid(sp, iVertexDensity);

	//计算线性方程组
	sp.getLinearSys();


}

void DepthSynWidget::SpOverlayGrid(SuperPixel &sp, int iVertexDensity)
{
	Rect rectBounding;
	if (sp.bExtension)
	{
		rectBounding = sp.ExtBounding;
	}
	else
	{
		rectBounding = sp.boundingRect;
	}

	//隔段的数目，断点数目 + 1
	int nGridWidth = rectBounding.width / iVertexDensity + 1;
	int nGridHeight = rectBounding.height / iVertexDensity + 1;

	float fXstep = (rectBounding.width - 1) / nGridWidth;
	float fYstep = (rectBounding.height - 1) / nGridHeight;

	sp.nXNum = nGridWidth + 1;
	sp.nYNum = nGridHeight + 1;
	sp.fXStep = fXstep;
	sp.fYStep = fYstep;

	int nVertex = 0;
	int nTri = 0;
	for (int x = 0; x <= nGridWidth; x++)
	{
		for (int y = 0; y <= nGridHeight; y++)
		{
			TriVertex* curTriVertex = new TriVertex(Point2i(rectBounding.x + x * fXstep, rectBounding.y + y * fYstep), nVertex);
			sp.vVertexpoint.push_back(curTriVertex);

			if (x != 0)
			{
				if (y != 0)
				{
					vector<int> vTriIndex;
					vTriIndex.push_back(nVertex);
					vTriIndex.push_back(nVertex - 1);
					vTriIndex.push_back(nVertex - nGridHeight - 1);
					sp.vTriangleIndex.push_back(vTriIndex);

					GridTriangle curTri(&sp);
					curTri.vVertex.push_back(sp.vVertexpoint[nVertex]);
					curTri.vVertex.push_back(sp.vVertexpoint[nVertex - 1]);
					curTri.vVertex.push_back(sp.vVertexpoint[nVertex - nGridHeight - 1]);
					sp.vTriangle.push_back(curTri);

					sp.vVertexpoint.at(nVertex)->parentTri.push_back(nTri);
					sp.vVertexpoint.at(nVertex - 1)->parentTri.push_back(nTri);
					sp.vVertexpoint.at(nVertex - nGridHeight - 1)->parentTri.push_back(nTri);


					++nTri;

				}
				if (y != nGridHeight)
				{
					vector<int> vTriIndex;
					vTriIndex.push_back(nVertex);
					vTriIndex.push_back(nVertex - nGridHeight - 1);
					vTriIndex.push_back(nVertex - nGridHeight);
					sp.vTriangleIndex.push_back(vTriIndex);

					GridTriangle curTri(&sp);
					curTri.vVertex.push_back(sp.vVertexpoint[nVertex]);
					curTri.vVertex.push_back(sp.vVertexpoint[nVertex - nGridHeight - 1]);
					curTri.vVertex.push_back(sp.vVertexpoint[nVertex - nGridHeight]);
					sp.vTriangle.push_back(curTri);

					sp.vVertexpoint.at(nVertex)->parentTri.push_back(nTri);
					sp.vVertexpoint.at(nVertex - nGridHeight - 1)->parentTri.push_back(nTri);
					sp.vVertexpoint.at(nVertex - nGridHeight)->parentTri.push_back(nTri);

					++nTri;

				}
			}

			nVertex++;
		}
	}

	sp.nVertexNum = nVertex;



	//三角结构里面的samplepoint
	for (int i = 0; i < sp.vpSamples.size(); i++)
	{

		float fX_index = (sp.vpSamples[i]->Point2D.x - rectBounding.x) / fXstep;
		float fY_index = (sp.vpSamples[i]->Point2D.y - rectBounding.y) / fYstep;


		//当样本点正好在网格的右、下 边缘时
		if (fY_index >= sp.nYNum - 1)
		{
			fY_index -= 1;
		}
		if (fX_index >= sp.nXNum - 1)
		{
			fX_index -= 1;
		}




		int nTriIndex = nGridHeight * 2 * int(fX_index) + 2 * int(fY_index);

		if ((fX_index - int(fX_index)) < (fY_index - int(fY_index)))
		{
			++nTriIndex;
		}

		sp.vTriangle[nTriIndex].vpSamplePoints.push_back(sp.vpSamples[i]);

	}

	//有可能三角形中 深度点的数目小于3 ，数目为0的从外界借，数目不为0 则内部引用
	for (int i = 0; i < sp.vTriangle.size(); i++)
	{
		if (sp.vTriangle[i].vpSamplePoints.size() < 3)
		{
			float fDepth;
			if (sp.vTriangle[i].vpSamplePoints.size() == 0)
			{
				Point2i pointV(sp.vTriangle[i].vVertex[0]->vertex);
				float fDis = 0;
				bool bExist = false;
				for (int k = 0; k < sp.vpSamples.size(); k++)
				{
					if (!sp.bHole)
					{
						bExist = true;
						if (fDis == 0)
						{
							fDis = (sp.vpSamples[k]->Point2D.x - pointV.x) * (sp.vpSamples[k]->Point2D.x - pointV.x) + (sp.vpSamples[k]->Point2D.y - pointV.y) * (sp.vpSamples[k]->Point2D.y - pointV.y);
							fDepth = sp.vpSamples[k]->fDepth;
						}
						else if ((sp.vpSamples[k]->Point2D.x - pointV.x) * (sp.vpSamples[k]->Point2D.x - pointV.x) + (sp.vpSamples[k]->Point2D.y - pointV.y) * (sp.vpSamples[k]->Point2D.y - pointV.y) < fDis)
						{
							fDis = (sp.vpSamples[k]->Point2D.x - pointV.x) * (sp.vpSamples[k]->Point2D.x - pointV.x) + (sp.vpSamples[k]->Point2D.y - pointV.y) * (sp.vpSamples[k]->Point2D.y - pointV.y);
							fDepth = sp.vpSamples[k]->fDepth;
						}
					}

					else if (fabsf(sp.vpSamples[k]->Point2D.x - pointV.x) < 2 * fXstep && fabsf(sp.vpSamples[k]->Point2D.y - pointV.y) < 2 * fYstep)
					{
						bExist = true;
						if (fDis = 0)
						{
							fDis = (sp.vpSamples[k]->Point2D.x - pointV.x) * (sp.vpSamples[k]->Point2D.x - pointV.x) + (sp.vpSamples[k]->Point2D.y - pointV.y) * (sp.vpSamples[k]->Point2D.y - pointV.y);
							fDepth = sp.vpSamples[k]->fDepth;
						}
						else if ((sp.vpSamples[k]->Point2D.x - pointV.x) * (sp.vpSamples[k]->Point2D.x - pointV.x) + (sp.vpSamples[k]->Point2D.y - pointV.y) * (sp.vpSamples[k]->Point2D.y - pointV.y) < fDis)
						{
							fDis = (sp.vpSamples[k]->Point2D.x - pointV.x) * (sp.vpSamples[k]->Point2D.x - pointV.x) + (sp.vpSamples[k]->Point2D.y - pointV.y) * (sp.vpSamples[k]->Point2D.y - pointV.y);
							fDepth = sp.vpSamples[k]->fDepth;
						}

					}


				}

				assert(bExist);
			}
			else
			{
				fDepth = sp.vTriangle[i].vpSamplePoints[0]->fDepth;
			}

			//把对应深度点插值到三角形中
			int x, y;
			SamplePoint *sam;

			x = sp.vTriangle[i].vVertex[0]->vertex.x / 2 + sp.vTriangle[i].vVertex[1]->vertex.x / 4 + sp.vTriangle[i].vVertex[2]->vertex.x / 4;
			y = sp.vTriangle[i].vVertex[0]->vertex.y / 2 + sp.vTriangle[i].vVertex[1]->vertex.y / 4 + sp.vTriangle[i].vVertex[2]->vertex.y / 4;
			sam = new SamplePoint();
			sam->Point2D = Point2i(x, y);
			sam->fDepth = fDepth;
			sam->bOrigin = false;
			sam->BackProjection(sp.m_spParentImg->src, sp.m_spParentImg->camera);
			sp.vTriangle[i].vpSamplePoints.push_back(sam);
			sp.vpSamples.push_back(sam);
			sp.m_spParentImg->m_spParentWidget->vvSamplePoint[sp.m_spParentImg->Index].push_back(sam);
			//sp里的sample信息已经足够多，暂且不必对vvsample进行补充？？


			x = sp.vTriangle[i].vVertex[0]->vertex.x / 4 + sp.vTriangle[i].vVertex[1]->vertex.x / 2 + sp.vTriangle[i].vVertex[2]->vertex.x / 4;
			y = sp.vTriangle[i].vVertex[0]->vertex.y / 4 + sp.vTriangle[i].vVertex[1]->vertex.y / 2 + sp.vTriangle[i].vVertex[2]->vertex.y / 4;
			sam = new SamplePoint();
			sam->Point2D = Point2i(x, y);
			sam->fDepth = fDepth;
			sam->bOrigin = false;
			sam->BackProjection(sp.m_spParentImg->src, sp.m_spParentImg->camera);
			sp.vTriangle[i].vpSamplePoints.push_back(sam);
			sp.vpSamples.push_back(sam);
			sp.m_spParentImg->m_spParentWidget->vvSamplePoint[sp.m_spParentImg->Index].push_back(sam);


			x = sp.vTriangle[i].vVertex[0]->vertex.x / 4 + sp.vTriangle[i].vVertex[1]->vertex.x / 4 + sp.vTriangle[i].vVertex[2]->vertex.x / 2;
			y = sp.vTriangle[i].vVertex[0]->vertex.y / 4 + sp.vTriangle[i].vVertex[1]->vertex.y / 4 + sp.vTriangle[i].vVertex[2]->vertex.y / 2;
			sam = new SamplePoint();
			sam->bOrigin = false;
			sam->Point2D = Point2i(x, y);
			sam->fDepth = fDepth;
			sam->BackProjection(sp.m_spParentImg->src, sp.m_spParentImg->camera);
			sp.vTriangle[i].vpSamplePoints.push_back(sam);
			sp.vpSamples.push_back(sam);
			sp.m_spParentImg->m_spParentWidget->vvSamplePoint[sp.m_spParentImg->Index].push_back(sam);



		}
	}




}

void DepthSynWidget::CalSPLinearSys(SuperPixel &sp)
{

}

void DepthSynWidget::GetKNovalNear(int k = 4)
{
	vSelectedImg.clear();
	vector<float> vNoval;
	for (int i = 0; i < 3; i++)
	{
		vNoval.push_back(this->camera->matRotation.at<float>(2, i));
	}

	map<float, int> mapAngleIndex;

	for (int i = 0; i < vCamera.size(); i++)
	{
		vector<float> vPriciple;
		for (int j = 0; j < 3; j++)
		{
			vPriciple.push_back(vCamera[i]->matRotation.at<float>(2, j));
		}

		mapAngleIndex.insert(pair<float, int>(GetAngleCos(vNoval, vPriciple), i));
	}

	auto it = mapAngleIndex.rbegin();
	for (int i = 0; i < k; i++)
	{
		vSelectedImg.push_back(vImgInfo[(*it++).second]);
	}
}

inline Vertex::Vertex() :nStart(0), nCount(0), next(NULL)
{
	;
}

Vertex::~Vertex()
{
	while (this->next)
	{
		Line *cur = this->next;
		this->next = cur->next;
		delete cur;
	}
}

void Vertex::PushBack(Line* pLine)
{
	if (next == NULL)
	{
		next = pLine;
		return;
	}
	Line* curLine = next;
	while (curLine->next != NULL)
	{
		curLine = curLine->next;
	}
	curLine->next = pLine;
}

SuperPixel::SuperPixel() :fMedianDepth(-1), bExtension(false), bHole(false), bOrigin(true), bNovalDepth(false)
{

}

SuperPixel::~SuperPixel()
{

}

void SuperPixel::getLinearSys()
{
	LinearSystem LinSys;
	//未知数实际上是 顶点数 的两倍，因为每个顶点 有 横纵坐标
	for (int m = 0; m < 2 * nVertexNum; m++)
	{
		vector<double> v(2 * nVertexNum);

		LinSys.A.push_back(v);

		LinSys.B.push_back(0);

		list<pair<Point3f*, float>> ListInfo;
		LinSys.BInfo.push_back(ListInfo);
	}


	// 	for (int i = 0; i < vVertexpoint.size();i++)
	// 	{
	// // 		int x = i / nYNum;
	// // 		int y = i % nYNum;
	// // 
	// // 		//每一个节点可能对应六个三角形 包含有该节点的Ep
	// // 		if (x!= 0)
	// // 		{
	// // 			if (y!= 0)
	// // 			{
	// // 				int nTriIndex =(x - 1) * 2 * (nYNum - 1 ) +  2 * y - 1;
	// // 			}
	// // 
	// // 			if (y != nYNum - 1)
	// // 			{
	// // 				int nTriIndex1 = (x - 1) * 2 * (nYNum - 1) + 2 * y;
	// // 				int nTriIndex2 = (x - 1) * 2 * (nYNum - 1) + 2 * y + 1;
	// // 			}
	// // 		}
	// // 
	// // 		if (x != nXNum - 1)
	// // 		{
	// // 			if (y != 0)
	// // 			{
	// // 				int nTriIndex1 = x * 2 * (nYNum - 1) + 2 * y - 2;
	// // 				int nTriIndex2 = x * 2 * (nYNum - 1) + 2 * y - 1;
	// // 
	// // 			}
	// // 
	// // 			if (y != nYNum - 1)
	// // 			{
	// // 				int nTriIndex = x * 2 * (nYNum - 1) + 2 * y;
	// // 			}
	// // 		}
	// 
	// 
	// 
	// 		auto it = vVertexpoint[i].parentTri.begin();
	// 		for (; it != vVertexpoint[i].parentTri.end();it++)
	// 		{
	// 			int nTriIndex = *it;
	// 			GridTriangle tri = vTriangle[nTriIndex];
	// 
	// 			//开始准备线性方程
	// 			for (int i = 0; i < tri.vSamplePoints.size();i++)
	// 			{
	// 				//投影点
	// 				Point2i p = tri.vSamplePoints[i]->Point2D;
	// 
	// 				//三角形顶点
	// 				TriVertex *v1 = tri.vVertex[0];
	// 				TriVertex *v2 = tri.vVertex[1];
	// 				TriVertex *v3 = tri.vVertex[2];
	// 
	// 
	// 				//获取 重心坐标系统 的坐标, int 对应 顶点编号，float对应坐标
	// 				pair<int, float> IndexCor1;
	// 				pair<int, float> IndexCor2;
	// 				pair<int, float> IndexCor3;
	// 
	// 				IndexCor1.first = v1->nIndex;
	// 				IndexCor2.first = v2->nIndex; 
	// 				IndexCor3.first = v3->nIndex;
	// 
	// 				IndexCor1.second = float((v2->vertex.y - v3->vertex.y) * (p.x - v3->vertex.x) + (v3->vertex.x - v2->vertex.x) * (p.y - v3->vertex.y)) / float((v2->vertex.y - v3->vertex.y) * (v1->vertex.x - v3->vertex.x) + (v3->vertex.x - v2->vertex.x) * (v1->vertex.y - v3->vertex.y));
	// 				IndexCor2.second = float((v3->vertex.y - v1->vertex.y) * (p.x - v3->vertex.x) + (v1->vertex.x - v3->vertex.x) * (p.y - v3->vertex.y)) / float((v2->vertex.y - v3->vertex.y) * (v1->vertex.x - v3->vertex.x) + (v3->vertex.x - v2->vertex.x) * (v1->vertex.y - v3->vertex.y));
	// 				IndexCor3.second = 1 - IndexCor1.second - IndexCor2.second;
	// 				
	// 
	// 				//妈的，好像有规律可以直接用循环的
	// 
	// 				LinSys.A[2 * IndexCor1.first][2 * IndexCor1.first] += IndexCor1.second * IndexCor1.second;
	// 				LinSys.A[2 * IndexCor1.first][2 * IndexCor2.first] += IndexCor1.second * IndexCor2.second;
	// 				LinSys.A[2 * IndexCor1.first][2 * IndexCor3.first] += IndexCor1.second * IndexCor3.second;
	// 
	// 				LinSys.A[2 * IndexCor2.first][2 * IndexCor1.first] += IndexCor2.second * IndexCor1.second;
	// 				LinSys.A[2 * IndexCor2.first][2 * IndexCor2.first] += IndexCor2.second * IndexCor2.second;
	// 				LinSys.A[2 * IndexCor2.first][2 * IndexCor3.first] += IndexCor2.second * IndexCor3.second;
	// 
	// 				LinSys.A[2 * IndexCor3.first][2 * IndexCor1.first] += IndexCor3.second * IndexCor1.second;
	// 				LinSys.A[2 * IndexCor3.first][2 * IndexCor2.first] += IndexCor3.second * IndexCor2.second;
	// 				LinSys.A[2 * IndexCor3.first][2 * IndexCor3.first] += IndexCor3.second * IndexCor3.second;
	// 
	// 
	// 
	// 
	// 
	// 				LinSys.A[2 * IndexCor1.first + 1][2 * IndexCor1.first + 1] += IndexCor1.second * IndexCor1.second;
	// 				LinSys.A[2 * IndexCor1.first + 1][2 * IndexCor2.first + 1] += IndexCor1.second * IndexCor2.second;
	// 				LinSys.A[2 * IndexCor1.first + 1][2 * IndexCor3.first + 1] += IndexCor1.second * IndexCor3.second;
	// 
	// 				LinSys.A[2 * IndexCor2.first + 1][2 * IndexCor1.first + 1] += IndexCor2.second * IndexCor1.second;
	// 				LinSys.A[2 * IndexCor2.first + 1][2 * IndexCor2.first + 1] += IndexCor2.second * IndexCor2.second;
	// 				LinSys.A[2 * IndexCor2.first + 1][2 * IndexCor3.first + 1] += IndexCor2.second * IndexCor3.second;
	// 
	// 				LinSys.A[2 * IndexCor3.first + 1][2 * IndexCor1.first + 1] += IndexCor3.second * IndexCor1.second;
	// 				LinSys.A[2 * IndexCor3.first + 1][2 * IndexCor2.first + 1] += IndexCor3.second * IndexCor2.second;
	// 				LinSys.A[2 * IndexCor3.first + 1][2 * IndexCor3.first + 1] += IndexCor3.second * IndexCor3.second;
	// 
	// 
	// 
	// 				LinSys.BInfo[2 * IndexCor1.first].push_back(pair<Point3f*, float>(&(tri.vSamplePoints[i]->Point3D), IndexCor1.second));
	// 				LinSys.BInfo[2 * IndexCor1.first + 1].push_back(pair<Point3f*, float>(&(tri.vSamplePoints[i]->Point3D), IndexCor1.second));
	// 
	// 				LinSys.BInfo[2 * IndexCor2.first].push_back(pair<Point3f*, float>(&(tri.vSamplePoints[i]->Point3D), IndexCor2.second));
	// 				LinSys.BInfo[2 * IndexCor2.first + 1].push_back(pair<Point3f*, float>(&(tri.vSamplePoints[i]->Point3D), IndexCor2.second));
	// 
	// 				LinSys.BInfo[2 * IndexCor3.first].push_back(pair<Point3f*, float>(&(tri.vSamplePoints[i]->Point3D), IndexCor3.second));
	// 				LinSys.BInfo[2 * IndexCor3.first + 1].push_back(pair<Point3f*, float>(&(tri.vSamplePoints[i]->Point3D), IndexCor3.second));
	// 				
	// 
	// 
	// 
	// 				
	// 
	// 			}
	// 
	// 		}
	// 	
	// 
	// 	}


	//对每一个三角网格进行循环 以及 求偏导，将结果累计到LinearSystem当中
	for (int k = 0; k < vTriangle.size(); k++)
	{
		GridTriangle tri = vTriangle[k];

		//开始准备线性方程
		for (int i = 0; i < tri.vpSamplePoints.size(); i++)
		{
			//投影点
			Point2i p = tri.vpSamplePoints[i]->Point2D;

			//三角形顶点
			TriVertex *v1 = tri.vVertex[0];
			TriVertex *v2 = tri.vVertex[1];
			TriVertex *v3 = tri.vVertex[2];


			//获取 重心坐标系统 的坐标, int 对应 顶点编号，float对应坐标
			pair<int, float> IndexCor1;
			pair<int, float> IndexCor2;
			pair<int, float> IndexCor3;

			IndexCor1.first = v1->nIndex;
			IndexCor2.first = v2->nIndex;
			IndexCor3.first = v3->nIndex;

			IndexCor1.second = float((v2->vertex.y - v3->vertex.y) * (p.x - v3->vertex.x) + (v3->vertex.x - v2->vertex.x) * (p.y - v3->vertex.y)) / float((v2->vertex.y - v3->vertex.y) * (v1->vertex.x - v3->vertex.x) + (v3->vertex.x - v2->vertex.x) * (v1->vertex.y - v3->vertex.y));
			IndexCor2.second = float((v3->vertex.y - v1->vertex.y) * (p.x - v3->vertex.x) + (v1->vertex.x - v3->vertex.x) * (p.y - v3->vertex.y)) / float((v2->vertex.y - v3->vertex.y) * (v1->vertex.x - v3->vertex.x) + (v3->vertex.x - v2->vertex.x) * (v1->vertex.y - v3->vertex.y));
			IndexCor3.second = 1 - IndexCor1.second - IndexCor2.second;


			//妈的，好像有规律可以直接用循环的

			LinSys.A[2 * IndexCor1.first][2 * IndexCor1.first] += IndexCor1.second * IndexCor1.second;
			LinSys.A[2 * IndexCor1.first][2 * IndexCor2.first] += IndexCor1.second * IndexCor2.second;
			LinSys.A[2 * IndexCor1.first][2 * IndexCor3.first] += IndexCor1.second * IndexCor3.second;

			LinSys.A[2 * IndexCor2.first][2 * IndexCor1.first] += IndexCor2.second * IndexCor1.second;
			LinSys.A[2 * IndexCor2.first][2 * IndexCor2.first] += IndexCor2.second * IndexCor2.second;
			LinSys.A[2 * IndexCor2.first][2 * IndexCor3.first] += IndexCor2.second * IndexCor3.second;

			LinSys.A[2 * IndexCor3.first][2 * IndexCor1.first] += IndexCor3.second * IndexCor1.second;
			LinSys.A[2 * IndexCor3.first][2 * IndexCor2.first] += IndexCor3.second * IndexCor2.second;
			LinSys.A[2 * IndexCor3.first][2 * IndexCor3.first] += IndexCor3.second * IndexCor3.second;





			LinSys.A[2 * IndexCor1.first + 1][2 * IndexCor1.first + 1] += IndexCor1.second * IndexCor1.second;
			LinSys.A[2 * IndexCor1.first + 1][2 * IndexCor2.first + 1] += IndexCor1.second * IndexCor2.second;
			LinSys.A[2 * IndexCor1.first + 1][2 * IndexCor3.first + 1] += IndexCor1.second * IndexCor3.second;

			LinSys.A[2 * IndexCor2.first + 1][2 * IndexCor1.first + 1] += IndexCor2.second * IndexCor1.second;
			LinSys.A[2 * IndexCor2.first + 1][2 * IndexCor2.first + 1] += IndexCor2.second * IndexCor2.second;
			LinSys.A[2 * IndexCor2.first + 1][2 * IndexCor3.first + 1] += IndexCor2.second * IndexCor3.second;

			LinSys.A[2 * IndexCor3.first + 1][2 * IndexCor1.first + 1] += IndexCor3.second * IndexCor1.second;
			LinSys.A[2 * IndexCor3.first + 1][2 * IndexCor2.first + 1] += IndexCor3.second * IndexCor2.second;
			LinSys.A[2 * IndexCor3.first + 1][2 * IndexCor3.first + 1] += IndexCor3.second * IndexCor3.second;



			LinSys.BInfo[2 * IndexCor1.first].push_back(pair<Point3f*, float>((Point3f*)tri.vpSamplePoints[i]->Point3D, IndexCor1.second));
			LinSys.BInfo[2 * IndexCor1.first + 1].push_back(pair<Point3f*, float>((Point3f*)tri.vpSamplePoints[i]->Point3D, IndexCor1.second));

			LinSys.BInfo[2 * IndexCor2.first].push_back(pair<Point3f*, float>((Point3f*)(tri.vpSamplePoints[i]->Point3D), IndexCor2.second));
			LinSys.BInfo[2 * IndexCor2.first + 1].push_back(pair<Point3f*, float>((Point3f*)(tri.vpSamplePoints[i]->Point3D), IndexCor2.second));

			LinSys.BInfo[2 * IndexCor3.first].push_back(pair<Point3f*, float>((Point3f*)(tri.vpSamplePoints[i]->Point3D), IndexCor3.second));
			LinSys.BInfo[2 * IndexCor3.first + 1].push_back(pair<Point3f*, float>((Point3f*)(tri.vpSamplePoints[i]->Point3D), IndexCor3.second));






		}

	}
	m_LinSys = LinSys;
}

void SuperPixel::solveLinearSys(CameraMatrix* _camera)
{
	if (this->bHole)
	{
		return;
	}
	CameraMatrix* camera = _camera;
	vWarpVertex.clear();
	m_LinSys.B.assign(m_LinSys.B.size(), 0);

	//先根据相机矩阵，完善B向量
	for (int m = 0; m < m_LinSys.BInfo.size(); m++)
	{
		//实际上只用计算偶数行就可以了
		if (m % 2)
		{
			continue;
		}
		for (auto it = m_LinSys.BInfo[m].begin(); it != m_LinSys.BInfo[m].end(); it++)
		{


			Mat_<float> X(3, 1);
			Mat_<float> x(3, 1, 0.0);

			X.at<float>(0, 0) = it->first->x;
			X.at<float>(1, 0) = it->first->y;
			X.at<float>(2, 0) = it->first->z;

			x = camera->matRotation * X;
			x = x + camera->T;

			x.at<float>(0, 0) /= (-x.at<float>(2, 0));
			x.at<float>(1, 0) /= (-x.at<float>(2, 0));

			if (camera->vK[0] != 0 || camera->vK[1] != 0)
			{
				float fxValue = powf(x.at<float>(0, 0), 2) + powf(x.at<float>(1, 0), 2);
				x.at<float>(0, 0) = x.at<float>(0, 0) * camera->fFocal * (1 + camera->vK.at(0) * fxValue + camera->vK.at(1) * powf(fxValue, 2));
				x.at<float>(1, 0) = x.at<float>(1, 0) * camera->fFocal * (1 + camera->vK.at(0) * fxValue + camera->vK.at(1) * powf(fxValue, 2));

			}
			else
			{
				x.at<float>(0, 0) *= camera->fFocal;
				x.at<float>(1, 0) *= camera->fFocal;
			}

			//照片坐标和 投影坐标有offset
			x.at<float>(0, 0) += float(this->m_spParentImg->src.cols + 1) / 2;
			x.at<float>(1, 0) += float(this->m_spParentImg->src.rows + 1) / 2;

			x.at<float>(1, 0) = this->m_spParentImg->src.rows - x.at<float>(1, 0);

			m_LinSys.B[m] += it->second * x.at<float>(0, 0);
			m_LinSys.B[m + 1] += it->second * x.at<float>(1, 0);

		}
	}
	//采用eigen求解线性方程组
	MatrixXd A(vVertexpoint.size(), vVertexpoint.size());
	VectorXd B1(vVertexpoint.size());
	VectorXd B2(vVertexpoint.size());
	VectorXd X1(vVertexpoint.size());
	VectorXd X2(vVertexpoint.size());


	for (int k = 0; k < m_LinSys.A.size(); k++)
	{
		int m = k / 2;
		if (!(k % 2))
		{
			B1(m) = m_LinSys.B[k];
			for (int l = 0; l < m_LinSys.A[k].size(); l++)
			{
				int n = l / 2;
				if (!(l % 2))
				{
					A(m, n) = m_LinSys.A[k][l];
				}
			}
		}
		else
		{
			B2(m) = m_LinSys.B[k];
		}
	}

	X1 = A.colPivHouseholderQr().solve(B1);
	X2 = A.colPivHouseholderQr().solve(B2);


	for (int i = 0; i < vVertexpoint.size(); i++)
	{
		TriVertex *ver = new TriVertex();
		ver->nIndex = vVertexpoint[i]->nIndex;
		ver->vertex.x = X1(i);
		ver->vertex.y = X2(i);

		vWarpVertex.push_back(ver);


	}

	//调试用
	return;

	//转化为cholmod模式，并且开始求解(格式纯粹是为了方便阅读)
	// 	cholmod_dense *dA1;
	// 	cholmod_dense *dA2;
	// 	cholmod_sparse *sA1;
	// 	cholmod_sparse *sA2;
	// 
	// 	cholmod_dense *X1,*r1;
	// 	cholmod_dense *X2,*r2;
	// 
	// 	cholmod_dense *B1;
	// 	cholmod_dense *B2;
	// 
	// 	cholmod_factor *L1;
	// 	cholmod_factor *L2;
	// 
	// 	double alpha[2] = { -1, 0 };
	// 	double beta[2] = { 1, 0 };
	// 
	// 	cholmod_common c;
	// 
	// 	cholmod_start(&c);
	// 
	// 	dA1 = cholmod_eye(vVertexpoint.size(), vVertexpoint.size(), CHOLMOD_REAL, &c);
	// 	dA2 = cholmod_eye(vVertexpoint.size(), vVertexpoint.size(), CHOLMOD_REAL, &c);
	// 
	// 	B1 = cholmod_ones(vVertexpoint.size(), 1, CHOLMOD_REAL, &c);
	// 	B2 = cholmod_ones(vVertexpoint.size(), 1, CHOLMOD_REAL, &c);
	// 
	// 	double *A1x = (double*)dA1->x;
	// 	double *B1x = (double*)B1->x;
	// 
	// 	double *A2x = (double*)dA2->x;
	// 	double *B2x = (double*)B2->x;
	// 
	// 
	// 	//初始化A和B
	// 	for (int k = 0; k < m_LinSys.A.size();k++)
	// 	{
	// 		int m = k / 2;
	// 		if (!(k % 2))
	// 		{
	// 			B1x[m] = m_LinSys.B[k];
	// 			for (int l = 0; l < m_LinSys.A[k].size();l++)
	// 			{
	// 				int n = l / 2;
	// 				if (!(l % 2))
	// 				{
	// 					A1x[m + n * dA1->d] = m_LinSys.A[k][l];
	// 				}
	// 			}
	// 		}
	// 		else
	// 		{
	// 			B2x[m] = m_LinSys.B[k];
	// 			for (int l = 0; l < m_LinSys.A[k].size(); l++)
	// 			{
	// 				int n = l / 2;
	// 				if (l % 2)
	// 				{
	// 					A2x[m + n * dA1->d] = m_LinSys.A[k][l];
	// 				}
	// 			}
	// 
	// 		}
	// 	}
	// 
	// 	cholmod_print_dense(dA1, "dA1", &c);
	// 	cholmod_print_dense(dA2, "dA2", &c);
	// 
	// 
	// 	sA1 = cholmod_dense_to_sparse(dA1, 1, &c);
	// 	sA2 = cholmod_dense_to_sparse(dA2, 1, &c);
	// 
	// 	cholmod_print_sparse(sA1, "sA1", &c);
	// 	cholmod_print_sparse(sA2, "sA2", &c);
	// 
	// 
	// 	sA1->stype = 1;
	// 	sA2->stype = 1;
	// 
	// 	L1 = cholmod_analyze(sA1, &c);
	// 	L2 = cholmod_analyze(sA2, &c);
	// 
	// 	X1 = cholmod_solve(CHOLMOD_A, L1, B1, &c);
	// 	X2 = cholmod_solve(CHOLMOD_A, L2, B2, &c);
	// 
	// 	//将方程的解，转化为顶点坐标
	// 	double* x1 = (double*)(X1->x);
	// 	double* x2 = (double*)(X2->x);
	// 
	// 	for (int i = 0; i < X1->nrow; i++)
	// 	{
	// 		TriVertex *vertex = new TriVertex(Point2i(x1[i],x2[i]),i);
	// 		vWarpVertex.push_back(vertex);
	// 	}
	// 
	// 
	// 	r1 = cholmod_copy_dense(B1, &c);
	// 	r2 = cholmod_copy_dense(B2, &c);
	// 
	// 	cholmod_sdmult(sA1, 0, alpha, beta, X1, r1, &c);
	// 	cholmod_norm_dense(r1, 1, &c);
	// 
	// 	cholmod_sdmult(sA2, 0, alpha, beta, X2, r2, &c);
	// 	cholmod_norm_dense(r2, 1, &c);
	// 
	// 	cholmod_free_dense(&dA1, &c);
	// 	cholmod_free_dense(&B1, &c);
	// 	cholmod_free_dense(&X1, &c);
	// 	cholmod_free_dense(&r1, &c);
	// 	cholmod_free_sparse(&sA1, &c);
	// 
	// 	cholmod_free_dense(&dA2, &c);
	// 	cholmod_free_dense(&B2, &c);
	// 	cholmod_free_dense(&X2, &c);
	// 	cholmod_free_dense(&r2, &c);
	// 	cholmod_free_sparse(&sA2, &c);





}

float SuperPixel::getNovalDepth(const CameraMatrix *camera)
{
	SamplePoint sampoint;
	int count = 0;
	for (auto it = listPoint.begin(); it != listPoint.end(); it++)
	{
		if (++count == listPoint.size() / 2)
		{
			sampoint.Point2D = *it;
			break;
		}

	}

	sampoint.BackProjection(this->m_spParentImg->src, camera);
	fNovalDepth = sampoint.fDepth;
	return fNovalDepth;

}

ImgInfo::ImgInfo()
{

}

ImgInfo::ImgInfo(DepthSynWidget* parent)
{
	m_spParentWidget = parent;
}

void ImgInfo::GenerateView()
{
	//对每一个超像素i，每一个三角网格j ，用新生成的三角顶点在 新视角上划分片区
	for (int i = 0; i < vSP.size(); i++)
	{
		for (int j = 0; j < vSP[i].vTriangle.size(); j++)
		{
			vector<Point2i> P(3);
			for (int k = 0; k < 3; k++)
			{
				P[k] = vSP[i].vTriangle[j].vVertex[k]->vertex;
			}

			//在新视角上划三角形
		}
	}
}

ImgInfo::~ImgInfo()
{

}

GridTriangle::GridTriangle(SuperPixel *parent)
{
	m_spParentSP = parent;
}

GridTriangle::~GridTriangle()
{

}

LinearSystem::LinearSystem()
{

}

LinearSystem::~LinearSystem()
{

}

TriVertex::TriVertex()
{

}

TriVertex::TriVertex(Point2d point, int index)
{
	vertex = point;
	nIndex = index;
}

TriVertex::~TriVertex()
{

}

inline SamplePoint::SamplePoint() :bOrigin(false)
{

}

inline SamplePoint::~SamplePoint()
{
	if (!bOrigin)
	{
		delete Point3D;
	}
}

void SamplePoint::BackProjection(Mat src, const CameraMatrix *camera)
{
	if (bOrigin)
	{
		return;
	}

	Mat_<float> X(3, 1);
	Mat_<float> x(3, 1, 0.0);

	// 	x.at<float>(1, 0) = src.rows - Point2D.y;

	x.at<float>(0, 0) = Point2D.x - float(src.cols + 1) / 2;
	x.at<float>(1, 0) = Point2D.y - float(src.rows + 1) / 2;
	x.at<float>(1, 0) = -x.at<float>(1, 0);
	float fM3 = sqrtf(powf(camera->matRotation.at<float>(2, 0), 2) + powf(camera->matRotation.at<float>(2, 1), 2) + powf(camera->matRotation.at<float>(2, 2), 2));
	x.at<float>(2, 0) = -fDepth * fM3 * (determinant(camera->matRotation) < 0 ? -1 : 1);

	x.at<float>(0, 0) /= camera->fFocal;
	x.at<float>(1, 0) /= camera->fFocal;
	
	if (camera->vK.at(0) != 0 || camera->vK.at(1) != 0)
	{
		//去畸变之后的坐标
		float xx = x.at<float>(0, 0);
		float xy = x.at<float>(1, 0);

		//执行去畸变之前 的perspective坐标
		float cx = xx;
		float cy = xy;
		//迭代 上一代的结果
		float lastx = cx;
		float lasty = cy;

		do 
		{
			//保存上一代的结果
			lastx = cx;
			lasty = cy;

			//开始计算
			cx = xx / (1 + camera->vK.at(0) * (cx * cx + cy * cy) + camera->vK.at(1) * (cx * cx + cy * cy)* (cx * cx + cy * cy));
			cy = xy / (1 + camera->vK.at(0) * (cx * cx + cy * cy) + camera->vK.at(1) * (cx * cx + cy * cy)* (cx * cx + cy * cy));
		} while ((lastx - cx) * (lastx - cx) + (lasty - cy) * (lasty - cy) >  0.000001);

		x.at<float>(0, 0) = cx;
		x.at<float>(1, 0) = cy;
	}
// 	x.at<float>(0, 0) = x.at<float>(0, 0) / (camera->fFocal/fdistortion);
// 	x.at<float>(1, 0) = x.at<float>(1, 0) / (camera->fFocal /** (1 + camera->vK.at(0) * fxValue + camera->vK.at(1) * powf(fxValue, 2))*/);

	x.at<float>(0, 0) *= (-x.at<float>(2, 0));
	x.at<float>(1, 0) *= (-x.at<float>(2, 0));

	x = x - camera->T;
	X = camera->matRotation.inv() * x;

	Point3D = new MyPoint3D(X.at<float>(0, 0), X.at<float>(1, 0), X.at<float>(2, 0));


}

bool SamplePoint::Projection(Mat src, const CameraMatrix *camera)
{
	if (bOrigin)
	{
		return true;
	}

	Mat_<float> X(3, 1);
	Mat_<float> x(3, 1, 0.0);

	X.at<float>(0, 0) = Point3D->x;
	X.at<float>(1, 0) = Point3D->y;
	X.at<float>(2, 0) = Point3D->z;
	// 			X.at<float>(3, 0) = 1;


	//二维空间x坐标的计算
	// 			x = camera.matRotation * X;
	// 			x = x + camera.T;
	// 			float fxValue = powf(x.at<float>(0, 0), 2) + powf(x.at<float>(1, 0), 2) + powf(x.at<float>(2, 0), 2);
	// 			x = x * camera.fFocal * (1 + camera.vK.at(0) * fxValue + camera.vK.at(1) * powf(fxValue, 2));
	// 			x.at<float>(2, 0) /= camera.fFocal;
	// 
	// 			//以上是原生结果，实际结果需要乘以 -1
	// 			x.at<float>(2, 0) *= -1;

	////////////////////////////////////////////////////////
	x = camera->matRotation * X;
	x = x + camera->T;
	x.at<float>(0, 0) /= (-x.at<float>(2, 0));
	x.at<float>(1, 0) /= (-x.at<float>(2, 0));
	float fxValue = powf(x.at<float>(0, 0), 2) + powf(x.at<float>(1, 0), 2);
	x.at<float>(0, 0) = x.at<float>(0, 0) * camera->fFocal * (1 + camera->vK.at(0) * fxValue + camera->vK.at(1) * powf(fxValue, 2));
	x.at<float>(1, 0) = x.at<float>(1, 0) * camera->fFocal * (1 + camera->vK.at(0) * fxValue + camera->vK.at(1) * powf(fxValue, 2));

	//一直到这里，x的第三个坐标依然是w
	float fDepth;
	float fM3 = sqrtf(powf(camera->matRotation.at<float>(2, 0), 2) + powf(camera->matRotation.at<float>(2, 1), 2) + powf(camera->matRotation.at<float>(2, 2), 2));
	fDepth = -x.at<float>(2, 0) * (determinant(camera->matRotation) < 0 ? -1 : 1) / fM3;

	Point point;

	//bundler的结果，坐标原点是图片正中心，需要将图片坐标转化到opencv坐标,如果转化的坐标超出了图片范围，则跳出循环

	point.x = (x.at<float>(0, 0) + float(src.cols + 1) / 2);
	point.y = (x.at<float>(1, 0) + float(src.rows + 1) / 2);

	point.y = src.rows - point.y;

	if (point.x < 0 || point.x >= src.cols || point.y < 0 || point.y >= src.rows)
	{
		return false;
	}

	this->fDepth = fDepth;
	this->Point2Df = point;
	this->Point2D = Point2i(point.x + 0.5, point.y + 0.5);
	return true;
}

inline MyPoint3D::MyPoint3D() :ConnectImgSp(NULL)
{

}

MyPoint3D::MyPoint3D(float x, float y, float z) : Point3f(x, y, z), ConnectImgSp(NULL)
{

}

void MyPoint3D::ClearConnect()
{
	delete[] ConnectImgSp;
}

MyPoint3D::~MyPoint3D()
{

}

BinGraph::BinGraph(const vector<ImgInfo*> &vImg)
{
	for (int i = 0; i < vImg.size(); i++)
	{
		vector<BinVertex*> vVertex;
		for (int j = 0; j < vImg[i]->vSP.size(); j++)
		{
			BinVertex *vertex = new BinVertex(i, j, vImg.size());
			vVertex.push_back(vertex);
		}
		this->graph.push_back(vVertex);
	}

}

BinGraph::~BinGraph()
{
	for (int i = 0; i < graph.size();i++)
	{
		for (int j = 0; j < graph[i].size();j++)
		{
			delete graph[i][j];
		}
	}
}

void BinGraph::Initialize(vector<MyPoint3D*> vPoints)
{
	for (int i = 0; i < vPoints.size(); i++)
	{

		int* p = vPoints[i]->ConnectImgSp;

		//插值点 ，没有连接关系
		if (p == NULL)
		{
			continue;
		}

		for (int m = 0; m < vPoints[i]->nSize - 1; m++)
		{
			if (p[m] != -1 && p[m] != -2)
			{
				for (int n = m + 1; n < vPoints[i]->nSize; n++)
				{
					if (p[n] != -1 && p[n] != -2)
					{
						AddEdge(m, p[m], n, p[n]);
						AddEdge(n, p[n], m, p[m]);
					}
				}
			}
		}

		vPoints[i]->ClearConnect();
	}
}

pair<bool, HashNode**> BinGraph::FindEdge(int i1, int j1, int i2, int j2)
{
	if (graph[i1][j1]->hash[i2] == NULL)
	{
		return pair<bool, HashNode**>(false, &(graph[i1][j1]->hash[i2]));
	}
	else
	{
		HashNode** p = &(graph[i1][j1]->hash[i2]);
		while (*p != NULL)
		{
			if ((*p)->nIndex == j2)
			{
				return pair<bool, HashNode**>(true, p);
			}
			p = &((*p)->next);

		}

		return pair<bool, HashNode**>(false, p);
	}
}

void BinGraph::AddEdge(int i1, int j1, int i2, int j2)
{
	pair<bool, HashNode**> result;
	result = FindEdge(i1, j1, i2, j2);

	if (result.first)
	{
		return;
	}
	else
	{
		HashNode* node = new HashNode(j2);
		*(result.second) = node;


		//测试用
		HashNode** p = &(graph[i1][j1]->hash[i2]);
		HashNode** p2 = result.second;

	}
}

BinVertex::BinVertex(int i, int j, int nHashSize)
{
	this->i = i;
	this->j = j;
	for (int k = 0; k < nHashSize; k++)
	{
		// 		HashNode *node = new HashNode(-1);
		hash.push_back(NULL);
	}
}

BinVertex::~BinVertex()
{
	for (int i = 0; i < hash.size();i++)
	{
		if (hash[i])
		{
			delete hash[i];
		}
	}
}

HashNode::HashNode(int i) :nIndex(i), next(NULL)
{

}

HashNode::~HashNode()
{
	HashNode* cur = next;
	while (cur)
	{
		next = cur->next;
		delete cur;
	}
}

CameraMatrix::CameraMatrix() :bCenter(false)
{
}

CameraMatrix::~CameraMatrix()
{

}

Point3f CameraMatrix::CalCenter()
{
		Mat c;
		c = -(this->matRotation.inv() * this->T);

		center = Point3f(c.at<float>(0, 0), c.at<float>(1, 0), c.at<float>(2, 0));
		bCenter = true;
		return center;

}

void CameraMatrix::NormolizeR()
{
	for (int m = 0; m < matRotation.rows;m++)
	{
		float norm = 0;
		for (int n = 0; n < matRotation.cols;n++)
		{
			norm += matRotation.at<float>(m, n) * matRotation.at<float>(m, n);
		}
		norm =  sqrtf(norm);
		if (norm)
		{
			for (int n = 0; n < matRotation.cols; n++)
			{
				matRotation.at<float>(m, n) = matRotation.at<float>(m, n) / norm;
			}

		}

	}
}

void depthSynThread::writeImform(const QString s,bool bPrefix)
{
	if (bDepthOut)
	{
		depthMutex.lock();
		if (bPrefix)
		{
			*(m_spParent->DepthsynText) << "img " + QString::number(i) + ":" << s.toLocal8Bit() << endl;
		}
		else
		{
			*(m_spParent->DepthsynText) <<  s.toLocal8Bit();
		}
		depthMutex.unlock();
	}
}

void depthSynThread::run()
{
	m_spParent->mutexSrc.lock();

	src = m_spParent->m_spOpenCVFrame->vMat[i];
	vLabelPixelCount = m_spParent->m_spOpenCVFrame->m_spSLIC->vvImageLabelPixelCount[i];
	LabelMat = (m_spParent->m_spOpenCVFrame->m_spSLIC->vLabelMat[i].clone());

	//调试用，判断是否有编号831超像素
// 	for (int m = 0; m < LabelMat.rows; m ++)
// 	{
// 		for (int n = 0; n < LabelMat.cols; n ++)
// 		{
// 			if (LabelMat.at<int>(m,n) == 831)
// 			{
// 				continue;
// 			}
// 		}
// 	}

	vPoints.assign(m_spParent->vPoints.begin(), m_spParent->vPoints.end());
	nNumSp = m_spParent->m_spOpenCVFrame->m_spSLIC->vNumSP[i];
	nNumCamera = m_spParent->vCamera.size();
	camera = m_spParent->vCamera[i];

	m_spParent->mutexSrc.unlock();

	m_spParent->PointsProjection(i, this);
	m_spParent->GetSPDepth(i, this);



	ReSegSp();










	for (int j = 0; j < vLabelPointCount.size(); j++)
	{
		if (vLabelPointCount[j] <= vLabelPixelCount[j] / 150)
		{
			vTargetSPLabel.push_back(j);
		}
	}
	vNoneOriginSPLabel = vTargetSPLabel;

	m_spParent->TargetSPBlend(i, this);

	m_spParent->CalLabHist(i, this);
	m_spParent->CreateAdjointGraph(i, this);
	

	int nTargetSize;
	do
	{
		nTargetSize = this->vTargetSPLabel.size();
		m_spParent->FindNearLabel(i, this);


		//对每一张图片，每一个TargetLabel 做dijskra


		for (int j = 0; j < vTargetSPLabel.size(); j++)
		{
			int nTarget = vTargetSPLabel[j];
			m_spParent->Dijskra(vVertexAdjoint, nTarget, mapTargetNearLabel[nTarget]);
		}


		//得到天空区域，每一个SP若为天空区域，则对应label的vImgSky为0
		if (!m_spParent->bSkyGraphCut)
		{
			m_spParent->getSkyLabel();	//得到天空区域的SP
			m_spParent->setSkyDepth();	//将天空区域的label写入深度信息


			//如果是调试模式，则将对应的图片输出
			if (m_spParent->bDebugSky)
			{
				m_spParent->writeSkyImg();
			}

		}


		// 	//开始进行深度合成
		m_spParent->GenEnableSynTarget(i, this);
		m_spParent->SetEnableDepth(i, this);



	} while (vTargetSPLabel.size() != nTargetSize);

	m_spParent->mutex.lock();

	m_spParent->vvLabelPointCount[i] = vLabelPixelCount;
	m_spParent->vmapImgTargetNearLabel[i] = mapTargetNearLabel;
	m_spParent->vvImgEnableTarget[i] = vEnableTarget;
	m_spParent->vmapImgSpDepthHis[i] = mapSpDepthHis;
	m_spParent->vmapDepthPercentile[i] = mapDepthPercentile;
	m_spParent->vDepthMat[i] = DepthMat;
	m_spParent->vvDepthPoint[i] = vDepthPoint;
	m_spParent->vvSamplePoint[i] = vSamplePoint;
	m_spParent->vvTargetSPLabel[i] = vTargetSPLabel;
	m_spParent->vvNoneOriginSPLabel[i] = vNoneOriginSPLabel;
	m_spParent->vTargetMaskMat[i] = TargetMaskMat;
	m_spParent->vBlendMat[i] = BlendMat;
	m_spParent->vvvImgLabelHis[i] = vvLabelHis;
	m_spParent->vGraph[i] = Graph;
	m_spParent->vvImgVertexAdjoint[i] = vVertexAdjoint;
	m_spParent->vmapImgSPDepth[i] = mapSPDepth;
	m_spParent->vvLabelPointCount[i] = vLabelPointCount;
	m_spParent->vLabelMat[i] = LabelMat;
	m_spParent->vSrc[i] = src;

	m_spParent->mutex.unlock();


	//////////////////////////////////////////////////////////////////////////////////////////////
	// 	m_spParent->mutexSrc.lock();
	// 	m_spParent->vSrc[i] = (m_spParent->m_spOpenCVFrame->vMat[i]);
	// 	m_spParent->mutexSrc.unlock();
	// 
	// 	m_spParent->mutexLabelMat.lock();
	// 	m_spParent->vLabelMat[i] = (m_spParent->m_spOpenCVFrame->m_spSLIC->vLabelMat[i].clone());
	// 	m_spParent->mutexLabelMat.unlock();
	// 
	// 	m_spParent->PointsProjection(i,this);
	// 
	// 	//get the target SP 
	// 
	// 
	// 	vector<int> vTargetSPLabel;
	// 	for (int j = 0; j < m_spParent->vvLabelPointCount.at(i).size(); j++)
	// 	{
	// 		if (m_spParent->vvLabelPointCount[i][j] <= m_spParent->m_spOpenCVFrame->m_spSLIC->vvImageLabelPixelCount[i][j] / 150)
	// 		{
	// 			vTargetSPLabel.push_back(j);
	// 		}
	// 	}
	// 	//这一步其实没必要，我就是想试试
	// 	qSort(vTargetSPLabel.begin(), vTargetSPLabel.end(), IntDescend); 
	// 
	// 	m_spParent->mutexTargetSPLabel.lock();
	// 	m_spParent->vvTargetSPLabel[i] = (vTargetSPLabel);
	// 	m_spParent->mutexTargetSPLabel.unlock();
	// 
	// 	m_spParent->mutexNoneOriginLabel.lock();
	// 	m_spParent->vvNoneOriginSPLabel[i].assign(m_spParent->vvTargetSPLabel[i].begin(), m_spParent->vvTargetSPLabel[i].end());
	// 	m_spParent->mutexNoneOriginLabel.unlock();
	// 
	// 	//测试程序，显示target SP 位置
	// 	m_spParent->TargetSPBlend(i);
	// 
	// 	//测试程序，用于显示同一点对应不同视角的位置
	// 	// 	for (int i = 0; i < vPoints.size(); i++)
	// 	// 	{
	// 	// 		int nCount = 0;
	// 	// 		for (int m = 0; m < vPoints[i]->nSize; m++)
	// 	// 		{
	// 	// 			if (vPoints[i]->ConnectImgSp[m] != -1)
	// 	// 			{
	// 	// 				++nCount;
	// 	// 			}
	// 	// 		}
	// 	// 
	// 	// 		if (nCount >= vPoints[i]->nSize - 1)
	// 	// 		{
	// 	// 			for (int k = 0; k < vPoints[i]->nSize; k++)
	// 	// 			{
	// 	// 				if (vPoints[i]->ConnectImgSp[k] != -1)
	// 	// 				{
	// 	// 					SamplePoint* sample = new SamplePoint();
	// 	// 					sample->Point3D = vPoints[i];
	// 	// 					if (sample->Projection(vBlendMat[k], vCamera[k]))
	// 	// 					{
	// 	// 						circle(vBlendMat[k], sample->Point2D, 5, Scalar(255, 255, 255), -1, 8);
	// 	// 					}
	// 	// 
	// 	// 
	// 	// 
	// 	// 				}
	// 	// 			}
	// 	// 			break;
	// 	// 		}
	// 	// 	}
	// 
	// 
	// 	// 开始寻找每一个TargetLabel对应的 最邻近label，这一步采用的是直方图寻找卡方距离最近的40个SP label（某些图片不到40个）
	// 	m_spParent->CalLabHist(i);
	// 	//计算邻接图，两种形式都有
	// 	m_spParent->CreateAdjointGraph(i);
	// 
	// 	m_spParent->GetSPDepth(i);
	// 
	// 	int nTargetSize;
	// 	do
	// 	{
	// 		m_spParent->mutexTargetSPLabel.lock();
	// 		nTargetSize = m_spParent->vvTargetSPLabel[i].size();
	// 		m_spParent->mutexTargetSPLabel.unlock();
	// 
	// 		m_spParent->FindNearLabel(i);
	// 
	// 
	// 		//对每一张图片，每一个TargetLabel 做dijskra
	// 
	// 		
	// 		for (int j = 0; j < m_spParent->vvTargetSPLabel[i].size(); j++)
	// 		{
	// 			int nTarget = m_spParent->vvTargetSPLabel[i][j];
	// 			m_spParent->Dijskra(m_spParent->vvImgVertexAdjoint[i], nTarget, m_spParent->vmapImgTargetNearLabel[i][nTarget]);
	// 		}
	// 
	// 
	// 		//得到天空区域，每一个SP若为天空区域，则对应label的vImgSky为0
	// 		if (!m_spParent->bSkyGraphCut)
	// 		{
	// 			m_spParent->getSkyLabel();	//得到天空区域的SP
	// 			m_spParent->setSkyDepth();	//将天空区域的label写入深度信息
	// 
	// 
	// 			//如果是调试模式，则将对应的图片输出
	// 			if (m_spParent->bDebugSky)
	// 			{
	// 				m_spParent->writeSkyImg();
	// 			}
	// 
	// 		}
	// 
	// 
	// 		// 	//开始进行深度合成
	// 		m_spParent->GenEnableSynTarget(i);
	// 		m_spParent->SetEnableDepth(i);
	// 
	// 
	// 
	// 	} while (m_spParent->vvTargetSPLabel[i].size() != nTargetSize);
	// 
	// 
	// 	// 	for (int i = 0; i < vSrc.size();i++)
	// 	// 	{
	// 	// 		Mat slicMask = m_spOpenCVFrame->m_spSLIC->vLabelMat[i];
	// 	// 		for (int m = 0; m < vvImgEnableTarget[i].size(); m++)
	// 	// 		{
	// 	// 			for (int r = 0; r < vBlendMat[i].rows;r++)
	// 	// 			{
	// 	// 				for (int c = 0; c < vBlendMat[i].cols;c++)
	// 	// 				{
	// 	// 					if (slicMask.at<int>(r,c) == vvImgEnableTarget[i][m])
	// 	// 					{
	// 	// 						vBlendMat[i].at<Vec3b>(r, c) = Vec3b(255, 255, 255);
	// 	// 					}
	// 	// 				}
	// 	// 			}
	// 	// 		}
	// 	// 
	// 	// 	}



}

void depthSynThread::ReSegSp()
{
	//开始执行 基于深度的超像素分割
	QString sSeg;
	int nCurIndex = vLabelPixelCount.size();
	//每一个超像素 对应的像素
	vector<vector<Point2i>> vvSpPixles(vLabelPixelCount.size());
	for (int m = 0; m < LabelMat.rows; m++)
	{
		for (int n = 0; n < LabelMat.cols; n++)
		{
			int nLabel = LabelMat.at<int>(m, n);
			vvSpPixles[nLabel].push_back(Point2i(n, m));
		}
	}

	//计算当前图片的深度百分制度
	float step = vDepthPoint.size() / 100.0f;
	qSort(vDepthPoint.begin(), vDepthPoint.end(), depthPointAscend);
	for (int m = 0; m < vDepthPoint.size() - 1; m++)
	{
		int nP = m / step;
		mapDepthPercentile.insert(pair<float, int >(vDepthPoint[m].fDepth, nP));
	}
	//最后一个深度特殊处理(有可能 m/step 为100)
	mapDepthPercentile.insert(pair<float, int >(vDepthPoint[vDepthPoint.size() - 1].fDepth, 99));


	//记录超像素三维点的位置
	map<int, vector<SamplePoint*>> mapSPSample;
	for (auto it = vSamplePoint.begin(); it != vSamplePoint.end(); it++)
	{
		if (mapSPSample.find((*it)->nSpIndex) == mapSPSample.end())
		{
			mapSPSample.insert(pair<int, vector<SamplePoint*>>((*it)->nSpIndex, vector<SamplePoint*>(1, *it)));
		}
		else
		{
			mapSPSample[(*it)->nSpIndex].push_back(*it);
		}
	}


	//将sp深度对应到直方图，然后看直方图的间隙，根据间隙判断是否分离

	for (auto it = mapSPDepth.begin(); it != mapSPDepth.end(); it++)
	{
		if (vLabelPointCount[it->first] < vLabelPixelCount[it->first] / 150)
		{
			continue;
		}
		vector<float> vHisDepth(100, 0);
		for (int i = 0; i < it->second.size(); i++)
		{
			vHisDepth[mapDepthPercentile[it->second[i].fDepth]]++;
		}

		//判断是否分离
		bool bSeg = false;
		bool bFlag = false;
		int nStart;
		int nPeak = 0;
		int nLastPeakPos = -1;
		vector<float> vSegDepth;
		int nMaxGap = 0;

		for (int m = 0; m < vHisDepth.size(); m++)
		{
			if (!bFlag && vHisDepth[m])
			{
				bFlag = true;
				nStart = m;
			}
			else if (bFlag && (!vHisDepth[m] || m == (vHisDepth.size() - 1)))
			{
				bFlag = false;
				nPeak++;
				if (nLastPeakPos < 0)
				{
					nLastPeakPos = m;
				}
				else
				{
					//判断峰之间的距离，若超过限制，则需要分割，且将分割深度推入
					int nGap = nStart - nLastPeakPos;
					if (nGap >= 35)
					{
						bSeg = true;
						vSegDepth.push_back(vDepthPoint[int((nStart + nLastPeakPos) / 2 * step) + 1].fDepth);
						if (nGap > nMaxGap)
						{
							nMaxGap = nGap;
						}
					}
					nLastPeakPos = m;
				}

			}
		}

		//开始根据是否分离 进行操作
		if (bSeg)
		{
			vSegIndex.push_back(it->first);
			vector<Point2i>* vCurPixels = &vvSpPixles[it->first];

			//部分变量容量调整
			int nAdd = vSegDepth.size();
			nNumSp += nAdd;
			vLabelPixelCount.resize(vLabelPixelCount.size() + nAdd);
			vLabelPointCount.resize(vLabelPointCount.size() + nAdd);

			for (int i = 0; i < nAdd; i++)
			{
				mapSPDepth.insert(pair<int, vector<DepthPoint>>(nCurIndex + i, vector<DepthPoint>(0)));
			}

			//对depthPoint 以及 samplePoint 进行改造,depthpoint是已经排好序的
			for (auto depthIt = mapSPDepth[it->first].begin(); depthIt != mapSPDepth[it->first].end();)
			{
				int nMyPos;
				for (nMyPos = 0; nMyPos < vSegDepth.size(); nMyPos++)
				{
					if (depthIt->fDepth < vSegDepth[nMyPos])
					{
						break;
					}
				}
				if (!nMyPos)
				{
					++depthIt;
					continue;
				}
				else
				{
					nMyPos += nCurIndex - 1;

					vLabelPointCount[it->first]--;
					vLabelPixelCount[it->first]--;

					vLabelPointCount[nMyPos]++;
					vLabelPixelCount[nMyPos]++;

					DepthMat.at<Vec3f>(depthIt->point)[0] = nMyPos;
					LabelMat.at<int>(depthIt->point) = nMyPos;

					mapSPDepth[nMyPos].push_back(*depthIt);
					depthIt = mapSPDepth[it->first].erase(depthIt);
				}
			}



			//像素级别的调整（找到最邻近sample）
			for (int i = 0; i < vCurPixels->size(); i++)
			{
				float fNearDis = LabelMat.cols * LabelMat.cols;
				SamplePoint* nearSample;
				Point2i curPoint = vCurPixels->at(i);
				int nMyIndex;
				for (int j = 0; j < mapSPSample[it->first].size(); j++)
				{
					SamplePoint* curSample = mapSPSample[it->first][j];
					float fDis = (curPoint.x - curSample->Point2D.x) * (curPoint.x - curSample->Point2D.x) + (curPoint.y - curSample->Point2D.y) * (curPoint.y - curSample->Point2D.y);
					if (fDis <= fNearDis)
					{
						fNearDis = fDis;
						nearSample = curSample;
					}
				}

				//三维点的信息已经改造过
				if (!fNearDis)
				{
					continue;
				}
				else
				{
					int nMyPos = LabelMat.at<int>(nearSample->Point2D);

					vLabelPixelCount[it->first]--;

					vLabelPixelCount[nMyPos]++;

					DepthMat.at<Vec3f>(curPoint)[0] = nMyPos;
					LabelMat.at<int>(curPoint) = nMyPos;

				}

			}
			//输出被选中超像素
			if (bDepthOut)
			{
				QString sCur;
				sCur = QString::number(it->first) + "(" + "Pos: " + QString::number(it->second[0].point.x) + "," + QString::number(it->second[0].point.y) + " Gap: " + QString::number(nMaxGap) + ")" + ":";
				for (int i = 0; i < vHisDepth.size(); i++)
				{
					sCur = sCur + QString::number(vHisDepth[i]) + " ";
				}
				writeImform(sCur);

			}

		}
		nCurIndex += vSegDepth.size();
	}


}

void warpThread::run()
{
	img->dst = Mat(img->src.rows, img->src.cols, CV_8UC3, Scalar(0, 0, 0));
	img->dstDepth = Mat(img->src.rows, img->src.cols, CV_32FC1, 0.0);
	Mat_<float> myMatWeight(img->src.rows, img->src.cols,-1);
	vector<vector<Point2i>> myPixelCorrespond(myMatWeight.rows, vector<Point2i>(myMatWeight.cols));

	for (int j = 0; j < img->vSP.size(); j++)
	{
		SuperPixel* sp = &img->vSP[j];
		//对于hole超像素，直接pass
		if (sp->bHole)
		{
			continue;
		}
		sp->solveLinearSys(&novalCamera);
		writeImform("SP" + QString::number(j) + "solved");

		for (int k = 0; k < sp->vTriangleIndex.size(); k++)
		{


			Point2d *p1 = &(sp->vWarpVertex[sp->vTriangleIndex[k][0]]->vertex);
			Point2d *p2 = &(sp->vWarpVertex[sp->vTriangleIndex[k][1]]->vertex);
			Point2d *p3 = &(sp->vWarpVertex[sp->vTriangleIndex[k][2]]->vertex);



			//三角网外包矩形
			int xLeft, xRight, yTop, yBottom;
			xLeft = p1->x < p2->x ? (p1->x < p3->x ? p1->x : p3->x) : (p2->x < p3->x ? p2->x : p3->x);
			xRight = p1->x > p2->x ? (p1->x > p3->x ? p1->x : p3->x) : (p2->x > p3->x ? p2->x : p3->x);
			yBottom = p1->y < p2->y ? (p1->y < p3->y ? p1->y : p3->y) : (p2->y < p3->y ? p2->y : p3->y);
			yTop = p1->y > p2->y ? (p1->y > p3->y ? p1->y : p3->y) : (p2->y > p3->y ? p2->y : p3->y);

			xLeft = xLeft < 0 ? 0 : xLeft;
			xRight = xRight > img->src.cols - 1 ? img->src.cols - 1 : xRight;
			yBottom = yBottom < 0 ? 0 : yBottom;
			yTop = yTop > img->src.rows - 1 ? img->src.rows - 1 : yTop;

			for (int y = yBottom; y <= yTop; y++)
			{
				for (int x = xLeft; x <= xRight; x++)
				{

					//获取重心坐标
					float alpha = float((p2->y - p3->y) * (x - p3->x) + (p3->x - p2->x) * (y - p3->y)) / float((p2->y - p3->y)*(p1->x - p3->x) + (p3->x - p2->x) * (p1->y - p3->y));

					float beta = float((p3->y - p1->y) * (x - p3->x) + (p1->x - p3->x) * (y - p3->y)) / float((p2->y - p3->y)*(p1->x - p3->x) + (p3->x - p2->x) * (p1->y - p3->y));

					float gamma = 1 - alpha - beta;

					//该点在三角形内，计算对应原图坐标
					if (alpha >= 0 && beta >= 0 && gamma >= 0)
					{
						//浮点型 对应原图点位置
						Point2d srcPoint;

						//离散型（最邻近） 对应原图点位置
						Point2i iSrcPoint;


						Point2d *sp1 = &(sp->vVertexpoint[sp->vTriangleIndex[k][0]]->vertex);
						Point2d *sp2 = &(sp->vVertexpoint[sp->vTriangleIndex[k][1]]->vertex);
						Point2d *sp3 = &(sp->vVertexpoint[sp->vTriangleIndex[k][2]]->vertex);

						srcPoint.x = alpha * sp1->x + beta * sp2->x + gamma * sp3->x;
						srcPoint.y = alpha * sp1->y + beta * sp2->y + gamma * sp3->y;

						iSrcPoint.x = int(srcPoint.x + 0.5);
						iSrcPoint.y = int(srcPoint.y + 0.5);

						//判断重投影点是否在超像素内，如果是 则 双线性插值 + 创立链接
						if ((LabelMat.at<int>(iSrcPoint) == j) || (sp->bExtension && sp->matExtSP.at<uchar>(iSrcPoint.y - sp->ExtBounding.y, iSrcPoint.x - sp->ExtBounding.x)))
						{
							//建立连接，同时计算权重
							

							SamplePoint curPoint;
							curPoint.Point2D = iSrcPoint;
							curPoint.bOrigin = false;
							curPoint.fDepth = sp->fMedianDepth;
							curPoint.BackProjection(img->src, img->camera);
							writeImform("pixel " + QString::number(iSrcPoint.x) +","  + QString::number(iSrcPoint.y) + " get");

							//点到原始相机的射线
							Point3f pRay = (*(dynamic_cast<Point3f*>(curPoint.Point3D)) - img->camera->CalCenter());
							vector<float> ray;
							ray.push_back(pRay.x);
							ray.push_back(pRay.y);
							ray.push_back(pRay.z);

							//点到novalview的射线
							Point3f cpRay = (*(dynamic_cast<Point3f*>(curPoint.Point3D)) - novalCamera.CalCenter());
							vector<float> cray;
							cray.push_back(cpRay.x);
							cray.push_back(cpRay.y);
							cray.push_back(cpRay.z);

							float fCos = GetAngleCos(ray, cray);
							float fAngleWeight = acos(GetAngleCos(ray, cray));

							float fDisWeigh;
							fDisWeigh = fabs(GetNorm(ray) - GetNorm(cray));

							//
							if (img->dstDepth.at<float>(y, x) != 0 && GetNorm(cray) > img->dstDepth.at<float>(y, x))
							{
								writeImform("pixel " + QString::number(iSrcPoint.x) + "," + QString::number(iSrcPoint.y) + " pass");
								continue;							
							}

							//本来应该是双线性插值 ,目前先用最邻近法
							img->dst.at<Vec3b>(y, x) = img->src.at<Vec3b>(iSrcPoint);
							img->dstDepth.at<float>(y, x) = GetNorm(cray);

// 							m_spParent->mutex.lock();
							myMatWeight.at<float>(y, x) = kAngleWeight * fAngleWeight + (1 - kAngleWeight) * fDisWeigh;
							myPixelCorrespond[y][x] = iSrcPoint;
// 							m_spParent->mutex.unlock();
							writeImform("pixel " + QString::number(iSrcPoint.x) +"," +  QString::number(iSrcPoint.y) + " done");

 						}

					}
				}
			}
		}

	}
	qWarpMutex.lock();
	for (int m = 0; m < myMatWeight.rows;m++)
	{
		for (int n = 0; n < matWeight.cols;n++)
		{
			matWeight.at<Vec4f>(m, n)[i] = myMatWeight.at<float>(m, n);
			pixelCorrespond[m][n][i] = myPixelCorrespond[m][n];
		}
	}

	qWarpMutex.unlock();
	
	writeImform("finished");
}

void warpThread::writeImform(const QString s)
{
	if (bWarpOut)
	{
		m_spParent->warpMutex.lock();
		*(m_spParent->warpText) << "img" + QString::number(this->img->Index) + ":"<< s.toLocal8Bit() << endl;
		m_spParent->warpMutex.unlock();
	}

}
