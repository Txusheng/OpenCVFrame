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

//�������ˣ���������ν
bool pairIFDescenf(const pair<int, float> &a, const pair<int, float> &b)
{
	return a.second < b.second;
}

//��ʵ���������ˣ���������ν
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
		//�����нǷ�Χ
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

	//��ֹ��� ��Ϊ�������� ������Χ -1~1
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


		//��ά�ռ�x����ļ���
		// 			x = camera.matRotation * X;
		// 			x = x + camera.T;
		// 			float fxValue = powf(x.at<float>(0, 0), 2) + powf(x.at<float>(1, 0), 2) + powf(x.at<float>(2, 0), 2);
		// 			x = x * camera.fFocal * (1 + camera.vK.at(0) * fxValue + camera.vK.at(1) * powf(fxValue, 2));
		// 			x.at<float>(2, 0) /= camera.fFocal;
		// 
		// 			//������ԭ�������ʵ�ʽ����Ҫ���� -1
		// 			x.at<float>(2, 0) *= -1;

		////////////////////////////////////////////////////////
		x = thread->camera->matRotation * X;
		x = x + thread->camera->T;
		x.at<float>(0, 0) /= (-x.at<float>(2, 0));
		x.at<float>(1, 0) /= (-x.at<float>(2, 0));
		float fxValue = powf(x.at<float>(0, 0), 2) + powf(x.at<float>(1, 0), 2);
		x.at<float>(0, 0) = x.at<float>(0, 0) * thread->camera->fFocal * (1 + thread->camera->vK.at(0) * fxValue + thread->camera->vK.at(1) * powf(fxValue, 2));
		x.at<float>(1, 0) = x.at<float>(1, 0) * thread->camera->fFocal * (1 + thread->camera->vK.at(0) * fxValue + thread->camera->vK.at(1) * powf(fxValue, 2));

		//һֱ�����x�ĵ�����������Ȼ��w
		float fDepth;
		float fM3 = sqrtf(powf(thread->camera->matRotation.at<float>(2, 0), 2) + powf(thread->camera->matRotation.at<float>(2, 1), 2) + powf(thread->camera->matRotation.at<float>(2, 2), 2));
		fDepth = -x.at<float>(2, 0) * (determinant(thread->camera->matRotation) < 0 ? -1 : 1) / fM3;

		Point point;

		//bundler�Ľ��������ԭ����ͼƬ�����ģ���Ҫ��ͼƬ����ת����opencv����,���ת�������곬����ͼƬ��Χ��������ѭ��

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

		//samplepoint �� ����
		SamplePoint* samplePoint = new SamplePoint;
		samplePoint->Point3D = thread->vPoints.at(j);
		//��ʼ�����ӣ���ҪifҲ��
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


	int n = 0;//��ʾvTargetSPLabel���±�
	int p = 0;//��ʾvSegIndex�±�
	for (int m = 0; m < thread->nNumSp; m++)
	{
		Vec<uchar, 3> r1;
		//���ȱʧΪ��ɫ
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
		//��ȳ���Ϊ��ɫ
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


	//��ÿһ�������ر�Ե�����ߣ����ڳ������ϱ�ǳ����صı��

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

	//�����ͼ��������
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
	//��������Ϊ12.8 ����Ϊ 256/20


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

	//��ͼƬ���ؽ���ѭ������histgramͳ�Ƴ���
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
	//����ͨ���� �����㲽��������Ҫע������Ϊ255��ֵӦ�����⴦��
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
	//���ȶ�ÿһ��target Label �ҵ� ֱ��ͼ���������С�� label 40 ��

	vector<int> vTargetLabel = thread->vTargetSPLabel;
	vector<vector<float>> vvLabelHist = thread->vvLabelHis;
	map<int, vector<int>> mapTargetNearLabel;
	for (int m = 0; m < vTargetLabel.size(); m++)
	{
		int n = 0;// n��ʾ��ѭ��label�Ĺ����У���ǰtarget Label ������λ��
		vector<int> vNearLabel;
		//�������з�target label �� target label ��Ӧ�� ��������


		vector<pair<int, float>> vLabelDis;//�ýṹ�� ����dis����

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




		//��С������ţ�����ȡǰ��ʮ��
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




	//ֻ��Ϊ�����ڽӣ�û��Ҫ���������ر�����
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
	//�Ƿ񱻼��뵽��̶��㼯��
	int *set;
	set = new int[vVertex.size()];

	//��ǰ����·������һ������
	int *path;
	path = new int[vVertex.size()];

	//·������
	float *cost;
	cost = new float[vVertex.size()];


	//��ʼ��
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


	//��ʼѭ���������·����

	for (int i = 0; i < vVertex.size() - 1; i++)
	{
		float fMinCost = 62500.0f;
		int nV;//�˴�ѭ������Ķ���

		//ѭ��ѡȡҪ�������ڽ�����ĵ�
		for (int j = 0; j < vVertex.size(); j++)
		{
			if (set[j] == 0 && cost[j] >= 0 && cost[j] < fMinCost)
			{
				fMinCost = cost[j];
				nV = j;
			}
		}
		set[nV] = 1;


		//�������ӵĶ������
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


	//�ҵ������е�cost�����ڿ�ʼ��NearLabel��ɸѡ

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

				//��ʼ���ѡȡ���ؽ��и�ֵ
				assert(vHash.size() > 15);
				int  nCountDown = (vHash.size() / 200 > 15) ? (vHash.size() / 200) : 15;//ʣ����Ҫ��ֵ��������Ŀ

				int nCount = 0;//�Ѿ���ֵ��������Ŀ

				while (nCountDown)
				{
					int index;	//Ҫ��ֵ�������±�
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


	//ÿ��label ��Ӧһ�����ֵ
	map<int, vector<DepthPoint>> mapLabelDepth;
	map<int, vector<DepthPoint>>::iterator itLabelDepth;

	Mat matDepthMat = thread->DepthMat;

	//���û���㹻��ȵ㣬����û��Ҫ����,�����ø�ͼƬ�����ֵ��Χ
	if (vDepthPoint.size() <= 10)
	{
		vmapImgSPDepth[i] = (mapLabelDepth);
		return;
	}

	//��VdepthPoint ���п��ţ�Ȼ��ȷ���ٷ�λ�����ٷ�λ��Ϣ���뵽map��
	qSort(vDepthPoint.begin(), vDepthPoint.end(), depthPointAscend);
	thread->vDepthPoint = vDepthPoint;

	//ÿһ��label ����һ�� ���ֵ
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


		//ÿһ��targetLabel��Ӧ����ȵ����Ŀ
		int nDepthCount = 0;

		for (int m = 0; m < itNearLabel->second.size(); m++)
		{


			//��ȡ��label��Ӧ���������
			vector<DepthPoint> vDepth(thread->mapSPDepth[itNearLabel->second[m]]);

			nDepthCount += vDepth.size();

			for (int n = 0; n < vDepth.size(); n++)
			{
				vHisDepth[mapDepthPercentile[vDepth[n].fDepth]]++;
			}
		}


		//��ȡ��ȷֲ� �� �����ܶ�ͼ,ͬʱ��ȡ��ֵλ�ú���Ŀ

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
	//�������





	vector<int> vEnableTarget(thread->vEnableTarget);
	map<int, vector<float>> mapSpDepthHist(thread->mapSpDepthHis);
	map<int, vector<int>> mapTargetNearLabel(thread->mapTargetNearLabel);
	map<int, vector<DepthPoint>> mapSpDepth(thread->mapSPDepth);
	map<float, int> mapDepthPercentile(thread->mapDepthPercentile);
	Mat src = thread->src;
	Mat LabelMat = thread->LabelMat;
	//����������޸Ĳ�����


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

					//�µ����ݽṹ sample point
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

	//ȥ����target���
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
		//�����ã������۲�һ���߳�
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

	//���Գ���������ʾͬһ���Ӧ��ͬ�ӽǵ�λ��
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



	//�����������ݿ���ͨ��action����������������д�� 
	m_spOpenCVFrame->ShowImage(vBlendMat);


}

void DepthSynWidget::slotPrepareLocalWarp()
{
	//�����Ϣ��ת¼
	warpInitialize(0.1);

	//���� ������ά��ĳ����� ������ͼ
	myGraph = new BinGraph(vImgInfo);
	myGraph->Initialize(vPoints);

	//��ȡÿһ�����ص�warp�ṹ�Լ�����ϵͳ
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


	//ʵ�飬���ض��Ƕ�����ͷ�����Է�����
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
	//��������� �������� ���¼����


	//��ѡ�ĸ��Ƕ�����������
	GetKNovalNear(4);

	//Ȩֵͼ��novalview ÿһ������ ��Ӧ������ͼ ��Ȩֵ
	Mat matWeight = Mat(vSrc[0].rows, vSrc[0].cols, CV_32FC4, Scalar(-1, -1, -1, -1));

	//Ȩֵͼ���м����
	Mat matMWeight = Mat(vSrc[0].rows, vSrc[0].cols, CV_32FC4, Scalar(0, 0, 0, 0));

	//novalview ÿһ������ ��Ӧ�� ԭͼ��λ��
	vector<vector<array<Point2i, 4>>> pixelCorrespond(vSrc[0].rows, vector<array<Point2i, 4>>(vSrc[0].cols));

	//���߳�����
	vector<warpThread*> vThreads(4);

	if (bWarpOut)
	{
		(*warpText) << tr("warp initialized") << endl;
	}

	//���ĸ���� ��Ӧ��ͼƬ �������Է�����Ľ� �Լ� ͶӰ��ͼƬ��
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
// 			//����hole�����أ�ֱ��pass
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
// 				//�������������
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
// 						//��ȡ��������
// 						float alpha = float((p2->y - p3->y) * (x - p3->x) + (p3->x - p2->x) * (y - p3->y)) / float((p2->y - p3->y)*(p1->x - p3->x) + (p3->x - p2->x) * (p1->y - p3->y));
// 
// 						float beta = float((p3->y - p1->y) * (x - p3->x) + (p1->x - p3->x) * (y - p3->y)) / float((p2->y - p3->y)*(p1->x - p3->x) + (p3->x - p2->x) * (p1->y - p3->y));
// 
// 						float gamma = 1 - alpha - beta;
// 
// 						//�õ����������ڣ������Ӧԭͼ����
// 						if (alpha >= 0 && beta >= 0 && gamma >= 0)
// 						{
// 							//������ ��Ӧԭͼ��λ��
// 							Point2d srcPoint;
// 
// 							//��ɢ�ͣ����ڽ��� ��Ӧԭͼ��λ��
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
// 							//�ж���ͶӰ���Ƿ��ڳ������ڣ������ �� ˫���Բ�ֵ + ��������
// 							if ((vLabelMat[img->Index].at<int>(iSrcPoint) == j) || (sp->bExtension && sp->matExtSP.at<uchar>(iSrcPoint.y - sp->ExtBounding.y, iSrcPoint.x - sp->ExtBounding.x)))
// 							{
// 								//�������ӣ�ͬʱ����Ȩ��
// 								pixelCorrespond[y][x].at(i) = iSrcPoint;
// 
// 								SamplePoint curPoint;
// 								curPoint.Point2D = iSrcPoint;
// 								curPoint.bOrigin = false;
// 								curPoint.fDepth = sp->fMedianDepth;
// 								curPoint.BackProjection(img->src, img->camera);
// 
// 								//�㵽ԭʼ���������
// 								Point3f pRay = (*(dynamic_cast<Point3f*>(curPoint.Point3D)) - img->camera->CalCenter());
// 								vector<float> ray;
// 								ray.push_back(pRay.x);
// 								ray.push_back(pRay.y);
// 								ray.push_back(pRay.z);
// 
// 								//�㵽novalview������
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
// 								//����Ӧ����˫���Բ�ֵ ,Ŀǰ�������ڽ���
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

	//ĿǰȨֵͼ��ʵ��costͼ����Ҫ�����Ȩֵͼ
	for (int m = 0; m < matWeight.rows; m++)
	{
		for (int n = 0; n < matWeight.cols; n++)
		{
			Vec4f vec = matWeight.at<Vec4f>(m, n);


			//���飬int��ʾid��float��ʾcost
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

				//���ݳ���������ͼ���ж�Ȩֵ������ϵ
				array<float, 2> aFactor = { 1.0, 1.0 };
				if (vIF.size() > 1)
				{
					//�����һ����Ϊimg��ţ��ڶ���Ϊsp���
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
						//��Ƚ���ĳ����ؼ�Ȩ
						if (vImgInfo[i1]->vSP[j1].getNovalDepth(camera) < vImgInfo[i2]->vSP[j2].getNovalDepth(camera))
						{
							aFactor[0] *= 2;
						}
						else
						{
							aFactor[1] *= 2;
						}

						//�ǲ�ֵ������ ��Ȩ
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


				//��ʵ���������switch������
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

	//�ϳ����ӽǣ�����
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

		//������
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
		map<int, char> mLabelType;// AΪͼƬ�϶ˣ�û�������Ϣ��BΪû�������Ϣ��CΪ�������Ϣ
		Mat LabelMat = vLabelMat[i].clone();

		typedef Graph<int, int, int> GraphType;
		GraphType *myGraph = new GraphType(LabelMat.cols * LabelMat.rows, 2 * LabelMat.cols * LabelMat.rows - LabelMat.cols - LabelMat.rows);
		int nFlow;	//�������Ȼ������û����

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

				//�õ���ǰ���ص��Ӧ���ͣ���ʼ�����ڵ��Ȩֵ��Ϊ�˱����ظ���ÿһ�����ص㶼������ ���ع�����Ȩֵ��
				myGraph->add_node();

				//����T link
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

				//���� N link
				if (n != 0)//����ߵ����ع�����
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

				if (m != 0)//���ұߵ����ع�����
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

		//������ջ���labelͼ�������ж�labelmat��label�Ƿ�Ϊ��յ�mat���������vector��
		Mat_<int> matSky(LabelMat.rows, LabelMat.cols);
		vector<int> vSky(m_spOpenCVFrame->m_spSLIC->vNumSP[i]);

		for (int m = 0; m < LabelMat.rows; m++)
		{
			for (int n = 0; n < LabelMat.cols; n++)
			{
				int nLabel = myGraph->what_segment(m * LabelMat.cols + n);
				matSky.at<int>(m, n) = nLabel;

				//��û�п��ܣ�һ��SP���� ��������գ�����û���϶�����գ�
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
		//�ҵ�ĿǰͼƬ�е������ȣ�Ȼ���������ȳ���1000 ��Ϊ�����ȣ��ǳ�Զ��
		vector<DepthPoint> vDepthPoint(vvDepthPoint[i]);

		//�������ͼһ����ȵ㶼û�У��ǾͲ������ˣ�
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

		//����������Ϊ ������ֱ�ӳ���1000�����Ϊ������ֱ�Ӽ���1000
		if (fMatDepth > 0)
		{
			fMatDepth *= 1000;
		}
		else
		{
			fMatDepth += 1000;
		}



		//��ʼ�����SP�е�������ؽ�����ȸ�ֵ
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

				//��ʼ���ѡȡ���ؽ��и�ֵ
				assert(vHash.size() > 15);
				int  nCountDown = (vHash.size() / 200 > 15) ? (vHash.size() / 200) : 15;//ʣ����Ҫ��ֵ��������Ŀ

				int nCount = 0;//�Ѿ���ֵ��������Ŀ

				while (nCountDown)
				{
					int index;	//Ҫ��ֵ�������±�
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

						//�µ����ݽṹ sample point
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
	//��ʼ��Ȩֵͼ��4��,Ĭ��noval view���Ӵ� �� ��һ��ԭͼ��С��ͬ

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


		//��ȵ㶼���ٵģ���û��Ҫ��
		if (vvSamplePoint[i].size() < 100)
		{
			img->bValid = false;
			vImgInfo.push_back(img);
			continue;
		}

		//��ͼƬ�����ϱ�Ե һ�����ؿ�ȣ�Ӧ�ò��������ôϸ�ĳ����ذ�

		for (int m = 0; m < vDepthMat[i].rows; m++)
		{
			for (int n = 0; n < vDepthMat[i].cols; n++)
			{
				int nLabel = img->labelMat.at<int>(m, n);

				//SP�ĵ�һ�����أ���ʼ��SP��boundingbox �Լ� SP��fMedianDepth
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
					//boundingbox ����
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

				//��������Ϣ���뵽list����
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

		//������Ȳ�ֵ֮����Ȼ��target��sp�����Ϊhole
		for (int j = 0; j < vvTargetSPLabel[i].size(); j++)
		{
			img->vSP[vvTargetSPLabel[i][j]].bHole = true;
		}

		//����ԭ����ȱ�������Ϣ��sp�����Ϊ��origin
		for (int j = 0; j < vvNoneOriginSPLabel[i].size(); j++)
		{
			img->vSP[vvNoneOriginSPLabel[i][j]].bOrigin = false;
		}

		vImgInfo.push_back(img);
	}



	//����������֮������ӹ�ϵ��ͼ��


}

void DepthSynWidget::SpWarpLinearSystem(SuperPixel &sp, int iVertexDensity, int iExtPixel)
{
	//Ԥ����
	//����ÿһSP��Ӧ��Сmask mat �Լ��� sample��
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



		//������
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

	//��sp������grid��ͬʱ����ÿһ�������е�sample point
	SpOverlayGrid(sp, iVertexDensity);

	//�������Է�����
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

	//���ε���Ŀ���ϵ���Ŀ + 1
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



	//���ǽṹ�����samplepoint
	for (int i = 0; i < sp.vpSamples.size(); i++)
	{

		float fX_index = (sp.vpSamples[i]->Point2D.x - rectBounding.x) / fXstep;
		float fY_index = (sp.vpSamples[i]->Point2D.y - rectBounding.y) / fYstep;


		//��������������������ҡ��� ��Եʱ
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

	//�п����������� ��ȵ����ĿС��3 ����ĿΪ0�Ĵ����裬��Ŀ��Ϊ0 ���ڲ�����
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

			//�Ѷ�Ӧ��ȵ��ֵ����������
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
			//sp���sample��Ϣ�Ѿ��㹻�࣬���Ҳ��ض�vvsample���в��䣿��


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
	//δ֪��ʵ������ ������ ����������Ϊÿ������ �� ��������
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
	// // 		//ÿһ���ڵ���ܶ�Ӧ���������� �����иýڵ��Ep
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
	// 			//��ʼ׼�����Է���
	// 			for (int i = 0; i < tri.vSamplePoints.size();i++)
	// 			{
	// 				//ͶӰ��
	// 				Point2i p = tri.vSamplePoints[i]->Point2D;
	// 
	// 				//�����ζ���
	// 				TriVertex *v1 = tri.vVertex[0];
	// 				TriVertex *v2 = tri.vVertex[1];
	// 				TriVertex *v3 = tri.vVertex[2];
	// 
	// 
	// 				//��ȡ ��������ϵͳ ������, int ��Ӧ �����ţ�float��Ӧ����
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
	// 				//��ģ������й��ɿ���ֱ����ѭ����
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


	//��ÿһ�������������ѭ�� �Լ� ��ƫ����������ۼƵ�LinearSystem����
	for (int k = 0; k < vTriangle.size(); k++)
	{
		GridTriangle tri = vTriangle[k];

		//��ʼ׼�����Է���
		for (int i = 0; i < tri.vpSamplePoints.size(); i++)
		{
			//ͶӰ��
			Point2i p = tri.vpSamplePoints[i]->Point2D;

			//�����ζ���
			TriVertex *v1 = tri.vVertex[0];
			TriVertex *v2 = tri.vVertex[1];
			TriVertex *v3 = tri.vVertex[2];


			//��ȡ ��������ϵͳ ������, int ��Ӧ �����ţ�float��Ӧ����
			pair<int, float> IndexCor1;
			pair<int, float> IndexCor2;
			pair<int, float> IndexCor3;

			IndexCor1.first = v1->nIndex;
			IndexCor2.first = v2->nIndex;
			IndexCor3.first = v3->nIndex;

			IndexCor1.second = float((v2->vertex.y - v3->vertex.y) * (p.x - v3->vertex.x) + (v3->vertex.x - v2->vertex.x) * (p.y - v3->vertex.y)) / float((v2->vertex.y - v3->vertex.y) * (v1->vertex.x - v3->vertex.x) + (v3->vertex.x - v2->vertex.x) * (v1->vertex.y - v3->vertex.y));
			IndexCor2.second = float((v3->vertex.y - v1->vertex.y) * (p.x - v3->vertex.x) + (v1->vertex.x - v3->vertex.x) * (p.y - v3->vertex.y)) / float((v2->vertex.y - v3->vertex.y) * (v1->vertex.x - v3->vertex.x) + (v3->vertex.x - v2->vertex.x) * (v1->vertex.y - v3->vertex.y));
			IndexCor3.second = 1 - IndexCor1.second - IndexCor2.second;


			//��ģ������й��ɿ���ֱ����ѭ����

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

	//�ȸ��������������B����
	for (int m = 0; m < m_LinSys.BInfo.size(); m++)
	{
		//ʵ����ֻ�ü���ż���оͿ�����
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

			//��Ƭ����� ͶӰ������offset
			x.at<float>(0, 0) += float(this->m_spParentImg->src.cols + 1) / 2;
			x.at<float>(1, 0) += float(this->m_spParentImg->src.rows + 1) / 2;

			x.at<float>(1, 0) = this->m_spParentImg->src.rows - x.at<float>(1, 0);

			m_LinSys.B[m] += it->second * x.at<float>(0, 0);
			m_LinSys.B[m + 1] += it->second * x.at<float>(1, 0);

		}
	}
	//����eigen������Է�����
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

	//������
	return;

	//ת��Ϊcholmodģʽ�����ҿ�ʼ���(��ʽ������Ϊ�˷����Ķ�)
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
	// 	//��ʼ��A��B
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
	// 	//�����̵Ľ⣬ת��Ϊ��������
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
	//��ÿһ��������i��ÿһ����������j ���������ɵ����Ƕ����� ���ӽ��ϻ���Ƭ��
	for (int i = 0; i < vSP.size(); i++)
	{
		for (int j = 0; j < vSP[i].vTriangle.size(); j++)
		{
			vector<Point2i> P(3);
			for (int k = 0; k < 3; k++)
			{
				P[k] = vSP[i].vTriangle[j].vVertex[k]->vertex;
			}

			//�����ӽ��ϻ�������
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
		//ȥ����֮�������
		float xx = x.at<float>(0, 0);
		float xy = x.at<float>(1, 0);

		//ִ��ȥ����֮ǰ ��perspective����
		float cx = xx;
		float cy = xy;
		//���� ��һ���Ľ��
		float lastx = cx;
		float lasty = cy;

		do 
		{
			//������һ���Ľ��
			lastx = cx;
			lasty = cy;

			//��ʼ����
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


	//��ά�ռ�x����ļ���
	// 			x = camera.matRotation * X;
	// 			x = x + camera.T;
	// 			float fxValue = powf(x.at<float>(0, 0), 2) + powf(x.at<float>(1, 0), 2) + powf(x.at<float>(2, 0), 2);
	// 			x = x * camera.fFocal * (1 + camera.vK.at(0) * fxValue + camera.vK.at(1) * powf(fxValue, 2));
	// 			x.at<float>(2, 0) /= camera.fFocal;
	// 
	// 			//������ԭ�������ʵ�ʽ����Ҫ���� -1
	// 			x.at<float>(2, 0) *= -1;

	////////////////////////////////////////////////////////
	x = camera->matRotation * X;
	x = x + camera->T;
	x.at<float>(0, 0) /= (-x.at<float>(2, 0));
	x.at<float>(1, 0) /= (-x.at<float>(2, 0));
	float fxValue = powf(x.at<float>(0, 0), 2) + powf(x.at<float>(1, 0), 2);
	x.at<float>(0, 0) = x.at<float>(0, 0) * camera->fFocal * (1 + camera->vK.at(0) * fxValue + camera->vK.at(1) * powf(fxValue, 2));
	x.at<float>(1, 0) = x.at<float>(1, 0) * camera->fFocal * (1 + camera->vK.at(0) * fxValue + camera->vK.at(1) * powf(fxValue, 2));

	//һֱ�����x�ĵ�����������Ȼ��w
	float fDepth;
	float fM3 = sqrtf(powf(camera->matRotation.at<float>(2, 0), 2) + powf(camera->matRotation.at<float>(2, 1), 2) + powf(camera->matRotation.at<float>(2, 2), 2));
	fDepth = -x.at<float>(2, 0) * (determinant(camera->matRotation) < 0 ? -1 : 1) / fM3;

	Point point;

	//bundler�Ľ��������ԭ����ͼƬ�����ģ���Ҫ��ͼƬ����ת����opencv����,���ת�������곬����ͼƬ��Χ��������ѭ��

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

		//��ֵ�� ��û�����ӹ�ϵ
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


		//������
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

	//�����ã��ж��Ƿ��б��831������
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


		//��ÿһ��ͼƬ��ÿһ��TargetLabel ��dijskra


		for (int j = 0; j < vTargetSPLabel.size(); j++)
		{
			int nTarget = vTargetSPLabel[j];
			m_spParent->Dijskra(vVertexAdjoint, nTarget, mapTargetNearLabel[nTarget]);
		}


		//�õ��������ÿһ��SP��Ϊ����������Ӧlabel��vImgSkyΪ0
		if (!m_spParent->bSkyGraphCut)
		{
			m_spParent->getSkyLabel();	//�õ���������SP
			m_spParent->setSkyDepth();	//����������labelд�������Ϣ


			//����ǵ���ģʽ���򽫶�Ӧ��ͼƬ���
			if (m_spParent->bDebugSky)
			{
				m_spParent->writeSkyImg();
			}

		}


		// 	//��ʼ������Ⱥϳ�
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
	// 	//��һ����ʵû��Ҫ���Ҿ���������
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
	// 	//���Գ�����ʾtarget SP λ��
	// 	m_spParent->TargetSPBlend(i);
	// 
	// 	//���Գ���������ʾͬһ���Ӧ��ͬ�ӽǵ�λ��
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
	// 	// ��ʼѰ��ÿһ��TargetLabel��Ӧ�� ���ڽ�label����һ�����õ���ֱ��ͼѰ�ҿ������������40��SP label��ĳЩͼƬ����40����
	// 	m_spParent->CalLabHist(i);
	// 	//�����ڽ�ͼ��������ʽ����
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
	// 		//��ÿһ��ͼƬ��ÿһ��TargetLabel ��dijskra
	// 
	// 		
	// 		for (int j = 0; j < m_spParent->vvTargetSPLabel[i].size(); j++)
	// 		{
	// 			int nTarget = m_spParent->vvTargetSPLabel[i][j];
	// 			m_spParent->Dijskra(m_spParent->vvImgVertexAdjoint[i], nTarget, m_spParent->vmapImgTargetNearLabel[i][nTarget]);
	// 		}
	// 
	// 
	// 		//�õ��������ÿһ��SP��Ϊ����������Ӧlabel��vImgSkyΪ0
	// 		if (!m_spParent->bSkyGraphCut)
	// 		{
	// 			m_spParent->getSkyLabel();	//�õ���������SP
	// 			m_spParent->setSkyDepth();	//����������labelд�������Ϣ
	// 
	// 
	// 			//����ǵ���ģʽ���򽫶�Ӧ��ͼƬ���
	// 			if (m_spParent->bDebugSky)
	// 			{
	// 				m_spParent->writeSkyImg();
	// 			}
	// 
	// 		}
	// 
	// 
	// 		// 	//��ʼ������Ⱥϳ�
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
	//��ʼִ�� ������ȵĳ����طָ�
	QString sSeg;
	int nCurIndex = vLabelPixelCount.size();
	//ÿһ�������� ��Ӧ������
	vector<vector<Point2i>> vvSpPixles(vLabelPixelCount.size());
	for (int m = 0; m < LabelMat.rows; m++)
	{
		for (int n = 0; n < LabelMat.cols; n++)
		{
			int nLabel = LabelMat.at<int>(m, n);
			vvSpPixles[nLabel].push_back(Point2i(n, m));
		}
	}

	//���㵱ǰͼƬ����Ȱٷ��ƶ�
	float step = vDepthPoint.size() / 100.0f;
	qSort(vDepthPoint.begin(), vDepthPoint.end(), depthPointAscend);
	for (int m = 0; m < vDepthPoint.size() - 1; m++)
	{
		int nP = m / step;
		mapDepthPercentile.insert(pair<float, int >(vDepthPoint[m].fDepth, nP));
	}
	//���һ��������⴦��(�п��� m/step Ϊ100)
	mapDepthPercentile.insert(pair<float, int >(vDepthPoint[vDepthPoint.size() - 1].fDepth, 99));


	//��¼��������ά���λ��
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


	//��sp��ȶ�Ӧ��ֱ��ͼ��Ȼ��ֱ��ͼ�ļ�϶�����ݼ�϶�ж��Ƿ����

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

		//�ж��Ƿ����
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
					//�жϷ�֮��ľ��룬���������ƣ�����Ҫ�ָ�ҽ��ָ��������
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

		//��ʼ�����Ƿ���� ���в���
		if (bSeg)
		{
			vSegIndex.push_back(it->first);
			vector<Point2i>* vCurPixels = &vvSpPixles[it->first];

			//���ֱ�����������
			int nAdd = vSegDepth.size();
			nNumSp += nAdd;
			vLabelPixelCount.resize(vLabelPixelCount.size() + nAdd);
			vLabelPointCount.resize(vLabelPointCount.size() + nAdd);

			for (int i = 0; i < nAdd; i++)
			{
				mapSPDepth.insert(pair<int, vector<DepthPoint>>(nCurIndex + i, vector<DepthPoint>(0)));
			}

			//��depthPoint �Լ� samplePoint ���и���,depthpoint���Ѿ��ź����
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



			//���ؼ���ĵ������ҵ����ڽ�sample��
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

				//��ά�����Ϣ�Ѿ������
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
			//�����ѡ�г�����
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
		//����hole�����أ�ֱ��pass
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



			//�������������
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

					//��ȡ��������
					float alpha = float((p2->y - p3->y) * (x - p3->x) + (p3->x - p2->x) * (y - p3->y)) / float((p2->y - p3->y)*(p1->x - p3->x) + (p3->x - p2->x) * (p1->y - p3->y));

					float beta = float((p3->y - p1->y) * (x - p3->x) + (p1->x - p3->x) * (y - p3->y)) / float((p2->y - p3->y)*(p1->x - p3->x) + (p3->x - p2->x) * (p1->y - p3->y));

					float gamma = 1 - alpha - beta;

					//�õ����������ڣ������Ӧԭͼ����
					if (alpha >= 0 && beta >= 0 && gamma >= 0)
					{
						//������ ��Ӧԭͼ��λ��
						Point2d srcPoint;

						//��ɢ�ͣ����ڽ��� ��Ӧԭͼ��λ��
						Point2i iSrcPoint;


						Point2d *sp1 = &(sp->vVertexpoint[sp->vTriangleIndex[k][0]]->vertex);
						Point2d *sp2 = &(sp->vVertexpoint[sp->vTriangleIndex[k][1]]->vertex);
						Point2d *sp3 = &(sp->vVertexpoint[sp->vTriangleIndex[k][2]]->vertex);

						srcPoint.x = alpha * sp1->x + beta * sp2->x + gamma * sp3->x;
						srcPoint.y = alpha * sp1->y + beta * sp2->y + gamma * sp3->y;

						iSrcPoint.x = int(srcPoint.x + 0.5);
						iSrcPoint.y = int(srcPoint.y + 0.5);

						//�ж���ͶӰ���Ƿ��ڳ������ڣ������ �� ˫���Բ�ֵ + ��������
						if ((LabelMat.at<int>(iSrcPoint) == j) || (sp->bExtension && sp->matExtSP.at<uchar>(iSrcPoint.y - sp->ExtBounding.y, iSrcPoint.x - sp->ExtBounding.x)))
						{
							//�������ӣ�ͬʱ����Ȩ��
							

							SamplePoint curPoint;
							curPoint.Point2D = iSrcPoint;
							curPoint.bOrigin = false;
							curPoint.fDepth = sp->fMedianDepth;
							curPoint.BackProjection(img->src, img->camera);
							writeImform("pixel " + QString::number(iSrcPoint.x) +","  + QString::number(iSrcPoint.y) + " get");

							//�㵽ԭʼ���������
							Point3f pRay = (*(dynamic_cast<Point3f*>(curPoint.Point3D)) - img->camera->CalCenter());
							vector<float> ray;
							ray.push_back(pRay.x);
							ray.push_back(pRay.y);
							ray.push_back(pRay.z);

							//�㵽novalview������
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

							//����Ӧ����˫���Բ�ֵ ,Ŀǰ�������ڽ���
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
