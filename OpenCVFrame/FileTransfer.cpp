#include "FileTransfer.h"
#include "opencvframe.h"

FT::FT(OpenCVFrame* parent) :QWidget(parent)
{
	m_spSfm = new sfm();
}

FT::~FT()
{

}

map<int, int> FT::NRDC2Match(string sSrc, string sRef, string sMatFile)
{
	bool bConSuccess;
	double threshold = 0.5;

	map<int, int> mapSrcRefCor;

	Mat src = imread(sSrc);
	Mat ref = imread(sRef);
	MATFile* nrdcFile = matOpen(sMatFile.c_str(), "r");
	int nNumMx;
	mxArray *pArrayDCF;
	mxArray *pArrayCon;
	double *pDCF;
	double *pCon;

	//arrayname仅仅是为了填充函数参数
	const char* arrayName;
	int nRow;
	int nCol;

	//裁剪后的维度
	int m;
	int n;

	int nOffset;

	if (nrdcFile == NULL)
	{
		QMessageBox::information(this, "Opening Mat File Failed!", "wo cao ni ma , wrong path!");
		return mapSrcRefCor;
	}

// 	char** dir = matGetDir(nrdcFile, &nNumMx);
	
	pArrayCon = matGetNextVariable(nrdcFile, &arrayName);
	pArrayDCF = matGetNextVariable(nrdcFile, &arrayName);
	int x = mxGetNumberOfDimensions(pArrayDCF);
	pDCF = mxGetPr(pArrayDCF);
	pCon = mxGetPr(pArrayCon);

	//开始进行匹配
	nRow = src.rows;
	nCol = src.cols;
	m = mxGetM(pArrayCon);
	n = mxGetN(pArrayCon);
	nOffset = (nRow - m + 1) / 2;

	for (int i = 0; i < m * n;i++)
	{
		if (*(pCon + i) >= threshold )
		{
			int mm = i % m;
			int mn = i / m;

			int curRow = mm + nOffset;
			int curCol = mn + nOffset;

			int refRow = nOffset + *(pDCF + m * n + i);
			int refCol = nOffset + *(pDCF + i);

			int nSrcId = curRow * nCol + curCol;
			int nRefId = refRow * nCol + refCol;

			mapSrcRefCor.insert(pair<int, int>(nSrcId, nRefId));

			//将匹配点信息推入sfm中
			m_spSfm->vPoints1.push_back(Point2f(curCol - n/2, curRow - m/2));
			m_spSfm->vPoints2.push_back(Point2f(refCol - n/2, refRow - m/2));
		}
	}
	
	//开始进行三维重构
	bConSuccess = m_spSfm->Reconstruction();

	QString qsSrc = QString::fromStdString(sSrc);
	m_spSfm->Output(true, true, qsSrc.left(qsSrc.lastIndexOf("/")), src, ref);
	
	matClose(nrdcFile);
	mxDestroyArray(pArrayCon);
	mxDestroyArray(pArrayDCF);
	return mapSrcRefCor;

}

void FT::slotNRDC2Match()
{
	m_spSfm;
	QStringList ssName =  QFileDialog::getOpenFileNames(this, tr("open result mat file and image files"), "D:/", tr("mat file ,src and ref images(*.mat *.jpg *.bmp)"));
	QString sMat, sSrc, sRef;
	
	for (auto it = ssName.begin(); it != ssName.end();it++)
	{
		if (it->endsWith(".mat"))
		{
			sMat = *it;
		}
		else if (it->contains("src",Qt::CaseInsensitive))
		{
			sSrc = *it;
		}
		else
		{
			sRef = *it;
		}
	}
	Mat matSrc = imread(sSrc.toStdString());
	
	map<int,int> result(NRDC2Match(sSrc.toStdString(), sRef.toStdString(), sMat.toStdString()));

	//开始写入匹配信息
	QFile File(sMat.left(sMat.lastIndexOf("/") + 1) + "matches.init.txt");
	File.open(QIODevice::WriteOnly);
	QTextStream text(&File);

	text << "0 1" << endl << result.size() << endl;
	for (auto it = result.begin();it != result.end();it++)
	{
		text << it->first << " " << it->second << endl;
	}
	File.close();

	//写入list文件以及key文件
	File.setFileName(sSrc.left(sSrc.lastIndexOf(".")) + ".key");
	File.open(QIODevice::WriteOnly);
	text.setDevice(&File);

	text << matSrc.rows * matSrc.cols << " " << "128" << endl;
	for (int m = 0; m < matSrc.rows;m++)
	{
		for (int n = 0; n < matSrc.cols;n++)
		{
			//row col scale rotation
			text << m << ".01 " << n << ".01 " << "100.01 " << "2.001" << endl;
			//特征向量
			for (int i = 0; i < 128;i++)
			{
				text << "1 ";
				if (i % 20 == 19)
				{
					text << endl;
				}
			}
			text << endl;
		}
	}

	File.setFileName(sRef.left(sRef.lastIndexOf(".")) + ".key");
	File.open(QIODevice::WriteOnly);
	text.setDevice(&File);

	text << matSrc.rows * matSrc.cols << " " << "128" << endl;
	for (int m = 0; m < matSrc.rows; m++)
	{
		for (int n = 0; n < matSrc.cols; n++)
		{
			//row col scale rotation
			text << m << ".01 " << n << ".01 " << "100.01 " << "2.001" << endl;
			//特征向量
			for (int i = 0; i < 128; i++)
			{
				text << "1 ";
				if (i % 20 == 19)
				{
					text << endl;
				}
			}
			text << endl;
		}
	}

	File.setFileName(sMat.left(sMat.lastIndexOf("/") + 1) + "list.txt");
	File.open(QIODevice::WriteOnly);
	text.setDevice(&File);

	text << "src/" + sSrc.right(sSrc.size() - sSrc.lastIndexOf("/") - 1) << " 0 " << "1401.71779"<<endl;
	text << "src/" + sRef.right(sRef.size() - sRef.lastIndexOf("/") - 1) << " 0 " << "1401.71779" << endl;


}

sfm::~sfm()
{

}

bool sfm::Reconstruction()
{
	if (vPoints2.size() != vPoints1.size() || !vPoints1.size())
	{
		return false;
	}

	//焦距暂且设置为4mm
	focal = 4 * 641 / 4.89;
	K = Mat(3, 3, CV_32FC1,Scalar(0));
	K.at<float>(0) = focal;
	K.at<float>(4) = focal;
	K.at<float>(8) = 1;
	
	Mat E = findEssentialMat(vPoints1, vPoints2, focal, Point2d(0, 0), RANSAC, 0.9999, 1.0, mask);

	float fFeasible = countNonZero(mask);
	if (fFeasible < 15 || fFeasible < vPoints1.size() * 0.5)
	{
		return false;
	}

	int nPass = recoverPose(E, vPoints1, vPoints2, R, T, 4, Point2d(0, 0), mask);

// 	if (nPass < fFeasible * 0.7)
// 	{
// 		return false;
// 	}

	//进行三角化
	Cam1 = Mat(3, 4, CV_32FC1);
	Cam2 = Mat(3, 4, CV_32FC1);

	Cam1(Range(0, 3), Range(0, 3)) = Mat::eye(3,3,CV_32FC1);
	Cam1.col(3) = Mat::zeros(3, 1, CV_32FC1);

	R.convertTo(Cam2(Range(0, 3), Range(0, 3)), CV_32FC1);
	T.convertTo(Cam2.col(3), CV_32FC1);

	Cam1 = K * Cam1;
	Cam2 = K * Cam2;

// 	//暂时先用所有的点进行三维重建
// 	vCorPoint1 = vPoints1;
// 	vCorPoint2 = vPoints2;
// 
	for (int i = 0; i < vPoints1.size();i++)
	{
		if (mask.at<char>(i,0))
		{
			vCorPoint1.push_back(vPoints1[i]);
			vCorPoint2.push_back(vPoints2[i]);
		}
	}
	triangulatePoints(Cam1, Cam2, vCorPoint1, vCorPoint2, matStructure);

	//将结果转化为3Dpoints
	for (int n = 0; n < matStructure.cols;n++)
	{
		v3DPoints.push_back(Point3f(matStructure.at<float>(0, n) / matStructure.at<float>(3, n), matStructure.at<float>(1, n) / matStructure.at<float>(3, n), matStructure.at<float>(2, n) / matStructure.at<float>(3, n)));
	}
}

void sfm::Output(bool bOut, bool bPly, QString dirPath, Mat src, Mat ref)
{
	if (bOut)
	{
		QFile fileOut(dirPath + "/bundle.out");
		fileOut.open(QIODevice::WriteOnly);
		QTextStream text(&fileOut);

		text << "# Bundle file v0.3" << endl;
		text << "2 " << v3DPoints.size() << endl;
		//第一个相机
		text << focal << " 0.0 0.0" << endl;
		for (int m = 0; m < 3; m++)
		{
			for (int n = 0; n < 3;n++)
			{
				if (m == n)
				{
					text << "1 ";
				}
				else
				{
					text << "0 ";
				}
			}
			text << endl;
		}
		text << "0.0 0.0 0.0" << endl;

		//第二个相机
		text << focal << " 0.0 0.0" << endl;
		for (int m = 0; m < R.rows;m++)
		{
			for (int n = 0; n < R.cols;n++)
			{
				text << R.at<double>(m, n) << " ";
			}
			text << endl;
		}
		for (int m = 0; m < T.rows;m++)
		{
			text << T.at<double>(m, 0) << " ";
		}
		text << endl;

		//点
		for (int i = 0; i < v3DPoints.size();i++)
		{
			text << v3DPoints[i].x << " " << v3DPoints[i].y << " " << v3DPoints[i].z << endl;
			Vec3b vBGR = (src.at<Vec3b>(vCorPoint1[i].y + src.rows / 2, vCorPoint1[i].x + src.cols / 2) + ref.at<Vec3b>(vCorPoint2[i].y + ref.rows / 2, vCorPoint2[i].x + ref.cols / 2)) / 2;
			text << vBGR[2] << " " << vBGR[1] << " " << vBGR[0] << endl;

			int idx1, idx2;
			idx1 = int(vCorPoint1[i].y + src.rows / 2) * src.cols + int(vCorPoint1[i].x + src.cols / 2);
			idx2 = int(vCorPoint2[i].y + ref.rows / 2) * ref.cols + int(vCorPoint2[i].x + ref.cols / 2);

			text << "2 " << "0 " << idx1 << " " << vCorPoint1[i].x << " " << vCorPoint1[i].y << " 1 " << idx2 << " " << vCorPoint2[i].x << " " << vCorPoint2[i].y << endl;
		}
	}
	if (bPly)
	{
		QFile fileOut(dirPath + "/points001.ply");
		fileOut.open(QIODevice::WriteOnly);
		QTextStream text(&fileOut);

		text << "ply" << endl
			<< "format ascii 1.0" << endl
			<< "element face 0" << endl
			<< "property list uchar int vertex_indices" << endl
			<< "element vertex " << v3DPoints.size() << endl
			<< "property float x" << endl
			<< "property float y" << endl
			<< "property float z" << endl
			<< "property uchar diffuse_red" << endl
			<< "property uchar diffuse_green" << endl
			<< "property uchar diffuse_blue" << endl
			<< "end_header" << endl;
		//输入相机位置
		text << "0.0 0.0 0.0 255 0 0" << endl;
		Mat C = -(R.inv() * T);
		text << C.at<double>(0, 0) << " " << C.at<double>(1, 0) << " " << C.at<double>(2, 0) << " 0 255 0" << endl;


		for (int i = 0; i < v3DPoints.size();i++)
		{
			Vec3b vBGR = (src.at<Vec3b>(vCorPoint1[i].y + src.rows / 2, vCorPoint1[i].x + src.cols / 2) + ref.at<Vec3b>(vCorPoint2[i].y + ref.rows / 2, vCorPoint2[i].x + ref.cols / 2)) / 2;
			text << v3DPoints[i].x << " " << v3DPoints[i].y << " " << v3DPoints[i].z << " " << vBGR[2] << " " << vBGR[1] << " " << vBGR[0] << endl;
		}
	}
}
