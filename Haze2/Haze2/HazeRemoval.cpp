#include "HazeRemoval.h"

using namespace cv;

void getFiles( string path, vector<string>& files)  
{  
	//文件句柄   
	intptr_t   hFile   =   0;  
	//文件信息   
	struct _finddatai64_t fileinfo;  
	string p;  
	if((hFile = _findfirsti64(p.assign(path).append("\\*").c_str(),&fileinfo)) !=  -1)  
	{  
		do  
		{  
			//如果是目录,迭代之   
			//如果不是,加入列表   
			if((fileinfo.attrib &  _A_SUBDIR))  
			{  
				if(strcmp(fileinfo.name,".") != 0  &&  strcmp(fileinfo.name,"..") != 0)  
					getFiles( p.assign(path).append("\\").append(fileinfo.name), files);  
			}  
			else  
			{  
				files.push_back(p.assign(path).append("\\").append(fileinfo.name) );

			}  
		}while(_findnexti64(hFile, &fileinfo)  == 0);  
		_findclose(hFile);  
	}  
} 

void UC3toVecFloat3(Mat src,Mat &dst)
{
	dst = Mat_<Vec<float,3>>(src.rows,src.cols);
	for (int i = 0;i < src.rows ;i++)
	{
		for (int j = 0; j < src.cols; j++)
		{
			dst.at<Vec<float,3>>(i,j)[0] = src.at<Vec<uchar,3>>(i,j)[0];
			dst.at<Vec<float,3>>(i,j)[1] = src.at<Vec<uchar,3>>(i,j)[1];
			dst.at<Vec<float,3>>(i,j)[2] = src.at<Vec<uchar,3>>(i,j)[2];

		}
	}
}

void VecFlaot3toUC3(Mat src,Mat &dst)
{
	dst = Mat::zeros(src.rows,src.cols,CV_8UC3);
	for (int i = 0;i < src.rows ;i++)
	{
		for (int j = 0; j < src.cols; j++)
		{
			dst.at<Vec<uchar,3>>(i,j)[0] = (uchar)src.at<Vec<float,3>>(i,j)[0];
			dst.at<Vec<uchar,3>>(i,j)[1] = (uchar)src.at<Vec<float,3>>(i,j)[1];
			dst.at<Vec<uchar,3>>(i,j)[2] = (uchar)src.at<Vec<float,3>>(i,j)[2];

		}
	}

}


void GetMinRGBImg(Mat src,Mat& maMinRGB)
{

	for (int i = 0;i < src.rows;i++)
	{
		for (int j = 0; j < src.cols; j++)
		{
			float fMin = 255.0f;
			Vec<float,3> vRGB = src.at<Vec<float,3>>(i,j);
			for (int k = 0; k < 3;k++)
			{
				if (vRGB[k] < fMin)
				{
					fMin = vRGB[k];
				}
			}
			maMinRGB.at<float>(i,j) = fMin;

		}
	}
}

void MinFilter(Mat &maMinRGB,int nRadius)
{
	Mat tempImg = Mat_<float>(maMinRGB.rows,maMinRGB.cols);

	for (int i = 0;i < maMinRGB.rows;i++)
	{
		for (int j = 0;j < maMinRGB.cols;j++)
		{
			float fMIn = 255.0f;
			for (int m = ((i - nRadius) > 0?(i - nRadius):0); m < ((i + nRadius + 1) < maMinRGB.rows?(i + nRadius + 1):(maMinRGB.rows)); m++)
			{
				for (int n = ((j - nRadius) > 0?(j - nRadius):0); n < ((j + nRadius + 1) < maMinRGB.cols?(j + nRadius + 1):(maMinRGB.cols )); n++)
				{
					float fCur = maMinRGB.at<float>(m,n);
					if (maMinRGB.at<float>(m,n) < fMIn)
					{
						fMIn = maMinRGB.at<float>(m,n);
					}
				}

			}
			tempImg.at<float>(i,j) = fMIn;

		}
	}
	for (int i = 0; i < maMinRGB.rows;i++)
	{
		for (int j = 0; j < maMinRGB.cols; j++)
		{
			maMinRGB.at<float>(i,j) = tempImg.at<float>(i,j);
		}
	}
}


int comp(Vec<float,3> a,Vec<float,3> b)
{
	return a[2] > b[2];
}

Vec<float, 3> GetA(Mat src, Mat maMinRGB, float fPercentage,int &nARow, int &nAcol)
{
	Mat curSrc;
	VecFlaot3toUC3(src, curSrc);
	cvtColor(src, curSrc, CV_BGR2HLS);


	vector<Vec<float,3>> v3PosA;//存储每个像素值的位置以及暗通道
	int nCount = src.cols * src.rows * fPercentage;//获取A值的像素比例
	Vec<float,3> vA(0.0f,0.0f,0.0f);
	for (int i = 0; i < maMinRGB.rows;i++)
	{
		for (int j = 0;j < maMinRGB.cols;j++)
		{
			Vec<float,3> v3CurPix;
			v3CurPix[0] = i;
			v3CurPix[1] = j;
			v3CurPix[2] = maMinRGB.at<float>(i,j);
			v3PosA.push_back(v3CurPix);
		}
	}
	sort(v3PosA.begin(),v3PosA.end(),comp);
	vector<Vec<float,3>>::iterator vIt = v3PosA.begin();


	//这里取的是平均值，完成之后可以做改进


	for (int i = 0; i < nCount ; i++)
	{
		vA[0] += src.at<Vec<float,3>>(v3PosA[i][0],v3PosA[i][1])[0];
		vA[1] += src.at<Vec<float,3>>(v3PosA[i][0],v3PosA[i][1])[1];
		vA[2] += src.at<Vec<float,3>>(v3PosA[i][0],v3PosA[i][1])[2];

	}
	vA[0] /= nCount;
	vA[1] /= nCount;
	vA[2] /= nCount;





	//重新计算A，不是用平均值，而是用像素最大值
	float fMax = 0.0;
	
	for (int i = 0; i < nCount;i++)
	{
		int nRow = v3PosA[i][0];
		int nCol = v3PosA[i][1];
		float fIlumination = curSrc.at<Vec<float,3>>(nRow,nCol)[1];
		if (fIlumination > fMax)
		{
			fMax = fIlumination;
			vA = src.at<Vec<float, 3>>(nRow, nCol);
			nARow = nRow;
			nAcol = nCol;
		}

	}


	//真实值 234，242,255
// 	vA *= 1.1;
	return vA;

}


Mat GetT(Mat src, Vec<float, 3> vA, int nRadius, float &fMinT, float &fMaxT)
{
	Mat mT = Mat_<float>(src.rows, src.cols);
	Mat mNorImg = src.clone();//归一化原图
	Mat mNorRGBMinImg = Mat_<float>(src.rows,src.cols);
	for (int i = 0; i < src.rows;i++)
	{
		for (int j = 0; j < src.cols; j++)
		{
			mNorImg.at<Vec<float,3>>(i,j)[0] /=vA[0];
			mNorImg.at<Vec<float,3>>(i,j)[1] /=vA[1];
			mNorImg.at<Vec<float,3>>(i,j)[2] /=vA[2];

		}
	}
	GetMinRGBImg(mNorImg,mNorRGBMinImg);
	MinFilter(mNorRGBMinImg,nRadius);

	for (int i = 0; i < mT.rows ; i++)
	{
		for (int j = 0; j < mT.cols ; j++)
		{
			float fCurT = 1 - 0.95 * mNorRGBMinImg.at<float>(i,j);
			if (fCurT < fMinT)
			{
				fMinT = fCurT;
			}
			else if (fCurT > fMaxT)
			{
				fMaxT = fCurT;
			}

			mT.at<float>(i,j) = fCurT;
		}
	}
	return mT;
}

float LaplaceFilter(Mat src,int nrow,int ncol,float fEpson,float fDelta)
{
	vector<Mat> I(9);         //存储每个像素值的颜色
	Mat cov = Mat_<float>(3,3); //协方差矩阵
	Mat u = Mat_<float>(3,1);                    //当前窗口中颜色的平均值
	Mat inv = Mat_<Vec<float,3>>(3,3); //某逆矩阵
	Mat U3 = Mat_<float>::eye(3,3);
	Mat A;//逆矩阵
	Mat B;//当前像素矩阵减去平均值
	Mat C;//矩阵处理之后

	//初始化I
	int k = 0;
	for (int i = nrow - 1;i <= nrow + 1;i++)
	{
		for (int j = ncol - 1;j <= ncol + 1; j++)
		{
			I[k] = Mat_<float>(3,1);
			I[k].at<float>(0,0) = src.at<Vec<float,3>>(i,j)[0];
			I[k].at<float>(1,0) = src.at<Vec<float,3>>(i,j)[1];
			I[k].at<float>(2,0) = src.at<Vec<float,3>>(i,j)[2];

			for (int m = 0; m < 3;m++)
			{
				//为均值做贡献
				u.at<float>(m,0) += I[k].at<float>(m,0); 
				for (int n = 0; n < 3;n++ )
				{
					//为协方差做贡献
					cov.at<float>(m,n) += (I[k].at<float>(m,0) * I[k].at<float>(n,0));
				}
			}
			k++;

		}
	}
	//得到标准的平均值
	for(int m = 0 ;m < 3;m++)
	{
		u.at<float>(m,0) /= 9;
	}
	//得到标准的协方差矩阵
	for(int m = 0 ;m < 3;m++)
	{
		for(int n = 0;n < 3;n++)
		{
			cov.at<float>(m,n) = (cov.at<float>(m,n) -9* u.at<float>(m)* u.at<float>(n))/9;
		}
	}

	A = (cov + fEpson / 9 * U3);
	invert(A,A);




	B = I[4] - u;
	C = A * B;
	for (int i = 0; i++ ; i < 9)
	{
		Mat D = I[i] - u;
		D = D.t();
		Mat E = D * C;
		if (i == 5)
		{
			return (1 - (1 + E.at<float>(0,0))/9);
		}
		else
		{
			return - (1 + E.at<float>(0,0))/9;
		}
	}

}

Mat LaplaceMetrix(Mat src,float fEpson,float fDelta)
{
	Mat Laplace = Mat_<float>(src.rows,src.cols);
	for (int i = 1; i < src.rows - 1; i++)
	{
		for (int j = 1;j < src.cols - 1;j++)
		{
			Laplace.at<float>(i,j) = LaplaceFilter(src,i,j,fEpson,fDelta);
		}
	}
	return Laplace;
}

void SoftMatting(Mat &T,Mat Laplace,float fDelta)
{
	Mat mT = T.clone();
	Mat U = Mat_<float>::eye(T.rows,T.cols);
	for (int i = 0; i < Laplace.rows; i++)
	{
		Laplace.at<float>(i,0) = 1;
		Laplace.at<float>(i,Laplace.cols - 1);
	}
	for (int j = 0; j < Laplace.cols;j++)
	{
		Laplace.at<float>(0,j) == 0;
		Laplace.at<float>(Laplace.rows - 1,j) == 0;
	}

	Mat A = Laplace + 0.001 * U;
	invert(A,A);
	mT = A *fDelta * T;
	T = mT.clone();

}


void SolveLaplaceMetrix(Mat src,Mat mT,Mat &T,float fEpson,float fDelta)
{
	Mat LaplaceMetrix = Mat_<float>(src.rows * src.cols,src.rows * src.cols);

	Mat mT1 = Mat_<float>(mT.rows * mT.cols,1);
	for (int i = 0;i < mT.rows;i++)
	{
		for (int j = 0;j < mT.cols;j++)
		{
			mT1.at<float>(i * mT.cols + j,0) = fDelta * mT.at<float>(i,j);
		}
	}
	Mat T1 = Mat_<float>(mT.rows * mT.cols,1);

	for (int nrow = 1; nrow < src.rows - 1 ; nrow ++)
	{
		for (int ncol = 1;ncol < src.cols - 1;ncol ++)
		{
			vector<Mat> I(9);         //存储每个像素值的颜色
			Mat cov = Mat_<float>(3,3); //协方差矩阵
			Mat u = Mat_<float>(3,1);                    //当前窗口中颜色的平均值
			Mat inv = Mat_<Vec<float,3>>(3,3); //某逆矩阵
			Mat U3 = Mat_<float>::eye(3,3);
			Mat A;//逆矩阵
			Mat B;//当前像素矩阵减去平均值
			Mat C;//矩阵处理之后

			//初始化I
			int k = 0;
			for (int i = nrow - 1;i <= nrow + 1;i++)
			{
				for (int j = ncol - 1;j <= ncol + 1; j++)
				{
					I[k] = Mat_<float>(3,1);
					I[k].at<float>(0,0) = src.at<Vec<float,3>>(i,j)[0];
					I[k].at<float>(1,0) = src.at<Vec<float,3>>(i,j)[1];
					I[k].at<float>(2,0) = src.at<Vec<float,3>>(i,j)[2];

					for (int m = 0; m < 3;m++)
					{
						//为均值做贡献
						u.at<float>(m,0) += I[k].at<float>(m,0); 
					}
					k++;

				}
			}
			//得到标准的平均值
			for(int m = 0 ;m < 3;m++)
			{
				u.at<float>(m,0) /= 9;
			}
			//得到标准的协方差矩阵
			for (int i = nrow - 1;i <= nrow + 1;i++)
			{
				for (int j = ncol - 1;j <= ncol + 1; j++)
				{
					for (int m = 0;m < 3;m++ )
					{
						for (int n = 0;n < 3;n++)
						{
							cov.at<float>(m,n) +=(src.at<Vec<float,3>>(i,j)[m] - u.at<float>(m,0))*(src.at<Vec<float,3>>(i,j)[n] - u.at<float>(n,0));
						}
					}
				}
			}
			cov = cov/9;



			A = (cov + fEpson / 9 * U3);
			invert(A,A);

			for (int i = nrow - 1 ; i <= nrow + 1;i++ )
			{
				for (int j = ncol - 1; j <= ncol + 1;j++ )
				{
					int index_i = i * src.cols + j;
					Mat I1 = Mat_<float>(3,1);
					I1.at<float>(0,0) = src.at<Vec<float,3>>(i,j)[0] - u.at<float>(0,0);
					I1.at<float>(1,0) = src.at<Vec<float,3>>(i,j)[1] - u.at<float>(1,0);
					I1.at<float>(2,0) = src.at<Vec<float,3>>(i,j)[2] - u.at<float>(2,0);
					for (int m = nrow - 1 ; m <= nrow + 1;m++ )
					{
						for (int n = ncol - 1; n <= ncol + 1;n++ )
						{
							Mat result;
							int index_j = m * src.cols + n;
							Mat I2 = Mat_<float>(3,1);
							I2.at<float>(0,0) = src.at<Vec<float,3>>(m,n)[0] - u.at<float>(0,0);
							I2.at<float>(1,0) = src.at<Vec<float,3>>(m,n)[1] - u.at<float>(1,0);
							I2.at<float>(2,0) = src.at<Vec<float,3>>(m,n)[2] - u.at<float>(2,0);
							
							result = I1.t() * A * I2;
							if (index_i == index_j)
							{
								LaplaceMetrix.at<float>(index_i,index_j) += 1 - (1 + result.at<float>(0,0))/9;
							}
							else
							{
								LaplaceMetrix.at<float>(index_i,index_j) += 0 - (1 + result.at<float>(0,0))/9;

							}
						}
					}

				}
			}
			Mat U = Mat_<float>::eye(LaplaceMetrix.rows,LaplaceMetrix.cols);
			LaplaceMetrix = LaplaceMetrix + U * fDelta;

			invert(LaplaceMetrix,LaplaceMetrix);
			T1 = LaplaceMetrix * mT1;
	
			for (int i = 0;i < mT.rows;i++)
			{
				for (int j = 0;j < mT.cols;j++)
				{
					T.at<float>(i,j) = T1.at<float>(i * mT.cols + j,0);
				}
			}



		}
	}
}

Mat ReSolveMatTandA(Mat J, Mat I, Vec<float, 3> &A,Mat &oriT ,int nARow, int nACol)
{
// 	Vec<float, 3> curA1(A);
// 	curA1 /= 255;
	A *= 1.1;
	for (int i = 0; i < A.channels;i++)
	{
		if (A[i] >255)
		{
			A[i] = 255;
		}
	}
// 	Vec<float, 3> curA2(A);
// 	curA2 /= 255;

	//首先重新计算A，然后重新计算t
// 	A = (I.at<Vec<float, 3>>(nARow, nACol) - (oriT.at<float>(nARow, nACol) * J.at<Vec<float, 3>>(nARow, nACol))) / (1 - oriT.at<float>(nARow, nACol));
	



	Mat_<float> T(J.rows, J.cols);
	for (int i = 0; i < J.rows;i++)
	 {
		for (int j = 0; j < J.cols; j++)
		{
			for (int m = 0; m < 3;m++)
			{
				float fCurT = (A[m] - I.at<Vec<float, 3>>(i, j)[m]) / (A[m] - J.at<Vec<float, 3>>(i, j)[m]);
				T.at<float>(i, j) = (A[m] - I.at<Vec<float, 3>>(i, j)[m]) / (A[m] - J.at<Vec<float, 3>>(i, j)[m]);
				T.at<float>(i, j) = (T.at<float>(i, j) > 1) ? 1 : T.at<float>(i, j);
				T.at<float>(i, j) = (T.at<float>(i, j) < 0.1) ? 0.1 : T.at<float>(i, j);
			}
			  
		}
	 }

	
// 	T = oriT;
	return T;
}

Scalar getMSSIM(const Mat& i1, const Mat& i2)
{
	const double C1 = 6.5025, C2 = 58.5225;
	/***************************** INITS **********************************/
	int d = CV_32F;

	Mat I1, I2;
	i1.convertTo(I1, d);           // cannot calculate on one byte large values
	i2.convertTo(I2, d);

	Mat I2_2 = I2.mul(I2);        // I2^2
	Mat I1_2 = I1.mul(I1);        // I1^2
	Mat I1_I2 = I1.mul(I2);        // I1 * I2

	/*************************** END INITS **********************************/

	Mat mu1, mu2;   // PRELIMINARY COMPUTING
	GaussianBlur(I1, mu1, Size(11, 11), 1.5);
	GaussianBlur(I2, mu2, Size(11, 11), 1.5);

	Mat mu1_2 = mu1.mul(mu1);
	Mat mu2_2 = mu2.mul(mu2);
	Mat mu1_mu2 = mu1.mul(mu2);

	Mat sigma1_2, sigma2_2, sigma12;

	GaussianBlur(I1_2, sigma1_2, Size(11, 11), 1.5);
	sigma1_2 -= mu1_2;

	GaussianBlur(I2_2, sigma2_2, Size(11, 11), 1.5);
	sigma2_2 -= mu2_2;

	GaussianBlur(I1_I2, sigma12, Size(11, 11), 1.5);
	sigma12 -= mu1_mu2;

	///////////////////////////////// FORMULA ////////////////////////////////
	Mat t1, t2, t3;

	t1 = 2 * mu1_mu2 + C1;
	t2 = 2 * sigma12 + C2;
	t3 = t1.mul(t2);              // t3 = ((2*mu1_mu2 + C1).*(2*sigma12 + C2))

	t1 = mu1_2 + mu2_2 + C1;
	t2 = sigma1_2 + sigma2_2 + C2;
	t1 = t1.mul(t2);               // t1 =((mu1_2 + mu2_2 + C1).*(sigma1_2 + sigma2_2 + C2))

	Mat ssim_map;
	divide(t3, t1, ssim_map);      // ssim_map =  t3./t1;

	Scalar mssim = mean(ssim_map); // mssim = average of ssim map
	return mssim;
}

