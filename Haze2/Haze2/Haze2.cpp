// Haze2.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "HazeRemoval.h"
#include <string.h>
#include <iostream>
#include <ostream>
#include <fstream>
using namespace std;
int _tmain(int argc, _TCHAR* argv[])
{
	
	string sFilePath = "H:\\Haze";
	vector<string> sFiles;
	getFiles(sFilePath,sFiles);
	
	ofstream fout;
	fout.open(sFilePath + "\\out.txt", ios_base::app);
	
	
	for (int i = 0; i < sFiles.size();i++)
	{
		string::size_type idx = sFiles[i].find("_result");
		string::size_type idx2 = sFiles[i].find("_dc");
		string::size_type idx3 = sFiles[i].find("_modified");
		string::size_type idx4 = sFiles[i].find("_true");
		string::size_type idx5 = sFiles[i].find(".txt");

		if (idx != string::npos || idx2 != string::npos || idx3 != string::npos || idx4 != string::npos || idx5 != string::npos)
		{
			continue;
		}


			Mat src = imread(sFiles[i]);



		string sTrueFile(sFiles[i]);
		
		Mat srcTrue = imread(sTrueFile.replace(sTrueFile.size() - 9, 5, "true"));

		std::cout<<sFiles[i]<<"\n";
		fout << sFiles[i] << "\n";

		string sResultFile = sFiles[i];
		sResultFile = sResultFile.replace(sResultFile.size() - 9,5,"_result");
		string sDarkChannel = sFiles[i];
		sDarkChannel = sDarkChannel.replace(sDarkChannel.size() - 9,5,"_dc");
		string sModifiedResult = sFiles[i];
		sModifiedResult = sModifiedResult.replace(sModifiedResult.size() - 9,5, "_modified");
		Mat src2;
		UC3toVecFloat3(src,src2);

		// 	src.convertTo(src2,CV_64FC3,1.0/255,0.0);
		Mat dst = Mat_<Vec<float,3>>(src2.rows,src2.cols);
		
		Mat matDC = Mat_<float>(src2.rows,src2.cols);
		Mat mT = Mat_<float>(src2.rows,src2.cols);
		Vec<float,3> vA;
		// 	imshow("src",src);
		GetMinRGBImg(src2,matDC);
		MinFilter(matDC,20);
		imwrite(sDarkChannel,matDC);

		float fMinT = 1.0;
		float fMaxT = 0.0;
		int nARow;
		int nACol;
		vA = GetA(src2,matDC,0.001,nARow,nACol);//此过程为第一次计算的A
		mT = GetT(src2,vA,10,fMinT,fMaxT);//计算过程中采用W为0.95,未设置t的下限，下限在计算的时候使用



		cout.setf(ios_base::fixed);
		cout.precision(2);
		fout.setf(ios_base::fixed);
		fout.precision(2);


		cout << "T ranges:" << endl << fMinT << " to " << fMaxT << endl << endl;
		fout << "T ranges:" << endl << fMinT << " to " << fMaxT << endl << endl;


		ximgproc::guidedFilter(src2, mT, mT, 15, 8);
		for (int i = 0; i < src.rows; i++)
		{
			for (int j = 0; j < src.cols; j++)
			{


				float fCurT = (mT.at<float>(i,j) > 0.1f? mT.at<float>(i,j): 0.1f);
				fCurT = (fCurT <= 1.0f ? fCurT : 1.0f);

				float fSrc0 = src2.at<Vec<float,3>>(i,j)[0];
				float fSrc1 = src2.at<Vec<float,3>>(i,j)[1];
				float fSrc2 = src2.at<Vec<float,3>>(i,j)[2];
				float fDst0 = (fSrc0 - vA[0])/fCurT + vA[0];
				float fDst1 = (fSrc1 - vA[1])/fCurT + vA[1];
				float fDst2 = (fSrc2 - vA[2])/fCurT + vA[2];


				fDst0 = (fDst0 <= 0 ? 0 : fDst0);
				fDst1 = (fDst1 <= 0 ? 0 : fDst1);
				fDst2 = (fDst2 <= 0 ? 0 : fDst2);


				dst.at<Vec<float,3>>(i,j)[0] = (fDst0 > 255.0?255.0:fDst0);
				dst.at<Vec<float,3>>(i,j)[1] = (fDst1 > 255.0?255.0:fDst1);
				dst.at<Vec<float,3>>(i,j)[2] = (fDst2 > 255.0?255.0:fDst2);


			}
		}
		Mat dst2;



		//在亮度调节之前，显示SSIM
		Scalar scalar1 = getMSSIM(srcTrue, dst);
		cout.setf(ios_base::fixed);
		cout.precision(2);
		cout << "origin without modified A , T range changed as 0.1 ~ 1:" << endl << scalar1.val[0] << " " << scalar1.val[1] << " " << scalar1.val[2]<<endl;
		cout << "avarage" << (scalar1.val[0] + scalar1.val[1] + scalar1.val[2]) / 3.0 << endl << endl;
	
		fout << "origin without modified A , T range changed as 0.1 ~ 1:" << endl << scalar1.val[0] << " " << scalar1.val[1] << " " << scalar1.val[2] << endl;
		fout << "avarage" << (scalar1.val[0] + scalar1.val[1] + scalar1.val[2]) / 3.0 << endl << endl;


		VecFlaot3toUC3(dst,dst2);
		// 	imshow("dst",dst2);

		dst2 *= 1.1;




		//亮度调节之后 ，显示SSIM
		Scalar scalar2 = getMSSIM(srcTrue, dst2);
		cout.setf(ios_base::fixed);
		cout.precision(2);
		cout << "origin with enhanced brightness:" << endl << scalar2.val[0] << " " << scalar2.val[1] << " " << scalar2.val[2] << endl;
		cout << "avarage" << (scalar2.val[0] + scalar2.val[1] + scalar2.val[2]) / 3.0 << endl << endl;


		fout << "origin with enhanced brightness:" << endl << scalar2.val[0] << " " << scalar2.val[1] << " " << scalar2.val[2] << endl;
		fout << "avarage" << (scalar2.val[0] + scalar2.val[1] + scalar2.val[2]) / 3.0 << endl << endl;

		imwrite(sResultFile,dst2);





	   



















		//从这里开始是后续过程
		Mat newDst = dst.clone();

		cout << "A    " << float(vA[0]) / 255 << " " << float(vA[1]) / 255 << " " << float(vA[2]) / 255 << " " << endl;
		fout << "A    " << float(vA[0]) / 255 << " " << float(vA[1]) / 255 << " " << float(vA[2]) / 255 << " " << endl;

		Mat newT = ReSolveMatTandA(dst, src2, vA,mT,nARow,nACol);
		newT = GetT(src2, vA, 10, fMinT, fMaxT);//计算过程中采用W为0.95,未设置t的下限，下限在计算的时候使用

		
		cout << "new A" << float(vA[0]) / 255 << " " << float(vA[1]) / 255 << " " << float(vA[2]) / 255 << " " << endl;
		fout << "new A" << float(vA[0]) / 255 << " " << float(vA[1]) / 255 << " " << float(vA[2]) / 255 << " " << endl;


		//对T进行导向滤波
		ximgproc::guidedFilter(src2, newT, newT, 15, 0.000001);

		
		for (int i = 0; i < src.rows; i++)
		{
			for (int j = 0; j < src.cols; j++)
			{
				//这里把下限也修改了，fuck
				float fCurT = (newT.at<float>(i, j) > 0.1f ? newT.at<float>(i, j) : 0.1f);
				fCurT = (fCurT <= 1.0f ? fCurT : 1.0f);

				float fSrc0 = src2.at<Vec<float, 3>>(i, j)[0];
				float fSrc1 = src2.at<Vec<float, 3>>(i, j)[1];
				float fSrc2 = src2.at<Vec<float, 3>>(i, j)[2];
				float fDst0 = (fSrc0 - vA[0]) / fCurT + vA[0];
				float fDst1 = (fSrc1 - vA[1]) / fCurT + vA[1];
				float fDst2 = (fSrc2 - vA[2]) / fCurT + vA[2];


// 				//还增加了亮度，呵呵
// 				fDst0 *= 1.1;
// 				fDst1 *= 1.1;
// 				fDst2 *= 1.1;

				fDst0 = (fDst0 <= 0 ? 0 : fDst0);
				fDst1 = (fDst1 <= 0 ? 0 : fDst1);
				fDst2 = (fDst2 <= 0 ? 0 : fDst2);


				newDst.at<Vec<float, 3>>(i, j)[0] = (fDst0 > 255.0 ? 255.0 : fDst0);
				newDst.at<Vec<float, 3>>(i, j)[1] = (fDst1 > 255.0 ? 255.0 : fDst1);
				newDst.at<Vec<float, 3>>(i, j)[2] = (fDst2 > 255.0 ? 255.0 : fDst2);


			}
		}
		
		//进行校正之后SSIM并输出
		Scalar scalar3 = getMSSIM(srcTrue, newDst);
		cout.setf(ios_base::fixed);
		cout.precision(2);
		cout << "modefied T:" << endl<<" " << scalar3.val[0] << " " << scalar3.val[1] << " " << scalar3.val[2] << endl;
		cout << "avarage" << (scalar3.val[0] + scalar3.val[1] + scalar3.val[2]) / 3.0 << endl << endl;

		fout << "modefied T:" << endl << " " << scalar3.val[0] << " " << scalar3.val[1] << " " << scalar3.val[2] << endl;
		fout << "avarage" << (scalar3.val[0] + scalar3.val[1] + scalar3.val[2]) / 3.0 << endl << endl;


		Mat newDst2;
		VecFlaot3toUC3(newDst, newDst2);
		// 	imshow("dst",dst2);

		imwrite(sModifiedResult, newDst2);

		fout << "\n\n\n\n\n\n";
		cout << "\n\n\n\n\n\n";


	}

	getchar();
	return 0;
}

