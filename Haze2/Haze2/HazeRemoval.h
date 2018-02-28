#include "stdafx.h"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include <math.h>
#include <cmath>
#include "io.h"
#include <opencv2/ximgproc.hpp>



using namespace cv;
using namespace std;
//将CV_8UC3类型的Mat转化为vec<float,3>类型的Mat
void UC3toVecFloat3(Mat src,Mat &dst);
//将浮点型Mat转化为uchar型
void VecFlaot3toUC3(Mat src,Mat &dst);

//获取当前路径下所有文件
void getFiles( string path, vector<string>& files);

//得到每个像素点RGB中最小的一位，并存储到maMINRGB中
void GetMinRGBImg(Mat src,Mat& maMinRGB);

//最小滤波
void MinFilter(Mat &maMinRGB,int nRadius);

//通过暗通道图片，获取A值(全球大气光成分，RGB表示)
Vec<float, 3> GetA(Mat src, Mat maMinRGB, float fPercentage, int &nARow, int &nAcol);

//通过得到的A值，利用公式，获取t值，以及t的取值范围
Mat GetT(Mat src,Vec<float,3> vA,int nRadius,float &fMinT,float &fMaxT);

//拉普拉斯掩膜 3*3
float LaplaceFilter(Mat src,int nrow,int ncol);

//获得L（matting Laplace Metrix)  vec<float,3>,最外面一圈像素不作处理先
Mat LaplaceMetrix(Mat src,float fEpson,float fDelta);

//solft matting 的T矩阵
void SoftMatting(Mat &T,Mat Laplace,float fDelta);

//解拉普拉斯矩阵，得到新的T
void SolveLaplaceMetrix(Mat src,Mat mT,Mat &T,float fEpson,float fDelta, int nARow, int nACol);




//根据第一次得到的J矩阵，重新计算t矩阵 并且根据T矩阵判定的深度，重新计算A
Mat ReSolveMatTandA(Mat J, Mat I, Vec<float, 3> &A,Mat &oriT, int nARow, int nACol);

//计算两幅图片的SSIM值，返回scalar类型，分别表示 BGR
Scalar getMSSIM(const Mat& i1, const Mat& i2);