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
//��CV_8UC3���͵�Matת��Ϊvec<float,3>���͵�Mat
void UC3toVecFloat3(Mat src,Mat &dst);
//��������Matת��Ϊuchar��
void VecFlaot3toUC3(Mat src,Mat &dst);

//��ȡ��ǰ·���������ļ�
void getFiles( string path, vector<string>& files);

//�õ�ÿ�����ص�RGB����С��һλ�����洢��maMINRGB��
void GetMinRGBImg(Mat src,Mat& maMinRGB);

//��С�˲�
void MinFilter(Mat &maMinRGB,int nRadius);

//ͨ����ͨ��ͼƬ����ȡAֵ(ȫ�������ɷ֣�RGB��ʾ)
Vec<float, 3> GetA(Mat src, Mat maMinRGB, float fPercentage, int &nARow, int &nAcol);

//ͨ���õ���Aֵ�����ù�ʽ����ȡtֵ���Լ�t��ȡֵ��Χ
Mat GetT(Mat src,Vec<float,3> vA,int nRadius,float &fMinT,float &fMaxT);

//������˹��Ĥ 3*3
float LaplaceFilter(Mat src,int nrow,int ncol);

//���L��matting Laplace Metrix)  vec<float,3>,������һȦ���ز���������
Mat LaplaceMetrix(Mat src,float fEpson,float fDelta);

//solft matting ��T����
void SoftMatting(Mat &T,Mat Laplace,float fDelta);

//��������˹���󣬵õ��µ�T
void SolveLaplaceMetrix(Mat src,Mat mT,Mat &T,float fEpson,float fDelta, int nARow, int nACol);




//���ݵ�һ�εõ���J�������¼���t���� ���Ҹ���T�����ж�����ȣ����¼���A
Mat ReSolveMatTandA(Mat J, Mat I, Vec<float, 3> &A,Mat &oriT, int nARow, int nACol);

//��������ͼƬ��SSIMֵ������scalar���ͣ��ֱ��ʾ BGR
Scalar getMSSIM(const Mat& i1, const Mat& i2);