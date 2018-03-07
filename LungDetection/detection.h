#include <iostream>
#include <core.hpp>
#include <opencv2/opencv.hpp>
#include <highgui.hpp>  
#include <imgproc.hpp>
#include "opencv2/imgcodecs.hpp"
#include "vector"
#include "set"

using namespace std;
using namespace cv;

const double pi(3.14159265);

struct feature
{
	double Area = 0, Area_erode = 0, Area_open = 0, Area_close = 0;//面积
	double ratio_erode;
	double ratio_open;
	double ratio_close;
	double Length = 0;//周长
	//内、外及边缘灰度均值与方差
	double edgesum = 0, edgevar = 0;
	double insidesum = 0, insidevar = 0;
	double outsum = 0, outvar = 0;
	//内、外及边缘灰度梯度均值
	double Gradient_edge_avg = 0, Gradient_in_avg = 0, Gradient_out_avg = 0;
	//非均匀性
	vector<double>avg;
	vector<double>var;
	//标准化径向长度均值与方差
	double M1 = 0, M2 = 0;
	//一维熵，二维熵
	double  M3_1D = 0, M3_2D = 0;
	//横纵比、凸壳度、紧致度、光滑度、粗糙度
	double M4, M5, M6, M7 = 0, M8, M9;
	//gabor滤波特征，共48个数值。6个尺度，8个方向，共48个滤波器
	vector<double> gaborFeature;
	//soble滤波，共三个数值。 X方向， Y方向， XY方向
	vector<double> sobelFeature;
	//灰度共生矩阵，共16个值。四个方向，顺序为水平，垂直，45度，135度。每个方向有能量、熵、对比度、逆差矩
	vector<double> glcmFeature;

	//TIC 相关特征
	//vector<double>PE;
	//double AWS;//平均廓清率AWS
	//double AWPD;	//绝对廓清增强比
	//double WR_in;	//吸收率
	//double WR_out;	//廓清率
	//vector<double>T;	//灰度比参数

	//三维特征,表面积，体积，熵
	double surface;
	double volume;
	double entropy3D;
	double maocidu3D;//三维毛刺度
	//导管会分割进来？各个特征的意义，把tic的东西加进来，三维毛刺度
};

Mat RegionGrow(Mat src, Point2i pt, int th);
void icvprCcaByTwoPass(const cv::Mat& _binImg, cv::Mat& _lableImg, int &maxlabel);
void FindLung(const cv::Mat& _lableImg, Mat &res, int maxlabel);
int otsuThreshold(Mat pic);


class Lungdetection{
private:
	Mat img;//original img
	Mat res;//result 255代表结节
	Mat ROI;//roi image
	Mat LungTagMap;//肺部的mask
	Mat LungImg;//
	Rect rec;//
	bool segmented = false;//是否已经做了分割，用于计算参数

public:
	Lungdetection(){};
	void SetImg(Mat src, Rect rec);
	void NoduleSeg();
	void LungSeg();
	void ShowResult();
	void feature2D(feature &ret);
};