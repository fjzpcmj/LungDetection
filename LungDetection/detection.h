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
	double Area = 0, Area_erode = 0, Area_open = 0, Area_close = 0;//���
	double ratio_erode;
	double ratio_open;
	double ratio_close;
	double Length = 0;//�ܳ�
	//�ڡ��⼰��Ե�ҶȾ�ֵ�뷽��
	double edgesum = 0, edgevar = 0;
	double insidesum = 0, insidevar = 0;
	double outsum = 0, outvar = 0;
	//�ڡ��⼰��Ե�Ҷ��ݶȾ�ֵ
	double Gradient_edge_avg = 0, Gradient_in_avg = 0, Gradient_out_avg = 0;
	//�Ǿ�����
	vector<double>avg;
	vector<double>var;
	//��׼�����򳤶Ⱦ�ֵ�뷽��
	double M1 = 0, M2 = 0;
	//һά�أ���ά��
	double  M3_1D = 0, M3_2D = 0;
	//���ݱȡ�͹�Ƕȡ����¶ȡ��⻬�ȡ��ֲڶ�
	double M4, M5, M6, M7 = 0, M8, M9;
	//gabor�˲���������48����ֵ��6���߶ȣ�8�����򣬹�48���˲���
	vector<double> gaborFeature;
	//soble�˲�����������ֵ�� X���� Y���� XY����
	vector<double> sobelFeature;
	//�Ҷȹ������󣬹�16��ֵ���ĸ�����˳��Ϊˮƽ����ֱ��45�ȣ�135�ȡ�ÿ���������������ء��Աȶȡ�����
	vector<double> glcmFeature;

	//TIC �������
	//vector<double>PE;
	//double AWS;//ƽ��������AWS
	//double AWPD;	//����������ǿ��
	//double WR_in;	//������
	//double WR_out;	//������
	//vector<double>T;	//�ҶȱȲ���

	//��ά����,��������������
	double surface;
	double volume;
	double entropy3D;
	double maocidu3D;//��άë�̶�
	//���ܻ�ָ�������������������壬��tic�Ķ����ӽ�������άë�̶�
};

Mat RegionGrow(Mat src, Point2i pt, int th);
void icvprCcaByTwoPass(const cv::Mat& _binImg, cv::Mat& _lableImg, int &maxlabel);
void FindLung(const cv::Mat& _lableImg, Mat &res, int maxlabel);
int otsuThreshold(Mat pic);


class Lungdetection{
private:
	Mat img;//original img
	Mat res;//result 255������
	Mat ROI;//roi image
	Mat LungTagMap;//�β���mask
	Mat LungImg;//
	Rect rec;//
	bool segmented = false;//�Ƿ��Ѿ����˷ָ���ڼ������

public:
	Lungdetection(){};
	void SetImg(Mat src, Rect rec);
	void NoduleSeg();
	void LungSeg();
	void ShowResult();
	void feature2D(feature &ret);
};