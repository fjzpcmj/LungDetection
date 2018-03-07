#include <iostream>
#include <core.hpp>
#include <opencv2/opencv.hpp>
#include <highgui.hpp>  
#include <imgproc.hpp>
#include "opencv2/imgcodecs.hpp"
#include "detection.h"

using namespace cv;
using namespace std;
Mat showImg;
Lungdetection det;

void onMouse(int event, int x, int y, int flags, void* param){
	static Point pre_pt = (-1, -1);//��ʼ����  
	static Point cur_pt = (-1, -1);//ʵʱ����  
	char temp[16];
	Mat img;
	if (event == CV_EVENT_LBUTTONDOWN)//������£���ȡ��ʼ���꣬����ͼ���ϸõ㴦��Բ  
	{
		showImg.copyTo(img);//��ԭʼͼƬ���Ƶ�img��  
		sprintf_s(temp, "(%d,%d)", x, y);
		pre_pt = Point(x, y);
		putText(img, temp, pre_pt, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255, 0), 1, 8);//�ڴ�������ʾ����  
		circle(img, pre_pt, 2, Scalar(255, 0, 0, 0), CV_FILLED, CV_AA, 0);//��Բ  
		cv::imshow("indexImage", img);
	}
	else if (event == CV_EVENT_MOUSEMOVE && !(flags & CV_EVENT_FLAG_LBUTTON))//���û�а��µ����������ƶ��Ĵ�����  
	{
		showImg.copyTo(img);
		sprintf_s(temp, "(%d,%d)", x, y);
		cur_pt = Point(x, y);
		putText(img, temp, cur_pt, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255, 0));//ֻ��ʵʱ��ʾ����ƶ�������  
		cv::imshow("indexImage", img);
	}
	else if (event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_LBUTTON))//�������ʱ������ƶ�������ͼ���ϻ�����  
	{
		showImg.copyTo(img);
		sprintf_s(temp, "(%d,%d)", x, y);
		cur_pt = Point(x, y);
		putText(img, temp, cur_pt, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255, 0));
		rectangle(img, pre_pt, cur_pt, Scalar(0, 255, 0, 0), 1, 8, 0);//����ʱͼ����ʵʱ��ʾ����϶�ʱ�γɵľ���  
		cv::imshow("indexImage", img);
	}
	else if (event == CV_EVENT_LBUTTONUP)//����ɿ�������ͼ���ϻ�����  
	{
		//outputIndex.clear();
		//outputMask.clear();
		destroyWindow("res");
		showImg.copyTo(img);
		sprintf_s(temp, "(%d,%d)", x, y);
		cur_pt = Point(x, y);
		putText(img, temp, cur_pt, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255, 0));
		circle(img, pre_pt, 2, Scalar(255, 0, 0, 0), CV_FILLED, CV_AA, 0);
		rectangle(img, pre_pt, cur_pt, Scalar(0, 255, 0, 0), 1, 8, 0);//���ݳ�ʼ��ͽ����㣬�����λ���img��  
		imshow("indexImage", img);
		//��ȡ���ΰ�Χ��ͼ�񣬲����浽dst��  
		int width = abs(pre_pt.x - cur_pt.x);
		int height = abs(pre_pt.y - cur_pt.y);
		if (width == 0 || height == 0)
		{
			printf("width == 0 || height == 0");
			return;
		}
		cout << "doing segmentation..." << endl;
		Rect rect = Rect(min(cur_pt.x, pre_pt.x), min(cur_pt.y, pre_pt.y), width, height);
		//Mat res = showImg(rect).clone();
		det.SetImg(showImg,rect);
		det.LungSeg();
		det.NoduleSeg();
		det.ShowResult();
		feature fea;
		det.feature2D(fea);
		//writeResult(result);
		waitKey(0);
		


	}
}

void main(){
	Mat temp;
	//������ָ�ķ�CTͼ��
	//renji/2009216\2014051\2221767\2226859\2340733
	showImg = imread("renji/2221767.jpg");
	cv::imshow("indexImage", showImg);
	setMouseCallback("indexImage", onMouse);
	waitKey(0);
}