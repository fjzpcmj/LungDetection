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
	static Point pre_pt = (-1, -1);//初始坐标  
	static Point cur_pt = (-1, -1);//实时坐标  
	char temp[16];
	Mat img;
	if (event == CV_EVENT_LBUTTONDOWN)//左键按下，读取初始坐标，并在图像上该点处划圆  
	{
		showImg.copyTo(img);//将原始图片复制到img中  
		sprintf_s(temp, "(%d,%d)", x, y);
		pre_pt = Point(x, y);
		putText(img, temp, pre_pt, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255, 0), 1, 8);//在窗口上显示坐标  
		circle(img, pre_pt, 2, Scalar(255, 0, 0, 0), CV_FILLED, CV_AA, 0);//划圆  
		cv::imshow("indexImage", img);
	}
	else if (event == CV_EVENT_MOUSEMOVE && !(flags & CV_EVENT_FLAG_LBUTTON))//左键没有按下的情况下鼠标移动的处理函数  
	{
		showImg.copyTo(img);
		sprintf_s(temp, "(%d,%d)", x, y);
		cur_pt = Point(x, y);
		putText(img, temp, cur_pt, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255, 0));//只是实时显示鼠标移动的坐标  
		cv::imshow("indexImage", img);
	}
	else if (event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_LBUTTON))//左键按下时，鼠标移动，则在图像上划矩形  
	{
		showImg.copyTo(img);
		sprintf_s(temp, "(%d,%d)", x, y);
		cur_pt = Point(x, y);
		putText(img, temp, cur_pt, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255, 0));
		rectangle(img, pre_pt, cur_pt, Scalar(0, 255, 0, 0), 1, 8, 0);//在临时图像上实时显示鼠标拖动时形成的矩形  
		cv::imshow("indexImage", img);
	}
	else if (event == CV_EVENT_LBUTTONUP)//左键松开，将在图像上划矩形  
	{
		//outputIndex.clear();
		//outputMask.clear();
		destroyWindow("res");
		showImg.copyTo(img);
		sprintf_s(temp, "(%d,%d)", x, y);
		cur_pt = Point(x, y);
		putText(img, temp, cur_pt, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255, 0));
		circle(img, pre_pt, 2, Scalar(255, 0, 0, 0), CV_FILLED, CV_AA, 0);
		rectangle(img, pre_pt, cur_pt, Scalar(0, 255, 0, 0), 1, 8, 0);//根据初始点和结束点，将矩形画到img上  
		imshow("indexImage", img);
		//截取矩形包围的图像，并保存到dst中  
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
	//读入待分割的肺CT图像
	//renji/2009216\2014051\2221767\2226859\2340733
	showImg = imread("renji/2221767.jpg");
	cv::imshow("indexImage", showImg);
	setMouseCallback("indexImage", onMouse);
	waitKey(0);
}