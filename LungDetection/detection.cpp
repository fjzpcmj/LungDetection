#include "detection.h"

void Lungdetection::SetImg(Mat src,Rect re){
	if (src.channels() == 3){
		cvtColor(src, src, CV_BGR2GRAY);
	}
	img = src.clone();
	ROI = img(re).clone();
	rec = re;
	LungSeg();
	cout << src.channels()<< img.channels() << ROI.channels() << endl;
}


/***************************************************************************************
Function:  分割算法
Input:     ROI 待处理原图像 
Output:    结节的所在的区域 结节是白色，其他区域是黑色
Description: 用大津算法进行二值化，之后腐蚀断开结节与肺壁的连接，在区域中央附近进行区域生长，得到结节位置
Return:    Mat
Others:    NULL
***************************************************************************************/
void Lungdetection::NoduleSeg(){
	Mat gray;
	Mat temp;
	Mat binary;
	int searchLength = 10;
	//cvtColor(ROI, gray, CV_BGR2GRAY);
	gray = LungImg(rec).clone();
	threshold(gray, binary, otsuThreshold(gray), 255, CV_THRESH_BINARY);
	erode(binary, temp, Mat());
	//imshow("Nodulebinary", binary);
	//imshow("temp", temp);
	int rows = ROI.rows;
	int cols = ROI.cols;
	int x = cols / 2;
	int y = rows / 2;
	int searchx = x;
	int searchy = y;
	int DIR[4][2] = { { 0, -1 }, { 1, 0 }, { 0, 1 },{ -1, 0 } };
	bool find = false;
	if (temp.at<uchar>(y, x) != 255){
		for (int i = 1; i < searchLength+1; i++){
			if (find){
				break;
			}
			for (int j = 0; j < 4; j++){
				searchx = x + i*DIR[j][0];
				searchy = y+i*DIR[j][1];
				if (searchx < 0 || searchy< 0 || searchx >(cols - 1) || (searchy > rows - 1))
					continue;
				if (temp.at<uchar>(searchy, searchx) == 255){
					x = searchx;
					y = searchy;
					find = true;
					break;
				}
			}
		}
	}
	else{
		find = true;
	}
	if (find){
		res = RegionGrow(temp, Point2i(x, y), 2);
		dilate(res, res, Mat());
		segmented = true;
	}
	else{
		cout << "find no thing" << endl;
		res = temp;
	}
}

void Lungdetection::ShowResult(){
	destroyWindow("res");
	imshow("res",res);
}

void Lungdetection::LungSeg(){
	Mat binary;
	Mat labelimg;
	Mat lung;//通过形态学得到的粗糙的肺实质区域
	//threshold(img, binary, 0, 255, CV_THRESH_OTSU | CV_THRESH_BINARY_INV);
	threshold(img, binary, otsuThreshold(img), 255, CV_THRESH_BINARY_INV);
	cout << "otsuThreshold :"<< otsuThreshold(img)<<endl;
	//erode(binary,binary,Mat());
	//为什么要开运算？
	morphologyEx(binary, binary, MORPH_OPEN, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));

	int maxlabel;
	icvprCcaByTwoPass(binary/255, labelimg,maxlabel);
	FindLung(labelimg, lung, maxlabel);
	//threshold(img, binary, 40, 255,CV_THRESH_BINARY);
	imshow("binary", binary);
	imshow("beforeLung", lung);
	
	morphologyEx(lung, lung, MORPH_CLOSE, getStructuringElement(MORPH_ELLIPSE, Size(20, 20)),Point(-1,-1),1);
	vector<vector<Point> > contours;
	findContours(lung.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0, 0));
	cout << "contours.size()"<<contours.size() << endl;
	vector<vector<Point> >hull(contours.size());
	// Int type hull
	vector<vector<int>> hullsI(contours.size());
	// Convexity defects
	vector<vector<Vec4i>> defects(contours.size());
	for (size_t i = 0; i < contours.size(); i++)
	{
		convexHull(Mat(contours[i]), hull[i], false);
		// find int type hull
		convexHull(Mat(contours[i]), hullsI[i], false);
		// get convexity defects
		convexityDefects(Mat(contours[i]), hullsI[i], defects[i]);
	}
	vector<Vec4i> it;
	vector<vector<int>> rmidx;//需要去除的contours上的点的index
	LungTagMap = Mat::zeros(lung.size(), CV_8UC1);
	for (size_t i = 0; i < contours.size(); i++)
	{
		vector<int> rmidx_i;
		it = defects[i];//第i个轮廓的缺陷
		cout << "i : " << i << endl;
		for (int j = 0; j < it.size(); j++){
			int startidx = it[j][0];
			Point ptStart(contours[i][startidx]);
			int endidx = it[j][1];;
			Point ptEnd(contours[i][endidx]);
			int faridx = it[j][2];
			Point ptFar(contours[i][faridx]);
			double thr = sqrt(pow(ptEnd.x - ptStart.x, 2) + pow(ptEnd.y - ptStart.y, 2)) / (endidx - startidx);
			if (thr < 0.9 && (-thr * 320 + 400)>(endidx - startidx)){
				rmidx_i.push_back(startidx);
				rmidx_i.push_back(endidx);
				cout << "startidx:" << startidx << endl;
				cout << "endidx:" << endidx << endl;
				cout << endl;
				/*line(drawing, ptStart, ptFar, CV_RGB(0, 255, 0), 2);
				line(drawing, ptEnd, ptFar, CV_RGB(0, 255, 0), 2);
				circle(drawing, ptStart, 4, Scalar(0, 255, 0), 2);
				circle(drawing, ptEnd, 4, Scalar(255, 0, 0), 2);
				circle(drawing, ptFar, 4, Scalar(100, 0, 255), 2);*/
			}
			cout << "threshold:" << thr << endl;
		}
		rmidx.push_back(rmidx_i);
	}
	vector<vector<Point> > newcontours;
	for (int i = 0; i < contours.size(); i++)
	{
		if (rmidx[i].empty()){//第i个轮廓是否有缺陷
			newcontours.push_back(contours[i]);//无缺陷
		}
		else{
			vector<Point> contours_i;
			int k = 0;
			for (int j = 0; j < contours[i].size(); j++){
				if (j <= rmidx[i][k] || j >= rmidx[i][k+1]){
					contours_i.push_back(contours[i][j]);
				}
				else{
					j = rmidx[i][k+1];
					j--;
					if (rmidx[i].size()>(k + 2)){
						k = k + 2;
					}				
				}
			}
			newcontours.push_back(contours_i);
		}		
	}
	//cout << "newcontours 0:" << newcontours[0][0].x <<"  "<< newcontours[0][0].y << endl;
	//cout << "contours 0:" << contours[0][0].x << "  " << contours[0][0].y << endl;
	drawContours(LungTagMap, newcontours, -1, Scalar::all(255), -1, 8);
	LungImg = img.mul(LungTagMap / 255);
	imshow("LungImg", LungImg);
	imshow("lung", lung);
}

//计算病灶的二维特征
//Image为输入的是原始图片
//binaryImage为二值图，其中值为255的代表结节，值为0的为正常组织
//输出存放在ret中，ret内的数据在feature数据类型的定义中有说明
void Lungdetection::feature2D(feature &ret)
{
	if (!segmented){
		cout << "please do segmentation first" << endl;
		return;
	}
	Mat Image = ROI;
	Mat binaryImage = res;
	Mat Image_seg;
	if (binaryImage.rows == 0){
		cout << "binaryImage not found" << endl;
		return;
	}
	Image_seg = Image.mul(binaryImage / 255);
	imshow("Image_seg", Image_seg);
	imshow("binaryImage", binaryImage);
	imshow("Image", Image);
	//waitKey(0);
	//struct result ret;
	//提取canny边缘
	Mat cannyImage, cannyerode;
	Canny(binaryImage, cannyImage, 200, 255, 3);
	//通过canny边缘图，获取边缘坐标，计算面积周长
	vector<vector<Point>>precontours;//contours坐标中x为列，y为行
	findContours(cannyImage, precontours, RETR_EXTERNAL, CHAIN_APPROX_NONE);

	int maxArea = 0;
	int contouridx = 0;
	for (int i = 0; i< precontours.size(); i++)
	{
		int area = contourArea(precontours[i]);
		if (area>maxArea){
			contouridx = i;
			maxArea = area;
		}
	}
	vector<vector<Point>>contours;
	contours.push_back(precontours[contouridx]);

	for (int i = 0; i < (int)contours.size(); i++)
	{
		ret.Length += contours[i].size();
		std::cout << "周长:" << contours[i].size() << endl;
	}
	int maxContoursIndex = 0;
	int maxContoursSize = 0;
	//求vector边缘点坐标的最小值和最大值
	vector<int> rows_peri;
	vector<int> cols_peri;
	for (int k = 0; k < contours.size(); k++){
		int peri_size = contours[k].size();
		if (peri_size > maxContoursSize){
			maxContoursIndex = k;
			maxContoursSize = peri_size;
		}
		for (int i = 0; i < peri_size; i++){
			int x = contours[k][i].x;
			int y = contours[k][i].y;
			//cout << "x: " << x << endl;
			//cout << "y: " << y << endl;
			rows_peri.push_back(y);
			cols_peri.push_back(x);
		}
	}
	sort(rows_peri.begin(), rows_peri.end());
	sort(cols_peri.begin(), cols_peri.end());
	int total_length = rows_peri.size();
	int row_min_perim = rows_peri[0];
	int row_max_perim = rows_peri[total_length - 1];
	int col_min_perim = cols_peri[0];
	int col_max_perim = cols_peri[total_length - 1];
	cout << "行最小值： " << row_min_perim << endl;
	cout << "行最大值： " << row_max_perim << endl;
	cout << "列最小值： " << col_min_perim << endl;
	cout << "列最大值： " << col_max_perim << endl;

	//计算gabor、sobel、glcm特征
	//Mat minImg = Image(Rect(Point(col_min_perim, row_min_perim), Point(col_max_perim, row_max_perim))).clone();
	//imshow("minImg", minImg);
	//ret.gaborFeature = gaborFeature(minImg);
	//ret.sobelFeature = sobelFeature(minImg);
	//ret.glcmFeature = glcmFeature(minImg);
	//cout << "gabor特征:";
	//for (int i = 0; i < 48; i++){
	//	cout << ret.gaborFeature[i] << ",";
	//}
	//cout << endl;


	//面积及计算
	Mat Image_seg_erode, Image_seg_open, Image_seg_close;

	erode(Image_seg, Image_seg_erode, Mat(5, 5, CV_8U), Point(-1, -1), 1);
	morphologyEx(Image_seg, Image_seg_open, MORPH_OPEN, Mat(3, 3, CV_8U), Point(-1, -1), 1);
	morphologyEx(Image_seg, Image_seg_close, MORPH_CLOSE, Mat(3, 3, CV_8U), Point(-1, -1), 1);

	for (int i = 0; i <Image_seg_erode.rows; i++)
	{
		for (int j = 0; j <Image_seg_erode.cols; j++)
		{
			if (Image_seg_erode.at<uchar>(i, j) != 0)
			{
				ret.Area_erode = ret.Area_erode + 1;
			}
			if (Image_seg_open.at<uchar>(i, j) != 0)
			{
				ret.Area_open = ret.Area_open + 1;
			}
			if (Image_seg_close.at<uchar>(i, j) != 0)
			{
				ret.Area_close = ret.Area_close + 1;
			}
			if (Image_seg.at<uchar>(i, j) != 0)
			{
				ret.Area = ret.Area + 1;
			}
		}
	}
	ret.ratio_erode = ret.Area_erode / ret.Area;
	ret.ratio_open = ret.Area_open / ret.Area;
	ret.ratio_close = ret.Area_close / ret.Area;
	std::cout << "面积:" << ret.Area << endl;
	cout << "腐蚀后面积:" << ret.Area_erode << "\t与原面积之比:" << ret.ratio_erode << endl;
	cout << "开运算后面积:" << ret.Area_open << "\t与原面积之比:" << ret.ratio_open << endl;
	cout << "闭运算后面积:" << ret.Area_close << "\t与原面积之比:" << ret.ratio_close << endl;

	//外接圆计算//
	float MaxD = -1;//最大直径
	vector<Point2f>center(contours.size());
	vector<float>radius(contours.size());
	Point2f p;
	for (int i = 0; i < contours.size(); i++)
	{
		minEnclosingCircle(contours[i], center[i], radius[i]);
		if (2 * radius[i] > MaxD){
			MaxD = 2 * radius[i];
			p.y = center[i].y + 1;
			p.x = center[i].x + 1;
		}

		//center y : row
		//center x: col
		//cout << "center y: " << p.y << endl;
		//cout << "center x: " << p.x << endl;
	}

	cout << "最大直径MaxD:" << MaxD << endl;
	cout << "中心点:" << p << endl;
	//-------------------------------------------------------------------------------------------------//
	//边缘灰度值相关参数计算,输入Image,cannyImage,binaryImage//
	int counter1 = 0;
	for (int j = 0; j < Image.rows; j++)//双层循环遍历边缘检测图像
	{
		for (int i = 0; i < Image.cols; i++)
		{
			if (cannyImage.at<uchar>(j, i) != 0)
			{
				ret.edgesum = ret.edgesum + Image.at<uchar>(j, i);
				counter1 = counter1 + 1;
			}
		}
	}
	ret.edgesum = ret.edgesum / counter1;
	for (int j = 0; j < Image.rows; j++)//双层循环遍历边缘检测图像
	{
		for (int i = 0; i < Image.cols; i++)
		{
			if (cannyImage.at<uchar>(j, i) != 0)
			{
				ret.edgevar = ret.edgevar + pow((Image.at<uchar>(j, i) - ret.edgesum), 2);
			}
		}
	}
	//内部灰度值相关参数计算//
	int counter2 = 0;
	for (int j = 0; j < Image.rows; j++)//双层循环遍历二值图像
	{
		for (int i = 0; i < Image.cols; i++)
		{
			if (binaryImage.at<uchar>(j, i) != 0 && cannyImage.at<uchar>(j, i) == 0)
			{
				ret.insidesum = ret.insidesum + Image.at<uchar>(j, i);
				counter2 = counter2 + 1;
			}
		}
	}
	ret.insidesum = ret.insidesum / counter2;
	for (int j = 0; j < Image.rows; j++)//双层循环遍历边缘检测图像
	{
		for (int i = 0; i < Image.cols; i++)
		{
			if (binaryImage.at<uchar>(j, i) != 0 && cannyImage.at<uchar>(j, i) == 0)
			{
				ret.insidevar = ret.insidevar + pow((Image.at<uchar>(j, i) - ret.insidesum), 2);
			}
		}
	}
	//外界灰度值相关参数计算
	int counter3 = 0;
	for (int j = 0; j < Image.rows; j++)//双层循环遍历图像
	{
		for (int i = 0; i < Image.cols; i++)
		{

			if (binaryImage.at<uchar>(j, i) == 0 && cannyImage.at<uchar>(j, i) == 0)
			{
				ret.outsum = ret.outsum + Image.at<uchar>(j, i);
				counter3 = counter3 + 1;
			}
		}
	}
	ret.outsum = ret.outsum / counter3;
	for (int j = 0; j < Image.rows; j++)//双层循环遍历图像
	{
		for (int i = 0; i < Image.cols; i++)
		{
			if (binaryImage.at<uchar>(j, i) == 0 && cannyImage.at<uchar>(j, i) == 0)
			{
				ret.outvar = ret.outvar + pow((Image.at<uchar>(j, i) - ret.outsum), 2);
			}
		}
	}
	//内部、轮廓、外界的灰度均值与方差//
	ret.edgevar = ret.edgevar / counter1;
	ret.insidevar = ret.insidevar / counter2;
	ret.outvar = ret.outvar / counter3;
	cout << "内部灰度均值:" << ret.insidesum << "\t内部灰度方差:" << ret.insidevar << endl;
	cout << "边缘灰度均值:" << ret.edgesum << "\t边缘灰度方差:" << ret.edgevar << endl;
	cout << "外部灰度均值:" << ret.outsum << "\t外部灰度方差:" << ret.outvar << endl;

	//-------------------------------------------------------------------------------------------------//
	//边缘区域、外边缘区域、内边缘区域梯度均值计算//
	vector<double>Gradient_edge, Gradient_out, Gradient_in;
	double dx, dy;
	double Gradient_edge_avg = 0, Gradient_out_avg = 0, Gradient_in_avg = 0;
	//边缘区域//
	for (int i = 0; i < contours[maxContoursIndex].size(); i++)
	{
		Point g = contours[maxContoursIndex].at(i);
		dy = Image.at<uchar>(g.y + 1, g.x) - Image.at<uchar>(g.y, g.x);
		dx = Image.at<uchar>(g.y, g.x + 1) - Image.at<uchar>(g.y, g.x);
		Gradient_edge.push_back(pow(pow(dy + dx, 2), 0.5));
	}
	for (int i = 0; i < Gradient_edge.size(); i++)
	{
		Gradient_edge_avg = Gradient_edge_avg + Gradient_edge[i];
	}
	Gradient_edge_avg = Gradient_edge_avg / Gradient_edge.size();
	//外、内边缘区域//
	//设置边缘线宽
	int edgewidth = 1;
	int area_current = ret.Area;
	if (area_current < 36 && area_current > 16){
		edgewidth = 2;
	}
	if (area_current < 64 && area_current >= 36){
		edgewidth = 3;
	}
	if (area_current >= 64){
		edgewidth = 4;
	}
	cout << "初始设置的边缘宽：" << edgewidth << endl;
	while (((row_min_perim - edgewidth - 1 < 0) || (col_min_perim - edgewidth - 1 < 0) || (row_max_perim + edgewidth + 1 >= Image.rows) || (col_max_perim + edgewidth + 1 >= Image.cols)) && edgewidth > 1) {
		cout << "图片剪裁过小！正在修改边缘宽度" << endl;
		edgewidth -= 1;
	}
	cout << "当前设置的边缘宽：" << edgewidth << endl;
	if ((row_min_perim - edgewidth - 1 < 0) || (col_min_perim - edgewidth - 1< 0) || (row_max_perim + edgewidth + 1 >= Image.rows) || (col_max_perim + edgewidth + 1 >= Image.cols)){
		//cout << "图片剪裁过小！请重新剪裁" << endl;
		fprintf(stderr, "图片剪裁过小！无法计算梯度，请重新剪裁\n");
		//return ret;
	}
	else{
		for (int i = 0; i < contours[maxContoursIndex].size(); i++)
		{
			Point g = contours[maxContoursIndex].at(i);
			if (g.y <= p.y && g.x>p.x)//相对于中心点的第一象限 g.y <= p.x&&g.x>p.y
			{
				//Point g = contours[0].at(i);
				dy = Image.at<uchar>(g.y + 1 - edgewidth, g.x + edgewidth) - Image.at<uchar>(g.y - edgewidth, g.x + edgewidth);
				dx = Image.at<uchar>(g.y - edgewidth, g.x + 1 + edgewidth) - Image.at<uchar>(g.y - edgewidth, g.x + edgewidth);
				Gradient_out.push_back(pow(pow(dy + dx, 2), 0.5));
				dy = Image.at<uchar>(g.y + 1 + edgewidth, g.x - edgewidth) - Image.at<uchar>(g.y + edgewidth, g.x - edgewidth);
				dx = Image.at<uchar>(g.y + edgewidth, g.x + 1 - edgewidth) - Image.at<uchar>(g.y + edgewidth, g.x - edgewidth);
				Gradient_in.push_back(pow(pow(dy + dx, 2), 0.5));
			}
			if (g.y < p.y&&g.x <= p.x)//相对于中心点的第二象限 g.y < p.x&&g.x <= p.y
			{
				dy = Image.at<uchar>(g.y + 1 - edgewidth, g.x - edgewidth) - Image.at<uchar>(g.y - edgewidth, g.x - edgewidth);
				dx = Image.at<uchar>(g.y - edgewidth, g.x + 1 - edgewidth) - Image.at<uchar>(g.y - edgewidth, g.x - edgewidth);
				Gradient_out.push_back(pow(pow(dy + dx, 2), 0.5));
				dy = Image.at<uchar>(g.y + 1 + edgewidth, g.x + edgewidth) - Image.at<uchar>(g.y + edgewidth, g.x + edgewidth);
				dx = Image.at<uchar>(g.y + edgewidth, g.x + 1 + edgewidth) - Image.at<uchar>(g.y + edgewidth, g.x + edgewidth);
				Gradient_in.push_back(pow(pow(dy + dx, 2), 0.5));
			}
			if (g.y >= p.y && g.x<p.x)//相对于中心点的第三象限 g.y >= p.x&&g.x<p.y
			{
				dy = Image.at<uchar>(g.y + 1 + edgewidth, g.x - edgewidth) - Image.at<uchar>(g.y + edgewidth, g.x - edgewidth);
				dx = Image.at<uchar>(g.y + edgewidth, g.x + 1 - edgewidth) - Image.at<uchar>(g.y + edgewidth, g.x - edgewidth);
				Gradient_out.push_back(pow(pow(dy + dx, 2), 0.5));
				dy = Image.at<uchar>(g.y + 1 - edgewidth, g.x + edgewidth) - Image.at<uchar>(g.y - edgewidth, g.x + edgewidth);
				dx = Image.at<uchar>(g.y - edgewidth, g.x + 1 + edgewidth) - Image.at<uchar>(g.y - edgewidth, g.x + edgewidth);
				Gradient_in.push_back(pow(pow(dy + dx, 2), 0.5));
			}
			if (g.y > p.y&&g.x >= p.x)//相对于中心点的第四象限 g.y > p.x&&g.x >= p.y
			{
				dy = Image.at<uchar>(g.y + 1 + edgewidth, g.x + edgewidth) - Image.at<uchar>(g.y + edgewidth, g.x + edgewidth);
				dx = Image.at<uchar>(g.y + edgewidth, g.x + 1 + edgewidth) - Image.at<uchar>(g.y + edgewidth, g.x + edgewidth);
				Gradient_out.push_back(pow(pow(dy + dx, 2), 0.5));
				dy = Image.at<uchar>(g.y + 1 - edgewidth, g.x - edgewidth) - Image.at<uchar>(g.y - edgewidth, g.x - edgewidth);
				dx = Image.at<uchar>(g.y - edgewidth, g.x + 1 - edgewidth) - Image.at<uchar>(g.y - edgewidth, g.x - edgewidth);
				Gradient_in.push_back(pow(pow(dy + dx, 2), 0.5));
			}
		}
		for (int i = 0; i < Gradient_out.size(); i++)
		{
			Gradient_out_avg = Gradient_out_avg + Gradient_out[i];
		}
		Gradient_out_avg = Gradient_out_avg / Gradient_out.size();
		for (int i = 0; i < Gradient_in.size(); i++)
		{
			Gradient_in_avg = Gradient_in_avg + Gradient_in[i];
		}
		Gradient_in_avg = Gradient_in_avg / Gradient_in.size();
		cout << "边缘梯度:" << Gradient_edge_avg << endl;
		cout << "内边缘梯度:" << Gradient_in_avg << endl;
		cout << "外边缘梯度:" << Gradient_out_avg << endl;
		ret.Gradient_edge_avg = Gradient_edge_avg;
		ret.Gradient_in_avg = Gradient_in_avg;
		ret.Gradient_out_avg = Gradient_out_avg;
	}
	//---------------------------------------------------------------------------------------------//
	//非均匀性//
	vector<double>orientation1, orientation2, orientation3, orientation4;
	vector<double>orientation5, orientation6, orientation7, orientation8;
	double row_temp = 15;//第j行
	double col_temp = 16;//第i列
	double avg_row = p.y;//中心点在第几行
	double avg_col = p.x;//中心点在第几列
	double dy_atan, dx_atan;//用来算atan
	if (ret.Area > 36)
	{
		for (int j = 0; j < binaryImage.rows; j++)//双层循环遍历图像
		{
			for (int i = 0; i < binaryImage.cols; i++)//双层循环遍历图像
			{
				if (binaryImage.at<uchar>(j, i) != 0)
				{
					row_temp = j;//第j行
					col_temp = i;//第i列
					//cout << "不等于0:"<<row_temp << "  " << col_temp << endl;
					if (j == avg_row && i == avg_col)//j行i列的点如果在中心点上，算入第一区域中
					{
						orientation1.push_back(Image.at<uchar>(j, i));
					}
					if (row_temp < avg_row && col_temp > avg_col)//第一象限
					{

						dy_atan = abs(avg_row - row_temp);
						dx_atan = abs(col_temp - avg_col);
						if ((atan(dy_atan / dx_atan)) <= (pi / 4.0))//0-45度区域
						{

							orientation1.push_back(Image.at<uchar>(row_temp, col_temp));
						}
						if ((atan(dy_atan / dx_atan)) > (pi / 4.0))//45-90度区域
						{
							orientation2.push_back(Image.at<uchar>(row_temp, col_temp));
						}
					}
					if ((row_temp == avg_row) && (col_temp > avg_col))//点在x正轴算第一区域
					{
						orientation1.push_back(Image.at<uchar>(row_temp, col_temp));
					}
					if (row_temp < avg_row && col_temp == avg_col)//点在y正轴算第二区域
					{
						orientation2.push_back(Image.at<uchar>(row_temp, col_temp));
					}
					if (row_temp < avg_row && col_temp < avg_col)//第二象限
					{
						dy_atan = abs(avg_row - row_temp);
						dx_atan = abs(col_temp - avg_col);
						if ((atan(dy_atan / dx_atan)) < (pi / 4))//135-180度区域
						{
							orientation4.push_back(Image.at<uchar>(row_temp, col_temp));
						}
						if ((atan(dy_atan / dx_atan)) >= (pi / 4))//90-135度区域
						{
							orientation3.push_back(Image.at<uchar>(row_temp, col_temp));
						}
					}
					if ((row_temp == avg_row) && (col_temp < avg_col))//点在x负轴算第4区域
					{
						orientation4.push_back(Image.at<uchar>(row_temp, col_temp));
					}
					if (row_temp > avg_row && col_temp < avg_col)//第三象限
					{
						dy_atan = abs(avg_row - row_temp);
						dx_atan = abs(col_temp - avg_col);
						if ((atan(dy_atan / dx_atan)) <= (pi / 4))
						{
							orientation5.push_back(Image.at<uchar>(row_temp, col_temp));
						}
						if ((atan(dy_atan / dx_atan)) >(pi / 4))
						{
							orientation6.push_back(Image.at<uchar>(row_temp, col_temp));
						}
					}
					if (row_temp > avg_row && col_temp == avg_col)//点在y负轴算第六区域
					{
						orientation6.push_back(Image.at<uchar>(row_temp, col_temp));
					}
					if (row_temp > avg_row && col_temp > avg_col)
					{
						dy_atan = abs(avg_row - row_temp);
						dx_atan = abs(col_temp - avg_col);
						if ((atan(dy_atan / dx_atan)) <= (pi / 4))
						{
							orientation7.push_back(Image.at<uchar>(row_temp, col_temp));
						}
						if ((atan(dy_atan / dx_atan)) > (pi / 4))
						{
							orientation8.push_back(Image.at<uchar>(row_temp, col_temp));
						}
					}
				}

			}
		}
		double sum = 0, squa = 0;
		for (int i = 0; i < orientation1.size(); i++)
		{
			sum = sum + orientation1[i];
		}
		for (int i = 0; i < orientation1.size(); i++)
		{
			squa = squa + pow((orientation1[i] - sum / orientation1.size()), 2);
		}
		(ret.avg).push_back(sum / orientation1.size());
		(ret.var).push_back(squa / orientation1.size());
		sum = 0, squa = 0;
		for (int i = 0; i < orientation2.size(); i++)
		{
			sum = sum + orientation2[i];
		}
		for (int i = 0; i < orientation2.size(); i++)
		{
			squa = squa + pow((orientation2[i] - sum / orientation2.size()), 2);
		}
		(ret.avg).push_back(sum / orientation2.size());
		(ret.var).push_back(squa / orientation2.size());
		sum = 0, squa = 0;
		for (int i = 0; i < orientation3.size(); i++)
		{
			sum = sum + orientation3[i];
		}
		for (int i = 0; i < orientation3.size(); i++)
		{
			squa = squa + pow((orientation3[i] - sum / orientation3.size()), 2);
		}
		(ret.avg).push_back(sum / orientation3.size());
		(ret.var).push_back(squa / orientation3.size());
		sum = 0, squa = 0;
		for (int i = 0; i < orientation4.size(); i++)
		{
			sum = sum + orientation4[i];
		}
		for (int i = 0; i < orientation4.size(); i++)
		{
			squa = squa + pow((orientation4[i] - sum / orientation4.size()), 2);
		}
		(ret.avg).push_back(sum / orientation4.size());
		(ret.var).push_back(squa / orientation4.size());
		sum = 0, squa = 0;
		for (int i = 0; i < orientation5.size(); i++)
		{
			sum = sum + orientation5[i];
		}
		for (int i = 0; i < orientation5.size(); i++)
		{
			squa = squa + pow((orientation5[i] - sum / orientation5.size()), 2);
		}
		(ret.avg).push_back(sum / orientation5.size());
		(ret.var).push_back(squa / orientation5.size());
		sum = 0, squa = 0;
		for (int i = 0; i < orientation6.size(); i++)
		{
			sum = sum + orientation6[i];
		}
		for (int i = 0; i <orientation6.size(); i++)
		{
			squa = squa + pow((orientation6[i] - sum / orientation6.size()), 2);
		}
		(ret.avg).push_back(sum / orientation6.size());
		(ret.var).push_back(squa / orientation6.size());
		sum = 0, squa = 0;
		for (int i = 0; i < orientation7.size(); i++)
		{
			sum = sum + orientation7[i];
		}
		for (int i = 0; i < orientation7.size(); i++)
		{
			squa = squa + pow((orientation7[i] - sum / orientation7.size()), 2);
		}
		(ret.avg).push_back(sum / orientation7.size());
		(ret.var).push_back(squa / orientation7.size());
		sum = 0, squa = 0;
		for (int i = 0; i < orientation8.size(); i++)
		{
			sum = sum + orientation8[i];
		}
		for (int i = 0; i < orientation8.size(); i++)
		{
			squa = squa + pow((orientation8[i] - sum / orientation8.size()), 2);
		}
		(ret.avg).push_back(sum / orientation8.size());
		(ret.var).push_back(squa / orientation8.size());
		for (int i = 0; i < 8; i++)
		{
			cout << i + 1 << "个方向灰度均值" << ret.avg[i] << "\t" << i + 1 << "个方向灰度方差" << ret.var[i] << endl;
		}

	}
	else
	{
		for (int j = 0; j < binaryImage.rows; j++)//双层循环遍历图像
		{
			for (int i = 0; i < binaryImage.cols; i++)//双层循环遍历图像
			{
				if (binaryImage.at<uchar>(j, i) != 0)
				{
					row_temp = j;//第j行
					col_temp = i;//第i列
					if (j == avg_row && i == avg_col)//j行i列的点如果在中心点上，算入第一象限中
					{
						orientation1.push_back(Image.at<uchar>(j, i));
					}
					if (row_temp < avg_row && col_temp > avg_col)//第一象限
					{
						orientation1.push_back(Image.at<uchar>(j, i));
					}
					if ((row_temp == avg_row) && (col_temp > avg_col))//点在x正轴算第一象限
					{
						orientation1.push_back(Image.at<uchar>(row_temp, col_temp));
					}
					if (row_temp < avg_row && col_temp == avg_col)//点在y正轴算第二象限
					{
						orientation2.push_back(Image.at<uchar>(row_temp, col_temp));
					}
					if (row_temp < avg_row && col_temp < avg_col)//第二象限
					{
						orientation2.push_back(Image.at<uchar>(row_temp, col_temp));
					}
					if ((row_temp == avg_row) && (col_temp < avg_col))//点在x负轴算第三象限
					{
						orientation3.push_back(Image.at<uchar>(row_temp, col_temp));
					}
					if (row_temp > avg_row && col_temp < avg_col)//第三象限
					{
						orientation3.push_back(Image.at<uchar>(row_temp, col_temp));
					}
					if (row_temp > avg_row && col_temp == avg_col)//点在y负轴算第四象限
					{
						orientation4.push_back(Image.at<uchar>(row_temp, col_temp));
					}
					if (row_temp > avg_row && col_temp > avg_col)
					{
						orientation4.push_back(Image.at<uchar>(row_temp, col_temp));
					}
				}
			}
		}

		double sum = 0, squa = 0;
		for (int i = 0; i < orientation1.size(); i++)
		{
			sum = sum + orientation1[i];
		}
		for (int i = 0; i < orientation1.size(); i++)
		{
			squa = squa + pow((orientation1[i] - sum / orientation1.size()), 2);
		}
		(ret.avg).push_back(sum / orientation1.size());
		(ret.var).push_back(squa / orientation1.size());
		sum = 0, squa = 0;
		for (int i = 0; i < orientation2.size(); i++)
		{
			sum = sum + orientation2[i];
		}
		for (int i = 0; i < orientation2.size(); i++)
		{
			squa = squa + pow((orientation2[i] - sum / orientation2.size()), 2);
		}
		(ret.avg).push_back(sum / orientation2.size());
		(ret.var).push_back(squa / orientation2.size());
		sum = 0, squa = 0;
		for (int i = 0; i < orientation3.size(); i++)
		{
			sum = sum + orientation3[i];
		}
		for (int i = 0; i < orientation3.size(); i++)
		{
			squa = squa + pow((orientation3[i] - sum / orientation3.size()), 2);
		}
		(ret.avg).push_back(sum / orientation3.size());
		(ret.var).push_back(squa / orientation3.size());
		sum = 0, squa = 0;
		for (int i = 0; i < orientation4.size(); i++)
		{
			sum = sum + orientation4[i];
		}
		for (int i = 0; i < orientation4.size(); i++)
		{
			squa = squa + pow((orientation4[i] - sum / orientation4.size()), 2);
		}
		(ret.avg).push_back(sum / orientation4.size());
		(ret.var).push_back(squa / orientation4.size());
		for (int i = 0; i < 4; i++)
		{
			cout << i + 1 << "个方向灰度均值" << ret.avg[i] << "\t" << i + 1 << "个方向灰度方差" << ret.var[i] << endl;
		}
	}


	//---------------------------------------------------------------------------------------------//
	//M1//
	Point2f p1;
	int x, y;
	vector<double>d;
	for (int i = 0; i <contours[maxContoursIndex].size(); i++)
	{
		p1 = contours[maxContoursIndex].at(i);
		d.push_back(sqrt(pow((p1.x - p.x), 2) + pow((p1.y - p.y), 2)) / MaxD);
	}

	for (int i = 0; i < contours[maxContoursIndex].size(); i++)
	{
		ret.M1 = ret.M1 + d[i];
	}
	ret.M1 = ret.M1 / contours[maxContoursIndex].size();

	//---------------------------------------------------------------------------------------------//
	//M2//
	for (int i = 0; i <contours[maxContoursIndex].size(); i++)
	{
		ret.M2 = ret.M2 + pow((d[i] - ret.M1), 2);
	}
	ret.M2 = sqrt(ret.M2 / (contours[maxContoursIndex].size() - 1));
	//---------------------------------------------------------------------------------------------//
	//M3//
	double maxd, mind, space;
	vector<double>prob(100, 0);
	for (int i = 0; i < contours[maxContoursIndex].size() - 1; i++)//判断M1中d向量的最大最小值，
	{
		maxd = d[i];
		mind = d[i];
		if (d[i + 1]>d[i])
		{
			maxd = d[i + 1];
		}
		if (d[i + 1]<d[i])
		{
			mind = d[i + 1];
		}
	}
	space = (maxd - mind) / 100;//100个区间的间隔大小
	for (int i = 0; i < 99; i++)//循环100个区间
	{
		for (int j = 0; j < contours[maxContoursIndex].size() - 1; j++)//
		{
			if (d[j] >= (mind + space*(i - 1)) && d[j] <= (mind + space*i))
			{
				prob[i] = prob[i] + 1;
			}
		}
	}
	for (int i = 0; i < 99; i++)
	{
		prob[i] = prob[i] / contours[maxContoursIndex].size();
		if (prob[i] == 0)
		{
			prob[i] = 1;
		}
	}
	for (int i = 0; i < 99; i++)
	{
		ret.M3_1D = ret.M3_1D + prob[i] * (log(prob[i]) / log(2));
	}
	ret.M3_1D = -ret.M3_1D;

	double P_i, P_j;
	Mat P_ij = Mat::zeros(Size(256, 256), CV_8UC1);//指定矩阵的大小和类型,并用指定的数据进行填充
	for (int i = 0; i < Image.rows; i++)
	{
		for (int j = 0; j < Image.cols; j++)
		{
			P_i = Image.at<uchar>(i, j);
			if (i == 0 && j == 0)
			{
				P_j = Image.at<uchar>(i, j + 1) + Image.at<uchar>(i + 1, j) + Image.at<uchar>(i + 1, j + 1);
			}
			if ((i == 0) && (j>0) && (j < Image.cols - 1))
			{
				P_j = Image.at<uchar>(i, j - 1) + Image.at<uchar>(i, j + 1) + Image.at<uchar>(i + 1, j - 1) + Image.at<uchar>(i + 1, j) + Image.at<uchar>(i + 1, j + 1);
			}
			if ((i == 0) && (j == (Image.cols - 1)))
			{
				P_j = Image.at<uchar>(i, j - 1) + Image.at<uchar>(i + 1, j) + Image.at<uchar>(i + 1, j - 1);
			}
			if ((i>0) && (j == 0) && (i<Image.rows - 1))
			{
				P_j = Image.at<uchar>(i - 1, j) + Image.at<uchar>(i - 1, j + 1) + Image.at<uchar>(i, j + 1) + Image.at<uchar>(i + 1, j) + Image.at<uchar>(i + 1, j + 1);
			}
			if ((i>0) && (j == (Image.cols - 1)) && (i < Image.rows - 1))
			{
				P_j = Image.at<uchar>(i - 1, j - 1) + Image.at<uchar>(i - 1, j) + Image.at<uchar>(i, j - 1) + Image.at<uchar>(i + 1, j - 1) + Image.at<uchar>(i + 1, j);
			}
			if ((i == (Image.rows - 1)) && (j == 0))
			{
				P_j = Image.at<uchar>(i - 1, j) + Image.at<uchar>(i - 1, j + 1) + Image.at<uchar>(i, j + 1);
			}
			if ((i == (Image.rows - 1)) && (j > 0) && (j < (Image.cols - 1)))
			{
				P_j = Image.at<uchar>(i - 1, j) + Image.at<uchar>(i - 1, j + 1) + Image.at<uchar>(i - 1, j - 1) + Image.at<uchar>(i, j - 1) + Image.at<uchar>(i, j + 1);
			}
			if ((i == (Image.rows - 1)) && (j == (Image.cols - 1)))
			{
				P_j = Image.at<uchar>(i - 1, j) + Image.at<uchar>(i - 1, j - 1) + Image.at<uchar>(i, j - 1);
			}
			if ((i>0) && (j > 0) && (i < (Image.rows - 1)) && (j < (Image.cols - 1)))
			{
				P_j = Image.at<uchar>(i - 1, j - 1) + Image.at<uchar>(i - 1, j) + Image.at<uchar>(i - 1, j + 1) + Image.at<uchar>(i, j + 1) + Image.at<uchar>(i, j - 1) + Image.at<uchar>(i + 1, j - 1) + Image.at<uchar>(i + 1, j) + Image.at<uchar>(i + 1, j + 1);
			}
			P_j = P_j / 8;
			P_ij.at<uchar>(P_i, P_j) = P_ij.at<uchar>(P_i, P_j) + 1;
		}
	}
	for (int i = 0; i < 256; i++)
	{
		for (int j = 0; j < 256; j++)
		{
			double pij = P_ij.at<uchar>(i, j);
			if (pij != 0)
			{
				pij = -(pij / (256 * 256))*log(pij / (256 * 256));
				ret.M3_2D = ret.M3_2D + pij;
			}
		}
	}
	//---------------------------------------------------------------------------------------------//
	//M4//
	Point2f fourPoint[4];
	vector<double>line;
	for (int i = 0; i < contours.size(); i++)
	{
		RotatedRect rectPoint = minAreaRect(contours[i]);
		//将rectPoint变量中存储的坐标值放到 fourPoint的数组中
		rectPoint.points(fourPoint);
	}
	for (int i = 1; i < 4; i++)
	{
		line.push_back(sqrt(pow((fourPoint[i].y - fourPoint[0].y), 2) + pow((fourPoint[i].x - fourPoint[0].x), 2)));
	}
	vector<double>::iterator biggest = std::max_element(std::begin(line), std::end(line));
	vector<double>::iterator smallest = std::min_element(std::begin(line), std::end(line));
	ret.M4 = *biggest / *smallest;

	//---------------------------------------------------------------------------------------------//
	//M5//
	double coverArea = pi*pow((MaxD / 2), 2);
	ret.M5 = (ret.Area) / coverArea;

	//---------------------------------------------------------------------------------------------//
	//M6//
	ret.M6 = ret.Area / (4 * pi*(ret.Length)*(ret.Length));

	//---------------------------------------------------------------------------------------------//
	//M7//
	for (int i = 1; i < (contours[maxContoursIndex].size() - 1); i++)
	{
		ret.M7 = ret.M7 + abs(d[i] - (d[i - 1] + d[i + 1]) / 2);
	}

	//---------------------------------------------------------------------------------------------//
	//M8//
	ret.M8 = ret.M1 / ret.M2;

	//---------------------------------------------------------------------------------------------//
	//M9//
	double M9_1 = 0, M9_2 = 0;
	int N = contours[maxContoursIndex].size();
	for (int i = 0; i < N; i++)
	{
		M9_1 = M9_1 + pow((d[i] - ret.M1), 4);
		M9_2 = M9_2 + pow((d[i] - ret.M1), 2);
	}
	ret.M9 = (pow(M9_1 / N, 0.25) - pow(M9_2 / N, 0.5)) / (ret.M2);

	//---------------------------------------------------------------------------------------------//


	//return ret;
}

/***************************************************************************************
Function:  区域生长算法
Input:     src 待处理原图像 pt 初始生长点 th 生长的阈值条件
Output:    肺实质的所在的区域 实质区是白色，其他区域是黑色
Description: 生长结果区域标记为白色(255),背景色为黑色(0)
Return:    Mat
Others:    NULL
***************************************************************************************/
Mat RegionGrow(Mat src, Point2i pt, int th)
{
	Point2i ptGrowing;                      //待生长点位置  
	int nGrowLable = 0;                             //标记是否生长过  
	int nSrcValue = 0;                              //生长起点灰度值  
	int nCurValue = 0;                              //当前生长点灰度值  
	Mat matDst = Mat::zeros(src.size(), CV_8UC1);   //创建一个空白区域，填充为黑色  
	//生长方向顺序数据  
	int DIR[8][2] = { { -1, -1 }, { 0, -1 }, { 1, -1 }, { 1, 0 }, { 1, 1 }, { 0, 1 }, { -1, 1 }, { -1, 0 } };
	vector<Point2i> vcGrowPt;                     //生长点栈  
	vcGrowPt.push_back(pt);                         //将生长点压入栈中  
	matDst.at<uchar>(pt.y, pt.x) = 255;               //标记生长点  
	nSrcValue = src.at<uchar>(pt.y, pt.x);            //记录生长点的灰度值  

	while (!vcGrowPt.empty())                       //生长栈不为空则生长  
	{
		pt = vcGrowPt.back();                       //取出一个生长点  
		vcGrowPt.pop_back();

		//分别对八个方向上的点进行生长  
		for (int i = 0; i<9; ++i)
		{
			ptGrowing.x = pt.x + DIR[i][0];
			ptGrowing.y = pt.y + DIR[i][1];
			//检查是否是边缘点  
			if (ptGrowing.x < 0 || ptGrowing.y < 0 || ptGrowing.x >(src.cols - 1) || (ptGrowing.y > src.rows - 1))
				continue;

			nGrowLable = matDst.at<uchar>(ptGrowing.y, ptGrowing.x);      //当前待生长点的灰度值  

			if (nGrowLable == 0)                    //如果标记点还没有被生长  
			{
				nCurValue = src.at<uchar>(ptGrowing.y, ptGrowing.x);
				if (abs(nSrcValue - nCurValue) < th)                 //在阈值范围内则生长  
				{
					matDst.at<uchar>(ptGrowing.y, ptGrowing.x) = 255;     //标记为白色  
					vcGrowPt.push_back(ptGrowing);                  //将下一个生长点压入栈中  
				}
			}
		}
	}
	return matDst.clone();
}

//Function:  连通域标记算法 Two-Pass法
//可以使用opencv自带的connectedComponents
void icvprCcaByTwoPass(const cv::Mat& _binImg, cv::Mat& _lableImg,int &maxlabel)
{
	// connected component analysis (4-component)  
	// use two-pass algorithm  
	// 1. first pass: label each foreground pixel with a label  
	// 2. second pass: visit each labeled pixel and merge neighbor labels  
	//   
	// foreground pixel: _binImg(x,y) = 1  
	// background pixel: _binImg(x,y) = 0  
	//输出的labelimg中背景的label是0

	if (_binImg.empty() ||
		_binImg.type() != CV_8UC1)
	{
		return;
	}

	// 1. first pass  

	_lableImg.release();
	_binImg.convertTo(_lableImg, CV_32SC1);

	int label = 1;  // start by 2  
	std::vector<int> labelSet;
	labelSet.push_back(0);   // background: 0  
	labelSet.push_back(1);   // foreground: 1  

	int rows = _binImg.rows - 1;
	int cols = _binImg.cols - 1;
	for (int i = 1; i < rows; i++)
	{
		int* data_preRow = _lableImg.ptr<int>(i - 1);
		int* data_curRow = _lableImg.ptr<int>(i);
		for (int j = 1; j < cols; j++)
		{
			if (data_curRow[j] == 1)
			{
				std::vector<int> neighborLabels;
				neighborLabels.reserve(2);
				int leftPixel = data_curRow[j - 1];
				int upPixel = data_preRow[j];
				if (leftPixel > 1)
				{
					neighborLabels.push_back(leftPixel);
				}
				if (upPixel > 1)
				{
					neighborLabels.push_back(upPixel);
				}

				if (neighborLabels.empty())
				{
					labelSet.push_back(++label);  // assign to a new label  
					data_curRow[j] = label;
					labelSet[label] = label;
				}
				else
				{
					std::sort(neighborLabels.begin(), neighborLabels.end());
					int smallestLabel = neighborLabels[0];
					data_curRow[j] = smallestLabel;

					// save equivalence  
					for (size_t k = 1; k < neighborLabels.size(); k++)
					{
						int tempLabel = neighborLabels[k];
						int& oldSmallestLabel = labelSet[tempLabel];
						if (oldSmallestLabel > smallestLabel)
						{
							labelSet[oldSmallestLabel] = smallestLabel;
							oldSmallestLabel = smallestLabel;
						}
						else if (oldSmallestLabel < smallestLabel)
						{
							labelSet[smallestLabel] = oldSmallestLabel;
						}
					}
				}
			}
		}
	}

	// update equivalent labels  
	// assigned with the smallest label in each equivalent label set  
	for (size_t i = 2; i < labelSet.size(); i++)
	{
		int curLabel = labelSet[i];
		int preLabel = labelSet[curLabel];
		while (preLabel != curLabel)
		{
			curLabel = preLabel;
			preLabel = labelSet[preLabel];
		}
		labelSet[i] = curLabel;
	}


	// 2. second pass  
	maxlabel=0;
	for (int i = 0; i < rows - rows / 4; i++) //减小扫描床的干扰
	{
		int* data = _lableImg.ptr<int>(i);
		for (int j = 0; j < cols; j++)//
		{
			int& pixelLabel = data[j];
			pixelLabel = labelSet[pixelLabel];
			if (pixelLabel >maxlabel){
				maxlabel = pixelLabel;
			}
		}
	}
	for (int i = rows - rows / 4; i < rows; i++){//减小扫描床的干扰
		int* data = _lableImg.ptr<int>(i);
		for (int j = 0; j < cols; j++)//
		{
			data[j]=2;
		}
	}
	cout<<endl<< maxlabel;
}

//Function:  筛选出面积符合条件的连通域，即为肺实质
void FindLung(const cv::Mat& _lableImg,Mat &res,int maxlabel){
	int rows = _lableImg.rows ;
	int cols = _lableImg.cols ;
	int *labelNum = new int[maxlabel+1]();
	int index;
	for (int i = 0; i < rows; i++){
		for (int j = 0; j < cols; j++){
			index = _lableImg.at<int>(i, j);
			labelNum[index]++;
		}
	}
	cout << "maxlabel" << maxlabel << endl;
	set<int> label;//存放符合大小条件的label
	for (int i = 1; i < maxlabel + 1; i++){ 
		if (labelNum[i]>7000 && labelNum[i] < 90000){
			label.insert(i);
			cout << labelNum[i] << "   " << i << endl;;
		}
	}
	res.release();
	res.create(_lableImg.size(), CV_8UC1);
	res.setTo(0);
	for (int i = 0; i < rows; i++){
		for (int j = 0; j < cols; j++){
			index = _lableImg.at<int>(i, j);
			if (label.count(index) == 1){
				res.at<uchar>(i, j) = 255;
			}
		}
	}
}

// 最大类间方差法 （扣除像素是0的元素）
//输入 灰度图
//返回阈值
int otsuThreshold(Mat pic)
{
	const int channels[1] = { 0 };
	const int histSize[1] = { 256 };
	float hranges[2] = { 0, 255 };
	const float* ranges[1] = { hranges };
	MatND hist;
	calcHist(&pic, 1, channels, Mat(), hist, 1, histSize, ranges);

	int T = 0;//阈值  
	double gSum0;//第一类灰度总值  
	double gSum1;//第二类灰度总值  
	double N0 = 0;//前景像素数  
	double N1 = 0;//背景像素数  
	double u0 = 0;//前景像素平均灰度  
	double u1 = 0;//背景像素平均灰度  
	double w0 = 0;//前景像素点数占整幅图像的比例为ω0  
	double w1 = 0;//背景像素点数占整幅图像的比例为ω1  
	double u = 0;//总平均灰度  
	double tempg = -1;//临时类间方差  
	double g = -1;//类间方差  
	int height = pic.rows;
	int width = pic.cols;
	double N = width*height - hist.at<float>(0);//总像素数扣除像素是0的元素  

	for (int i = 1; i<256; i++)
	{
		gSum0 = 0;
		gSum1 = 0;
		N0 += hist.at<float>(i);
		N1 = N - N0;
		if (0 == N1)break;//当出现前景无像素点时，跳出循环  
		w0 = N0 / N;
		w1 = 1 - w0;
		for (int j = 0; j <= i; j++)
		{
			gSum0 += j*hist.at<float>(j);
		}
		u0 = gSum0 / N0;
		for (int k = i + 1; k<256; k++)
		{
			gSum1 += k*hist.at<float>(k);
		}
		u1 = gSum1 / N1;
		//u = w0*u0 + w1*u1;  
		g = w0*w1*(u0 - u1)*(u0 - u1);
		if (tempg<g)
		{
			tempg = g;
			T = i;
		}
	}
	return T;
}