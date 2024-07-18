#include<iostream>
#include<cmath>
#include<opencv2/opencv.hpp>
#include<string>

using namespace std;
using namespace cv;

/*
导入图片、视频
图片灰度化
边缘检测
显示ROI区域
轮廓提取（椭圆拟合）
直线拟合
./data/video_1.mp4
./data/test_image2.png
960 540
*/

Point left_line[2];
Point right_line[2];
void process(Mat& frame, Point* left_line, Point* right_line);
Mat fitLines(Mat& image, Point* lift_line, Point* right_line);

int main()
{
	int num = 0;
	cout << "如果您需要检测图片请输入1,需要检测视频请输入2"<<endl;
	cin >> num;
	getchar();
	if (num == 1)//图片
	{
		string user_path,user_path_save;
		cout << "请输入图片路径：  ";
		getline(cin, user_path);
		//getchar();
		cout << "图片保存路径(文件格式为.jpg)：  ";
		getline(cin, user_path_save);
		Mat frame = imread(user_path);
		if (frame.empty())
		{
			cerr << "输入图片路径有误！！！";
			return -1;
		}

		int width = frame.cols;//宽
		int hight = frame.rows;//高
		cout << "宽：  " << width << "\t高：   " << hight << endl;

		//输出一下原图
		namedWindow("原图", WINDOW_FREERATIO);
		imshow("原图", frame);
		//waitKey(0);
		//初始化
		left_line[0] = { Point(0,0) };
		left_line[0] = { Point(0,0) };
		right_line[0] = { Point(0,0) };
		right_line[0] = { Point(0,0) };

		//调用函数处理图片
		process(frame, left_line, right_line);
		imwrite(user_path_save,frame);
		waitKey(0);
		//释放所有窗口
		destroyAllWindows();
		return 0;
	}
	else if (num == 2)//视频
	{
		string user_path,user_path_save;
		cout << "请输入视频路径：  ";
		getline(cin,user_path);
		//getchar();
		cout << "请视频保存路径(文件格式为.avi)：  ";
		getline(cin,user_path_save);
		VideoWriter writer;
		// 保存视频文件
		writer.open(user_path_save, VideoWriter::fourcc('M', 'J', 'P', 'G'), 25, Size(960, 540));
		//读取视频
		VideoCapture capture(user_path);
		int height = capture.get(CAP_PROP_FRAME_HEIGHT);//高
		int width = capture.get(CAP_PROP_FRAME_WIDTH);//宽
		int count = capture.get(CAP_PROP_FRAME_COUNT);//视频总帧数
		int fps = capture.get(CAP_PROP_FPS);//视频频率

		cout << "宽：" << width << "\t高：" << height
			<< "\t视频总帧数：" << count << "\t视频频率：" << fps << endl;
		//初始化
		left_line[0] = { Point(0,0) };
		left_line[0] = { Point(0,0) };
		right_line[0] = { Point(0,0) };
		right_line[0] = { Point(0,0) };
		
		//循环读取视频
		Mat frame;
		while (true) {
			int ret = capture.read(frame);//用ret来记录读取的视频
			if (!ret) {
				break;
			}
			process(frame, left_line, right_line);
			writer.write(frame);
			char c = waitKey(5);
			if (c == 27) {				//esc退出循环读取				
				break;
			}
		}
		//处理并释放空间
		capture.release();
		writer.release();
		destroyAllWindows();
		return 0;
	}
	else
	{
		cout << "输入错误！！！";
		return -1;
	}
}
void process(Mat& frame, Point* left_line, Point* right_line) {
	//gray 灰度   binary 边缘
	Mat gray, binary;
	//灰度化	
	cvtColor(frame, gray, COLOR_BGR2GRAY);
	imshow("灰度图片", gray);
	//canny算子边缘检测
	Canny(gray, binary, 100,150);
	imshow("边缘检测", binary);

	// 定义ROI区域的顶点
	int width = frame.cols;
	int height = frame.rows;
	Point roi_points[1][4];
	roi_points[0][0] = (Point(width/2-100, height/2+50));
	roi_points[0][1] = (Point(120, height-20));
	roi_points[0][2] = (Point(width-100, height-20));
	roi_points[0][3] = (Point(width/2+100, height/2+50));

	// 创建掩码
	Mat mask = Mat::zeros(binary.size(), CV_8UC1);
	const Point* ppt[1] = { roi_points[0] };
	int npt[] = { 4 };

	// 填充掩码
	fillPoly(mask, ppt, npt, 1, Scalar(255));
	//fillPoly(mask,);
	imshow("掩码图像", mask);

	// 对二值化(边缘)填入掩码
	Mat roi_binary;
	bitwise_and(binary, mask, roi_binary);
	imshow("ROI区域（提取出的区域）", roi_binary);

	//寻找轮廓findContours
	//二值化图像中查找对象的轮廓，并将找到的轮廓保存在contours这个容器中
	//二值化图片	保存容器	检测外侧	压缩
	vector<vector<Point>> contours;
	findContours(roi_binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	//创建一个空图片和灰度图像信息保持一致
	Mat out_image = Mat::zeros(gray.size(), gray.type());//椭圆处理后放这里面
	for (int i = 0; i < contours.size(); i++)
	{
		//遍历筛选一些轮廓
		//计算面积与周长
		double length = arcLength(contours[i], true);
		double area = contourArea(contours[i]);
		//cout << "周长 length:" << length << endl;
		//cout << "面积 area:" << area << endl;
		//轮廓的边框
		Rect rect = boundingRect(contours[i]);
		int h = gray.rows - 50;
		//轮廓分析
		if (length < 5.0 || area < 10.0) {
			continue;
		}
		if (rect.y > h) {
			continue;
		}
		//最小包围矩形
		RotatedRect rrt = minAreaRect(contours[i]);
		//cout << "最小包围矩形 angle:" << rrt.angle << endl;
		double angle = abs(rrt.angle);
		//angle < 50.0 || angle>89.0
		if (angle < 20.0 || angle>84.0) {
			continue;
		}
		if (contours[i].size() > 5) {
			//拟合椭圆
			RotatedRect errt = fitEllipse(contours[i]);
			//cout << "用椭圆拟合err.angle:" << errt.angle << endl;
			if ((errt.angle < 5.0) || (errt.angle > 160.0))
			{
				if (80.0 < errt.angle && errt.angle < 100.0) {
					continue;
				}
			}
		}

		//cout << "开始绘制：" << endl;
		drawContours(out_image, contours, i, Scalar(255), 2, 8);
		imshow("轮廓处理", out_image);	//显示轮廓处理后的结果
	}
	Mat result = fitLines(out_image, left_line, right_line);
	imshow("画出线条", result);	//显示fitlines直线拟合后的结果

	Mat dst;
	addWeighted(frame, 0.8, result, 0.5, 0, dst);	//权重
	imshow("原图中勾勒出车道线", dst);	//显示添加权重后的结果
	frame = dst;
}
Mat fitLines(Mat& image, Point* left_line, Point* right_line) {
	int height = image.rows;
	int width = image.cols;

	Mat out = Mat::zeros(image.size(), CV_8UC3);
	//图像中心
	int cx = width / 2;
	int cy = height / 2;

	vector<Point> left_pts;
	vector<Point> right_pts;
	Vec4f left;
	//遍历左下部分，找到左车道线
	for (int i = 100; i < (cx - 10); i++)
	{
		for (int j = cy; j < height; j++)
		{
			int pv = image.at<uchar>(j, i);
			if (pv == 255)
			{	//提取左车道线的点集
				left_pts.push_back(Point(i, j));
			}
		}
	}
	//遍历右下部分找到点，找到右下部分
	for (int i = cx; i < (width - 20); i++)
	{
		for (int j = cy; j < height; j++)
		{
			int pv = image.at<uchar>(j, i);
			if (pv == 255)
			{	//提取右车道线的点集
				right_pts.push_back(Point(i, j));
			}
		}
	}

	//拟合左车道
	if (left_pts.size() > 2)		//点数大于2
	{
		//拟合一条直接到左车道点集上
		fitLine(left_pts, left, DIST_L1, 0, 0.01, 0.01);

		double k1 = left[1] / left[0];
		double step = left[3] - k1 * left[2];

		int x1 = int((height - step) / k1);
		int y2 = int((cx - 25) * k1 + step);

		Point left_spot_1 = Point(x1, height);
		Point left_spot_end = Point((cx - 25), y2);

		line(out, left_spot_1, left_spot_end, Scalar(128, 0, 128), 8, 8, 0);
		//承接上一张的点
		left_line[0] = left_spot_1;
		left_line[1] = left_spot_end;
	}
	else
	{
		line(out, left_line[0], left_line[1], Scalar(128, 0, 128), 8, 8, 0);
	}
	//拟合右车道
	if (right_pts.size() > 2)
	{
		Point spot_1 = right_pts[0];
		Point spot_end = right_pts[right_pts.size() - 1];

		int x1 = spot_1.x;
		int y1 = spot_1.y;

		int x2 = spot_end.x;
		int y2 = spot_end.y;
		// 计算斜线的倾斜角度（弧度）
		double angle = atan2(y2 - y1, x2 - x1);

		// 将弧度转换为角度
		double angle_deg = angle * 180 / CV_PI;

		// 过滤掉倾斜角度小于25度的斜线
		if (abs(angle_deg) >= 25.0)
		{
			line(out, spot_1, spot_end, Scalar(0, 225, 0), 8, 8, 0);
			right_line[0] = spot_1;
			right_line[1] = spot_end;
		}
		line(out, spot_1, spot_end, Scalar(0, 225, 0), 8, 8, 0);
		right_line[0] = spot_1;
		right_line[1] = spot_end;
	}
	else
	{
		line(out, right_line[0], right_line[1], Scalar(0, 225, 0), 8, 8, 0);
	}

	return out;
}