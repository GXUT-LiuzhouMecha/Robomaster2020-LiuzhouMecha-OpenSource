/**************************************************************

MIT License

Copyright(c) 2020 GXUT - 龙城机甲 

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

**************************************************************/

#include "SentinelCamera.h"

//模拟场地尺寸
const double site_wdith = 1.0;
const double site_height = 1.5;
//图像尺寸
const int image_wdith = 640;
const int image_height = 480;

SentinelCamera::SentinelCamera(int enemy_color)
{
	enemy_color_ = enemy_color;
	PersctiveMatInit();
}
void SentinelCamera::PersctiveMatInit()
{
	cv::Point2f src_Points[4], dst_Points[4];

	//取点坐标
	src_Points[0] = cv::Point2f(102, 132);
	src_Points[1] = cv::Point2f(358, 38);
	src_Points[2] = cv::Point2f(462, 544);
	src_Points[3] = cv::Point2f(51, 558);

	//对应坐标
	dst_Points[0] = cv::Point2f(0, 0);
	dst_Points[1] = cv::Point2f(480, 0);
	dst_Points[2] = cv::Point2f(480, 640);
	dst_Points[3] = cv::Point2f(0, 640);

	persctive_mat_ = cv::getPerspectiveTransform(src_Points, dst_Points);

}
cv::Mat SentinelCamera::DistillationColor(const cv::Mat &src_img, unsigned int color, bool using_hsv)
{
	if (using_hsv) {
		cv::Mat img_hsv;
		cv::cvtColor(src_img, img_hsv, CV_BGR2HSV);
		if (color == 0) {
			cv::Mat img_hsv_blue, img_threshold_blue;
			img_hsv_blue = img_hsv.clone();
			cv::Mat blue_low(cv::Scalar(90, 150, 46));
			cv::Mat blue_higher(cv::Scalar(140, 255, 255));
			cv::inRange(img_hsv_blue, blue_low, blue_higher, img_threshold_blue);
			return img_threshold_blue;
		}
		else {
			cv::Mat img_hsv_red1, img_hsv_red2, img_threshold_red, img_threshold_red1, img_threshold_red2;
			img_hsv_red1 = img_hsv.clone();
			img_hsv_red2 = img_hsv.clone();
			cv::Mat red1_low(cv::Scalar(0, 43, 46));
			cv::Mat red1_higher(cv::Scalar(3, 255, 255));

			cv::Mat red2_low(cv::Scalar(170, 43, 46));
			cv::Mat red2_higher(cv::Scalar(180, 255, 255));
			cv::inRange(img_hsv_red1, red1_low, red1_higher, img_threshold_red1);
			cv::inRange(img_hsv_red2, red2_low, red2_higher, img_threshold_red2);
			img_threshold_red = img_threshold_red1 | img_threshold_red2;
			//cv::imshow("img_threshold_red", img_threshold_red);
			return img_threshold_red;
		}
	}
	else {
		std::vector<cv::Mat> bgr;
		cv::split(src_img, bgr);
		if (color == 1) {
			cv::Mat result_img;
			cv::subtract(bgr[2], bgr[1], result_img);
			return result_img;
		}
		else if (color == 0) {
			cv::Mat result_img;
			cv::subtract(bgr[0], bgr[2], result_img);
			return result_img;
		}
	}
}

void SentinelCamera::CoordinateTransform(std::vector<cv::Point2f> &points)
{
	if (points.size() == 0)
	{
		return;
	}
	cv::perspectiveTransform(points, points, persctive_mat_);

	//图像坐标转换场地坐标 单位m
	for (int i =0; i < points.size() ;i++)
	{
		points[i].x = points[i].x / image_height * site_wdith;
		points[i].y = site_height - points[i].y / image_wdith * site_height;
	}

}
std::vector<cv::Point2f> SentinelCamera::ImageProcessing(cv::Mat src_img)
{
	cv::Mat img_gay = DistillationColor(src_img, enemy_color_, 1);

	cv::Mat img_threshold;
	threshold(img_gay, img_threshold, 50, 255, cv::THRESH_BINARY);

	cv::Mat kernel_dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(50, 50));
	dilate(img_threshold, img_threshold, kernel_dilate);

	cv::Mat kernel_erode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(50, 30));
	erode(img_threshold, img_threshold, kernel_erode);

	std::vector<std::vector<cv::Point>> contours;
	findContours(img_threshold, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	sort(contours.begin(), contours.end(), [](std::vector<cv::Point> v1, std::vector<cv::Point> v2)
	{
		return contourArea(v1) > contourArea(v2);
	});

	std::vector<cv::Point2f> car_coordinates;

	for (int i = 0; i < contours.size(); i++)
	{
		if (contourArea(contours[i]) > 300)
		{
			rectangle(src_img, boundingRect(contours[i]), cv::Scalar(0, 255, 0), 2);

			cv::RotatedRect rotated_rect = cv::minAreaRect(cv::Mat(contours[i]));
		
			car_coordinates.push_back(rotated_rect.center);

			if (car_coordinates.size() == 2)
			{
				break;
			}
		}
	}

	CoordinateTransform(car_coordinates);

	return car_coordinates;
}
void SentinelCamera::CoordinateTransformTest(cv::Mat src_img, cv::Point2f point)
{
	std::vector<cv::Point2f> points;
	points.push_back(point);

	std::vector<cv::Point2f> img_points;
	std::vector<cv::Point2f> real_points;
	real_points.push_back(point);

	cv::perspectiveTransform(points, img_points, persctive_mat_);
	CoordinateTransform(real_points);

	cv::Mat dst_img;
	cv::warpPerspective(src_img, dst_img, persctive_mat_, cv::Size(src_img.cols, src_img.rows), cv::INTER_LINEAR);
	cv::circle(dst_img, img_points[0], 5, cv::Scalar(255, 0, 0), -1);
	putText(dst_img, "Image coordinates(" + std::to_string(int(img_points[0].x)) + "," + std::to_string(int(img_points[0].y)) + ")", cv::Point(50,50), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0, 255));
	putText(dst_img, "Real coordinates(" + std::to_string((real_points[0].x)) + "," + std::to_string((real_points[0].y)) + ")", cv::Point(50, 80), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0, 255));

	imshow("坐标转换测试", dst_img);
}