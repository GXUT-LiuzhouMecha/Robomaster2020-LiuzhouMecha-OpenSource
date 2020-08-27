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

const int BLUE = 0;
const int RED = 1;

//鼠标获取函数
void onMouse(int event, int x, int y, int flags, void* param)
{
	cv::Point &mouse_point = *(cv::Point*)param;

	if (event == cv::EVENT_LBUTTONDOWN)
	{
		mouse_point.x = x;
		mouse_point.y = y;
	}

}

int main()
{
	cv::VideoCapture video(0);

	if (!video.isOpened())
	{
		std::cout << "video open error!" << std::endl;
		return 0;
	}

	cv::Mat frame;

	SentinelCamera sentinel_camera(BLUE);

	cv::Point mouse_point;

	while (1)
	{
		//video >> frame;
		//测试图像
		frame = cv::imread("image/image1.jpg");

		if (frame.empty())
		{
			std::cout << "frame is empty!" << std::endl;
			break;
		}

		//std::vector<cv::Point2f> car_coord = sentinel_camera.ImageProcessing(frame);

		//坐标转换测试
		sentinel_camera.CoordinateTransformTest(frame, mouse_point);

		//鼠标左击坐标点获取
 		cv::setMouseCallback("frame", onMouse, &mouse_point);
 		cv::putText(frame, "(" + std::to_string(mouse_point.x) + "," + std::to_string(mouse_point.y) + ")", cv::Point(50,50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255, 255),2);
		cv::circle(frame, mouse_point, 5, cv::Scalar(0, 0, 255),-1);
 		imshow("frame", frame);


		if (cv::waitKey(1000.0 / 60) == 27)
		{
			std::cout << "ESC退出!" << std::endl;
			break;
		}
	}
	
	return 0;
}