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

#include <opencv2\imgproc\types_c.h>
#include "opencv2/opencv.hpp"  


class SentinelCamera
{
public:
	SentinelCamera(int enemy_color);

public:
   /*
    *	@Brief: 变换矩阵初始化，手动取点
    */
	void PersctiveMatInit();

   /*
    *	@Brief: 图像灰度化，
	*	@Input:	彩色图像
    *			提取颜色
    *			是否采用hsv空间模式
	*	@Return: 灰度图像
    */
	cv::Mat DistillationColor(const cv::Mat &src_img, unsigned int color, bool using_hsv);

   /*
    *	@Brief: 坐标转换
    *	@Input: 坐标向量
    */
	void CoordinateTransform(std::vector<cv::Point2f> &points);

	/*
     *	@Brief: 坐标转换测试，通过透视变换图像对比
     *	@Input: 转换图像
	 *          坐标点    
     */
	void CoordinateTransformTest(cv::Mat src_img,cv::Point2f point);

	/*
     *	@Brief: 图像处理主函数
     *	@Input: 输入图像
     *  @Return: 目标在真实场地的坐标
     */
	std::vector<cv::Point2f> ImageProcessing(cv::Mat src_img);

public:
	int enemy_color_;
	cv::Mat persctive_mat_;
	cv::Point point_;
	
};

