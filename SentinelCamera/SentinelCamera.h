/**************************************************************

MIT License

Copyright(c) 2020 GXUT - ���ǻ��� 

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
    *	@Brief: �任�����ʼ�����ֶ�ȡ��
    */
	void PersctiveMatInit();

   /*
    *	@Brief: ͼ��ҶȻ���
	*	@Input:	��ɫͼ��
    *			��ȡ��ɫ
    *			�Ƿ����hsv�ռ�ģʽ
	*	@Return: �Ҷ�ͼ��
    */
	cv::Mat DistillationColor(const cv::Mat &src_img, unsigned int color, bool using_hsv);

   /*
    *	@Brief: ����ת��
    *	@Input: ��������
    */
	void CoordinateTransform(std::vector<cv::Point2f> &points);

	/*
     *	@Brief: ����ת�����ԣ�ͨ��͸�ӱ任ͼ��Ա�
     *	@Input: ת��ͼ��
	 *          �����    
     */
	void CoordinateTransformTest(cv::Mat src_img,cv::Point2f point);

	/*
     *	@Brief: ͼ����������
     *	@Input: ����ͼ��
     *  @Return: Ŀ������ʵ���ص�����
     */
	std::vector<cv::Point2f> ImageProcessing(cv::Mat src_img);

public:
	int enemy_color_;
	cv::Mat persctive_mat_;
	cv::Point point_;
	
};

