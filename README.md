### 1.1 Overview
哨岗摄像头通过USB线与场外计算设备连接，程序运行于Windows环境VS2017,依赖OpenCV4.3.0.
总体步骤：
赛前调试取点->敌方机器人颜色提取->闭运算->寻找轮廓->筛选轮廓->提取轮廓中心坐标->坐标转换

### 1.2 赛前调试
安装好哨岗摄像头，使其覆盖整个赛场，运行程序，鼠标左键依次获取记录赛场4个角落在图像中的坐标，用于计算透视变换矩阵。

### 1.3 颜色提取
通过hsv空间提取敌方机器人颜色，使用了官方源码中的提取函数。

### 1.4 坐标转换
通过变换矩阵进行透视变换将哨岗摄像头视角坐标转换为鸟瞰视角坐标，根据此坐标与场地大小关系得出敌方机器人在场地中的坐标。


## 2.API
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