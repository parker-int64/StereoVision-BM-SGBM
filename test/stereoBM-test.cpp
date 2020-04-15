#include <iostream>
#include <opencv2\opencv.hpp>
 
using namespace std;
using namespace cv;
 
void calDispWithBM(Mat imgL, Mat imgR, Mat &imgDisparity8U)
{
	Mat imgDisparity16S = Mat(imgL.rows, imgL.cols, CV_16S);
 
	//--Call the constructor for StereoBM
	cv::Size imgSize = imgL.size();
	int numberOfDisparities = ((imgSize.width / 8) + 15) & -16;
	cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(16, 9);
 
	//--Calculate the disparity image
 
	/*
	左右视图的有效像素区域，一般由双目校正阶段的 cvStereoRectify 函数传递，也可以自行设定。
	一旦在状态参数中设定了 roi1 和 roi2，OpenCV 会通过cvGetValidDisparityROI 函数计算出视差图的有效区域，在有效区域外的视差值将被清零。
	*/
	cv::Rect roi1, roi2;
	bm->setROI1(roi1);
	bm->setROI2(roi2);
 
	//==预处理滤波参数
	/*
	预处理滤波器的类型，主要是用于降低亮度失真（photometric distortions）、消除噪声和增强纹理等,
	有两种可选类型：CV_STEREO_BM_NORMALIZED_RESPONSE（归一化响应） 或者 CV_STEREO_BM_XSOBEL（水平方向Sobel算子，默认类型）,
	该参数为 int 型;
	*/
	bm->setPreFilterType(0);
 
	/*预处理滤波器窗口大小，容许范围是[5,255]，一般应该在 5x5..21x21 之间，参数必须为奇数值, int 型*/
	bm->setPreFilterSize(9);
 
	/*预处理滤波器的截断值，预处理的输出值仅保留[-preFilterCap, preFilterCap]范围内的值，参数范围：1 - 31,int 型*/
	bm->setPreFilterCap(31);
 
	//==SAD 参数
	/*SAD窗口大小，容许范围是[5,255]，一般应该在 5x5 至 21x21 之间，参数必须是奇数，int 型*/
	bm->setBlockSize(9);
 
	/*最小视差，默认值为 0, 可以是负值，int 型*/
	bm->setMinDisparity(0);
 
	/*视差窗口，即最大视差值与最小视差值之差, 窗口大小必须是 16 的整数倍，int 型*/
	bm->setNumDisparities(numberOfDisparities);
 
	//==后处理参数
	/*
	低纹理区域的判断阈值:
	如果当前SAD窗口内所有邻居像素点的x导数绝对值之和小于指定阈值，则该窗口对应的像素点的视差值为 0
	（That is, if the sum of absolute values of x-derivatives computed over SADWindowSize by SADWindowSize
	pixel neighborhood is smaller than the parameter, no disparity is computed at the pixel），
	该参数不能为负值，int 型;
	*/
	bm->setTextureThreshold(10);
 
	/*
	视差唯一性百分比:
	视差窗口范围内最低代价是次低代价的(1 + uniquenessRatio/100)倍时，最低代价对应的视差值才是该像素点的视差，
	否则该像素点的视差为 0 （the minimum margin in percents between the best (minimum) cost function value and the second best value to accept
	the computed disparity, that is, accept the computed disparity d^ only if SAD(d) >= SAD(d^) x (1 + uniquenessRatio/100.) for any d != d*+/-1 within the search range ），
	该参数不能为负值，一般5-15左右的值比较合适，int 型*/
	bm->setUniquenessRatio(15);
 
	/*检查视差连通区域变化度的窗口大小, 值为 0 时取消 speckle 检查，int 型*/
	bm->setSpeckleWindowSize(100);
 
	/*视差变化阈值，当窗口内视差变化大于阈值时，该窗口内的视差清零，int 型*/
	bm->setSpeckleRange(32);
 
	/*
	左视差图（直接计算得出）和右视差图（通过cvValidateDisparity计算得出）之间的最大容许差异。
	超过该阈值的视差值将被清零。该参数默认为 -1，即不执行左右视差检查。int 型。
	注意在程序调试阶段最好保持该值为 -1，以便查看不同视差窗口生成的视差效果。
	*/
	bm->setDisp12MaxDiff(1);
 
	/*计算视差*/
	bm->compute(imgL, imgR, imgDisparity16S);
 
	//-- Check its extreme values
	double minVal; double maxVal;
	minMaxLoc(imgDisparity16S, &minVal, &maxVal);
 
	cout << minVal << "\t" << maxVal << endl;
 
	//--Display it as a CV_8UC1 image：16位有符号转为8位无符号
	imgDisparity16S.convertTo(imgDisparity8U, CV_8U, 255 / (numberOfDisparities*16.));
    
    waitKey(0);
}
 
 
int main()
{
	//--读取图像
	Mat imgL = imread("D:/Matlab_calib_photos/photos-3/left0.jpg", 0);
	Mat imgR = imread("D:/Matlab_calib_photos/photos-3/right0.jpg", 0);
 
	//--And create the image in which we will save our disparities
	Mat imgDisparity8U = Mat(imgL.rows, imgL.cols, CV_8UC1);
 
	calDispWithBM(imgL, imgR, imgDisparity8U);
    return 0;
}