#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
using namespace std;
using namespace cv;

vector<vector<Point2f> >corners_l_array, corners_r_array;
int array_index = 0;


int  main()
{
	Mat input_image;
	Mat image_left, image_right;
	Mat frame_l, frame_r;
	VideoCapture cam(0);


    //这个写大点
	Rect left_rect(0, 0, 639, 480);  //创建一个Rect框，属于cv中的类，四个参数代表x,y,width,height  
	Rect right_rect(640, 0, 639, 480);

	if (!cam.isOpened())
		exit(0);


	//把分辨率弄大点
	cam.set(CAP_PROP_FRAME_WIDTH, 1280);
	cam.set(CAP_PROP_FRAME_HEIGHT, 480);

	//读取摄像头校准文件
	Mat cameraMatrix[2], distCoeffs[2];
	FileStorage fs("D:/intrinsics.yml", FileStorage::READ);
	if (fs.isOpened())
	{
		fs["M1"] >> cameraMatrix[0];
		fs["D1"] >> distCoeffs[0];
		fs["M2"] >> cameraMatrix[1];
		fs["D2"] >> distCoeffs[1];
		fs.release();
	}
	else
		cout << "Error: can not read the intrinsic parameters\n";

	Mat R, T, E, F;
	Mat R1, R2, P1, P2, Q;
	Rect validRoi[2];
	Size imageSize(320, 240);

	fs.open("D:/extrinsics.yml", FileStorage::READ);
	if (fs.isOpened())
	{
		fs["R"] >> R;
		fs["T"] >> T;
		fs["R1"] >> R1;
		fs["R2"] >> R2;
		fs["P1"] >> P1;
		fs["P2"] >> P2;
		fs["Q"] >> Q;
		fs.release();
	}
	else
		cout << "Error: can not read the extrinsic parameters\n";

	stereoRectify(cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, R1, R2, P1, P2, Q,
		CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);


	// OpenCV can handle left-right
	// or up-down camera arrangements
	bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

	// COMPUTE AND DISPLAY RECTIFICATION
	Mat rmap[2][2];
	// IF BY CALIBRATED (BOUGUET'S METHOD)

	//Precompute maps for cv::remap()
	initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

	Mat canvas;
	double sf;
	int w, h;
	if (!isVerticalStereo)
	{
		sf = 600. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width*sf);
		h = cvRound(imageSize.height*sf);
		canvas.create(h, w * 2, CV_8UC3);
	}
	else
	{
		sf = 300. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width*sf);
		h = cvRound(imageSize.height*sf);
		canvas.create(h * 2, w, CV_8UC3);
	}

	destroyAllWindows();

	Mat imgLeft, imgRight;

	int ndisparities = 16 * 5;   /**< Range of disparity */
	int SADWindowSize = 31; /**< Size of the block window. Must be odd */
	Ptr<StereoBM> sbm = StereoBM::create(ndisparities, SADWindowSize);
	sbm->setMinDisparity(0);
	//sbm->setNumDisparities(64);
	sbm->setTextureThreshold(10);
	sbm->setDisp12MaxDiff(-1);
	sbm->setPreFilterCap(31);
	sbm->setUniquenessRatio(25);
	sbm->setSpeckleRange(32);
	sbm->setSpeckleWindowSize(100);


	Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 64, 7,
		10 * 7 * 7,
		40 * 7 * 7,
		1, 63, 10, 100, 32, StereoSGBM::MODE_SGBM);


	Mat rimg, cimg;
	Mat Mask;
	while (1)
	{
		cam >> input_image;
		frame_l = Mat(input_image, left_rect).clone();
		frame_r = Mat(input_image, right_rect).clone();//分离左右视野
		std::cout << "Original frame Type: CV_"<<frame_l.type()<<"UC"<<frame_l.channels()<<std::endl;
		if (frame_l.empty() || frame_r.empty())
			continue;

		remap(frame_l, rimg, rmap[0][0], rmap[0][1], INTER_LINEAR);
		rimg.copyTo(cimg);
		Mat canvasPart1 = !isVerticalStereo ? canvas(Rect(w * 0, 0, w, h)) : canvas(Rect(0, h * 0, w, h));
		resize(cimg, canvasPart1, canvasPart1.size(), 0, 0, INTER_AREA);
		Rect vroi1(cvRound(validRoi[0].x*sf), cvRound(validRoi[0].y*sf),
			cvRound(validRoi[0].width*sf), cvRound(validRoi[0].height*sf));

		remap(frame_r, rimg, rmap[1][0], rmap[1][1], INTER_LINEAR);
		rimg.copyTo(cimg);
		Mat canvasPart2 = !isVerticalStereo ? canvas(Rect(w * 1, 0, w, h)) : canvas(Rect(0, h * 1, w, h));
		resize(cimg, canvasPart2, canvasPart2.size(), 0, 0, INTER_AREA);
		Rect vroi2 = Rect(cvRound(validRoi[1].x*sf), cvRound(validRoi[1].y*sf),
			cvRound(validRoi[1].width*sf), cvRound(validRoi[1].height*sf));

		Rect vroi = vroi1 & vroi2;

		imgLeft = canvasPart1(vroi).clone();
		imgRight = canvasPart2(vroi).clone();
		std::cout << "clone() option imgLeft Type: CV_"<<imgLeft.type()<<"UC"<<imgLeft.channels()<<std::endl;
		rectangle(canvasPart1, vroi1, Scalar(0, 0, 255), 3, 8);
		rectangle(canvasPart2, vroi2, Scalar(0, 0, 255), 3, 8);

		if (!isVerticalStereo)
			for (int j = 0; j < canvas.rows; j += 32)
				line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
		else
			for (int j = 0; j < canvas.cols; j += 32)
				line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);


		cvtColor(imgLeft, imgLeft, COLOR_BGR2GRAY);
		cvtColor(imgRight, imgRight, COLOR_BGR2GRAY);
		std::cout << "Atfer cvtColor() imgLeft Type: CV_"<<imgLeft.type()<<"UC"<<imgLeft.channels()<<std::endl;

		//-- And create the image in which we will save our disparities
		Mat imgDisparity16S = Mat(imgLeft.rows, imgLeft.cols, CV_16S);
		Mat imgDisparity8U = Mat(imgLeft.rows, imgLeft.cols, CV_8UC1);
		Mat sgbmDisp16S = Mat(imgLeft.rows, imgLeft.cols, CV_16S);
		Mat sgbmDisp8U = Mat(imgLeft.rows, imgLeft.cols, CV_8UC1);

		if (imgLeft.empty() || imgRight.empty())
		{
			std::cout << " --(!) Error reading images " << std::endl; return -1;
		} else {
			std::cout << "imgLeft Type: CV_"<<imgLeft.type()<<"UC"<<imgLeft.channels()<<std::endl;
		}
		sbm->compute(imgLeft, imgRight, imgDisparity16S);

		imgDisparity16S.convertTo(imgDisparity8U, CV_8UC1, 255.0 / 1000.0);
		cv::compare(imgDisparity16S, 0, Mask, CMP_GE);
		applyColorMap(imgDisparity8U, imgDisparity8U, COLORMAP_HSV);
		Mat disparityShow;
		imgDisparity8U.copyTo(disparityShow, Mask);




		sgbm->compute(imgLeft, imgRight, sgbmDisp16S);

		sgbmDisp16S.convertTo(sgbmDisp8U, CV_8UC1, 255.0 / 1000.0);
		cv::compare(sgbmDisp16S, 0, Mask, CMP_GE);
		applyColorMap(sgbmDisp8U, sgbmDisp8U, COLORMAP_COOL);
		Mat  sgbmDisparityShow,sgnm[3];
		sgbmDisp8U.copyTo(sgbmDisparityShow, Mask);


		//计算过程pixle像素，alpha，intercept未知。
		int pix = sgbmDisparityShow.at<Vec3b>(170, 150)[1];//分离出视野中心点的灰度值（170，150）是视野中心点坐标【1】表示是第二个通道
		double alpha = 0.35;
		double intercept = 24.75; //计算距离需要使用的两个参数
		//int distance = pix * alpha + intercept;//实际距离
		//cout << distance << "cm" << endl;
		cout << "即将显示各种图" << endl;
		cout << "按Q或者ESC退出预览" << endl;

		split(sgbmDisparityShow, sgnm);
		imshow("bmDisparity", disparityShow);//原始左右摄像头图像显示
		imshow("sgbmDisparity", sgbmDisparityShow);//热能图显示
		imshow("sgbm", sgnm[1]);//灰度显示
		imshow("rectified", canvas);//经过处理的热能图
		char c = (char)waitKey(1);
		if (c == 27 || c == 'q' || c == 'Q')
			break;
	}
	return 0;
}