#include "../include/process_frame.hpp"
#define WINDOW1_NAME "Control Panel"
#define CVUI_IMPLEMENTATION
#include "../include/cvui.h"
using namespace cv;
void process_frame::stereo_total(){
#ifdef DEBUG
    ocl::setUseOpenCL(true);        //开启OpenCL
    control = Mat(Size(1360,840),CV_8UC3);
    control = Scalar(150, 150, 150);
    cvui::init(WINDOW1_NAME);
#endif // DEBUG
    //定义要用的变量
    // const int imageWidth = 640;                             //摄像头的分辨率  
    // const int imageHeight = 480;
    int imageWidth = captureWidth / 2;
    int imageHeight = captureHeight;
    Size imageSize = Size(imageWidth, imageHeight);
    //定义相机矩阵，分内参和外参。标定数据来自Matlab
    cameraMatrixL = (Mat_<double>(3, 3) << 452.8131, 0, 303.0163,
                                            0, 452.8235, 228.0770,
                                            0, 0, 1);
      
    distCoeffL = (Mat_<double>(4, 1) << -0.0241,0.4826,0,0);
    cameraMatrixR = (Mat_<double>(3, 3) << 453.6927, 0, 311.4093,
                                            0, 453.9168, 237.1017,
                                            0, 0, 1);
    distCoeffR = (Mat_<double>(4, 1) << 0.0171,-0.0338,0,0);
    T = (Mat_<double>(3, 1) << -70.2056, 0.1296, 0.6029);//T平移向量
    rec = (Mat_<double>(3, 1) << 0.0076, -0.0140, 0.00240);//rec旋转向量
    Rodrigues(rec,R); //做一次罗德里格旋转变换得到旋转矩阵
    //输入的旋转向量(3 * 1)
    //输出的旋转矩阵(3 * 1)
    //参数矩阵定义完毕
    //选用算法sgbm
    //载入矩阵
    /*---------------双目矫正-------------*/
    stereoRectify(cameraMatrixL, distCoeffL, 
                  cameraMatrixR, distCoeffR, 
                  imageSize, R, T, Rl, Rr, Pl, Pr, Q, 
                  CALIB_ZERO_DISPARITY,
                  0, imageSize, &validROIL, &validROIR);

    initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pr, imageSize, CV_32FC1, mapLx, mapLy);
    initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);

    // StereoBM参数列表
    blockSizeBM = 5;
    preFilterTypeBM = 1;   //StereoBM::PREFILTER_XSOBEL
    preFilterSizeBM = 11;
    preFilterCapBM = 31;
    minDisparityBM = 0;
    numDisparitiesBM = 16;
    textureThresholdBM = 0;
    speckleWindowSizeBM = 100;
    speckleRangeBM = 32;
    disp12MaxDiffBM = -1;
    uniquenessRatioBM = 10; //5-15 int   

    // StereoSGBM参数列表
    minDisparitySGBM = 0;
    SADWindowSizeSGBM = 11;
    numDisparitiesSGBM = 16;
    preFilterCapSGBM = 15;
    uniquenessRatioSGBM = 6;
    speckleRangeSGBM = 2;
    speckleWindowSizeSGBM = 100;
    disp12MaxDiffSGBM = 1;
    modeSGBM = StereoSGBM::MODE_HH;

    // 功能模块变量
    switchAlg = false;
    stroagePathBM = "../data/StereoBM-Parameters.yml";
    stroagePathSGBM = "../data/StereoSGBM-Parameters.yml";
    //循环开始
    while(1){
        while(prdIdx - csmIdx == 0);
        data[csmIdx % BUFFER_SIZE].img_l.copyTo(left_src);
        data[csmIdx % BUFFER_SIZE].img_r.copyTo(right_src);
        ++csmIdx;
        double start = cv::getTickCount();
        //颜色空间变换
        cvtColor(left_src,gray_left,COLOR_BGR2GRAY);
        cvtColor(right_src,gray_right,COLOR_BGR2GRAY);
        // GaussianBlur(gray_left,gray_left,Size(5,5),0,0);
        // GaussianBlur(gray_right,gray_right,Size(5,5),0,0);
        medianBlur(gray_left,gray_left,5);
        medianBlur(gray_right,gray_right,5);
        //经过remap，左右相机的图像已经共面并且行对准了
        remap(gray_left, rectify_left, mapLx, mapLy, INTER_LINEAR);
        remap(gray_right, rectify_right, mapRx, mapRy, INTER_LINEAR);
        // StereoBM要求图片类型必须是CV_8UC1,在此处做转换，注意，简单用convertTo()可能不行
        leftBM = Mat(Size(rectify_left.cols,rectify_left.rows),CV_8UC1);
        rectify_left.convertTo(leftBM,CV_8UC1);
        rightBM = Mat(Size(rectify_right.cols,rectify_right.rows),CV_8UC1);
        rectify_right.convertTo(rightBM,CV_8UC1);

#ifdef DEBUG
        if(rectify_left.empty() || rectify_right.empty()){
            std::cout << "ERROR: Rectify images are empty" << std::endl;
            return ;
        }
        cvtColor(rectify_left,rectify_left,COLOR_GRAY2BGR);
        cvtColor(rectify_right,rectify_right,COLOR_GRAY2BGR); 
        sf = 600.0 / MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width * sf);
        h = cvRound(imageSize.height * sf);
        canvas.create(h, w * 2, CV_8UC3);//创建画布
        //左图在画布上
        Mat canvasPart = canvas(Rect(0, 0, w, h));
        resize(rectify_left, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
        Rect vroiL (cvRound(validROIL.x * sf), cvRound(validROIL.y * sf),
                cvRound(validROIL.width * sf), cvRound(validROIL.height * sf));

        //右图画在画布上
        canvasPart = canvas(Rect(w, 0, w, h));
        resize(rectify_right, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
        Rect vroiR (cvRound(validROIR.x * sf), cvRound(validROIR.y * sf),
                cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));

        //画上极线
        for(int i = 0;i < canvas.rows;i += 32){
            line(canvas, Point(0,i), Point(canvas.cols, i), Scalar(0,255,0), 1, 8);
        }
        double time = (double)(cv::getTickCount() - start ) / cv::getTickFrequency();
        double FPS = 1 / time;
        char fps_str[10];
        sprintf(fps_str,"FPS:%.2f",FPS);
        putText(canvas,fps_str,Point(0,30),FONT_HERSHEY_COMPLEX_SMALL,1.0,Scalar(255,255,0),1,8);
        cvui::rect(control,70,45,canvas.cols,canvas.rows,0x00ffff);
        cvui::image(control,70,45,canvas);
        visual_debug();
        cvui::imshow(WINDOW1_NAME, control);
#endif // DEBUG        
        char key = (char)waitKey(1);
        //setUseOptimized(true);
        //stereoBinaryBM(leftBM,rightBM);
        switchAlg == true ? stereo_matchSGBM(rectify_left,rectify_right) : stereo_matchBM(leftBM,rightBM,validROIL,validROIR);
        resolve_coodinates();
#ifdef DEBUG        
        double time_consume = (double)(getTickCount() - start1) / getTickFrequency();
        double fps1 = 1 / time_consume;
        std::cout << "Total time comsume:"<<time_consume<<", FPS:"<<fps1<<std::endl;
#endif //DEBUG        
        if(key == 'S' || key == 's')switchAlg = !switchAlg;
        else if(key == 27 || key == 'q' || key == 'Q' || exitButton)break;     
    }
    cap.release();
    destroyAllWindows();
}

/*--------BM算法--------*/
void process_frame::stereo_matchBM(cv::Mat rectify_left,cv::Mat rectify_right,cv::Rect ROI1,cv::Rect ROI2){
    bm = StereoBM::create(numDisparitiesBM,blockSizeBM);
    bm->setBlockSize(blockSizeBM);
    bm->setROI1(ROI1);
    bm->setROI2(ROI2);
    bm->setPreFilterType(preFilterTypeBM);
    bm->setPreFilterSize(preFilterSizeBM);
    bm->setPreFilterCap(preFilterCapBM);
    bm->setMinDisparity(minDisparityBM);
    bm->setNumDisparities(numDisparitiesBM);
    bm->setTextureThreshold(textureThresholdBM);
    bm->setSpeckleWindowSize(speckleWindowSizeBM);
    bm->setSpeckleRange(speckleRangeBM);
    bm->setDisp12MaxDiff(disp12MaxDiffBM);
    bm->setUniquenessRatio(uniquenessRatioBM);
    bm->compute(rectify_left,rectify_right,disp);
    disp.convertTo(disp8,CV_8U,255 / (numDisparitiesBM * 16.));
    if(saveButton){
        bm->save(stroagePathBM);
    }else if(loadButton){
        FileStorage fs(stroagePathBM,FileStorage::READ);
#ifdef DEBUG
        if(!fs.isOpened()){
            std::cout << "Error:File open failed" << std::endl;
        }
#endif // DEBUG
        FileNode node = fs["my_object"];
        node["minDisparity"] >> minDisparityBM;
        node["numDisparities"] >> numDisparitiesBM;
        node["blockSize"] >> blockSizeBM;
        node["speckleWindowSize"] >> speckleWindowSizeBM;
        node["speckleRange"] >> speckleRangeBM;
        node["disp12MaxDiff"] >> disp12MaxDiffBM;
        node["preFilterType"] >> preFilterTypeBM;
        node["preFilterSize"] >> preFilterSizeBM;
        node["preFilterCap"] >> preFilterCapBM;
        node["textureThreshold"] >> textureThresholdBM;
        node["uniquenessRatio"] >> uniquenessRatioBM;
        fs.release();
    } 
    imshow("DisparityBM",disp8);
}

/*--------SGBM算法--------*/
void process_frame::stereo_matchSGBM(cv::Mat rectify_left,cv::Mat rectify_right){
    sgbm = StereoSGBM::create(minDisparitySGBM, numDisparitiesSGBM, SADWindowSizeSGBM);
    int P1 = 8 * rectify_left.channels() * SADWindowSizeSGBM * SADWindowSizeSGBM;
    int P2 = 32 * rectify_left.channels() * SADWindowSizeSGBM * SADWindowSizeSGBM;
    sgbm->setP1(P1);
    sgbm->setP2(P2);
    sgbm->setPreFilterCap(preFilterCapSGBM);
    sgbm->setMinDisparity(minDisparitySGBM);
    sgbm->setNumDisparities(numDisparitiesSGBM);
    sgbm->setUniquenessRatio(uniquenessRatioSGBM);
    sgbm->setSpeckleRange(speckleRangeSGBM);
    sgbm->setSpeckleWindowSize(speckleWindowSizeSGBM);
    sgbm->setDisp12MaxDiff(disp12MaxDiffSGBM);
    sgbm->setMode(modeSGBM);
    sgbm->compute(rectify_left, rectify_right, disp); 
    disp.convertTo(disp8U,CV_8U,255 / (numDisparitiesSGBM * 16.));
    imshow("DisparitySGBM",disp8U);
    if(saveButton){
        sgbm->save(stroagePathSGBM);
    }else if(loadButton){
        FileStorage fs(stroagePathSGBM,FileStorage::READ);
#ifdef DEBUG
        if(!fs.isOpened()){
            std::cout << "Error:File open failed" << std::endl;
        }
#endif // DEBUG
        FileNode node = fs["my_object"];
        node["minDisparity"] >> minDisparitySGBM;
        node["numDisparities"] >> numDisparitiesSGBM;
        node["blockSize"] >> SADWindowSizeSGBM;
        node["speckleWindowSize"] >> speckleWindowSizeSGBM;
        node["speckleRange"] >> speckleRangeSGBM;
        node["disp12MaxDiff"] >> disp12MaxDiffSGBM;
        node["preFilterCap"] >> preFilterCapSGBM;
        node["uniquenessRatio"] >> uniquenessRatioSGBM;
        node["mode"] >> modeSGBM;
        fs.release();
    }
}


/*
void process_frame::stereoBinaryBM(cv::Mat rectify_left,cv::Mat rectify_right){
    Binarybm = stereo::StereoBinaryBM::create(numDisparitiesBM,blockSizeBM);
    Binarybm->setBlockSize(blockSizeBM);
    Binarybm->setPreFilterType(preFilterTypeBM);
    Binarybm->setPreFilterSize(preFilterSizeBM);
    Binarybm->setPreFilterCap(preFilterCapBM);
    Binarybm->setMinDisparity(minDisparityBM);
    Binarybm->setNumDisparities(numDisparitiesBM);
    Binarybm->setTextureThreshold(textureThresholdBM);
    Binarybm->setSpeckleWindowSize(speckleWindowSizeBM);
    Binarybm->setSpeckleRange(speckleRangeBM);
    Binarybm->setDisp12MaxDiff(0);
    Binarybm->setUniquenessRatio(uniquenessRatioBM);
    Binarybm->compute(rectify_left,rectify_right,disp);
    disp.convertTo(disp8,CV_8U,255 / (numDisparitiesBM * 16.));
//     if(saveButton){
//         Binarybm->save(stroagePathBM);
//     }else if(loadButton){
//         FileStorage fs(stroagePathBM,FileStorage::READ);
// #ifdef DEBUG
//         if(!fs.isOpened()){
//             std::cout << "Error:File open failed" << std::endl;
//         }
// #endif // DEBUG
//         FileNode node = fs["my_object"];
//         node["minDisparity"] >> minDisparityBM;
//         node["numDisparities"] >> numDisparitiesBM;
//         node["blockSize"] >> blockSizeBM;
//         node["speckleWindowSize"] >> speckleWindowSizeBM;
//         node["speckleRange"] >> speckleRangeBM;
//         node["disp12MaxDiff"] >> disp12MaxDiffBM;
//         node["preFilterType"] >> preFilterTypeBM;
//         node["preFilterSize"] >> preFilterSizeBM;
//         node["preFilterCap"] >> preFilterCapBM;
//         node["textureThreshold"] >> textureThresholdBM;
//         node["uniquenessRatio"] >> uniquenessRatioBM;
//         fs.release();
//     } 
    imshow("DisparityBinaryBM",disp8);
}


void process_frame::stereoBinarySGBM(cv::Mat rectify_left,cv::Mat rectify_right){
    
}
*/

/*---------------世界坐标-------------*/
void process_frame::resolve_coodinates(){
    // TODO:解析三维坐标
    coordinatesPath = "../data/coord3D.txt";
    Point input = Point(320,240);
    reprojectImageTo3D(disp, xyz, Q);
    xyz = xyz * 16;
    double x = xyz.at<Vec3f>(input)[0];
    double y = xyz.at<Vec3f>(input)[1];
    double z = xyz.at<Vec3f>(input)[2];
    of.open(coordinatesPath);
    of << x << "," << y << "," << z << std::endl;
    //三个数直接发
    //of << x << y<< z << std::endl;
    of.close();
#ifdef DEBUG
    std::cout << input <<" in world coordinate is "<<xyz.at<Vec3f>(input)<<std::endl;
    std::cout <<"x = "<< x << ",y = " << y << ",z = " << z << std::endl;
#endif // DEBUG

}

/*-----------------------CVUI调参界面-------------------*/
void process_frame::visual_debug(){
    cvui::text(control,400,10,"Dynamic parameters control panel",1,0xCECECE);
    if(switchAlg == false){
        // Windows1,BM和SGBM参数设置
        cvui::window(control,30,520,800,310,"StereoBM Parameters Settings");
        cvui::beginColumn(control,50,545,100,200);
        cvui::text("PREPROCESS",0.6,0xffff00);
        cvui::space(10);
        cvui::text("preFilterType");
        cvui::space(35);
        cvui::text("preFilterSize");
        cvui::space(35);
        cvui::text("preFilterCap");
        cvui::space(35);
        cvui::text("blockSize",0.4,0xdb70db);
        cvui::space(35);
        cvui::text("minDisparity");
        cvui::space(35);
        cvui::text("numDisparities",0.4,0xdb70db);
        cvui::endColumn();
        cvui::beginColumn(control,180,545,100,200);
        cvui::space(10);
        cvui::trackbar(100,&preFilterTypeBM,0,1,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,1);
        cvui::trackbar(200,&preFilterSizeBM,5,255,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,2);
        cvui::trackbar(200,&preFilterCapBM,1,63,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,1);
        cvui::trackbar(200,&blockSizeBM,0,25,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,2);
        cvui::trackbar(200,&minDisparityBM,-10,10,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,1);
        cvui::trackbar(200,&numDisparitiesBM,0,72,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,16);
        cvui::endColumn();
        cvui::beginColumn(control,480,545,100,200);
        cvui::text("POSTPROCESS",0.6,0xffff00);
        cvui::space(20);
        cvui::text("textureThreshold");
        cvui::space(35);
        cvui::text("uniquenessRatio",0.4,0xdb70db);
        cvui::space(35);
        cvui::text("speckleWindowSize");
        cvui::space(35);
        cvui::text("speckleRange");
        cvui::space(35);
        cvui::text("disp12MaxDiff");
        cvui::endColumn();
        cvui::beginColumn(control,590,545,100,200);
        cvui::space(15);
        cvui::trackbar(200,&textureThresholdBM,0,50,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,1);
        cvui::trackbar(200,&uniquenessRatioBM,5,15,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,2);
        cvui::trackbar(200,&speckleWindowSizeBM,0,10,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,1);
        cvui::trackbar(200,&speckleRangeBM,0,10,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,1);
        cvui::trackbar(200,&disp12MaxDiffBM,-1,10,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,1);
        cvui::endColumn();   
        destroyWindow("DisparitySGBM");
    } else if(switchAlg == true) {
        cvui::window(control,30,520,800,310,"StereoSGBM Parameters Settings");
        cvui::beginColumn(control,50,545,100,200);
        cvui::text("PREPROCESS",0.6,0xffff00);
        cvui::space(10);
        cvui::text("preFilterCap");
        cvui::space(35);
        cvui::text("SADWindowSize",0.4,0xdb70db);
        cvui::space(35);
        cvui::text("minDisparity");
        cvui::space(35);
        cvui::text("numDisparities",0.4,0xdb70db);
        cvui::space(35);
        cvui::text("mode");
        cvui::endColumn();
        cvui::beginColumn(control,180,545,100,200);
        cvui::space(10);
        cvui::trackbar(200,&preFilterCapSGBM,1,72,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,1);
        cvui::trackbar(200,&SADWindowSizeSGBM,1,11,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,1);
        cvui::trackbar(200,&minDisparitySGBM,-10,10,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,1);
        cvui::trackbar(200,&numDisparitiesSGBM,0,72,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,16);
        cvui::trackbar(200,&modeSGBM,0,3,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,1);
        cvui::endColumn();
        cvui::beginColumn(control,480,545,100,200);
        cvui::text("POSTPROCESS",0.6,0xffff00);
        cvui::space(20);
        cvui::text("uniquenessRatio",0.4,0xdb70db);
        cvui::space(35);
        cvui::text("speckleRange");
        cvui::space(35);
        cvui::text("speckleWindowSize");
        cvui::space(35);
        cvui::text("disp12MaxDiff");
        cvui::endColumn();
        cvui::beginColumn(control,590,545,100,200);
        cvui::space(15);
        cvui::trackbar(200,&uniquenessRatioSGBM,5,15,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,2);
        cvui::trackbar(200,&speckleRangeSGBM,0,10,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,1);
        cvui::trackbar(200,&speckleWindowSizeSGBM,0,10,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,1);
        cvui::trackbar(200,&disp12MaxDiffSGBM,-1,10,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,1);
        cvui::endColumn();
        destroyWindow("DisparityBM");
    }
    //Windows 2，匹配算法选择
    cvui::window(control,850,520,250,100,"Stereo Matching Method");
    switchAlg == false ? cvui::text(control,860,550,"Current Method:StereoBM") : cvui::text(control,860,550,"Current Method:StereoSGBM");
    cvui::text(control,860,580,"Press 'S' to switch method");
    cvui::beginRow(control,860,600,100,20,5);
    cvui::text("Pink",0.4,0xdb70db);
    cvui::text("variables are important");
    cvui::endRow();
    cvui::window(control,850,630,250,200,"Other Debug Options");
    cvui::beginColumn(control,880,670,100,100,10);
    saveButton = cvui::button("Sava Parameters");
    loadButton = cvui::button("Load Parameters");
    calibButton = cvui::button("Run Stereo Calibration");
    exitButton = cvui::button("Exit");
    cvui::endColumn();
    cvui::update(); 
}

