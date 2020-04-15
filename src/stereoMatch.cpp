#include "stereoMatch.hpp"
#include <pthread.h>
#define CVUI_IMPLEMENTATION
#include "cvui.h"
#define WINDOW1_NAME "Control Panel"
using namespace cv;
stereoMatch::stereoMatch(){

    imageWidth = captureWidth / 2;
    imageHeight = captureHeight;
    imageSize = Size(imageWidth, imageHeight);

    cameraMatrixL = (Mat_<double>(3, 3) << 228.8230, 0, 151.7397,
                                            0, 228.7386, 116.2399,
                                            0, 0, 1);
      
    cameraMatrixR = (Mat_<double>(3, 3) << 231.1488, 0, 150.5619,
                                            0, 231.0705, 119.2507,
                                            0, 0, 1);

    distCoeffL = (Mat_<double>(4, 1) << 0.1094, -0.0779, 0, 0);
    distCoeffR = (Mat_<double>(4, 1) << 0.1163, -0.2442, 0, 0);
    T = (Mat_<double>(3, 1) << -68.1526, 0.1022, -0.0327);              //T平移向量
    rec = (Mat_<double>(3, 1) << 0.0037, 0.0056, -0.0027);              //rec旋转向量
    Rodrigues(rec,R);                                                   //做一次罗德里格旋转变换得到旋转矩阵



    // SteroBM参数如下
    blockSizeBM             = 9;
    preFilterTypeBM         = 1;   // StereoBM::PREFILTER_XSOBEL
    preFilterSizeBM         = 11;
    preFilterCapBM          = 31;
    minDisparityBM          = 0;
    numDisparitiesBM        = 16;
    textureThresholdBM      = 0;
    speckleWindowSizeBM     = 100;
    speckleRangeBM          = 48;
    disp12MaxDiffBM         = -1;
    uniquenessRatioBM       = 10; //5-15 int   




    // StereoSGBM参数如下
    minDisparitySGBM        = 0;
    SADWindowSizeSGBM       = 3;
    numDisparitiesSGBM      = 48;
    preFilterCapSGBM        = 61;
    uniquenessRatioSGBM     = 6;
    speckleRangeSGBM        = 2;
    speckleWindowSizeSGBM   = 100;
    disp12MaxDiffSGBM       = -1;
    modeSGBM                = StereoSGBM::MODE_HH;


    // 功能模块变量
    switchAlg = true;
    stroagePathBM   = "../data/StereoBM-Parameters.yml";
    stroagePathSGBM = "../data/StereoSGBM-Parameters.yml";
    sf = 240. / MAX(imageSize.width, imageSize.height);
    w = cvRound(imageSize.width*sf);
	h = cvRound(imageSize.height*sf);
	canvas.create(h, w * 2, CV_8UC3);
    
}

void stereoMatch::stereoTotal(){



#ifdef DEBUG
    control = Mat(Size(1180,620),CV_8UC3);
    control = Scalar(150, 150, 150);
    cvui::init(WINDOW1_NAME);
#endif // DEBUG


    stereoMatch();  //载入参数

    stereoRectify(cameraMatrixL, distCoeffL, 
                  cameraMatrixR, distCoeffR, 
                  imageSize, R, T, Rl, Rr, Pl, Pr, Q, 
                  CALIB_ZERO_DISPARITY,
                  0, imageSize, &validROIL, &validROIR);
    
    initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pr, imageSize, CV_16SC2, mapLx, mapLy);
    initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_16SC2, mapRx, mapRy);

    while(true){

        double start = getTickCount();
        while(prdIdx - csmIdx == 0);
        data[csmIdx % BUFFER_SIZE].img_l.copyTo(left_src);
        data[csmIdx % BUFFER_SIZE].img_r.copyTo(right_src);
        ++csmIdx;
        cvtColor(left_src,gray_left,COLOR_BGR2GRAY);
        cvtColor(right_src,gray_right,COLOR_BGR2GRAY);
        remap(gray_left, rectify_left, mapLx, mapLy, INTER_LINEAR);
        remap(gray_right, rectify_right, mapRx, mapRy, INTER_LINEAR);


#ifdef DEBUG
        Mat tempL = Mat(Size(rectify_left.cols,rectify_left.rows),CV_8UC3);
        Mat tempR = Mat(Size(rectify_right.cols,rectify_right.rows),CV_8UC3);
        cvtColor(rectify_left,tempL,COLOR_GRAY2BGR);
        cvtColor(rectify_right,tempR,COLOR_GRAY2BGR); 
        Mat canvasPart = canvas(Rect(0, 0, w, h));
        resize(tempL, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
        Rect vroiL  = Rect(cvRound(validROIL.x * sf), cvRound(validROIL.y * sf),
                cvRound(validROIL.width * sf), cvRound(validROIL.height * sf));
        rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);

        canvasPart = canvas(Rect(w, 0, w, h));
        resize(tempR, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
        Rect vroiR = Rect(cvRound(validROIR.x * sf), cvRound(validROIR.y * sf),
                cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));
        rectangle(canvasPart, vroiR, Scalar(0, 0, 255), 3, 8);
        for(int i = 0;i < canvas.rows;i += 32){
            line(canvas, Point(0,i), Point(canvas.cols, i), Scalar(0,255,0), 1, 8);
        }
        cvui::rect(control,70,75,canvas.cols,canvas.rows,0xffffff);
        cvui::image(control,70,75,canvas);
        visualDebug();
        cvui::imshow(WINDOW1_NAME, control);
#endif // DEBUG

        char key = (char)waitKey(1);
        switchAlg == true ? stereoMatchSGBM(rectify_left,rectify_right) : stereoMatchBM(rectify_left,rectify_right,validROIL,validROIR);
        resolveCoodinates();
        launchSerial(outputBuffer);  // 启动串口发送
        double timeCosume = (double)( getTickCount() - start ) / getTickFrequency();
        double fps = 1 / timeCosume;
#ifdef DEBUG
        cvui::window(control,780,70,320,200,"Information Output");
        cvui::beginColumn(control,810,100,200,150,20);
        cvui::printf(0.4,0xff0000,"FPS: %.2f",fps);
        cvui::text("Current Resolution:640x320");
        cvui::printf(0.4,0xCECECE,"x = %.2f",x);
        cvui::printf(0.4,0xCECECE,"y = %.2f",y);
        cvui::printf(0.4,0xCECECE,"z = %.2f",z);
        cvui::endColumn();
#endif // DEBUG
        // std::cout << "FPS:" << fps <<std::endl;
        if(key == 'S' || key == 's')switchAlg = !switchAlg;
        else if(key == 27 || key == 'q' || key == 'Q' || exitButton)break;
    }
}

void stereoMatch::stereoMatchBM(cv::Mat rectify_left, cv::Mat rectify_right, cv::Rect ROI1, cv::Rect ROI2){

    bm = StereoBM::create();
    bm->setROI1(ROI1);
    bm->setROI2(ROI2);
    bm->setBlockSize(blockSizeBM);
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
    disp = Mat(Size(rectify_left.cols,rectify_right.rows),CV_16S);    
    bm->compute(rectify_left,rectify_right,disp);
    disp.convertTo(disp8,CV_8U,255 / (numDisparitiesBM * 16.));
    imshow("DisparityBM",disp8);
#ifdef DEBUG
    if(saveButton){
        bm->save(stroagePathBM);
    }else if(loadButton){
        FileStorage fs(stroagePathBM,FileStorage::READ);
        if(!fs.isOpened()){
            std::cout << "Error:File open failed" << std::endl;
        }
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
#else
    imshow("DisparityBM",disp8);
#endif
    
}

void stereoMatch::stereoMatchSGBM(cv::Mat rectify_left, cv::Mat rectify_right){
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
#ifdef DEBUG
    if(saveButton){
        sgbm->save(stroagePathSGBM);
    }else if(loadButton){
        FileStorage fs(stroagePathSGBM,FileStorage::READ);
        if(!fs.isOpened()){
            std::cout << "Error:Failed to open the file!" << std::endl;
        }
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
#else
    imshow("DisparitySGBM",disp8U);
#endif 


}

void stereoMatch::resolveCoodinates(){

#ifdef DEBUG
    Point input = Point(160,120);
    reprojectImageTo3D(disp, xyz, Q,true);
    xyz = xyz * 16;
    x = xyz.at<Vec3f>(input)[0];
    y = xyz.at<Vec3f>(input)[1];
    z = xyz.at<Vec3f>(input)[2];
    memset(outputBuffer,'\0',35);
    sprintf(outputBuffer,"%.2f\n%.2f\n%.2f\n",x,y,z);
    // std::cout << "x = "<< x <<",y = " << y << ",z = "<< z << std::endl;
#else
    Point input = Point(0,0);
    cv::FileStorage fsCoord("../data/coord2D.yml",FileStorage::READ); 
    if(!fsCoord.isOpened()){
        std::cout << "ERROR: failed to open the file!" << std::endl;
    } else{
        fsCoord["POINT_2D"] >> input;
        fsCoord.release();
        if(input == Point(0,0)){
            outputBuffer[35] = {'0'}; //没接到2D点，3D点全为0
        } else {
            reprojectImageTo3D(disp, xyz, Q,true);
            xyz = xyz * 16;
            x = xyz.at<Vec3f>(input)[0];
            y = xyz.at<Vec3f>(input)[1];
            z = xyz.at<Vec3f>(input)[2];
            //TODO:数据处理
            std::cout << "x = "<< x <<",y = " << y << ",z = "<< z << std::endl;
            memset(outputBuffer,'\0',35);
            sprintf(outputBuffer,"%.2f\n%.2f\n%.2f\n",x,y,z);
        }
        
    }
#endif // DEBUG

}

void stereoMatch::visualDebug(){
    cvui::text(control,400,10,"Dynamic parameters control panel",1,0xCECECE);
    if(switchAlg == false){
        // Windows1,BM和SGBM参数设置
        cvui::window(control,30,320,800,310,"StereoBM Parameters Settings");
        cvui::beginColumn(control,50,345,100,200);
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
        cvui::beginColumn(control,180,345,100,200);
        cvui::space(10);
        cvui::trackbar(100,&preFilterTypeBM,0,1,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,1);
        cvui::trackbar(200,&preFilterSizeBM,5,255,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,2);
        cvui::trackbar(200,&preFilterCapBM,1,63,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,1);
        cvui::trackbar(200,&blockSizeBM,0,25,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,2);
        cvui::trackbar(200,&minDisparityBM,-10,10,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,1);
        cvui::trackbar(200,&numDisparitiesBM,0,72,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,16);
        cvui::endColumn();
        cvui::beginColumn(control,480,345,100,200);
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
        cvui::beginColumn(control,590,345,100,200);
        cvui::space(15);
        cvui::trackbar(200,&textureThresholdBM,0,50,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,1);
        cvui::trackbar(200,&uniquenessRatioBM,5,15,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,2);
        cvui::trackbar(200,&speckleWindowSizeBM,0,10,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,1);
        cvui::trackbar(200,&speckleRangeBM,0,10,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,1);
        cvui::trackbar(200,&disp12MaxDiffBM,-1,10,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,1);
        cvui::endColumn();   
        destroyWindow("DisparitySGBM");
    } else if(switchAlg == true) {
        cvui::window(control,30,320,800,310,"StereoSGBM Parameters Settings");
        cvui::beginColumn(control,50,345,100,200);
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
        cvui::beginColumn(control,180,345,100,200);
        cvui::space(10);
        cvui::trackbar(200,&preFilterCapSGBM,1,72,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,1);
        cvui::trackbar(200,&SADWindowSizeSGBM,1,11,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,1);
        cvui::trackbar(200,&minDisparitySGBM,-10,10,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,1);
        cvui::trackbar(200,&numDisparitiesSGBM,0,72,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,16);
        cvui::trackbar(200,&modeSGBM,0,3,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,1);
        cvui::endColumn();
        cvui::beginColumn(control,480,345,100,200);
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
        cvui::beginColumn(control,590,345,100,200);
        cvui::space(15);
        cvui::trackbar(200,&uniquenessRatioSGBM,5,15,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,2);
        cvui::trackbar(200,&speckleRangeSGBM,0,10,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,1);
        cvui::trackbar(200,&speckleWindowSizeSGBM,0,10,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,1);
        cvui::trackbar(200,&disp12MaxDiffSGBM,-1,10,1,"%.0Lf",cvui::TRACKBAR_DISCRETE,1);
        cvui::endColumn();
        destroyWindow("DisparityBM");
    }

    //Windows 2，匹配算法选择
    cvui::window(control,850,320,250,100,"Stereo Matching Method");
    switchAlg == false ? cvui::text(control,860,350,"Current Method:StereoBM") : cvui::text(control,860,350,"Current Method:StereoSGBM");
    cvui::text(control,860,380,"Press 'S' to switch method");
    cvui::beginRow(control,860,400,100,20,5);
    cvui::text("Pink",0.4,0xdb70db);
    cvui::text("variables are important");
    cvui::endRow();
    cvui::window(control,850,430,250,200,"Other Debug Options");
    cvui::beginColumn(control,880,470,100,100,10);
    saveButton = cvui::button("Sava Parameters");
    loadButton = cvui::button("Load Parameters");
    exitButton = cvui::button("Exit");
    cvui::endColumn();
    cvui::update(); 
}


void stereoMatch::launchSerial(char *outputbuffer){
    serial serialPort;
    serialPort.openPort();
    std::thread writer(serial::writePort,&serialPort,outputBuffer);
    writer.join();
}