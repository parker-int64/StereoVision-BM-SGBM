#include "receiver.hpp"
volatile unsigned int prdIdx;           //定义在此
volatile unsigned int csmIdx;           //定义在此
struct imgData data[BUFFER_SIZE];       //定义结构体
using namespace cv;

receiver::receiver(){
    captureHeight = 240;
    captureWidth = 640; 
}

void receiver::stroage(){
#ifdef USE_VIDEO
    cap.open("../data/test.avi");
    if (!cap.isOpened()){
        std::cerr << "Error: Video file not found." << std::endl;
        exit(0);
    }
#else
    receiver();
    cap.set(CAP_DSHOW,1);
    cap.set(CAP_PROP_FRAME_WIDTH,captureWidth);
    cap.set(CAP_PROP_FRAME_HEIGHT,captureHeight);
    cap.open(0);
    if (!cap.isOpened()){
        std::cerr << "Error: Camera is not connected." << std::endl;
        exit(0);
    }
#endif // USE_VIDEO
    Rect left_rect(0,0,320,240);
    Rect right_rect(320,0,320,240);
    while(1){
        cap >> src_frame;
        image_left = Mat (src_frame,left_rect).clone();
        image_right = Mat (src_frame,right_rect).clone();
        while(prdIdx - csmIdx >= BUFFER_SIZE);
        image_left.copyTo(data[prdIdx % BUFFER_SIZE].img_l);
        image_right.copyTo(data[prdIdx % BUFFER_SIZE].img_r);
        ++prdIdx;
    }
}