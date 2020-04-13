#include "../include/receive_frame.hpp"

volatile unsigned int prdIdx;           //定义在此
volatile unsigned int csmIdx;           //定义在此
struct imgData data[BUFFER_SIZE];       //定义结构体
using namespace cv;
void receive_frame::receiver(){
#ifdef USE_VIDEO
    cap.open("../data/test.avi");
    if (!cap.isOpened()){
        std::cerr << "Error: Video file not found." << std::endl;
        exit(0);
    }
#else
    cap.set(CAP_DSHOW,1);
    captureHeight = 480;
    captureWidth = 1280;
    cap.open(0);
    // cap.set(CAP_PROP_FRAME_WIDTH,640);
    // cap.set(CAP_PROP_FRAME_HEIGHT,320);
    cap.set(CAP_PROP_FRAME_WIDTH,captureWidth);
    cap.set(CAP_PROP_FRAME_HEIGHT,captureHeight);
    if (!cap.isOpened()){
        std::cerr << "Error: Camera is not connected." << std::endl;
        exit(0);
    }
#endif // USE_VIDEO
    Rect left_rect(0,0,640,480);
    Rect right_rect(640,0,640,480);
    while(1){
        cap >> src_frame;
        //此步在于分离左右视图，并将其存入image_left与image_right
        image_left = Mat (src_frame,left_rect).clone();
        image_right = Mat (src_frame,right_rect).clone();
        //分离每帧出去，采用copyTo复制
        //copyTo复制是采用共享空间。
        //而clone则是完全拷贝
        while(prdIdx - csmIdx >= BUFFER_SIZE);
        image_left.copyTo(data[prdIdx % BUFFER_SIZE].img_l);
        image_right.copyTo(data[prdIdx % BUFFER_SIZE].img_r);
        ++prdIdx;
    }
}