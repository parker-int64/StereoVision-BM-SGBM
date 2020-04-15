#ifndef __RECEIVER_HPP
#define __RECEIVER_HPP
#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>
#define BUFFER_SIZE 1
class receiver{
    public:
        receiver();
        void stroage();
        cv::VideoCapture cap; 
        int captureHeight;
        int captureWidth; 
    private:
        cv::Mat src_frame;         //源帧
        cv::Mat image_left;        //双目左图
        cv::Mat image_right;       //双目右图
        
};
extern volatile unsigned int prdIdx;        //produce index
extern volatile unsigned int csmIdx;        //consume index
extern struct imgData data[BUFFER_SIZE];    //结构体声明
struct imgData{
    cv::Mat img_l;
    cv::Mat img_r; 
};
#endif // !__RECEIVER_HPP