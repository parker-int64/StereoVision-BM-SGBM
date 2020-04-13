#ifndef _RECEIVE_FRAME_HPP_
#define _RECEIVE_FRAME_HPP_
#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>
#define BUFFER_SIZE 1
class receive_frame{
    public:
        void receiver();
        cv::VideoCapture cap; 
        int captureHeight;
        int captureWidth; 
    private:
        cv::Mat src_frame;         //源帧
        cv::Mat image_left;        //双目左图
        cv::Mat image_right;       //双目右图
        
};
//声明在此
extern volatile unsigned int prdIdx; //produce index
extern volatile unsigned int csmIdx; //consume index
//结构体声明
extern struct imgData data[BUFFER_SIZE];
struct imgData{
    cv::Mat img_l;
    cv::Mat img_r; 
};//结构体用以储存第一个线程分离出的帧

#endif // !__RECEIVE_FRAME__