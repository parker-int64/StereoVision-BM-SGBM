#ifndef __STEREO_MATCH_HPP
#define __STEREO_MATCH_HPP
#include "receiver.hpp"
#include "serial.h"
// #include "socketServer.h"
class stereoMatch : public receiver{
    public:   
        stereoMatch();
        void visualDebug();  
        void stereoTotal();
        void stereoMatchBM(cv::Mat rectify_left,cv::Mat rectify_right,cv::Rect ROI1,cv::Rect ROI2);
        void stereoMatchSGBM(cv::Mat rectify_left,cv::Mat rectify_right);
        void resolveCoodinates();
        void launchSerial(char *outputbuffer);
        // void udpReceive(int &x,int &y);
        char outputBuffer[35];                  // 写入串口的信息
    private:
        /*矫正过程中的局部变量*/
        cv::Mat control;                        //调试控制面板
        cv::Mat left_src;                       //源图左
        cv::Mat right_src;                      //源图右
        cv::Mat gray_left;                      //灰度图左
        cv::Mat gray_right;                     //灰度图右
        cv::Mat rectify_left;                   //矫正图左（灰度图）
        cv::Mat rectify_right;                  //矫正图右（灰度图）
        cv::Mat after_rectifyL;                 //矫正之后，用于显示图左
        cv::Mat after_rectifyR;                 //矫正之后，用于显示图右
        cv::Rect validROIL;                     //图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域左  
        cv::Rect validROIR;                     //裁剪后区域右
        cv::Mat mapLx, mapLy, mapRx, mapRy;     //映射表  
        cv::Mat Rl, Rr, Pl, Pr, Q;              //校正旋转矩阵R，投影矩阵P，重投影矩阵Q
        cv::Mat xyz;                            //世界坐标系
        cv::Mat cameraMatrixL;                  //相机矩阵左
        cv::Mat cameraMatrixR;                  //相机矩阵右
        cv::Mat distCoeffL;                     //畸变矩阵左
        cv::Mat distCoeffR;                     //畸变矩阵右
        cv::Mat T;                              //平移向量T
        cv::Mat rec;                            //旋转向量rec
        cv::Mat R;                              //旋转矩阵
        cv::Mat canvas;                         // 画布，用以将矫正后左右图放至统一视图上
        cv::Mat disp;                           // 深度图
        double sf;
        int w,h;                                //宽高
        cv::Size imageSize;
        int imageWidth;
        int imageHeight;

        /*双目匹配BM局部变量*/
        cv::Mat leftBM;                         //参与最后计算的左图，格式CV_8UC1
        cv::Mat rightBM;                        //参与最后计算的右图，格式CV_8UC1
        cv::Mat disp8;                          //最后的差异图
        int numDisparitiesBM;
        int blockSizeBM;                        // 
        int preFilterTypeBM;                    // 预滤波类型
        int preFilterSizeBM;                    // 预滤波核大小
        int preFilterCapBM;                     // 预处理滤波器的截断值，预处理的输出值仅保留[-preFilterCap, preFilterCap]范围内的值
        int minDisparityBM;                     // 最小视差，默认值为0，可以为负
        int textureThresholdBM;                 //
        int speckleWindowSizeBM;                //
        int speckleRangeBM;                     //
        int disp12MaxDiffBM;                    //
        int uniquenessRatioBM;                  //
        cv::Ptr<cv::StereoBM> bm;

        /*双目匹配SBGM局部变量*/
        cv::Mat disp8U;
        int minDisparitySGBM;                       // 最小视差，默认值为 0, 可以是负值
        int nDisparitiesSGBM;                       // 视差窗口，即最大视差值与最小视差值之差,窗口大小必须是16的整数倍
        int SADWindowSizeSGBM;                      // SAD窗口大小，容许范围是[1,11]，一般应该在 3x3 至 11x11 之间，参数必须是奇数
        int numDisparitiesSGBM;                     // 问题变量,视差窗口，即最大视差值与最小视差值之差, 窗口大小必须是 16 的整数倍
        int P1;                                     /*  控制视差变化平滑性的参数。P1、P2的值越大，视差越平滑。
                                                    P1是相邻像素点视差增/减 1 时的惩罚系数；
                                                    P2是相邻像素点视差变化值大于1时的惩罚系数。
                                                    P2必须大于P1。*/ 
        int P2;                                 
        int preFilterCapSGBM;                       // 预处理滤波器的截断值，预处理的输出值仅保留[-preFilterCap, preFilterCap]范围内的值，参数范围：1 - 31
        int uniquenessRatioSGBM;                    // 视差唯一性百分比，该参数不能为负值，一般5-15左右
        int speckleRangeSGBM;                       // 视差变化阈值，当窗口内视差变化大于阈值时，该窗口内的视差清零
        int speckleWindowSizeSGBM;                  // 检查视差连通区域变化度的窗口大小, 值为 0 时取消 speckle 检查
        int disp12MaxDiffSGBM;                      // 左右一致性检测中最大容许误差值
        int modeSGBM;                               // 匹配模式
        cv::Ptr<cv::StereoSGBM> sgbm;
        /*
        在上述参数中，对视差生成效果影响较大的主要参数是
        SADWindowSize、numDisparities 
        和 uniquenessRatio 三个
        */

        /*调试及功能块变量*/
        bool isVerticalStereo;
        bool saveButton;                             // 储存参数
        bool loadButton;                             // 加载参数
        bool exitButton;                             // 退出按钮
        bool switchAlg;                              // 切换BM和SGBM算法
        std::string stroagePathBM;                   // 存储路径BM
        std::string stroagePathSGBM;                 // SGBM 
    protected:
        double x;                       // x
        double y;                       // y
        double z;                       // z      
};
#endif // !__STEREO_MATCH_HPP