#include<opencv2/opencv.hpp>
#include<opencv2/core/ocl.hpp>
using namespace cv;
int main(){
    bool ocl = cv::ocl::haveOpenCL();
    bool status = useOptimized();
    bool SSEStatus =  checkHardwareSupport(CV_SSE);
    std::cout<<"OpenCV useOptimized status: "<< status << std::endl;
    std::cout <<"OpenCV haveOpenCL status: "<< ocl<<std::endl;
    std::cout<<"CV_SSE status: "<<checkHardwareSupport(CV_SSE) <<std::endl;
    std::cout<<"CV_MMX status: "<<checkHardwareSupport(CV_MMX) <<std::endl;
    std::cout<<"CV_SSE2 status: "<<checkHardwareSupport(CV_SSE2) <<std::endl;
    std::cout<<"CV_SSE3 status: "<<checkHardwareSupport(CV_SSE3) <<std::endl;
    std::cout<<"CV_SSE4_1 status: "<<checkHardwareSupport(CV_SSE4_1) <<std::endl;
    std::cout<<"CV_SSE4_2 status: "<<checkHardwareSupport(CV_SSE4_2) <<std::endl;
    std::cout<<"CV_SSSE3 status: "<<checkHardwareSupport(CV_SSSE3) <<std::endl;
    std::cout<<"CV_POPCNT status: "<<checkHardwareSupport(CV_POPCNT) <<std::endl;
    std::cout<<"CV_AVX status: "<<checkHardwareSupport(CV_AVX) <<std::endl;
    std::cout<<"CV_AVX2 status: "<<checkHardwareSupport(CV_AVX2) <<std::endl;
    std::cout<<"CV_CPU_SSE2 status: "<<checkHardwareSupport(CV_CPU_SSE2) <<std::endl;
    
    return 0;
}