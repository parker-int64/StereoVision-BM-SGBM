#include<iostream>
#define WINDOW1_NAME "Control Panel"
#define CVUI_IMPLEMENTATION
#include"../include/cvui.h"
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;
int main(){
    cvui::init(WINDOW1_NAME);
    Mat control = Mat(Size(1360,800),CV_8UC3);
    control = Scalar(49, 52, 49);
    while(1){
        cvui::text(control, 80, 7, "Dynamic parameters control panel",1,0xCECECE);
        cvui::imshow(WINDOW1_NAME, control);
        char key = (char)waitKey(1);
        if(key == 'q' || key == 'Q' || key == 27)break;
    }
    return 0;
}