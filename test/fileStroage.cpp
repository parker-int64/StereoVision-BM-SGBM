#include <opencv2/opencv.hpp>
using namespace cv;
int main(int argc, const char** argv) {
    Point in = Point(0,0);
    FileStorage fs("./data/coord2D.yml",FileStorage::WRITE | FileStorage::APPEND);
    fs.writeComment("Comment here");
    fs << "Defalut" << Point(0,0);
    while(true){
        int x = rand()%100;
        int y = rand()%100;
        
        fs << "POINT_2D" << Point(x,y);
        fs.release();
        FileStorage fsRead("./data/coord2D.yml",FileStorage::READ);
        fsRead["POINT_2D"] >> in;
        fsRead.release();
        std::cout << "Point(x,y): " << Point(x,y) <<std::endl;
        std::cout << "Point in: "  << in <<std::endl;
    }
    return 0;
}