#include "../include/receive_frame.hpp"
#include "../include/process_frame.hpp"

int main(int argc, char** argv) {
    std::cout << "starting program..." << std::endl;
    process_frame pf;
    std::thread t1 (receive_frame::receiver,&pf);
    std::thread t2 (process_frame::stereo_total,&pf);
    t1.join();
    t2.join();
    return 0;
}