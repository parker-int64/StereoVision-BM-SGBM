#include "receiver.hpp"
#include "stereoMatch.hpp"
#include "serial.h"

int main(int argc, char** argv) {
    std::cout << "Starting program..." << std::endl;
    stereoMatch match;
    std::thread t1 (receiver::stroage,&match);
    std::thread t2 (stereoMatch::stereoTotal,&match);
    t1.join();
    t2.join();

    
    return 0;
}