#include <iostream>
#include <Darwin.h>

int main(int argc, char *argv[]) {
    std::cout << "Starting CM730 Platform Test" << std::endl;
    
    Darwin::Darwin darwin("/dev/ttyUSB0");
    
    
}
