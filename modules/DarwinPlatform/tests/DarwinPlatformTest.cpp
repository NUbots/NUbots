#include <iostream>
#include <Darwin.h>

int main(int argc, char *argv[]) {
    std::cout << "Starting CM730 Platform Test" << std::endl;
    
    // Connect to the darwin's CM730
    Darwin::Darwin darwin("/dev/ttyUSB0");
    
    Darwin::BulkReadResults r = darwin.bulkRead();
}
