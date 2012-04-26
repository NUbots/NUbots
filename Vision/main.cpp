#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "VisionWrapper/visioncontrolwrapperpc.h"

using namespace std;
using namespace cv;

int main(void)
{
    VisionControlWrapper* vision = VisionControlWrapper::getInstance();
    char c=0;
    int error=0;
    while(c!=27 && error==0) {
        error = vision->runFrame();
        c = waitKey(1);
    }
    if(error != 0)
        cout << "Error: " << error << endl;
}
