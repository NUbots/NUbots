#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "VisionBlackboard/visioncontroller.h"

using namespace std;
using namespace cv;

int main(void)
{
    VisionController* vision = VisionController::getInstance();
    return vision->run();
}
