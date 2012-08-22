#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "VisionWrapper/visioncontrolwrapperpc.h"

using namespace std;
using namespace cv;

int main(void)
{
    char ESC_KEY            = 27,
         STEP_KEY           = ' ',
         STEP_TOGGLE_KEY    = 's';

    VisionControlWrapper* vision = VisionControlWrapper::getInstance();
    //DataWrapper* visiondata = DataWrapper::getInstance();
    char c=0;
    int error=0;
    bool stepping = true;
    while(c!=ESC_KEY && error==0) {
        //visiondata->updateFrame();
        error = vision->runFrame();
        if(stepping) {
            c=0;
            while(c!=STEP_KEY && c!=ESC_KEY && c!=STEP_TOGGLE_KEY) {
                c=waitKey();
            }
        }
        else {
            c = waitKey(1);
        }
        if(c==STEP_TOGGLE_KEY) {
            stepping = !stepping;
        }
    }
    if(error != 0)
        cout << "Error: " << error << endl;
}
