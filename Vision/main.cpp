#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "VisionWrapper/visioncontrolwrapperpc.h"

using namespace std;
using namespace cv;

int main(void)
{
    VisionControlWrapper* vision = VisionControlWrapper::getInstance();
    //DataWrapper* visiondata = DataWrapper::getInstance();
    char c=0;
    int error=0;
    while(c!=27 && error==0) {
        //visiondata->updateFrame();
        error = vision->runFrame();
        c = waitKey();
    }
    if(error != 0)
        cout << "Error: " << error << endl;


    //IMAGE STREAM EDITOR
//    ifstream in;
//    ofstream out;
//    in.open("/home/shannon/nubot/imageout.strm");
//    out.open("/home/shannon/nubot/imageout2.strm");
//    NUImage img;
//    Mat cvimg;
//    LookUpTable lut;
//    namedWindow("demo");
//    bool saving = false;
//    char c;

//    lut.loadLUTFromFile("/home/shannon/nubot/default.lut");

//    while(in.good()) {
//        in >> img;
//        lut.classifyImage(img, cvimg);
//        imshow("demo", cvimg);
//        c = cv::waitKey();
//        if(c==32)
//            saving = !saving;
//        if(saving)
//            out << img;
//        if(c==27)
//            break;
//    }
//    in.close();
//    out.close();
}
