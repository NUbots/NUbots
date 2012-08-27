#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv/highgui.h>

#include "../Vision/VisionWrapper/visioncontrolwrappertraining.h"
#include "../Infrastructure/NUSensorsData/NUSensorsData.h"
#include "../Infrastructure/NUImage/NUImage.h"

int runDave();
int runDefault();
vector<double> checkImageStream(string name);
vector<double> checkSensorStream(string name);

int main()
{
    vector<double> imagetimes = checkImageStream(string(getenv("HOME")) + string("/Images/JoWedNight/image.strm"));
    vector<double> sensortimes = checkSensorStream(string(getenv("HOME")) + string("/Images/JoWedNight/sensor.strm"));
    cout << imagetimes << endl;
    cout << sensortimes << endl;
    double max_diff = 0.0;
    double sum = 0.0;
    double geo_mean = 1.0;
    double diff;
    int frames = min(imagetimes.size(), sensortimes.size());
    for(int i = 0; i<frames; i++) {
        diff = abs(imagetimes[i] - sensortimes[i]);
        max_diff = max(diff, max_diff);
        if(diff > 100)
            cout << "im: " << imagetimes[i] << " sen: " << sensortimes[i] << " diff: " << diff << endl;
        sum += diff;
        geo_mean *= pow(diff, 1.0/frames);
    }
    cout << "max difference: " << max_diff << " average: " << sum/frames << " geo mean: " << geo_mean << endl;
    max_diff = 0.0;
    sum = 0.0;
    geo_mean = 1.0;
    for(int i = 0; i<imagetimes.size()-1; i++) {
        diff = abs(imagetimes[i] - imagetimes[i+1]);
        max_diff = max(diff, max_diff);
        sum += diff;
        geo_mean *= pow(diff, 1.0/(imagetimes.size()-1));
    }
    cout << "max difference: " << max_diff << " average: " << sum/(imagetimes.size()-1) << " geo mean: " << geo_mean << endl;
    max_diff = 0.0;
    sum = 0.0;
    geo_mean = 1.0;
    for(int i = 0; i<sensortimes.size()-1; i++) {
        diff = abs(sensortimes[i] - sensortimes[i+1]);
        max_diff = max(diff, max_diff);
        sum += diff;
        geo_mean *= pow(diff, 1.0/(sensortimes.size()-1));
    }
    cout << "max difference: " << max_diff << " average: " << sum/(sensortimes.size()-1) << " geo mean: " << geo_mean << endl;
    //return runDave();
    //return runDefault();
}


vector<double> checkImageStream(string name)
{
    vector<double> times;
    ifstream strm(name.c_str());
    NUImage image;
    double last_time = -1;
    int duplicates = 0;
    int frames = 0;
    while(strm.good()) {
        strm >> image;
        if(last_time == image.GetTimestamp())
            duplicates++;
        frames++;
        last_time = image.GetTimestamp();
        times.push_back(last_time);
        strm.peek();
    }
    cout << "Image stream: " << name << " frames: " << frames << " duplicates: " << duplicates << std::endl;
    return times;
}

vector<double> checkSensorStream(string name)
{
    vector<double> times;
    ifstream strm(name.c_str());
    NUSensorsData sensors;
    vector<float> data;
    double last_time = -1;
    int duplicates = 0;
    int frames = 0;
    while(strm.good()) {
        try {
            strm >> sensors;
        }
        catch(exception e) {
            break;
        }
        if(last_time == sensors.GetTimestamp())
            duplicates++;
        if(last_time == -1) {
            sensors.getAccelerometer(data);
                for(int i=0; i<data.size(); i++) {
                    cout << data.at(i) << endl;
                }
        }
        frames++;
        last_time = sensors.GetTimestamp();
        times.push_back(last_time);
        strm.peek();
    }
    cout << "Sensor stream: " << name << " frames: " << frames << " duplicates: " << duplicates << std::endl;
    return times;
}

int runDave()
{
    int N=10;
    string dir = getenv("HOME") + string("/nubot/");
    string outfilename;
    ofstream out;
    VisionControlWrapper* vision = VisionControlWrapper::getInstance();
    int failed_LUTS = 0;
    for(int x=1; x<=N; x++) {
        for(int y=5; y <= 50; y+=5) {
            vision->resetHistory();
            vision->restartStream();
            stringstream oss;
            oss << dir << "lut" << x << "_" << y;
            cout << oss.str() << ".lut" << endl;
            if(!vision->setLUT(oss.str() + string(".lut")))
                failed_LUTS++;
            //run vision over all frames
            while(vision->runFrame() == 0);
            outfilename = oss.str() + string(".txt");
            out.open(outfilename.c_str());
            vision->writeDetections(out);
            out.close();
        }
    }
    if(failed_LUTS > 0)
        cerr << failed_LUTS << " LUTs failed to load" << endl;
    cout << "done" << endl;
    return 0;
}

int runDefault()
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
                c = cv::waitKey();
            }
        }
        else {
            c = cv::waitKey(1);
        }
        if(c==STEP_TOGGLE_KEY) {
            stepping = !stepping;
        }
    }
    if(error != 0)
        cout << "Error: " << error << endl;
    return error;
}
