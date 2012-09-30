#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include <boost/foreach.hpp>

#include "../Vision/VisionWrapper/visioncontrolwrappertraining.h"
#include "../Infrastructure/NUSensorsData/NUSensorsData.h"
#include "../Infrastructure/NUImage/NUImage.h"

#include "mainwindow.h"
#include <QApplication>

int runDave();
int runDefault();
vector<double> checkImageStream(string name);
vector<double> checkSensorStream(string name);

int main()
{
    ifstream performance_log(("/home/shannon/Images/FYP/Final100/VisionPSO_performance.log"));
    int garbage;
    float val;
    vector<float> perf;
    while(performance_log.good()) {
        performance_log >> garbage >> val;
        perf.push_back(val);
        performance_log.ignore(2, '\n');
        performance_log.peek();
    }
    performance_log.close();
    cout << perf.size();

    ofstream plog("/home/shannon/Images/FYP/Final100/VisionPSO_performance.log");
    plog.setf(ios_base::fixed);
    for(unsigned int i=0; i<perf.size(); i++) {
        plog << i << " " << perf[i] << endl;
    }
    plog.close();

//    ifstream in("/home/shannon/Images/FYP/Final100/opt_labels.strm");
//    ofstream out("/home/shannon/Images/FYP/Final100/opt_labels_short.strm");
//    vector<vector<VisionFieldObject*> > labels;
//    VisionControlWrapper* vision = VisionControlWrapper::getInstance();
//    vision->readLabels(in, labels);
//    cout << labels.size() << endl;
//    BOOST_FOREACH(vector<VisionFieldObject*> v, labels) {
//        out << v.size() << endl;
//        BOOST_FOREACH(VisionFieldObject* vfo, v) {
//            vfo->printLabelBrief(out);
//            out << endl;
//        }
//    }

//    QApplication app(NULL);
//    MainWindow mw;
//    mw.show();
//    return app.exec();
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
                for(unsigned int i=0; i<data.size(); i++) {
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
            vision->writeBatchDetections(out);
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
