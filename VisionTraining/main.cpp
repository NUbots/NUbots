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

void splitSamples();
void convertLabels();
int runDave();
int runDefault();
vector<double> checkImageStream(string name);
vector<double> checkSensorStream(string name);

int main()
{
    QApplication app(NULL);
    MainWindow mw;
    mw.show();
    return app.exec();
}

void splitSamples()
{
    ifstream in_labels("/home/shannon/Images/FYP/Final100/opt_labels.strm");
    ifstream in_image("/home/shannon/Images/FYP/Final100/image.strm");
    ofstream test_labels("/home/shannon/Images/FYP/Final100/test_labels.strm");
    ofstream test_image("/home/shannon/Images/FYP/Final100/test_image.strm");
    ofstream train_labels("/home/shannon/Images/FYP/Final100/train_labels.strm");
    ofstream train_image("/home/shannon/Images/FYP/Final100/train_image.strm");
    vector<vector<pair<VisionFieldObject::VFO_ID,Vector2<double> > > > labels;
    VisionControlWrapper::getInstance()->readLabels(in_labels, labels);

    for(int i=0; i<labels.size() && in_image.good(); i++) {
        NUImage img;
        in_image >> img;
        if(i%2==0) {
            test_image << img;
            test_labels << labels[i].size() << endl;
            for(int k=0; k<labels[i].size(); k++) {
                test_labels << VisionFieldObject::getVFOName(labels[i][k].first) << " " << labels[i][k].second << endl;
            }
        }
        else {
            train_image << img;
            train_labels << labels[i].size() << endl;
            for(int k=0; k<labels[i].size(); k++) {
                train_labels << VisionFieldObject::getVFOName(labels[i][k].first) << " " << labels[i][k].second << endl;
            }
        }
    }
    in_labels.close();
    in_image.close();
    test_labels.close();
    test_image.close();
    train_labels.close();
    train_image.close();
}

void convertLabels()
{
    ifstream in("/home/shannon/Images/FYP/Final100/opt_labels.strm");
    ofstream out("/home/shannon/Images/FYP/Final100/opt_labels_short.strm");
    vector<vector<VisionFieldObject*> > labels;
    VisionControlWrapper* vision = VisionControlWrapper::getInstance();
    vision->readLabels(in, labels);
    cout << labels.size() << endl;
    BOOST_FOREACH(vector<VisionFieldObject*> v, labels) {
        out << v.size() << endl;
        BOOST_FOREACH(VisionFieldObject* vfo, v) {
            vfo->printLabelBrief(out);
            out << endl;
        }
    }
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
