#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include <boost/foreach.hpp>

#include "../Vision/VisionWrapper/visioncontrolwrappertraining.h"
#include "../Infrastructure/NUSensorsData/NUSensorsData.h"
#include "../Infrastructure/NUImage/NUImage.h"
#include "../Vision/visionconstants.h"

#include "mainwindow.h"
#include <QApplication>

void splitSamples();
void convertLabels();
int runDave();
int runDefault();
void printBest();
pair<float, vector<Parameter> > getBest(istream &params_stream, istream &performance_stream);
vector<double> checkImageStream(string name);
vector<double> checkSensorStream(string name);

int main()
{
    QApplication app(NULL);
    MainWindow mw;
    mw.show();
    return app.exec();
    //printBest();
}

void printBest()
{
    ifstream para("/home/shannon/Images/VisionPGA_progress.log");
    ifstream perf("/home/shannon/Images/VisionPGA_training_performance.log");
    pair<float, vector<Parameter> > best = getBest(para, perf);
    cout << best.first << endl;
    cout << best.second << endl;

    VisionConstants::DO_RADIAL_CORRECTION = false;
    //! Goal filtering constants
    VisionConstants::THROWOUT_ON_ABOVE_KIN_HOR_GOALS = false;
    VisionConstants::THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_GOALS = false;
    VisionConstants::THROWOUT_DISTANT_GOALS = false;
    VisionConstants::THROWOUT_INSIGNIFICANT_GOALS = true;
    VisionConstants::THROWOUT_NARROW_GOALS = true;
    VisionConstants::THROWOUT_SHORT_GOALS = true;
    //! Beacon filtering constants
    VisionConstants::THROWOUT_ON_ABOVE_KIN_HOR_BEACONS = false;
    VisionConstants::THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BEACONS = false;
    VisionConstants::THROWOUT_DISTANT_BEACONS = false;
    VisionConstants::THROWOUT_INSIGNIFICANT_BEACONS = true;
    //! Ball filtering constants
    VisionConstants::THROWOUT_ON_ABOVE_KIN_HOR_BALL = false;
    VisionConstants::THROWOUT_ON_DISTANCE_METHOD_DISCREPENCY_BALL = false;
    VisionConstants::THROWOUT_SMALL_BALLS = true;
    VisionConstants::THROWOUT_INSIGNIFICANT_BALLS = true;
    VisionConstants::THROWOUT_DISTANT_BALLS = false;
    //! ScanLine options
    VisionConstants::HORIZONTAL_SCANLINE_SPACING = 3;
    VisionConstants::VERTICAL_SCANLINE_SPACING = 3;
    VisionConstants::GREEN_HORIZON_SCAN_SPACING = 11;
    //! Split and Merge constants
    VisionConstants::SAM_MAX_POINTS = 1000;
    VisionConstants::SAM_MAX_LINES = 150;
    VisionConstants::SAM_CLEAR_SMALL = true;
    VisionConstants::SAM_CLEAR_DIRTY = true;

    VisionConstants::setAllOptimisable(Parameter::getAsVector(best.second));
    VisionConstants::print(cout);
}

pair<float, vector<Parameter> > getBest(istream& params_stream, istream& performance_stream)
{
    vector<Parameter> cur, best;
    float f_cur, f_best = 0;
    int garbage;
    performance_stream.ignore(200, '\n');
    while(params_stream.good() && performance_stream.good()) {
        params_stream >> cur;
        performance_stream >> garbage >> f_cur;
        if(f_cur > f_best) {
            f_best = f_cur;
            best = cur;
        }
    }
    return pair<float, vector<Parameter> >(f_best, best);
}

void splitSamples()
{
    ifstream in_labels("/home/shannon/Images/FYP/Final100/opt_labels.strm");
    ifstream in_image("/home/shannon/Images/FYP/Final100/image.strm");
    ofstream test_labels("/home/shannon/Images/FYP/Final100/test_labels.strm");
    ofstream test_image("/home/shannon/Images/FYP/Final100/test_image.strm");
    ofstream train_labels("/home/shannon/Images/FYP/Final100/train_labels.strm");
    ofstream train_image("/home/shannon/Images/FYP/Final100/train_image.strm");
    vector<vector<VisionFieldObject* > > labels;
    VisionControlWrapper::getInstance()->readLabels(in_labels, labels);

    for(int i=0; i<labels.size() && in_image.good(); i++) {
        NUImage img;
        in_image >> img;
        if(i%2==0) {
            test_image << img;
            test_labels << labels[i].size() << endl;
            for(int k=0; k<labels[i].size(); k++) {
                labels[i][k]->printLabel(test_labels);
                test_labels << endl;
            }
        }
        else {
            train_image << img;
            train_labels << labels[i].size() << endl;
            for(int k=0; k<labels[i].size(); k++) {
                labels[i][k]->printLabel(train_labels);
                train_labels << endl;
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
