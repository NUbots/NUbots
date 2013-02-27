#include <boost/foreach.hpp>
#include "datawrapperrpi.h"
#include "Infrastructure/NUImage/ColorModelConversions.h"
#include "debug.h"
#include "nubotdataconfig.h"

#include "Vision/VisionTypes/coloursegment.h"
#include "Vision/visionconstants.h"

//for controlling GPIO pins
#include <wiringPi.h>

DataWrapper* DataWrapper::instance = 0;

string DataWrapper::getIDName(DEBUG_ID id) {
    switch(id) {
    case DBID_IMAGE:
        return "DBID_IMAGE";
    case DBID_H_SCANS:
        return "DBID_H_SCANS";
    case DBID_V_SCANS:
        return "DBID_V_SCANS";
    case DBID_SEGMENTS:
        return "DBID_SEGMENTS";
    case DBID_MATCHED_SEGMENTS:
        return "DBID_MATCHED_SEGMENTS";
    case DBID_HORIZON:
        return "DBID_HORIZON";
    case DBID_GREENHORIZON_SCANS:
        return "DBID_GREENHORIZON_SCANS";
    case DBID_GREENHORIZON_FINAL:
        return "DBID_GREENHORIZON_FINAL";
    case DBID_OBSTACLE_POINTS:
        return "DBID_OBSTACLE_POINTS";
    case DBID_FILTERED_SEGMENTS:
        return "DBID_FILTERED_SEGMENTS";
    case DBID_GOALS:
        return "DBID_GOALS";
    case DBID_BEACONS:
        return "DBID_BEACONS";
    case DBID_BALLS:
        return "DBID_BALLS";
    case DBID_OBSTACLES:
        return "DBID_OBSTACLES";
    default:
        return "NOT VALID";
    }
}

void getPointsAndColoursFromSegments(const vector< vector<ColourSegment> >& segments, vector<cv::Scalar>& colours, vector<PointType>& pts)
{
    unsigned char r, g, b;

    BOOST_FOREACH(const vector<ColourSegment>& line, segments) {
        BOOST_FOREACH(const ColourSegment& seg, line) {
            getColourAsRGB(seg.getColour(), r, g, b);
            pts.push_back(seg.getStart());
            pts.push_back(seg.getEnd());
            colours.push_back(cv::Scalar(b,g,r));
        }
    }
}

DataWrapper::DataWrapper(bool disp_on, bool cam)
{
    m_display_on = disp_on;
    if(cam)
        m_input_method = CAMERA;  //CAMERA, STREAM
    else
        m_input_method = STREAM;
    //frame grab methods
    switch(m_input_method) {
    case STREAM:
        streamname = string(getenv("HOME")) + string("/nubot/image.strm");
        LUTname = string(getenv("HOME")) + string("/nubot/default.lut");
        configname = string(getenv("HOME")) + string("/nubot/") + string("VisionOptions.cfg");
        imagestrm.open(streamname.c_str());
        m_current_image = new NUImage();
        if(imagestrm.is_open()) {
            imagestrm >> *m_current_image;
        }
        else {
            errorlog << "DataWrapper::DataWrapper() - failed to load stream: " << streamname << endl;
        }
        #ifdef DEBUG_VISION_VERBOSITY_ON
            debug << streamname << endl;
            debug << LUTname << endl;
            debug << configname << endl;
        #endif
        break;
    case CAMERA:
        m_camera = new PCCamera();
        m_current_image = m_camera->grabNewImage();
        LUTname = string(getenv("HOME")) +  string("/nubot/default.lut");
        break;
    }

    m_writing = true;
    if(m_writing)
        out_stream.open("/home/pi/image.strm");

    LUT.loadLUTFromFile(LUTname);

    //set up fake horizon
    kinematics_horizon.setLine(0, 1, 50);

    VisionConstants::loadFromFile(configname);

    if(!loadLUTFromFile(LUTname)){
        errorlog << "DataWrapper::DataWrapper() - failed to load LUT: " << LUTname << endl;
    }
    
    if(m_display_on) {
        results_window_name = "Results";
        namedWindow(results_window_name, CV_WINDOW_KEEPRATIO);
        results_img.create(m_current_image->getHeight(), m_current_image->getWidth(), CV_8UC3);
    }
    
    numFramesDropped = numFramesProcessed = 0;

    m_gpio = (wiringPiSetupSys() != -1);
    if(m_gpio) {
        system("gpio -g mode 17 out");
        system("gpio -g mode 18 out");
        system("gpio -g mode 22 out");
        system("gpio -g mode 23 out");
        system("gpio export 17 out");
        system("gpio export 18 out");
        system("gpio export 22 out");
        system("gpio export 23 out");
//        pinMode(17,OUTPUT);
//        pinMode(18,OUTPUT);
//        pinMode(22,OUTPUT);
//        pinMode(23,OUTPUT);
    }
}

DataWrapper::~DataWrapper()
{
    if(m_writing)
        out_stream.close();
    delete m_camera;
}

DataWrapper* DataWrapper::getInstance(bool disp_on, bool cam)
{
    if(!instance)
        instance = new DataWrapper(disp_on, cam);
    else {
        instance->m_display_on = disp_on;
    }

    return instance;
}

/**
*   @brief Fetches the next frame from the webcam.
*/
NUImage* DataWrapper::getFrame()
{
    return m_current_image;
}

//! @brief Generates spoofed camera transform vector.
bool DataWrapper::getCTGVector(vector<float> &ctgvector)
{
    //bool isOK = getSensorsData()->get(NUSensorsData::CameraToGroundTransform, ctgvector);
    //return isOK;
    ctgvector.clear();
    ctgvector.push_back(0);
    ctgvector.push_back(0);
    ctgvector.push_back(0);
    ctgvector.push_back(0);
    return false;
}

//! @brief Generates spoofed camera transform vector.
bool DataWrapper::getCTVector(vector<float> &ctvector)
{
    //bool isOK = getSensorsData()->get(NUSensorsData::CameraToGroundTransform, ctgvector);
    //return isOK;
    ctvector.clear();
    ctvector.push_back(0);
    ctvector.push_back(0);
    ctvector.push_back(0);
    ctvector.push_back(0);
    return false;
}


//! @brief Generates spoofed camera height.
bool DataWrapper::getCameraHeight(float& height)
{
    height = 0;
    return false;
}


//! @brief Generates spoofed camera pitch.
bool DataWrapper::getCameraPitch(float& pitch)
{
    pitch = 0;
    return false;
}

//! @brief Generates spoofed body pitch.
bool DataWrapper::getBodyPitch(float& pitch)
{
    pitch = 0;
    return false;
}

//! @brief Returns spoofed kinecv::Matics horizon.
const Horizon& DataWrapper::getKinematicsHorizon()
{ 
    return kinematics_horizon;
}

//! @brief Returns camera settings.
CameraSettings DataWrapper::getCameraSettings()
{
    return m_camera->getSettings();
}

const LookUpTable& DataWrapper::getLUT() const
{
    return LUT;
}

void DataWrapper::publish(const vector<const VisionFieldObject*> &visual_objects)
{
    BOOST_FOREACH(const VisionFieldObject* vfo, visual_objects) {
        publish(vfo);
    }
}

void DataWrapper::publish(const VisionFieldObject* visual_object)
{
    visual_object->printLabel(debug);
    debug << endl;
}

//! Outputs debug data to the appropriate external interface
void DataWrapper::debugRefresh()
{
    if(m_display_on) {
        debugPublish(DBID_CLASSED_IMAGE);
    }
}

//! Outputs supply data to the appropriate external interface
bool DataWrapper::debugPublish(DEBUG_ID id)
{
    //warning if called after another publish method for the same window it will overwrite the window
    if(m_display_on) {
        switch(id) {
        case DBID_CLASSED_IMAGE:
            LUT.classifyImage(*m_current_image, results_img);
            imshow(results_window_name, results_img);
            break;
        default:
            return false;
        }
        return true;
    }
    return false;
}

bool DataWrapper::debugPublish(DEBUG_ID id, NUImage const* const img)
{
    if(m_display_on && false) {
        if(id != DBID_IMAGE) {
            errorlog << "DataWrapper::debugPublish - Called with invalid id" << endl;
            return false;
        }

        cv::Mat& img_map = results_img;    //get image from pair
        string& window = results_window_name; //get window name from pair

        unsigned char* ptr,
                r, g, b;

        for(int y=0; y<img->getHeight(); y++) {
            ptr = img_map.ptr<unsigned char>(y);
            for(int x=0; x<img->getWidth(); x++) {
                ColorModelConversions::fromYCbCrToRGB((*img)(x,y).y, (*img)(x,y).cb, (*img)(x,y).cr, r, g, b);
                ptr[3*x]   = b;
                ptr[3*x+1] = g;
                ptr[3*x+2] = r;
            }
        }
        imshow(window, img_map);
        return true;
    }
    return false;
}

bool DataWrapper::debugPublish(const vector<Ball>& data) {
    #if VISION_WRAPPER_VERBOSITY > 1
        if(data.empty()) {
            debug << "DataWrapper::debugPublish - empty vector DEBUG_ID = " << getIDName(DBID_BALLS) << endl;
            return false;
        }
    #endif

    if(m_gpio && !data.empty())
        digitalWrite(GPIO_BALL, 1);
    else
        digitalWrite(GPIO_BALL, 0);
    
    if(m_display_on && false) {
        cv::Mat& img = results_img;    //get image from pair
        string& window = results_window_name; //get window name from pair

        BOOST_FOREACH(Ball b, data) {
            b.render(img);
        }

        imshow(window, img);    //refresh this particular debug window
        return true;
    }
    return false;
}

bool DataWrapper::debugPublish(const vector<Beacon>& data) {
    
#if VISION_WRAPPER_VERBOSITY > 1
    if(data.empty()) {
        debug << "DataWrapper::debugPublish - empty vector DEBUG_ID = " << getIDName(DBID_GOALS) << endl;
        return false;
    }
#endif

//    if(m_gpio && !data.empty())
//        digitalWrite(GPIO_BEACON, 1);
//    else
//        digitalWrite(GPIO_BEACON, 0);

    if(m_display_on && false) {
        cv::Mat& img = results_img;    //get image from pair
        string& window = results_window_name; //get window name from pair

        BOOST_FOREACH(Beacon b, data) {
            b.render(img);
        }

        imshow(window, img);    //refresh this particular debug window
        return true;
    }
    return false;
}

bool DataWrapper::debugPublish(const vector<Goal>& data) {

#if VISION_WRAPPER_VERBOSITY > 1
    if(data.empty()) {
        debug << "DataWrapper::debugPublish - empty vector DEBUG_ID = " << getIDName(DBID_GOALS) << endl;
        return false;
    }
#endif

    digitalWrite(GPIO_BGOAL, 0);
    digitalWrite(GPIO_YGOAL, 0);
    if(m_gpio && !data.empty()) {
        BOOST_FOREACH(Goal post, data) {
            if(isBlueGoal(post.getID()))
                digitalWrite(GPIO_BGOAL, 1);
            else
                digitalWrite(GPIO_YGOAL, 1);
        }
    }

    if(m_display_on && false) {
        cv::Mat& img = results_img;    //get image from pair
        string& window = results_window_name; //get window name from pair

        BOOST_FOREACH(Goal post, data) {
            post.render(img);
        }

        imshow(window, img);    //refresh this particular debug window
        return true;
    }
    return false;
}

bool DataWrapper::debugPublish(const vector<Obstacle>& data)
{
#if VISION_WRAPPER_VERBOSITY > 1
    if(data.empty()) {
        debug << "DataWrapper::debugPublish - empty vector DEBUG_ID = " << getIDName(DBID_OBSTACLES) << endl;
        return false;
    }
#endif

//    if(m_gpio && !data.empty())
//        digitalWrite(GPIO_OBSTACLE, 1);

    if(m_display_on && false) {
        cv::Mat& img = results_img;    //get image from pair
        string& window = results_window_name; //get window name from pair

        BOOST_FOREACH(Obstacle o, data) {
            o.render(img);
        }

        imshow(window, img);    //refresh this particular debug window
        return true;
    }
    return false;
}

bool DataWrapper::debugPublish(const vector<FieldLine> &data)
{
#if VISION_WRAPPER_VERBOSITY > 1
    if(data.empty()) {
        debug << "DataWrapper::debugPublish - empty vector DEBUG_ID = " << getIDName(DBID_LINES) << endl;
        return false;
    }
#endif

    if(m_gpio && !data.empty())
        digitalWrite(GPIO_LINE, 1);
    else
        digitalWrite(GPIO_LINE, 0);

    if(m_display_on && false) {
        cv::Mat& img = results_img;    //get image from pair
        string& window = results_window_name; //get window name from pair

        BOOST_FOREACH(FieldLine l, data) {
            l.render(img);
        }

        imshow(window, img);    //refresh this particular debug window
        return true;
    }
    return false;
}

bool DataWrapper::debugPublish(DEBUG_ID id, const vector<Vector2<double> >& data_points)
{
    vector<PointType>::const_iterator it;

#if VISION_WRAPPER_VERBOSITY > 1
    if(data_points.empty()) {
        debug << "DataWrapper::debugPublish - empty vector DEBUG_ID = " << getIDName(id) << endl;
        return false;
    }
#endif

    if(m_display_on && false) {
        cv::Mat& img = results_img;    //get image from pair
        string& window = results_window_name; //get window name from pair


        #if VISION_WRAPPER_VERBOSITY > 2
            debug << id << endl;
            debug << colour[0] << "," << colour[1] << "," << colour[2] << "," << colour[3] << "\t";
            debug << data_points << endl;
        #endif

        switch(id) {
        case DBID_H_SCANS:
            BOOST_FOREACH(const PointType& pt, data_points) {
                cv::line(img, cv::Point2i(0, pt.y), cv::Point2i(img.cols, pt.y), cv::Scalar(127,127,127), 1);
            }
            break;
        case DBID_V_SCANS:
            BOOST_FOREACH(const PointType& pt, data_points) {
                cv::line(img, cv::Point2i(pt.x, pt.y), cv::Point2i(pt.x, img.rows), cv::Scalar(127,127,127), 1);
            }
            break;
        case DBID_MATCHED_SEGMENTS:
            BOOST_FOREACH(const PointType& pt, data_points) {
                cv::circle(img, cv::Point2i(pt.x, pt.y), 1, cv::Scalar(255,255,0), 4);
            }
            break;
        case DBID_HORIZON:
            line(img, cv::Point2i(data_points.front().x, data_points.front().y), cv::Point2i(data_points.back().x, data_points.back().y), cv::Scalar(255,255,0), 1);
            break;
        case DBID_GREENHORIZON_SCANS:
            BOOST_FOREACH(const PointType& pt, data_points) {
                line(img, cv::Point2i(pt.x, kinematics_horizon.findYFromX(pt.x)), cv::Point2i(pt.x, img.rows), cv::Scalar(127,127,127), 1);
                //cv::circle(img, cv::Point2i(pt.x, pt.y), 1, cv::Scalar(127,127,127), 2);
                cv::circle(img, cv::Point2i(pt.x, pt.y), 1, cv::Scalar(255,0,255), 2);
            }
            break;
        case DBID_GREENHORIZON_FINAL:
            for(it=data_points.begin(); it<data_points.end(); it++) {
                if (it > data_points.begin()) {
                    line(img, cv::Point2i((it-1)->x, (it-1)->y), cv::Point2i(it->x, it->y), cv::Scalar(255,0,255), 2);
                }
                //cv::circle(img, cv::Point2i(it->x, it->y), 1, cv::Scalar(255,0,255), 2);
            }
            break;
        case DBID_OBSTACLE_POINTS:
            BOOST_FOREACH(const PointType& pt, data_points) {
                cv::circle(img, cv::Point2i(pt.x, pt.y), 1, cv::Scalar(0,0,255), 4);
            }
            break;
        default:
            errorlog << "DataWrapper::debugPublish - Called with invalid id" << endl;
            return false;
        }

        imshow(window, img);    //refresh this particular debug window

        return true;
    }
    return false;
}

//! Outputs debug data to the appropriate external interface
bool DataWrapper::debugPublish(DEBUG_ID id, const SegmentedRegion& region)
{
    if(m_display_on && false) {
        vector<PointType> data_points;
        vector<cv::Scalar> colours;
        vector<PointType>::const_iterator it;
        vector<cv::Scalar>::const_iterator c_it;

        getPointsAndColoursFromSegments(region.getSegments(), colours, data_points);

        if(data_points.empty() || colours.empty()) {
            errorlog << "DataWrapper::debugPublish - empty vector DEBUG_ID = " << getIDName(id) << endl;
            return false;
        }

        cv::Mat& img = results_img;    //get image from pair
        string& window = results_window_name; //get window name from pair


        #if VISION_WRAPPER_VERBOSITY > 2
            debug << id << endl;
            debug << colours.front()[0] << "," << colours.front()[1] << "," << colours.front()[2] << "," << colours.front()[3] << "\t";
            debug << data_points << endl;
        #endif

        switch(id) {
        case DBID_SEGMENTS:
            c_it = colours.begin();
            for (it = data_points.begin(); it < data_points.end()-1; it+=2) {
                //draws a line between each consecutive pair of points of the corresponding colour
                line(img, cv::Point2i(it->x, it->y), cv::Point2i((it+1)->x, (it+1)->y), *c_it, 1);
                c_it++;
            }
            break;
        case DBID_FILTERED_SEGMENTS:
            c_it = colours.begin();
            for (it = data_points.begin(); it < data_points.end()-1; it+=2) {
                //draws a line between each consecutive pair of points of the corresponding colour
                line(img, cv::Point2i(it->x, it->y), cv::Point2i((it+1)->x, (it+1)->y), *c_it, 1);
                c_it++;
            }
            break;
        default:
            errorlog << "DataWrapper::debugPublish - Called with invalid id" << endl;
            return false;
        }

        imshow(window, img);    //refresh this particular debug window

        return true;
    }
    return false;
}

bool DataWrapper::updateFrame()
{
    switch(m_input_method) {
    case CAMERA:
        m_current_image = m_camera->grabNewImage();   //force get new frame
        if(m_writing)
            out_stream << m_current_image << std::flush;
        break;
    case STREAM:
        VisionConstants::loadFromFile(configname);
        if(!imagestrm.is_open()) {
            errorlog << "No image stream" << endl;
            return false;
        }
        try {
            imagestrm >> *m_current_image;
            //doGaussian(m_current_image);
        }
        catch(std::exception& e){
            cout << "Stream error - resetting: " << e.what() << endl;
            imagestrm.clear() ;
            imagestrm.seekg(0, ios::beg);
            imagestrm >> *m_current_image;
        }
        break;
    }

    numFramesProcessed++;
    return true;
}

/**
*   @brief loads the colour look up table
*   @param filename The filename for the LUT stored on disk
*   @note Taken from original vision system
*/
bool DataWrapper::loadLUTFromFile(const string& fileName)
{
    return LUT.loadLUTFromFile(fileName);
}
