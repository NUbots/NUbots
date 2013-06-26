#include <boost/foreach.hpp>
#include "datawrapperpc.h"
#include "Infrastructure/NUImage/ColorModelConversions.h"
#include "debug.h"
#include "debugverbosityvision.h"
#include "nubotdataconfig.h"

#include "Vision/VisionTypes/coloursegment.h"
#include "Vision/visionconstants.h"

#include <QFileDialog>
#include <QInputDialog>
#include <QMessageBox>

DataWrapper* DataWrapper::instance = 0;

std::string DataWrapper::getIDName(DEBUG_ID id) {
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
    case DBID_OBJECT_POINTS:
        return "DBID_OBJECT_POINTS";
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
    case DBID_GOAL_LINES_START:
        return "DBID_GOAL_LINES_START";
    case DBID_GOAL_LINES_END:
        return "DBID_GOAL_LINES_END";
    default:
        return "NOT VALID";
    }
}

void getPointsAndColoursFromSegments(const std::vector< std::vector<ColourSegment> >& segments, std::vector<cv::Scalar>& colours, std::vector<Point >& pts)
{
    unsigned char r, g, b;
    
    BOOST_FOREACH(const std::vector<ColourSegment>& line, segments) {
        BOOST_FOREACH(const ColourSegment& seg, line) {
            getColourAsRGB(seg.getColour(), r, g, b);
            pts.push_back(seg.getStart());
            pts.push_back(seg.getEnd());
            colours.push_back(cv::Scalar(b,g,r));
        }
    }
}

void doGaussian(NUImage* img) {
    unsigned char* ptr;
            //r, g, b,QFileDialog::getOpenFileName(NULL, "Select image stream", QString(getenv("HOME")) + QString("/nubot/"),  "Stream Files (*.strm)").toStdString();
            //Y, cb, cr;
    std::vector<const VisionFieldObject*>::const_iterator it;
    cv::Mat mat(img->getHeight(), img->getWidth(), CV_8UC3);

    for(int y=0; y<img->getHeight(); y++) {
        ptr = mat.ptr<unsigned char>(y);
        for(int x=0; x<img->getWidth(); x++) {
            ptr[3*x]   = (*img)(x,y).y;
            ptr[3*x+1] = (*img)(x,y).cb;
            ptr[3*x+2] = (*img)(x,y).cr;
        }
    }

    cv::GaussianBlur(mat, mat, cv::Size (5, 5), 0);

    for(int y=0; y<img->getHeight(); y++) {
        ptr = mat.ptr<unsigned char>(y);
        for(int x=0; x<img->getWidth(); x++) {
            Pixel px;
            px.yCbCrPadding = ptr[3*x];
            px.cb = ptr[3*x+1];
            px.y = ptr[3*x];
            px.cr = ptr[3*x+2];
            img->setPixel(x, y, px);
        }
    }
}

DataWrapper::DataWrapper()
{
    //frame grab methods
    QString camoption("Camera"),
            strmoption("Image stream");
    QStringList l;
    l.append(camoption);
    l.append(strmoption);
    //get the input choice from the user
    QString s = QInputDialog::getItem(NULL, "Select Input Method", "Select input method", l, 0, false);
    if(s.compare(camoption) == 0)
        m_method = CAMERA;
    else if(s.compare(strmoption) == 0)
        m_method = STREAM;
    else
        m_method = STREAM;

    switch(m_method) {
    case CAMERA:
        m_camera = new PCCamera();
        m_current_image = *(m_camera->grabNewImage());
        LUTname = std::string(getenv("HOME")) +  std::string("/nubot/default.lut");
        break;
    case STREAM:
        using_sensors = (QMessageBox::question(NULL, "", "Use sensor log?", QMessageBox::Yes | QMessageBox::No) == QMessageBox::Yes);

        if(QMessageBox::question(NULL, "", "Manually select files?", QMessageBox::Yes | QMessageBox::No) == QMessageBox::Yes) {
            streamname = QFileDialog::getOpenFileName(NULL, "Select image stream", QString(getenv("HOME")) + QString("/nubot/"),  "Stream Files (*.strm)").toStdString();
            if(using_sensors)
                sensorstreamname = QFileDialog::getOpenFileName(NULL, "Select sensor stream", QString(getenv("HOME")) + QString("/nubot/"),  "Stream Files (*.strm)").toStdString();
            LUTname = QFileDialog::getOpenFileName(NULL, "Select Lookup Table", QString(getenv("HOME")) + QString("/nubot/"),  "LUT Files (*.lut)").toStdString();
            configname = QFileDialog::getOpenFileName(NULL, "Select Configuration File", QString(getenv("HOME")) + QString("/nubot/Config/Darwin/"), "config Files (*.cfg)").toStdString();
        }
        else {
            streamname = std::string(getenv("HOME")) + std::string("/nubot/image.strm");
            if(using_sensors)
                sensorstreamname = std::string(getenv("HOME")) + std::string("/nubot/sensor.strm");
            LUTname = std::string(getenv("HOME")) + std::string("/nubot/default.lut");
            configname = std::string(getenv("HOME")) + std::string("/nubot/") + std::string("VisionOptions.cfg");
        }
        imagestrm.open(streamname.c_str());
        if(using_sensors)
            sensorstrm.open(sensorstreamname.c_str());

        if(!sensorstrm.is_open()) {
            QMessageBox::warning(NULL, "Error", QString("Failed to read sensors from: ") + QString(sensorstreamname.c_str()) + QString(" defaulting to sensors off."));
            using_sensors = false;
        }

        if(!imagestrm.is_open()) {
            errorlog << "DataWrapper::DataWrapper() - failed to load stream: " << streamname << std::endl;
        }
        break;
    }

    //set up fake horizon
    kinematics_horizon.setLine(0, 1, 0);

    updateFrame();

    if(!loadLUTFromFile(LUTname)){
        errorlog << "DataWrapper::DataWrapper() - failed to load LUT: " << LUTname << std::endl;
    }

    //create debug windows
    debug_window_num = 8;
    debug_windows = new pair<std::string, cv::Mat>[debug_window_num];
    debug_windows[0].first = "Scans";
    debug_windows[1].first = "Classified";
    debug_windows[2].first = "Green Horizon and Obstacle Detection";
    debug_windows[3].first = "Scanlines";
    debug_windows[4].first = "Transitions";
    debug_windows[5].first = "Final Results";
    debug_windows[6].first = "Filtered Scanlines";
    debug_windows[7].first = "Goal Processing";
    for(int i=0; i<debug_window_num; i++) {
        namedWindow(debug_windows[i].first, CV_WINDOW_KEEPRATIO);
        debug_windows[i].second.create(m_current_image.getHeight(), m_current_image.getWidth(), CV_8UC3);
    }

    //create map into images
    debug_map[DBID_IMAGE].push_back(                &debug_windows[0]);
    debug_map[DBID_IMAGE].push_back(                &debug_windows[2]);
    //debug_map[DBID_IMAGE].push_back(                &debug_windows[3]);
    debug_map[DBID_IMAGE].push_back(                &debug_windows[4]);
    debug_map[DBID_IMAGE].push_back(                &debug_windows[5]);
    //debug_map[DBID_IMAGE].push_back(                &debug_windows[7]);

    debug_map[DBID_CLASSED_IMAGE].push_back(        &debug_windows[1]);

    debug_map[DBID_H_SCANS].push_back(              &debug_windows[0]);
    debug_map[DBID_V_SCANS].push_back(              &debug_windows[0]);

    debug_map[DBID_SEGMENTS].push_back(             &debug_windows[3]);
    debug_map[DBID_MATCHED_SEGMENTS].push_back(     &debug_windows[4]);

    debug_map[DBID_HORIZON].push_back(              &debug_windows[0]);
    debug_map[DBID_GREENHORIZON_SCANS].push_back(   &debug_windows[0]);
    debug_map[DBID_GREENHORIZON_FINAL].push_back(   &debug_windows[2]);
    debug_map[DBID_GREENHORIZON_FINAL].push_back(   &debug_windows[2]);

    debug_map[DBID_OBJECT_POINTS].push_back(        &debug_windows[2]);

    debug_map[DBID_FILTERED_SEGMENTS] .push_back(   &debug_windows[6]);

    debug_map[DBID_GOALS].push_back(                &debug_windows[5]);
    debug_map[DBID_BEACONS].push_back(              &debug_windows[5]);
    debug_map[DBID_BALLS].push_back(                &debug_windows[5]);
    debug_map[DBID_OBSTACLES].push_back(            &debug_windows[5]);
    debug_map[DBID_LINES].push_back(                &debug_windows[5]);


    debug_map[DBID_GOAL_LINES_START].push_back(     &debug_windows[7]);
    debug_map[DBID_GOAL_LINES_END].push_back(       &debug_windows[7]);

    results_img.create(m_current_image.getHeight(), m_current_image.getWidth(), CV_8UC3);
    
    numFramesDropped = numFramesProcessed = 0;

}

DataWrapper::~DataWrapper()
{
    delete[] debug_windows;
}

//void DataWrapper::generateImageFromMat(cv::Mat& frame)
//{
//    int size = frame.rows*frame.cols;
//    if (frame.rows != m_current_image.getHeight() or frame.cols != m_current_image.getWidth())
//    {
//        if(m_yuyv_buffer)
//            delete m_yuyv_buffer;
//        m_yuyv_buffer = new unsigned char[size*2];
//        m_current_image.MapYUV422BufferToImage(m_yuyv_buffer, frame.cols, frame.rows);
//    }

//    int i_YUV = 0;			// the index into the yuyv_buffer
//    unsigned char y1,u1,v1,y2,u2,v2;
//    for (int i=0; i<frame.rows; i++)
//    {
//        for (int j=frame.cols/2; j>0; j--)		// count down to flip the image around
//        {
//            ColorModelConversions::fromRGBToYCbCr(frame.at<Vec3b>(i,j<<1)[2], frame.at<Vec3b>(i,j<<1)[1], frame.at<Vec3b>(i,j<<1)[0], y1, u1, v1);
//            ColorModelConversions::fromRGBToYCbCr(frame.at<Vec3b>(i,(j<<1)-1)[2], frame.at<Vec3b>(i,(j<<1)-1)[1], frame.at<Vec3b>(i,(j<<1)-1)[0], y2, u2, v2);
//            m_yuyv_buffer[i_YUV++] = y1;
//            m_yuyv_buffer[i_YUV++] = (u1+u2)>>1;
//            m_yuyv_buffer[i_YUV++] = y2;
//            m_yuyv_buffer[i_YUV++] = (v1+v2)>>1;
//        }
//    }
//}

DataWrapper* DataWrapper::getInstance()
{
    if(!instance)
        instance = new DataWrapper();
    return instance;
}

/**
*   @brief Fetches the next frame from the webcam.
*/
NUImage* DataWrapper::getFrame()
{
    return &m_current_image;
}

//! @brief Generates spoofed camera transform std::vector.
bool DataWrapper::getCTGVector(std::vector<float> &ctgvector)
{
    if(using_sensors) {
        return m_sensor_data.get(NUSensorsData::CameraToGroundTransform, ctgvector);
    }
    else {
        ctgvector.assign(4, 0);
        return false;
    }
}

//! @brief Generates spoofed camera transform std::vector.
bool DataWrapper::getCTVector(std::vector<float> &ctvector)
{
    if(using_sensors) {
        return m_sensor_data.get(NUSensorsData::CameraTransform, ctvector);
    }
    else {
        ctvector.assign(4, 0);
        return false;
    }
}


//! @brief Generates spoofed camera height.
bool DataWrapper::getCameraHeight(float& height)
{
    if(using_sensors)
        return m_sensor_data.getCameraHeight(height);
    else
        return false;
}


//! @brief Generates spoofed camera pitch.
bool DataWrapper::getCameraPitch(float& pitch)
{
    if(using_sensors)
        return m_sensor_data.getPosition(NUSensorsData::HeadPitch, pitch);
    else
        return false;
}

//! @brief Generates spoofed body pitch.
bool DataWrapper::getBodyPitch(float& pitch)
{
    if(using_sensors) {
        std::vector<float> orientation;
        bool valid = m_sensor_data.get(NUSensorsData::Orientation, orientation);
        if(valid && orientation.size() > 2) {
            pitch = orientation.at(1);
            return true;
        }
    }
    return false;
}

//! @brief Returns spoofed kinecv::Matics horizon.
const Horizon& DataWrapper::getKinematicsHorizon()
{ 
    return kinematics_horizon;
}

////! @brief Returns image width.
//unsigned int DataWrapper::getImageWidth()
//{
//    return m_image_width;
//}

////! @brief Returns image height.
//unsigned int DataWrapper::getImageHeight()
//{
//    return m_image_height;
//}

//! @brief Returns camera settings.
CameraSettings DataWrapper::getCameraSettings()
{
    switch(m_method) {
    case CAMERA:
        return m_camera->getSettings();
    case STREAM:
        return CameraSettings();
    }
}

const LookUpTable& DataWrapper::getLUT() const
{
    return LUT;
}

void DataWrapper::publish(const std::vector<const VisionFieldObject*> &visual_objects)
{
    //std::cout << visual_objects.size() << " visual objects seen" << std::endl;
}

void DataWrapper::publish(const VisionFieldObject* visual_object)
{
    //std::cout << "Visual object seen at " << visual_object->getLocationPixels() << std::endl;
    #if VISION_WRAPPER_VERBOSITY > 0
    visual_object->printLabel(debug);
    debug << std::endl;
    #endif
}

//! Outputs debug data to the appropriate external interface
void DataWrapper::debugRefresh()
{
    for(int i=0; i<debug_window_num; i++) {
        debug_windows[i].second = cv::Scalar(0,0,0);
    }

    debugPublish(DBID_CLASSED_IMAGE);
}

//! Outputs supply data to the appropriate external interface
bool DataWrapper::debugPublish(DEBUG_ID id)
{
    //warning if called after another publish method for the same window it will overwrite the window
    map<DEBUG_ID, std::vector< pair<std::string, cv::Mat>* > >::iterator map_it;
    switch(id) {
    case DBID_CLASSED_IMAGE:
        //Get window assigned to id
        map_it = debug_map.find(id);
        if(map_it == debug_map.end()) {
            errorlog << "DataWrapper::debugPublish - Missing window definition DEBUG_ID = " << getIDName(id) << std::endl;
            return false;
        }

        //for all images
        for(unsigned int i=0; i<map_it->second.size(); i++) {
            std::string& window = map_it->second[i]->first; //get window name from pair
            cv::Mat& img = map_it->second[i]->second; //get window name from pair

            LUT.classifyImage(m_current_image, img);

            imshow(window, img);
        }
        break;
    default:
        errorlog << "DataWrapper::debugPublish called with invalid DEBIG_ID: " << getIDName(id) << std::endl;
    }
    return true;
}

bool DataWrapper::debugPublish(const std::vector<Ball>& data) {
    #if VISION_WRAPPER_VERBOSITY > 1
        if(data.empty()) {
            debug << "DataWrapper::debugPublish - empty std::vector DEBUG_ID = " << getIDName(DBID_BALLS) << std::endl;
            return false;
        }
    #endif
    
    //Get window assigned to id
    map<DEBUG_ID, std::vector< pair<std::string, cv::Mat>* > >::iterator map_it = debug_map.find(DBID_BALLS);
    if(map_it == debug_map.end()) {
        errorlog << "DataWrapper::debugPublish - Missing window definition DEBUG_ID = " << getIDName(DBID_BALLS) << std::endl;
        return false;
    }

    //for all images
    for(unsigned int i=0; i<map_it->second.size(); i++) {
        std::string& window = map_it->second[i]->first; //get window name from pair
        cv::Mat& img = map_it->second[i]->second; //get window name from pair

        BOOST_FOREACH(Ball b, data) {
            b.render(img);
        }

        imshow(window, img);
    }
    return true;
}

//bool DataWrapper::debugPublish(const std::vector<Beacon>& data) {
    
//#if VISION_WRAPPER_VERBOSITY > 1
//    if(data.empty()) {
//        debug << "DataWrapper::debugPublish - empty std::vector DEBUG_ID = " << getIDName(DBID_BEACONS) << std::endl;
//        return false;
//    }
//#endif
    
//    //Get window assigned to id
//    map<DEBUG_ID, std::vector< pair<std::string, cv::Mat>* > >::iterator map_it = debug_map.find(DBID_BEACONS);
//    if(map_it == debug_map.end()) {
//        errorlog << "DataWrapper::debugPublish - Missing window definition DEBUG_ID = " << getIDName(DBID_BEACONS) << std::endl;
//        return false;
//    }

//    //for all images
//    for(unsigned int i=0; i<map_it->second.size(); i++) {
//        std::string& window = map_it->second[i]->first; //get window name from pair
//        cv::Mat& img = map_it->second[i]->second; //get window name from pair

//        BOOST_FOREACH(Beacon b, data) {
//            b.render(img);
//        }

//        imshow(window, img);
//    }
//    return true;
//}

bool DataWrapper::debugPublish(const std::vector<Goal>& data) {

#if VISION_WRAPPER_VERBOSITY > 1
    if(data.empty()) {
        debug << "DataWrapper::debugPublish - empty std::vector DEBUG_ID = " << getIDName(DBID_GOALS) << std::endl;
        return false;
    }
#endif
    
    //Get window assigned to id
    map<DEBUG_ID, std::vector< pair<std::string, cv::Mat>* > >::iterator map_it = debug_map.find(DBID_GOALS);
    if(map_it == debug_map.end()) {
        errorlog << "DataWrapper::debugPublish - Missing window definition DEBUG_ID = " << getIDName(DBID_GOALS) << std::endl;
        return false;
    }

    //for all images
    for(unsigned int i=0; i<map_it->second.size(); i++) {
        std::string& window = map_it->second[i]->first; //get window name from pair
        cv::Mat& img = map_it->second[i]->second; //get window name from pair

        BOOST_FOREACH(Goal post, data) {
            post.render(img);
        }

        imshow(window, img);
    }
    return true;
}

bool DataWrapper::debugPublish(const std::vector<Obstacle>& data)
{

#if VISION_WRAPPER_VERBOSITY > 1
    if(data.empty()) {
        debug << "DataWrapper::debugPublish - empty std::vector DEBUG_ID = " << getIDName(DBID_OBSTACLES) << std::endl;
        return false;
    }
#endif
    //Get window assigned to id
    map<DEBUG_ID, std::vector< pair<std::string, cv::Mat>* > >::iterator map_it = debug_map.find(DBID_OBSTACLES);
    if(map_it == debug_map.end()) {
        errorlog << "DataWrapper::debugPublish - Missing window definition DEBUG_ID = " << getIDName(DBID_OBSTACLES) << std::endl;
        return false;
    }

    //for all images
    for(unsigned int i=0; i<map_it->second.size(); i++) {
        std::string& window = map_it->second[i]->first; //get window name from pair
        cv::Mat& img = map_it->second[i]->second; //get window name from pair

        BOOST_FOREACH(Obstacle o, data) {
            o.render(img);
        }

        imshow(window, img);
    }
    return true;
}

bool DataWrapper::debugPublish(const std::vector<FieldLine> &data)
{
#if VISION_WRAPPER_VERBOSITY > 1
    if(data.empty()) {
        debug << "DataWrapper::debugPublish - empty std::vector DEBUG_ID = " << getIDName(DBID_LINES) << std::endl;
        return false;
    }
#endif

    //Get window assigned to id
    map<DEBUG_ID, std::vector< pair<std::string, cv::Mat>* > >::iterator map_it = debug_map.find(DBID_LINES);
    if(map_it == debug_map.end()) {
        errorlog << "DataWrapper::debugPublish - Missing window definition DEBUG_ID = " << getIDName(DBID_LINES) << std::endl;
        return false;
    }

    //for all images
    for(unsigned int i=0; i<map_it->second.size(); i++) {
        std::string& window = map_it->second[i]->first; //get window name from pair
        cv::Mat& img = map_it->second[i]->second; //get window name from pair

        BOOST_FOREACH(FieldLine l, data) {
            l.render(img);
        }

        imshow(window, img);
    }
    return true;
}

bool DataWrapper::debugPublish(DEBUG_ID id, const std::vector<Point >& data_points)
{
    map<DEBUG_ID, std::vector<pair<std::string, cv::Mat>* > >::iterator map_it;
    std::vector<Point >::const_iterator it;

#if VISION_WRAPPER_VERBOSITY > 1
    if(data_points.empty()) {
        debug << "DataWrapper::debugPublish - empty std::vector DEBUG_ID = " << getIDName(id) << std::endl;
        return false;
    }
#endif

    //Get window assigned to id
    map_it = debug_map.find(id);
    if(map_it == debug_map.end()) {
        errorlog << "DataWrapper::debugPublish - Missing window definition DEBUG_ID = " << getIDName(id) << std::endl;
        return false;
    }


    //for all images
    for(unsigned int i=0; i<map_it->second.size(); i++) {
        cv::Mat& img = map_it->second[i]->second;    //get image from pair
        std::string& window = map_it->second[i]->first; //get window name from pair


    #if VISION_WRAPPER_VERBOSITY > 2
        debug << id << std::endl;
        debug << colour[0] << "," << colour[1] << "," << colour[2] << "," << colour[3] << "\t";
        debug << data_points << std::endl;
    #endif

        switch(id) {
        case DBID_H_SCANS:
            BOOST_FOREACH(const Point& pt, data_points) {
                cv::line(img, cv::Point2i(0, pt.y), cv::Point2i(img.cols, pt.y), cv::Scalar(127,127,127), 1);
            }
            break;
        case DBID_V_SCANS:
            BOOST_FOREACH(const Point& pt, data_points) {
                cv::line(img, cv::Point2i(pt.x, pt.y), cv::Point2i(pt.x, img.rows), cv::Scalar(127,127,127), 1);
            }
            break;
        case DBID_MATCHED_SEGMENTS:
            BOOST_FOREACH(const Point& pt, data_points) {
                cv::circle(img, cv::Point2i(pt.x, pt.y), 1, cv::Scalar(255,255,0), 4);
            }
            break;
        case DBID_HORIZON:
            line(img, cv::Point2i(data_points.front().x, data_points.front().y), cv::Point2i(data_points.back().x, data_points.back().y), cv::Scalar(255,255,0), 1);
            break;
        case DBID_GREENHORIZON_SCANS:
            BOOST_FOREACH(const Point& pt, data_points) {
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
        case DBID_OBJECT_POINTS:
            BOOST_FOREACH(const Point& pt, data_points) {
                cv::circle(img, cv::Point2i(pt.x, pt.y), 1, cv::Scalar(0,0,255), 4);
            }
            break;
        default:
            errorlog << "DataWrapper::debugPublish - Called with invalid id" << std::endl;
            return false;
        }

        imshow(window, img);    //refresh this particular debug window
    }

    return true;
}

//! Outputs debug data to the appropriate external interface
bool DataWrapper::debugPublish(DEBUG_ID id, const SegmentedRegion& region)
{
    std::vector<Point > data_points;
    std::vector<cv::Scalar> colours;
    map<DEBUG_ID, std::vector<pair<std::string, cv::Mat>* > >::iterator map_it;
    std::vector<Point >::const_iterator it;
    std::vector<cv::Scalar>::const_iterator c_it;

    getPointsAndColoursFromSegments(region.getSegments(), colours, data_points);
    
    if(data_points.empty() || colours.empty()) {
        errorlog << "DataWrapper::debugPublish - empty std::vector DEBUG_ID = " << getIDName(id) << std::endl;
        return false;
    }

    //Get window assigned to id
    map_it = debug_map.find(id);
    if(map_it == debug_map.end()) {
        errorlog << "DataWrapper::debugPublish - Missing window definition DEBUG_ID = " << getIDName(id) << std::endl;
        return false;
    }

    //for all images
    for(unsigned int i=0; i<map_it->second.size(); i++) {
        cv::Mat& img = map_it->second[i]->second;    //get image from pair
        std::string& window = map_it->second[i]->first; //get window name from pair


    #if VISION_WRAPPER_VERBOSITY > 2
        debug << id << std::endl;
        debug << colours.front()[0] << "," << colours.front()[1] << "," << colours.front()[2] << "," << colours.front()[3] << "\t";
        debug << data_points << std::endl;
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
            errorlog << "DataWrapper::debugPublish - Called with invalid id" << std::endl;
            return false;
        }

        imshow(window, img);    //refresh this particular debug window
    }

    return true;
}

bool DataWrapper::debugPublish(DEBUG_ID id, NUImage const* const img)
{
    map<DEBUG_ID, std::vector<pair<std::string, cv::Mat>* > >::iterator map_it;
    if(id != DBID_IMAGE) {
        errorlog << "DataWrapper::debugPublish - Called with invalid id" << std::endl;
        return false;
    }
    //Get window assigned to id
    map_it = debug_map.find(id);
    if(map_it == debug_map.end()) {
        errorlog << "DataWrapper::debugPublish - Missing window definition DEBUG_ID = " << getIDName(id) << std::endl;
        return false;
    }

    //for all images
    for(unsigned int i=0; i<map_it->second.size(); i++) {
        cv::Mat& img_map = map_it->second[i]->second;    //get image from pair
        std::string& window = map_it->second[i]->first; //get window name from pair

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
    }
    return true;
}

bool DataWrapper::debugPublish(DEBUG_ID id, const std::vector<LSFittedLine>& data)
{
#if VISION_WRAPPER_VERBOSITY > 1
    if(data.empty()) {
        debug << "DataWrapper::debugPublish - empty std::vector DEBUG_ID = " << getIDName(DBID_LINES) << std::endl;
        return false;
    }
#endif

    //Get window assigned to id
    map<DEBUG_ID, std::vector< pair<std::string, cv::Mat>* > >::iterator map_it = debug_map.find(id);
    if(map_it == debug_map.end()) {
        errorlog << "DataWrapper::debugPublish - Missing window definition DEBUG_ID = " << getIDName(id) << std::endl;
        return false;
    }

    //for all images
    for(unsigned int i=0; i<map_it->second.size(); i++) {
        std::string& window = map_it->second[i]->first; //get window name from pair
        cv::Mat& img = map_it->second[i]->second; //get window name from pair
        cv::Scalar colour;

        switch(id) {
        case DBID_GOAL_LINES_START:
            colour = cv::Scalar(255,255,0);
            break;
        case DBID_GOAL_LINES_END:
            colour = cv::Scalar(255,0,255);
            break;
        default:
            errorlog << "DataWrapper::debugPublish - Called with invalid id" << std::endl;
            return false;
        }

        BOOST_FOREACH(LSFittedLine l, data) {
            FieldLine ln(l);
            ln.render(img, colour);
            BOOST_FOREACH(Point pt, l.getPoints()) {
                cv::circle(img, cv::Point2i(pt.x, pt.y), 1, cv::Scalar(255,255,255) - colour);
            }

            Vector2<Point> pts = l.getEndPoints();
            //std::cout << pts << std::endl;
            cv::circle(img, cv::Point2i(pts[0].x, pts[0].y), 1, cv::Scalar(0,255,255));
            cv::circle(img, cv::Point2i(pts[1].x, pts[1].y), 1, cv::Scalar(0,255,255));

            Point t1 = l.projectOnto(pts[0]),
                  t2 = l.projectOnto(pts[1]);

            //std::cout << t1 << " " << t2 << std::endl;

            cv::circle(img, cv::Point2i(t1.x, t1.y), 3, cv::Scalar(0,0,255));
            cv::circle(img, cv::Point2i(t2.x, t2.y), 3, cv::Scalar(0,0,255));
            //l.render(img, colour);
        }

        imshow(window, img);
    }

    return true;
}

bool DataWrapper::updateFrame()
{
    switch(m_method) {
    case CAMERA:
        m_current_image = *(m_camera->grabNewImage());   //force get new frame
        break;
    case STREAM:
        VisionConstants::loadFromFile(configname);
        if(!imagestrm.is_open()) {
            errorlog << "No image stream - " << streamname << std::endl;
            return false;
        }
        if(using_sensors && !sensorstrm.is_open()) {
            errorlog << "No sensor stream - " << sensorstreamname << std::endl;
            return false;
        }
        try {
            imagestrm >> m_current_image;
        }
        catch(std::exception& e) {
            errorlog << "Image stream error - resetting: " << e.what() << std::endl;
            imagestrm.clear() ;
            imagestrm.seekg(0, ios::beg);
            imagestrm >> m_current_image;
            if(using_sensors) {
                sensorstrm.clear() ;
                sensorstrm.seekg(0, ios::beg);
            }
        }
        if(using_sensors) {
            try {
                sensorstrm >> m_sensor_data;
            }
            catch(std::exception& e){
                errorlog << "Sensor stream error: " << e.what() << std::endl;
            }
        }
        break;
    }

    //overwrite sensor horizon if using sensors
    std::vector<float> hor_data;
    if(using_sensors && m_sensor_data.getHorizon(hor_data)) {
        kinematics_horizon.setLine(hor_data.at(0), hor_data.at(1), hor_data.at(2));
    }

    numFramesProcessed++;

    return true;
}

/**
*   @brief loads the colour look up table
*   @param filename The filename for the LUT stored on disk
*   @note Taken from original vision system
*/
bool DataWrapper::loadLUTFromFile(const std::string& fileName)
{
    return LUT.loadLUTFromFile(fileName);
}

void DataWrapper::ycrcb2ycbcr(cv::Mat *img_ycrcb)
{
    std::vector<cv::Mat> planes;
    split(*img_ycrcb, planes);
    cv::Mat temp = planes[0];
    planes[0] = planes[2];
    planes[2] = temp;
    merge(planes, *img_ycrcb);
}
