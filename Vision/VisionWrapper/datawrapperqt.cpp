#include <boost/foreach.hpp>
#include "datawrapperqt.h"
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

void getPointsAndColoursFromSegments(const vector< vector<ColourSegment> >& segments, vector<cv::Scalar>& colours, vector<Point >& pts)
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
        LUTname = string(getenv("HOME")) +  string("/nubot/default.lut");
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
            streamname = string(getenv("HOME")) + string("/nubot/image.strm");
            if(using_sensors)
                sensorstreamname = string(getenv("HOME")) + string("/nubot/sensor.strm");
            LUTname = string(getenv("HOME")) + string("/nubot/default.lut");
            configname = string(getenv("HOME")) + string("/nubot/") + string("VisionOptions.cfg");
        }
        imagestrm.open(streamname.c_str());
        if(using_sensors)
            sensorstrm.open(sensorstreamname.c_str());

        if(!sensorstrm.is_open()) {
            QMessageBox::warning(NULL, "Error", QString("Failed to read sensors from: ") + QString(sensorstreamname.c_str()) + QString(" defaulting to sensors off."));
            using_sensors = false;
        }

        if(!imagestrm.is_open()) {
            errorlog << "DataWrapper::DataWrapper() - failed to load stream: " << streamname << endl;
        }
        break;
    }

    //set up fake horizon
    kinematics_horizon.setLine(0, 1, 0);

    updateFrame();

    if(!loadLUTFromFile(LUTname)){
        errorlog << "DataWrapper::DataWrapper() - failed to load LUT: " << LUTname << endl;
    }
    numFramesDropped = numFramesProcessed = 0;
}

DataWrapper::~DataWrapper()
{
}

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

//! @brief Generates spoofed camera transform vector.
bool DataWrapper::getCTGVector(vector<float> &ctgvector)
{
    if(using_sensors) {
        return m_sensor_data.get(NUSensorsData::CameraToGroundTransform, ctgvector);
    }
    else {
        ctgvector.assign(4, 0);
        return false;
    }
}

//! @brief Generates spoofed camera transform vector.
bool DataWrapper::getCTVector(vector<float> &ctvector)
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
        vector<float> orientation;
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

void DataWrapper::publish(const vector<const VisionFieldObject*> &visual_objects)
{
    //cout << visual_objects.size() << " visual objects seen" << std::endl;
}

void DataWrapper::publish(const VisionFieldObject* visual_object)
{
    //cout << "Visual object seen at " << visual_object->getLocationPixels() << std::endl;
    #if VISION_WRAPPER_VERBOSITY > 0
    visual_object->printLabel(debug);
    debug << endl;
    #endif
}

//! Outputs debug data to the appropriate external interface
void DataWrapper::debugRefresh()
{

}

void DataWrapper::debugPublish(const vector<Ball>& data) {

}

//bool DataWrapper::debugPublish(const vector<Beacon>& data) {

//#if VISION_WRAPPER_VERBOSITY > 1
//    if(data.empty()) {
//        debug << "DataWrapper::debugPublish - empty vector DEBUG_ID = " << getIDName(DBID_BEACONS) << endl;
//        return false;
//    }
//#endif

//    //Get window assigned to id
//    map<DEBUG_ID, vector< pair<string, cv::Mat>* > >::iterator map_it = debug_map.find(DBID_BEACONS);
//    if(map_it == debug_map.end()) {
//        errorlog << "DataWrapper::debugPublish - Missing window definition DEBUG_ID = " << getIDName(DBID_BEACONS) << endl;
//        return false;
//    }

//    //for all images
//    for(unsigned int i=0; i<map_it->second.size(); i++) {
//        string& window = map_it->second[i]->first; //get window name from pair
//        cv::Mat& img = map_it->second[i]->second; //get window name from pair

//        BOOST_FOREACH(Beacon b, data) {
//            b.render(img);
//        }

//        imshow(window, img);
//    }
//    return true;
//}

void DataWrapper::debugPublish(const vector<Goal>& data)
{

}

void DataWrapper::debugPublish(const vector<Obstacle>& data)
{

}

void DataWrapper::debugPublish(const vector<FieldLine> &data)
{

}

void DataWrapper::debugPublish(DEBUG_ID id, const vector<Point >& data_points)
{
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
        errorlog << "DataWrapper::debugPublish - Called with invalid id" << endl;
        return false;
    }
    return true;
}

//! Outputs debug data to the appropriate external interface
void DataWrapper::debugPublish(DEBUG_ID id, const SegmentedRegion& region)
{
    unsigned char r, g, b;
    BOOST_FOREACH(const vector<ColourSegment>& line, region.getSegments()) {
        BOOST_FOREACH(const ColourSegment& seg, line) {
            getColourAsRGB(seg.getColour(), r, g, b);
            gui.addToLayer(id, QLineF(seg.getStart().x, seg.getStart().y, seg.getEnd().x, seg.getEnd().y), QColor(r, g, b));
        }
    }
}

void DataWrapper::debugPublish(DEBUG_ID id, NUImage const* const img)
{
    //for all images
    for(unsigned int i=0; i<map_it->second.size(); i++) {
        cv::Mat& img_map = map_it->second[i]->second;    //get image from pair
        string& window = map_it->second[i]->first; //get window name from pair

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

    //clear old display
    if(!m_plain_scene.items().empty())
        m_plain_scene.removeItem(m_plain_scene.items().first());

    //generate QT display objects
    QImage q_plain((uchar*) plain.data, plain.cols, plain.rows, plain.step, QImage::Format_RGB888);
    QImage q_classified((uchar*) classified.data, classified.cols, classified.rows, classified.step, QImage::Format_RGB888);

    q_plain = q_plain.rgbSwapped();
    q_classified = q_classified.rgbSwapped();

    m_plain_pixmap.setPixmap(QPixmap::fromImage(q_plain));
    m_classified_pixmap.setPixmap(QPixmap::fromImage(q_classified));
    m_plain_scene.addItem(&m_plain_pixmap);
    m_classified_scene.addItem(&m_classified_pixmap);

    //set displays
    ui->imageView->setScene(&m_plain_scene);
    ui->imageView_2->setScene(&m_classified_scene);
}

void DataWrapper::debugPublish(DEBUG_ID id, const vector<LSFittedLine>& data)
{
    QColor colour1, colour2;

    switch(id) {
    case DBID_GOAL_LINES_START:
        colour1 = QColor(255,255,0);
        colour2 = QColor(0,0,255);
        break;
    case DBID_GOAL_LINES_END:
        colour1 = QColor(255,0,255);
        colour2 = QColor(0,255,0);
        break;
    default:
        errorlog << "DataWrapper::debugPublish - Called with invalid id" << endl;
        return;
    }

    BOOST_FOREACH(LSFittedLine l, data) {
        Vector2<Point> endpts = l.getEndPoints();
        gui.addToLayer(id, QLineF(endpts.x.x, endpts.x.y, endpts.y.x, endpts.y.y), colour1);
        gui.addToLayer(id, QPointF(endpts.x.x, endpts.x.y), QColor(0,255,255));
        gui.addToLayer(id, QPointF(endpts.y.x, endpts.y.y), QColor(0,255,255));
        Point t1 = l.projectOnto(endpts[0]),
              t2 = l.projectOnto(endpts[1]);
        gui.addToLayer(id, QPointF(t1.x, t1.y), QColor(0,0,255));
        gui.addToLayer(id, QPointF(t2.x, t2.y), QColor(0,0,255));
        BOOST_FOREACH(Point pt, l.getPoints()) {
            gui.addToLayer(id, QPointF(pt.x, pt.y), colour2);
        }
    }
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
            errorlog << "No image stream - " << streamname << endl;
            return false;
        }
        if(using_sensors && !sensorstrm.is_open()) {
            errorlog << "No sensor stream - " << sensorstreamname << endl;
            return false;
        }
        try {
            imagestrm >> m_current_image;
        }
        catch(std::exception& e) {
            errorlog << "Image stream error - resetting: " << e.what() << endl;
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
                errorlog << "Sensor stream error: " << e.what() << endl;
            }
        }
        break;
    }

    //overwrite sensor horizon if using sensors
    vector<float> hor_data;
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
bool DataWrapper::loadLUTFromFile(const string& fileName)
{
    return LUT.loadLUTFromFile(fileName);
}

void DataWrapper::ycrcb2ycbcr(cv::Mat *img_ycrcb)
{
    vector<cv::Mat> planes;
    split(*img_ycrcb, planes);
    cv::Mat temp = planes[0];
    planes[0] = planes[2];
    planes[2] = temp;
    merge(planes, *img_ycrcb);
}
