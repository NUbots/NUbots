#include <boost/foreach.hpp>
#include "datawrappertraining.h"
#include "debug.h"
#include "nubotdataconfig.h"

#include "Vision/VisionTypes/coloursegment.h"

DataWrapper* DataWrapper::instance = 0;

string DataWrapper::getIDName(DATA_ID id) {
    switch(id) {
    case DID_IMAGE:
        return "DID_IMAGE";
    case DID_CLASSED_IMAGE:
        return "DID_CLASSED_IMAGE";
    default:
        return "NOT VALID";
    }
}

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
    default:
        return "NOT VALID";
    }
}

DataWrapper::DataWrapper()
{
    //defaults for stream and LUT info
    image_stream_name = string(getenv("HOME")) +  string("/nubot/image.strm");
    sensor_stream_name = string(getenv("HOME")) + string("/nubot/sensor.strm");
    LUTname = string(getenv("HOME")) +  string("/nubot/default.lut");
    imagestrm.open(image_stream_name.c_str());
    sensorstrm.open(sensor_stream_name.c_str());
    m_current_image = new NUImage();
    m_sensor_data = new NUSensorsData();
    if(!imagestrm.is_open())
        errorlog << "DataWrapper::DataWrapper() - failed to load image stream: " << image_stream_name << endl;
    if(!sensorstrm.is_open())
        errorlog << "DataWrapper::DataWrapper() - failed to load sensor stream: " << sensor_stream_name << endl;
    if(!loadLUTFromFile(LUTname)){
        errorlog << "DataWrapper::DataWrapper() - failed to load LUT: " << LUTname << endl;
    }

    //set up fake horizon
    kinematics_horizon.setLineFromPoints(Point(0,0), Point(319,0));
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
    return m_current_image->getCameraSettings();
}

const LookUpTable& DataWrapper::getLUT() const
{
    return LUT;
}

void DataWrapper::publish(const vector<const VisionFieldObject*> &visual_objects)
{
    detections.insert(detections.end(), visual_objects.begin(), visual_objects.end());    //add onto detections list - invalid
}

void DataWrapper::publish(const VisionFieldObject* visual_object)
{
    detections.push_back(visual_object);    //add onto detections list - invalid
}

bool DataWrapper::debugPublish(const vector<Ball>& data)
{
    ball_detections.insert(ball_detections.end(), data.begin(), data.end());
    return true;
}

bool DataWrapper::debugPublish(const vector<Beacon>& data)
{
    beacon_detections.insert(beacon_detections.end(), data.begin(), data.end());
    return true;
}

bool DataWrapper::debugPublish(const vector<Goal>& data)
{
    goal_detections.insert(goal_detections.end(), data.begin(), data.end());
    return true;
}

bool DataWrapper::debugPublish(const vector<Obstacle>& data)
{
    obstacle_detections.insert(obstacle_detections.end(), data.begin(), data.end());
    return true;
}

bool DataWrapper::debugPublish(const vector<FieldLine>& data)
{
    line_detections.insert(line_detections.end(), data.begin(), data.end());
    return true;
}

bool DataWrapper::updateFrame()
{
    if(numFramesProcessed > 0) {
        //add old detections to history and start new log
        ball_detection_history.push_back(ball_detections);
        ball_detections.clear();
        goal_detection_history.push_back(goal_detections);
        goal_detections.clear();
        beacon_detection_history.push_back(beacon_detections);
        beacon_detections.clear();
        obstacle_detection_history.push_back(obstacle_detections);
        obstacle_detections.clear();
        line_detection_history.push_back(line_detections);
        line_detections.clear();

        //detection_history.push_back(detections);
        detections.clear();
    }
    numFramesProcessed++;
    imagestrm.peek();
    bool image_good = false;
    if(imagestrm.is_open() && imagestrm.good()) {
        imagestrm >> *m_current_image;
        image_good = true;
        debug << "DataWrapper::updateFrame - failed to load image stream: " << image_stream_name << endl;
    }
    if(sensorstrm.is_open() && sensorstrm.good()) {
        sensorstrm >> *m_sensor_data;
        debug << "DataWrapper::updateFrame - failed to load sensor stream: " << sensor_stream_name << endl;
    }
    return image_good;
}

void DataWrapper::resetHistory()
{
    ball_detection_history.clear();
    goal_detection_history.clear();
    beacon_detection_history.clear();
    obstacle_detection_history.clear();
    line_detection_history.clear();
}

void DataWrapper::printHistory(ostream& out)
{
    BOOST_FOREACH(vector<Ball> vb, ball_detection_history) {
        out << vb;
    }
    BOOST_FOREACH(vector<Goal> vg, goal_detection_history) {
        out << vg;
    }
    BOOST_FOREACH(vector<Beacon> vb, beacon_detection_history) {
        out << vb;
    }
    BOOST_FOREACH(vector<Obstacle> vo, obstacle_detection_history) {
        out << vo;
    }
    BOOST_FOREACH(vector<FieldLine> vl, line_detection_history) {
        BOOST_FOREACH(FieldLine l, vl) {
            out << l;
        }
    }
}

/**
*   @brief loads the colour look up table.
*   @param filename The filename for the LUT stored on disk.
*   @return The success of the operation.
*/
bool DataWrapper::loadLUTFromFile(const string& filename)
{
    return LUT.loadLUTFromFile(filename);
}

/**
*   @brief Sets the image stream to read from.
*   @param filename The filename for the stream stored on disk.
*   @return The success of the operation.
*/
bool DataWrapper::setImageStream(const string &filename)
{
    image_stream_name = filename;
    imagestrm.close();
    imagestrm.open(image_stream_name.c_str());
    numFramesDropped = numFramesProcessed = 0;
    if(!imagestrm.is_open()) {
        errorlog << "DataWrapper::DataWrapper() - failed to load stream: " << image_stream_name << endl;
        return false;
    }
    return true;
}

/**
*   @brief Sets the sensor stream to read from.
*   @param filename The filename for the stream stored on disk.
*   @return The success of the operation.
*/
bool DataWrapper::setSensorStream(const string &filename)
{
    sensor_stream_name = filename;
    sensorstrm.close();
    sensorstrm.open(sensor_stream_name.c_str());
    if(!sensorstrm.is_open()) {
        errorlog << "DataWrapper::DataWrapper() - failed to load stream: " << sensor_stream_name << endl;
        return false;
    }
    if(numFramesProcessed > 0) {
        errorlog << "DataWrapper::DataWrapper() - attempted to set sensor stream midway through reading image stream: " << sensor_stream_name << endl;
        return false;
    }
    return true;
}

void DataWrapper::resetStream()
{
    numFramesDropped = numFramesProcessed = 0;
    imagestrm.clear();
    imagestrm.seekg(0, ios::beg);
}

void DataWrapper::printLabels(ostream& out) const
{
    out << detections.size() << endl;
    for(unsigned int i=0; i<detections.size(); i++) {
        detections.at(i)->printLabel(out);
        out << endl;
    }
}

bool DataWrapper::readLabels(istream& in, vector< vector<VisionFieldObject*> >& labels) const
{
    VisionFieldObject* next;
    vector<VisionFieldObject*> next_vec;
    int n;
    while(in.good()) {
        in >> n;
        next_vec.clear();
        for(int i=0; i<n; i++) {
            string name;
            VisionFieldObject::VFO_ID id;
            in >> name;
            id = VisionFieldObject::getVFOFromName(name);
            if(VisionFieldObject::isBeacon(id)) {
                Beacon* b = new Beacon(id);
                in >> b->m_location_pixels >> b->m_size_on_screen;
                next = dynamic_cast<VisionFieldObject*>(b);
            }
            else if(VisionFieldObject::isGoal(id)) {
                Goal* g = new Goal(id);
                in >> g->m_location_pixels >> g->m_size_on_screen;
                next = dynamic_cast<VisionFieldObject*>(g);
            }
            else if(id == VisionFieldObject::BALL) {
                Ball* b = new Ball();
                in >> b->m_location_pixels >> b->m_diameter;
                next = dynamic_cast<VisionFieldObject*>(b);
            }
            else if(id == VisionFieldObject::OBSTACLE) {
                Obstacle* o = new Obstacle();
                in >> o->m_location_pixels >> o->m_size_on_screen;\
                next = dynamic_cast<VisionFieldObject*>(o);
            }
            else if(id == VisionFieldObject::FIELDLINE) {
                FieldLine* l = new FieldLine();
                in >> l->m_rho >> l->m_phi;
                next = dynamic_cast<VisionFieldObject*>(l);
            }
            else {
                return false;
            }
            next_vec.push_back(next);
        }
        labels.push_back(next_vec);
        in.ignore(2,'\n');
        in.peek();
    }
    return labels.size() > 0;
}

bool DataWrapper::readLabels(istream& in, vector< vector< pair<VisionFieldObject::VFO_ID, Vector2<double> > > >& labels) const
{
    pair<VisionFieldObject::VFO_ID, Vector2<double> > next;
    vector< pair<VisionFieldObject::VFO_ID, Vector2<double> > > next_vec;
    int n;
    while(in.good()) {
        in >> n;
        next_vec.clear();
        for(int i=0; i<n; i++) {
            string name;
            VisionFieldObject::VFO_ID id;
            in >> name >> next.second;
            next.first = VisionFieldObject::getVFOFromName(name);
            next_vec.push_back(next);
        }
        labels.push_back(next_vec);
        in.ignore(2,'\n');
        in.peek();
    }
    return labels.size() > 0;
}
