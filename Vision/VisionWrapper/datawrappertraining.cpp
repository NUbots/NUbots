#include <boost/foreach.hpp>
#include "datawrappertraining.h"
#include "debug.h"
#include "nubotdataconfig.h"

#include "Vision/VisionTypes/coloursegment.h"
#include "Infrastructure/NUImage/ColorModelConversions.h"

DataWrapper* DataWrapper::instance = 0;

/** @brief Enum to string converter for a DEBUG_ID
*/
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
    //defaults for streams and LUTs
    image_stream_name = string(getenv("HOME")) +  string("/nubot/image.strm");
    LUTname = string(getenv("HOME")) +  string("/nubot/default.lut");
    imagestrm.open(image_stream_name.c_str());
    m_current_image = new NUImage();
    if(!imagestrm.is_open())
        errorlog << "DataWrapper::DataWrapper() - failed to load image stream: " << image_stream_name << endl;
    if(!loadLUTFromFile(LUTname)){
        errorlog << "DataWrapper::DataWrapper() - failed to load LUT: " << LUTname << endl;
    }

    //set up fake kinematic horizon
    kinematics_horizon.setLineFromPoints(Point(0,0), Point(319,0));
    numFramesProcessed = 0;
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

//! @brief Generates spoofed camera to ground transform vector.
bool DataWrapper::getCTGVector(vector<float> &ctgvector)
{
    //return isOK;
    ctgvector.clear();
    ctgvector.push_back(0);
    ctgvector.push_back(0);
    ctgvector.push_back(0);
    ctgvector.push_back(0);
    return false;
}

//! @brief Generates spoofed camera to body transform vector.
bool DataWrapper::getCTVector(vector<float> &ctvector)
{
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

//! @brief Returns kinematics horizon.
const Horizon& DataWrapper::getKinematicsHorizon()
{
    return kinematics_horizon;
}

//! @brief Returns camera settings.
CameraSettings DataWrapper::getCameraSettings()
{
    return m_current_image->getCameraSettings();
}

//! @brief Returns LUT
const LookUpTable& DataWrapper::getLUT() const
{
    return LUT;
}

//! @brief Stores detections for later.
void DataWrapper::publish(const vector<const VisionFieldObject*> &visual_objects)
{
    detections.insert(detections.end(), visual_objects.begin(), visual_objects.end());    //add onto detections list - invalid
}

//! @brief Stores detections for later.
void DataWrapper::publish(const VisionFieldObject* visual_object)
{
    detections.push_back(visual_object);    //add onto detections list - invalid
}

//! @brief Stores detections for later.
bool DataWrapper::debugPublish(const vector<Ball>& data)
{
    ball_detections.insert(ball_detections.end(), data.begin(), data.end());
    return true;
}

//! @brief Stores detections for later.
bool DataWrapper::debugPublish(const vector<Beacon>& data)
{
    beacon_detections.insert(beacon_detections.end(), data.begin(), data.end());
    return true;
}

//! @brief Stores detections for later.
bool DataWrapper::debugPublish(const vector<Goal>& data)
{
    goal_detections.insert(goal_detections.end(), data.begin(), data.end());
    return true;
}

//! @brief Stores detections for later.
bool DataWrapper::debugPublish(const vector<Obstacle>& data)
{
    obstacle_detections.insert(obstacle_detections.end(), data.begin(), data.end());
    return true;
}

//! @brief Stores detections for later.
bool DataWrapper::debugPublish(const vector<FieldLine>& data)
{
    line_detections.insert(line_detections.end(), data.begin(), data.end());
    return true;
}

//! @brief Updates the data copies from the external system - in this case streams.
bool DataWrapper::updateFrame()
{
    if(numFramesProcessed > 0) {
        //add old detections to history and start new log
        ball_detection_history.push_back(ball_detections);
        goal_detection_history.push_back(goal_detections);
        beacon_detection_history.push_back(beacon_detections);
        obstacle_detection_history.push_back(obstacle_detections);
        line_detection_history.push_back(line_detections);
        resetDetections();
    }

    //update counters
    numFramesProcessed++;
    imagestrm.peek();
    bool image_good = false;
    //read an image from the stream
    if(imagestrm.is_open() && imagestrm.good()) {
        imagestrm >> *m_current_image;
        image_good = true;
    }
    else {
        debug << "DataWrapper::updateFrame - failed to read image stream: " << image_stream_name << endl;
    }
    return image_good;  //indicate succesful or failed read
}

//! @brief Updates the data copies from the external system - in this case with a provided image.
void DataWrapper::updateFrame(NUImage& img)
{
    if(numFramesProcessed > 0) {
        //add old detections to history and start new log
        ball_detection_history.push_back(ball_detections);
        goal_detection_history.push_back(goal_detections);
        beacon_detection_history.push_back(beacon_detections);
        obstacle_detection_history.push_back(obstacle_detections);
        line_detection_history.push_back(line_detections);
        resetDetections();
    }
    numFramesProcessed++;
    m_current_image = &img;
}

//! @brief Clears the detection history logs.
void DataWrapper::resetHistory()
{
    ball_detection_history.clear();
    goal_detection_history.clear();
    beacon_detection_history.clear();
    obstacle_detection_history.clear();
    line_detection_history.clear();
}

//! @brief Clears the detection logs.
void DataWrapper::resetDetections()
{
    ball_detections.clear();
    goal_detections.clear();
    beacon_detections.clear();
    obstacle_detections.clear();
    line_detections.clear();
    detections.clear();
}

/*! @brief Prints the detection history to stream.
  * @param out The stream to print to.
  */
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
    numFramesProcessed = 0;
    if(!imagestrm.is_open()) {
        errorlog << "DataWrapper::DataWrapper() - failed to load stream: " << image_stream_name << endl;
        return false;
    }
    return true;
}

/**
*   @brief Resets the stream to the initial point.
*/
void DataWrapper::resetStream()
{
    numFramesProcessed = 0;
    imagestrm.clear();
    imagestrm.seekg(0, ios::beg);
}

/**
*   @brief Prints the detections as labels to stream.
*   @param out The stream to print to.
*/
void DataWrapper::printLabels(ostream& out) const
{
    out << detections.size() << endl;
    for(unsigned int i=0; i<detections.size(); i++) {
        detections.at(i)->printLabel(out);
        out << endl;
    }
}

/**
*   @brief Reads labels in from stream as VisionFieldObjects.
*   @param in The stream to read from.
*   @param labels The resulting labels (ouput parameter).
*   @return The success of the operation.
*/
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
            //get ID
            in >> name;
            id = VisionFieldObject::getVFOFromName(name);

            //generate new field object based on ID and read in following parameters
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
                in >> o->m_location_pixels >> o->m_size_on_screen;
                next = dynamic_cast<VisionFieldObject*>(o);
            }
            else if(id == VisionFieldObject::FIELDLINE) {
                FieldLine* l = new FieldLine();
                Vector2<double> vals;
                in >> vals;
                l->m_rho = vals.x;
                l->m_phi = vals.y;
                //in >> l->m_rho >> l->m_phi;
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

/**
*   @brief Reads labels in from stream as ID/parameter pairs.
*   @param in The stream to read from.
*   @param labels The resulting labels (ouput parameter).
*   @return The success of the operation.
*/
bool DataWrapper::readLabels(istream& in, vector< vector< pair<VisionFieldObject::VFO_ID, Vector2<double> > > >& labels) const
{
    pair<VisionFieldObject::VFO_ID, Vector2<double> > next;
    vector< pair<VisionFieldObject::VFO_ID, Vector2<double> > > next_vec;
    vector< vector<VisionFieldObject* > > objects;

    readLabels(in, objects);
    BOOST_FOREACH(vector<VisionFieldObject*> vec, objects) {
        next_vec.clear();
        BOOST_FOREACH(VisionFieldObject* vfo, vec) {
            next.first = vfo->getID();
            next.second = vfo->getShortLabel();
            next_vec.push_back(next);
        }
        labels.push_back(next_vec);
    }

    return labels.size() > 0;
}

/**
*   @brief Renders the current image and detections to the supplied cv::Mat
*   @param mat The cv::Mat to render to (output parameter).
*   @return The success of the operation.
*/
bool DataWrapper::renderFrame(cv::Mat &mat, bool lines_only)
{
    //cannot render frames not processed
    if(numFramesProcessed == 0) {
        return false;
    }
    unsigned char* ptr, //pointer to image row
            r, g, b;    //r,g,b conversion values
    vector<const VisionFieldObject*>::const_iterator it;

    //initialise mat
    mat.create(m_current_image->getHeight(), m_current_image->getWidth(), CV_8UC3);

    //convert and copy pixel data
    for(int y=0; y<m_current_image->getHeight(); y++) {
        ptr = mat.ptr<unsigned char>(y);
        for(int x=0; x<m_current_image->getWidth(); x++) {
            //convert to RGB
            ColorModelConversions::fromYCbCrToRGB((*m_current_image)(x,y).y, (*m_current_image)(x,y).cb, (*m_current_image)(x,y).cr, r, g, b);
            //store as BGR
            ptr[3*x]   = b;
            ptr[3*x+1] = g;
            ptr[3*x+2] = r;
        }
    }

    //render the detected object
    for(it=detections.begin(); it<detections.end(); it++) {
        if(lines_only) {
            if((*it)->getID() == VisionFieldObject::FIELDLINE) {
                (*it)->render(mat);
            }
        }
        else {
            (*it)->render(mat);
        }
    }
    return true;
}
