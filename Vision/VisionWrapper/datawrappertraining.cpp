#include <boost/foreach.hpp>
#include "datawrappertraining.h"
#include "debug.h"
#include "nubotdataconfig.h"

#include "Vision/VisionTypes/coloursegment.h"
#include "Infrastructure/NUImage/ColorModelConversions.h"

#include <QPainter>
#include <QPen>

DataWrapper* DataWrapper::instance = 0;

DataWrapper::DataWrapper()
{
    //defaults for streams and LUTs
    image_stream_name = std::string(getenv("HOME")) +  std::string("/nubot/image.strm");
    sensor_stream_name = std::string(getenv("HOME")) +  std::string("/nubot/sensor.strm");
    LUTname = std::string(getenv("HOME")) +  std::string("/nubot/default.lut");
    imagestrm.open(image_stream_name.c_str());
    sensorstrm.open(sensor_stream_name.c_str());

    valid = true;

    if(!m_camspecs.LoadFromConfigFile((std::string(CONFIG_DIR) + std::string("CameraSpecs.cfg")).c_str())) {
        errorlog << "DataWrapper::DataWrapper() - failed to load camera specifications: " << std::string(CONFIG_DIR) + std::string("CameraSpecs.cfg") << std::endl;
        valid = false;
    }

    if(!imagestrm.is_open()) {
        errorlog << "DataWrapper::DataWrapper() - failed to load image stream: " << image_stream_name << std::endl;
        valid = false;
    }
    if(!sensorstrm.is_open()) {
        errorlog << "DataWrapper::DataWrapper() - failed to load sensor stream: " << sensor_stream_name << std::endl;
        valid = false;
    }
    if(!loadLUTFromFile(LUTname)) {
        errorlog << "DataWrapper::DataWrapper() - failed to load LUT: " << LUTname << std::endl;
        valid = false;
    }

    numFramesProcessed = 0;
}

DataWrapper::~DataWrapper()
{
    imagestrm.close();
    sensorstrm.close();
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
bool DataWrapper::getCTGVector(std::vector<float> &ctgvector)
{
    return m_current_sensors.get(NUSensorsData::CameraToGroundTransform, ctgvector);
}

//! @brief Generates spoofed camera transform vector.
bool DataWrapper::getCTVector(std::vector<float> &ctvector)
{
    return m_current_sensors.get(NUSensorsData::CameraTransform, ctvector);
}


//! @brief Generates spoofed camera height.
bool DataWrapper::getCameraHeight(float& height)
{
    return m_current_sensors.getCameraHeight(height);
}


//! @brief Generates spoofed camera pitch.
bool DataWrapper::getCameraPitch(float& pitch)
{
    return m_current_sensors.getPosition(NUSensorsData::HeadPitch, pitch);
}

//! @brief Generates spoofed camera yaw.
bool DataWrapper::getCameraYaw(float& yaw)
{
    return m_current_sensors.getPosition(NUSensorsData::HeadYaw, yaw);
}

//! @brief Generates spoofed body pitch.
bool DataWrapper::getBodyPitch(float& pitch)
{
    std::vector<float> orientation;
    bool valid = m_current_sensors.get(NUSensorsData::Orientation, orientation);
    if(valid && orientation.size() > 2) {
        pitch = orientation.at(1);
        return true;
    }
    return false;
}

Vector2<double> DataWrapper::getCameraFOV() const
{
    return Vector2<double>(m_camspecs.m_horizontalFov, m_camspecs.m_verticalFov);
}

//! @brief Returns spoofed kinecv::Matics horizon.
const Horizon& DataWrapper::getKinematicsHorizon()
{
    return kinematics_horizon;
}

//! @brief Returns camera settings.
CameraSettings DataWrapper::getCameraSettings()
{
    return m_current_image.getCameraSettings();
}

//! @brief Returns LUT
const LookUpTable& DataWrapper::getLUT() const
{
    return LUT;
}

//! @brief Stores detections for later.
void DataWrapper::publish(const std::vector<const VisionFieldObject*> &visual_objects)
{
    detections.insert(detections.end(), visual_objects.begin(), visual_objects.end());    //add onto detections std::list - invalid
}

//! @brief Stores detections for later.
void DataWrapper::publish(const VisionFieldObject* visual_object)
{
    detections.push_back(visual_object);    //add onto detections std::list - invalid
}

//! @brief Stores detections for later.
void DataWrapper::debugPublish(const std::vector<Ball>& data)
{
    ball_detections.insert(ball_detections.end(), data.begin(), data.end());
}

//! @brief Stores detections for later.
void DataWrapper::debugPublish(const std::vector<Goal>& data)
{
    goal_detections.insert(goal_detections.end(), data.begin(), data.end());
}

//! @brief Stores detections for later.
void DataWrapper::debugPublish(const std::vector<Obstacle>& data)
{
    obstacle_detections.insert(obstacle_detections.end(), data.begin(), data.end());
}

//! @brief Stores detections for later.
void DataWrapper::debugPublish(const std::vector<FieldLine>& data)
{
    line_detections.insert(line_detections.end(), data.begin(), data.end());
}

void DataWrapper::debugPublish(const std::vector<CentreCircle>& data)
{
    centrecircle_detections.insert(centrecircle_detections.end(), data.begin(), data.end());
}

void DataWrapper::debugPublish(const std::vector<CornerPoint>& data)
{
    corner_detections.insert(corner_detections.end(), data.begin(), data.end());
}

//! @brief Updates the data copies from the external system - in this case streams.
bool DataWrapper::updateFrame()
{
    if(valid)
    {
        NUImage img;
        NUSensorsData sensors;
        imagestrm.peek();
        sensorstrm.peek();
        bool image_good = false;
        bool sensors_good = false;
        // read an image from the stream
        if(imagestrm.good()) {
            imagestrm >> img;
            image_good = true;
        }
        else {
            debug << "DataWrapper::updateFrame - failed to read image stream: " << image_stream_name << std::endl;
        }

        // read sensors from the stream
        if(sensorstrm.good()) {
            sensorstrm >> sensors;
            sensors_good = true;
        }
        else {
            debug << "DataWrapper::updateFrame - failed to read sensor stream: " << sensor_stream_name << std::endl;
        }

        if(image_good && sensors_good)
        {
            updateFrame(img, sensors);
        }

        return image_good && sensors_good;  // indicate succesful or failed reads
    }
    else
    {
        return false;
    }

}

//! @brief Updates the data copies from the external system - in this case with a provided image.
void DataWrapper::updateFrame(NUImage& img, NUSensorsData& sensors)
{
    if(numFramesProcessed > 0) {
        //add old detections to history and start new log
        ball_detection_history.push_back(ball_detections);
        goal_detection_history.push_back(goal_detections);
        obstacle_detection_history.push_back(obstacle_detections);
        line_detection_history.push_back(line_detections);
        centrecircle_detection_history.push_back(centrecircle_detections);
        corner_detection_history.push_back(corner_detections);
        resetDetections();
    }
    numFramesProcessed++;

    // update image and sensors data
    m_current_image = img;
    m_current_sensors = sensors;

    // update counters
    numFramesProcessed++;

    //overwrite sensor horizon
    std::vector<float> hor_data;
    if(m_current_sensors.getHorizon(hor_data)) {
        kinematics_horizon.setLine(hor_data.at(0), hor_data.at(1), hor_data.at(2));
    }

    bool camera_pitch_valid, camera_yaw_valid, camera_height_valid, body_pitch_valid;
    float camera_pitch, camera_yaw, camera_height, body_pitch;
    bool ctg_valid;
    std::vector<float> ctg_vector;

    //get data copies from wrapper
    camera_pitch_valid = getCameraPitch(camera_pitch);
    camera_yaw_valid = getCameraYaw(camera_yaw);
    camera_height_valid = getCameraHeight(camera_height);
    body_pitch_valid = getBodyPitch(body_pitch);
    ctg_valid = getCTGVector(ctg_vector);

    //setup transformer
    transformer.setKinematicParams(camera_pitch_valid, camera_pitch,
                                   camera_yaw_valid, camera_yaw,
                                   camera_height_valid, camera_height,
                                   body_pitch_valid, body_pitch,
                                   ctg_valid, ctg_vector);
    transformer.setCamParams(Vector2<double>(m_current_image.getWidth(), m_current_image.getHeight()), getCameraFOV());
}

//! @brief Clears the detection history logs.
void DataWrapper::resetHistory()
{
    ball_detection_history.clear();
    goal_detection_history.clear();
    obstacle_detection_history.clear();
    line_detection_history.clear();
    centrecircle_detection_history.clear();
    corner_detection_history.clear();
}

//! @brief Clears the detection logs.
void DataWrapper::resetDetections()
{
    ball_detections.clear();
    goal_detections.clear();
    obstacle_detections.clear();
    line_detections.clear();
    centrecircle_detections.clear();
    corner_detections.clear();
    detections.clear();
}

/*! @brief Prints the detection history to stream.
  * @param out The stream to print to.
  */
void DataWrapper::printHistory(std::ostream& out)
{
    BOOST_FOREACH(std::vector<Ball> vb, ball_detection_history) {
        out << vb;
    }
    BOOST_FOREACH(std::vector<Goal> vg, goal_detection_history) {
        out << vg;
    }
    BOOST_FOREACH(std::vector<Obstacle> vo, obstacle_detection_history) {
        out << vo;
    }
    BOOST_FOREACH(std::vector<FieldLine> vl, line_detection_history) {
        BOOST_FOREACH(FieldLine l, vl) {
            out << l;
        }
    }    
    BOOST_FOREACH(std::vector<CentreCircle> vcc, centrecircle_detection_history) {
        BOOST_FOREACH(CentreCircle cc, vcc) {
            out << cc;
        }
    }
    BOOST_FOREACH(std::vector<CornerPoint> vc, corner_detection_history) {
        BOOST_FOREACH(CornerPoint c, vc) {
            out << c;
        }
    }
}

/**
*   @brief loads the colour look up table.
*   @param filename The filename for the LUT stored on disk.
*   @return The success of the operation.
*/
bool DataWrapper::loadLUTFromFile(const std::string& filename)
{
    return LUT.loadLUTFromFile(filename);
}

/**
*   @brief Sets the image stream to read from.
*   @param filename The filename for the stream stored on disk.
*   @return The success of the operation.
*/
bool DataWrapper::setImageStream(const std::string &filename)
{
    image_stream_name = filename;
    imagestrm.close();
    imagestrm.open(image_stream_name.c_str());
    numFramesProcessed = 0;
    if(!imagestrm.is_open()) {
        errorlog << "DataWrapper::DataWrapper() - failed to load image stream: " << image_stream_name << std::endl;
        return false;
    }
    return true;
}

/**
*   @brief Sets the sensor stream to read from.
*   @param filename The filename for the stream stored on disk.
*   @return The success of the operation.
*/
bool DataWrapper::setSensorStream(const std::string &filename)
{
    sensor_stream_name = filename;
    sensorstrm.close();
    sensorstrm.open(sensor_stream_name.c_str());
    numFramesProcessed = 0;
    if(!sensorstrm.is_open()) {
        errorlog << "DataWrapper::DataWrapper() - failed to load sensor stream: " << sensor_stream_name << std::endl;
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
    sensorstrm.clear();
    sensorstrm.seekg(0, ios::beg);
}

/**
*   @brief Prints the detections as labels to stream.
*   @param out The stream to print to.
*/
void DataWrapper::printLabels(std::ostream& out) const
{
    out << detections.size() << std::endl;
    for(unsigned int i=0; i<detections.size(); i++) {
        detections.at(i)->printLabel(out);
        out << std::endl;
    }
}

/**
*   @brief Reads labels in from stream as VisionFieldObjects.
*   @param in The stream to read from.
*   @param labels The resulting labels (ouput parameter).
*   @return The success of the operation.
*/
bool DataWrapper::readLabels(std::istream& in, std::vector< std::vector<VisionFieldObject*> >& labels) const
{
    VisionFieldObject* next;
    std::vector<VisionFieldObject*> next_vec;
    int n;
    while(in.good()) {
        in >> n;
        next_vec.clear();
        for(int i=0; i<n; i++) {
            std::string name;
            VFO_ID id;
            // get ID
            in >> name;
            id = VFOFromName(name);

            //generate new field object based on ID and read in following parameters
            if(isGoal(id)) {
                Point pos, size;
                in >> pos >> size;
                next = new Goal(id, Quad(pos.x - 0.5*size.x,
                                         pos.y - size.y,
                                         pos.x + 0.5*size.x,
                                         pos.y));
            }
            else if(id == BALL) {
                Point centre;
                double diam;
                in >> centre >> diam;
                next = new Ball(centre, diam);
            }
            else if(id == OBSTACLE) {
                Point pos, size;
                in >> pos >> size;
                next = new Obstacle(pos, size.x, size.y);
            }
            else if(id == FIELDLINE) {
                Vector2<GroundPoint> gp;
                in >> gp;
                next = new FieldLine(gp);
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

///**
//*   @brief Reads labels in from stream as ID/parameter pairs.
//*   @param in The stream to read from.
//*   @param labels The resulting labels (ouput parameter).
//*   @return The success of the operation.
//*/
//bool DataWrapper::readLabels(std::istream& in, std::vector< std::vector< pair<VFO_ID, Vector2<double> > > >& labels) const
//{
//    pair<VFO_ID, Vector2<double> > next;
//    std::vector< pair<VFO_ID, Vector2<double> > > next_vec;
//    std::vector< std::vector<VisionFieldObject* > > objects;

//    readLabels(in, objects);
//    BOOST_FOREACH(std::vector<VisionFieldObject*> vec, objects) {
//        next_vec.clear();
//        BOOST_FOREACH(VisionFieldObject* vfo, vec) {
//            next.first = vfo->getID();
//            next.second = vfo->getShortLabel();
//            next_vec.push_back(next);
//        }
//        labels.push_back(next_vec);
//    }

//    return labels.size() > 0;
//}

/**
*   @brief Renders the current image and detections to the supplied cv::Mat
*   @param mat The cv::Mat to render to (output parameter).
*   @return The success of the operation.
*
*   @note THIS DOES NOT HANDLE CORNERS OR CENTRE CIRCLE YET
*/
bool DataWrapper::renderFrame(QImage& img, bool lines_only)
{
    //cannot render frames not processed
    if(numFramesProcessed == 0) {
        return false;
    }
    unsigned char r, g, b;    //r,g,b conversion values
    int h = m_current_image.getHeight();
    int w = m_current_image.getWidth();
    std::vector<const VisionFieldObject*>::const_iterator it;

    //initialise image
    img = QImage(w, h, QImage::Format_RGB888);

    //convert and copy pixel data
    for(int y = 0; y < h; y++) {
        for(int x = 0; x < w; x++) {
            //convert to RGB
            ColorModelConversions::fromYCbCrToRGB(m_current_image(x,y).y, m_current_image(x,y).cb, m_current_image(x,y).cr, r, g, b);
            img.setPixel(x, y, qRgb(r, g, b));
        }
    }

    //render the detected object
    BOOST_FOREACH(const VisionFieldObject* vfo, detections)
    {
        VFO_ID id = vfo->getID();
        QPainter painter(&img);

        if(!lines_only)
        {
            if(id == BALL)
            {
                painter.setPen(QColor(255,160,0));
                painter.drawEllipse(QPointF(vfo->getLocationPixels().x, vfo->getLocationPixels().y), vfo->getScreenSize().x, vfo->getScreenSize().y);
            }
            else if(id == GOAL_L || id == GOAL_R)
            {
                painter.setPen(Qt::NoPen);
                painter.setBrush(QBrush(Qt::yellow));
                painter.drawRect(vfo->getLocationPixels().x, vfo->getLocationPixels().y, vfo->getScreenSize().x, vfo->getScreenSize().y);
            }
            else if(id == GOAL_U)
            {
                painter.setPen(QPen(Qt::yellow));
                painter.setBrush(Qt::NoBrush);
                painter.drawRect(vfo->getLocationPixels().x, vfo->getLocationPixels().y, vfo->getScreenSize().x, vfo->getScreenSize().y);
            }
            else if(id == OBSTACLE)
            {
                painter.setPen(QPen(Qt::white));
                painter.setBrush(Qt::NoBrush);
                painter.drawRect(vfo->getLocationPixels().x, vfo->getLocationPixels().y, vfo->getScreenSize().x, vfo->getScreenSize().y);
            }
        }

        // render lines regardless
        if(id == FIELDLINE)
        {
            FieldLine* line = (FieldLine*) vfo;
            painter.setPen(QPen(QBrush(Qt::red), 3));
            painter.drawLine(line->getEndPoints().x.screen.x, line->getEndPoints().x.screen.y, line->getEndPoints().y.screen.x, line->getEndPoints().y.screen.y);
        }
    }
    return true;
}
