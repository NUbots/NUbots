#include <boost/foreach.hpp>
#include "datawrappernuview.h"
#include "Infrastructure/NUImage/ColorModelConversions.h"
#include "debug.h"
#include "debugverbosityvision.h"
#include "nubotdataconfig.h"
#include "Kinematics/Kinematics.h"

#include "Vision/visionconstants.h"
#include "Vision/VisionTypes/coloursegment.h"
#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/Jobs/CameraJobs/ChangeCameraSettingsJob.h"


DataWrapper* DataWrapper::instance = 0;

void getPointsAndColoursFromSegments(const std::vector< std::vector<ColourSegment> >& segments, std::vector<Colour>& colours, std::vector<Point>& pts)
{
    BOOST_FOREACH(const std::vector<ColourSegment>& line, segments) {
        BOOST_FOREACH(const ColourSegment& seg, line) {
            pts.push_back(seg.getStart());
            pts.push_back(seg.getEnd());
            colours.push_back(seg.getColour());
        }
    }
}

DataWrapper::DataWrapper()
{
    camera_data.LoadFromConfigFile((std::string(CONFIG_DIR) + std::string("CameraSpecs.cfg")).c_str());
    VisionConstants::loadFromFile(std::string(CONFIG_DIR) + std::string("VisionOptions.cfg"));

    std::string sen_calib_name = std::string(CONFIG_DIR) + std::string("SensorCalibration.cfg");

    debug << "opening sensor calibration config: " << sen_calib_name << std::endl;
    if( ! m_sensor_calibration.ReadSettings(sen_calib_name)) {
        errorlog << "DataWrapper::DataWrapper() - failed to load sensor calibration: " << sen_calib_name << ". Using default values." << std::endl;
        m_sensor_calibration = SensorCalibration();
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

//void DataWrapper::setCallBack(virtualNUbot* virtual_nubot)
//{
//    m_virtual_nubot = virtual_nubot;
//}

/**
*   @brief Fetches the next frame from the webcam.
*/
const NUImage* DataWrapper::getFrame()
{
    return m_current_image;
}

//! @brief Retrieves the camera height returns it.
float DataWrapper::getCameraHeight() const
{
    return m_camera_height;
}

//! @brief Retrieves the camera pitch returns it.
float DataWrapper::getHeadPitch() const
{
    return m_head_pitch;
}

//! @brief Retrieves the camera yaw returns it.
float DataWrapper::getHeadYaw() const
{
    return m_head_yaw;
}

//! @brief Retrieves the body pitch returns it.
Vector3<float> DataWrapper::getOrientation() const
{
    return m_orientation;
}

Vector3<double> DataWrapper::getNeckPosition() const
{
    return m_neck_position;
}

Vector2<double> DataWrapper::getCameraFOV() const
{
    return Vector2<double>(camera_data.m_horizontalFov, camera_data.m_verticalFov);
}

const Horizon& DataWrapper::getKinematicsHorizon()
{
    #if VISION_WRAPPER_VERBOSITY > 1
        debug << "DataWrapper::getKinematicsHorizon() - Begin" << std::endl;
    #endif
    if(sensor_data->getHorizon(m_horizon_coefficients)) {
        #if VISION_WRAPPER_VERBOSITY > 1
            debug << "DataWrapper::getKinematicsHorizon() - success" << std::endl;
        #endif
        m_kinematics_horizon.setLine(m_horizon_coefficients.at(0), m_horizon_coefficients.at(1), m_horizon_coefficients.at(2));
        m_kinematics_horizon.exists = true;
    }
    else {
        #if VISION_WRAPPER_VERBOSITY > 1
            debug << "DataWrapper::getKinematicsHorizon() - failed" << std::endl;
        #endif
        m_kinematics_horizon.setLineFromPoints(Point(0, 0), Point(m_current_image->getWidth(), 0));

        m_kinematics_horizon.exists = false;
    }
    
    return m_kinematics_horizon;
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
    return CameraSettings();
}

const LookUpTable& DataWrapper::getLUT() const
{
    return LUT;
}

void DataWrapper::publish(const std::vector<const VisionFieldObject*> &visual_objects)
{
    for(int i=0; i<visual_objects.size(); i++) {
        visual_objects.at(i)->addToExternalFieldObjects(field_objects, m_timestamp);
    }
}

void DataWrapper::publish(const VisionFieldObject* visual_object)
{
    std::cout << m_timestamp << std::endl;
    visual_object->addToExternalFieldObjects(field_objects, m_timestamp);
}

void DataWrapper::debugPublish(DEBUG_ID id, const std::vector<Point> &data_points)
{
    #if VISION_WRAPPER_VERBOSITY > 2
        debug << id << std::endl;
        debug << data_points << std::endl;
    #endif

    switch(id) {
    case DBID_H_SCANS:
        debug << "DataWrapper::debugPublish - DBID_H_SCANS printing not implemented" << std::endl;
        break;
    case DBID_V_SCANS:
        debug << "DataWrapper::debugPublish - DBID_V_SCANS printing not implemented" << std::endl;
        break;
    case DBID_HORIZON:
        debug << "DataWrapper::debugPublish - DBID_HORIZON printing handled externally to vision" << std::endl;
        break;
    case DBID_GREENHORIZON_SCANS:
        emit pointsUpdated(data_points, GLDisplay::greenHorizonScanPoints);
        break;
    case DBID_GREENHORIZON_FINAL:
        emit pointsUpdated(data_points, GLDisplay::greenHorizonPoints);
        break;
    case DBID_MATCHED_SEGMENTS:
        emit pointsUpdated(data_points, GLDisplay::Transitions);
        break;
    case DBID_OBSTACLE_POINTS:
        debug << "DataWrapper::debugPublish - DBID_OBSTACLE_POINTS printing not implemented" << std::endl;
        break;
    default:
        errorlog << "DataWrapper::debugPublish - Called with invalid id" << std::endl;
    }
}

//! Outputs debug data to the appropriate external interface
void DataWrapper::debugPublish(DEBUG_ID id, const SegmentedRegion& region)
{

    switch(id) {
    case DBID_SEGMENTS:
        emit segmentsUpdated(region.getSegments(), GLDisplay::Segments);
//        c_it = colours.begin();
//        for (it = data_points.begin(); it < data_points.end()-1; it+=2) {
//            //draws a line between each consecutive pair of points of the corresponding colour
//            line(img, cv::Point2i(it->x, it->y), cv::Point2i((it+1)->x, (it+1)->y), *c_it, 1);
//            c_it++;
//        }
        break;
    case DBID_FILTERED_SEGMENTS:
        emit segmentsUpdated(region.getSegments(), GLDisplay::FilteredSegments);
//        c_it = colours.begin();
//        for (it = data_points.begin(); it < data_points.end()-1; it+=2) {
//            //draws a line between each consecutive pair of points of the corresponding colour
//            line(img, cv::Point2i(it->x, it->y), cv::Point2i((it+1)->x, (it+1)->y), *c_it, 1);
//            c_it++;
//        }
        break;
    default:
        errorlog << "DataWrapper::debugPublish - Called with invalid id" << std::endl;
    }
}

void DataWrapper::debugPublish(DEBUG_ID id, const NUImage* const img)
{

}

void DataWrapper::debugPublish(DEBUG_ID id, const std::vector<LSFittedLine> &data)
{
    switch(id) {
    case DBID_GOAL_LINES_START:
        emit linesUpdated(data, GLDisplay::GoalEdgeLinesStart);
        break;
    case DBID_GOAL_LINES_END:
        emit linesUpdated(data, GLDisplay::GoalEdgeLinesEnd);
        break;
    default:
        errorlog << "DataWrapper::debugPublish - Called with invalid id" << std::endl;
    }
}

void DataWrapper::plotCurve(std::string name, std::vector< Vector2<double> > pts)
{
    QVector<QPointF> qpts;
    BOOST_FOREACH(const Point& p, pts) {
        qpts.push_back(QPointF(p.x, p.y));
    }

    emit plotUpdated(QString(name.c_str()), qpts);
}

void DataWrapper::plotHistogram(std::string name, const Histogram1D &hist, Colour colour)
{
    errorlog << "plotHistogram - not implemented for NUView" << std::endl;
}

void DataWrapper::plotLineSegments(std::string name, std::vector<Point> pts)
{
    errorlog << "plotLineSegments - not implemented for NUView" << std::endl;
}

bool DataWrapper::updateFrame()
{
    // allow dynamic reloading of file values
    camera_data.LoadFromConfigFile((std::string(CONFIG_DIR) + std::string("CameraSpecs.cfg")).c_str());
    VisionConstants::loadFromFile(std::string(CONFIG_DIR) + std::string("VisionOptions.cfg"));
    //should check actions but for some reason it keeps coming in as null
    if (m_current_image == NULL || sensor_data == NULL || field_objects == NULL)
    {
        #if VISION_WRAPPER_VERBOSITY > 1
            debug << "DataWrapper::updateFrame(): null reference from BB" << std::endl;
        #endif
        // keep object times updated.
        if(field_objects && sensor_data)
        {
            field_objects->preProcess(sensor_data->GetTimestamp());
            field_objects->postProcess(sensor_data->GetTimestamp());
        }
        return false;
    }

    vector<float> orientation(3, 0);

    //update kinematics snapshot
    if(!sensor_data->getCameraHeight(m_camera_height))
        errorlog << "DataWrapperDarwin - updateFrame() - failed to get camera height from NUSensorsData" << std::endl;
    if(!sensor_data->getPosition(NUSensorsData::HeadPitch, m_head_pitch))
        errorlog << "DataWrapperDarwin - updateFrame() - failed to get head pitch from NUSensorsData" << std::endl;
    if(!sensor_data->getPosition(NUSensorsData::HeadYaw, m_head_yaw))
        errorlog << "DataWrapperDarwin - updateFrame() - failed to get head yaw from NUSensorsData" << std::endl;
    if(!sensor_data->getOrientation(orientation))
        errorlog << "DataWrapperDarwin - updateFrame() - failed to get orientation from NUSensorsData" << std::endl;

    vector<float> left, right;
    if(sensor_data->get(NUSensorsData::LLegTransform, left) and sensor_data->get(NUSensorsData::RLegTransform, right))
    {
        m_neck_position = Kinematics::CalculateNeckPosition(Matrix4x4fromVector(left), Matrix4x4fromVector(right), m_sensor_calibration.m_neck_position_offset);
    }
    else
    {
        errorlog << "DataWrapperDarwin - updateFrame() - failed to get left or right leg transforms from NUSensorsData" << std::endl;
        // Default in case kinemtaics not available. Base height of darwin.
        m_neck_position = Vector3<double>(0.0, 0.0, 39.22);
    }

    m_timestamp = m_current_image->GetTimestamp();
    //cout << "pre: " << m_timestamp << std::endl;
    field_objects->preProcess(m_timestamp);
    return true;
}

void DataWrapper::postProcess()
{
    if (m_current_image != NULL && field_objects != NULL)
    {
        //cout << "post: " << m_current_image->GetTimestamp() << std::endl;
        field_objects->postProcess(m_current_image->GetTimestamp());
    }
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

void DataWrapper::setRawImage(const NUImage* image)
{
    m_current_image = image;
}

void DataWrapper::setSensorData(NUSensorsData* sensors)
{
    sensor_data = sensors;
}

void DataWrapper::setFieldObjects(FieldObjects *fieldObjects)
{
    field_objects = fieldObjects;
}

void DataWrapper::setLUT(unsigned char *vals)
{
    LUT.set(vals);
}

void DataWrapper::classifyImage(ClassifiedImage &target) const
{
    if(m_current_image != NULL) {
        int width = m_current_image->getWidth();
        int height = m_current_image->getHeight();

        target.setImageDimensions(width,height);
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                target.image[y][x] = LUT.classifyPixel((*m_current_image)(x,y));
            }
        }
    }
}

void DataWrapper::classifyPreviewImage(ClassifiedImage &target,unsigned char* temp_vals) const
{
    int width = m_current_image->getWidth();
    int height = m_current_image->getHeight();

    target.setImageDimensions(width,height);
    LookUpTable tempLUT(temp_vals);
    //qDebug() << "Begin Loop:";
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            target.image[y][x] = tempLUT.classifyPixel((*m_current_image)(x,y));
        }
    }
}

void DataWrapper::saveAnImage()
{
    #if VISION_WRAPPER_VERBOSITY > 1
        debug << "DataWrapper::SaveAnImage(). Starting..." << std::endl;
    #endif

    if (!imagefile.is_open())
        imagefile.open((std::string(DATA_DIR) + std::string("image.strm")).c_str());
    if (!sensorfile.is_open())
        sensorfile.open((std::string(DATA_DIR) + std::string("sensor.strm")).c_str());

    if (imagefile.is_open() and numSavedImages < 2500)
    {
        if(sensorfile.is_open())
        {
            sensorfile << (*sensor_data) << flush;
        }
        NUImage buffer;
        buffer.cloneExisting(*m_current_image);
        imagefile << buffer;
        numSavedImages++;

        if (isSavingImagesWithVaryingSettings)
        {
            CameraSettings tempCameraSettings = m_current_image->getCameraSettings();
            if (numSavedImages % 10 == 0 )
            {
                tempCameraSettings.p_exposure.set(currentSettings.p_exposure.get() - 0);
            }
            else if (numSavedImages % 10 == 1 )
            {
                tempCameraSettings.p_exposure.set(currentSettings.p_exposure.get() - 50);
            }
            else if (numSavedImages % 10 == 2 )
            {
                tempCameraSettings.p_exposure.set(currentSettings.p_exposure.get() - 25);
            }
            else if (numSavedImages % 10 == 3 )
            {
                tempCameraSettings.p_exposure.set(currentSettings.p_exposure.get() - 0);
            }
            else if (numSavedImages % 10 == 4 )
            {
                tempCameraSettings.p_exposure.set(currentSettings.p_exposure.get() + 25);
            }
            else if (numSavedImages % 10 == 5 )
            {
                tempCameraSettings.p_exposure.set(currentSettings.p_exposure.get() + 50);
            }
            else if (numSavedImages % 10 == 6 )
            {
                tempCameraSettings.p_exposure.set(currentSettings.p_exposure.get() + 100);
            }
            else if (numSavedImages % 10 == 7 )
            {
                tempCameraSettings.p_exposure.set(currentSettings.p_exposure.get() + 150);
            }
            else if (numSavedImages % 10 == 8 )
            {
                tempCameraSettings.p_exposure.set(currentSettings.p_exposure.get() + 200);
            }
            else if (numSavedImages % 10 == 9 )
            {
                tempCameraSettings.p_exposure.set(currentSettings.p_exposure.get() + 300);
            }

            //Set the Camera Setttings using Jobs:
            ChangeCameraSettingsJob* newJob = new ChangeCameraSettingsJob(tempCameraSettings);
            Blackboard->Jobs->addCameraJob(newJob);
            //m_camera->setSettings(tempCameraSettings);
        }
    }
    #if VISION_WRAPPER_VERBOSITY > 1
        debug << "DataWrapper::SaveAnImage(). Finished" << std::endl;
    #endif
}
