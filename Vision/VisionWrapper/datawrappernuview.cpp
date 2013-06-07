#include <boost/foreach.hpp>
#include "datawrappernuview.h"
#include "Infrastructure/NUImage/ColorModelConversions.h"
#include "debug.h"
#include "debugverbosityvision.h"
#include "nubotdataconfig.h"

#include "Vision/visionconstants.h"
#include "Vision/VisionTypes/coloursegment.h"
#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/Jobs/CameraJobs/ChangeCameraSettingsJob.h"


DataWrapper* DataWrapper::instance = 0;

void getPointsAndColoursFromSegments(const vector< vector<ColourSegment> >& segments, vector<Colour>& colours, vector<Point>& pts)
{
    BOOST_FOREACH(const vector<ColourSegment>& line, segments) {
        BOOST_FOREACH(const ColourSegment& seg, line) {
            pts.push_back(seg.getStart());
            pts.push_back(seg.getEnd());
            colours.push_back(seg.getColour());
        }
    }
}

DataWrapper::DataWrapper()
{
    camera_data.LoadFromConfigFile((string(CONFIG_DIR) + string("CameraSpecs.cfg")).c_str());
    VisionConstants::loadFromFile(string(CONFIG_DIR) + string("VisionOptions.cfg"));
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
bool DataWrapper::getCameraHeight()
{
    return m_camera_height;
}

//! @brief Retrieves the camera pitch returns it.
bool DataWrapper::getHeadPitch()
{
    return m_head_pitch;
}

//! @brief Retrieves the camera yaw returns it.
bool DataWrapper::getHeadYaw()
{
    return m_head_yaw;
}

//! @brief Retrieves the body pitch returns it.
Vector3<float> DataWrapper::getOrientation()
{
    return m_orientation;
}

//! @brief Returns the neck position snapshot.
Vector3<float> DataWrapper::getOrientation()
{
    return m_orientation;
}

const Horizon& DataWrapper::getKinematicsHorizon()
{
    #if VISION_WRAPPER_VERBOSITY > 1
        debug << "DataWrapper::getKinematicsHorizon() - Begin" << endl;
    #endif
    if(sensor_data->getHorizon(m_horizon_coefficients)) {
        #if VISION_WRAPPER_VERBOSITY > 1
            debug << "DataWrapper::getKinematicsHorizon() - success" << endl;
        #endif
        m_kinematics_horizon.setLine(m_horizon_coefficients.at(0), m_horizon_coefficients.at(1), m_horizon_coefficients.at(2));
        m_kinematics_horizon.exists = true;
    }
    else {
        #if VISION_WRAPPER_VERBOSITY > 1
            debug << "DataWrapper::getKinematicsHorizon() - failed" << endl;
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

void DataWrapper::publish(const vector<const VisionFieldObject*> &visual_objects)
{
    for(int i=0; i<visual_objects.size(); i++) {
        visual_objects.at(i)->addToExternalFieldObjects(field_objects, m_timestamp);
    }
}

void DataWrapper::publish(const VisionFieldObject* visual_object)
{
    cout << m_timestamp << endl;
    visual_object->addToExternalFieldObjects(field_objects, m_timestamp);
}

void DataWrapper::debugPublish(DEBUG_ID id, const vector<Point> &data_points)
{
    #if VISION_WRAPPER_VERBOSITY > 2
        debug << id << endl;
        debug << data_points << endl;
    #endif

    switch(id) {
    case DBID_H_SCANS:
        debug << "DataWrapper::debugPublish - DBID_H_SCANS printing not implemented" << endl;
        break;
    case DBID_V_SCANS:
        debug << "DataWrapper::debugPublish - DBID_V_SCANS printing not implemented" << endl;
        break;
    case DBID_HORIZON:
        debug << "DataWrapper::debugPublish - DBID_HORIZON printing handled externally to vision" << endl;
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
        debug << "DataWrapper::debugPublish - DBID_OBSTACLE_POINTS printing not implemented" << endl;
        break;
    default:
        errorlog << "DataWrapper::debugPublish - Called with invalid id" << endl;
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
        errorlog << "DataWrapper::debugPublish - Called with invalid id" << endl;
    }
}

void DataWrapper::debugPublish(DEBUG_ID id, const NUImage* const img)
{

}

void DataWrapper::debugPublish(DEBUG_ID id, const vector<LSFittedLine> &data)
{
    switch(id) {
    case DBID_GOAL_LINES_START:
        emit linesUpdated(data, GLDisplay::GoalEdgeLinesStart);
        break;
    case DBID_GOAL_LINES_END:
        emit linesUpdated(data, GLDisplay::GoalEdgeLinesEnd);
        break;
    default:
        errorlog << "DataWrapper::debugPublish - Called with invalid id" << endl;
    }
}

void DataWrapper::plotCurve(string name, vector< Vector2<double> > pts)
{
    QVector<QPointF> qpts;
    BOOST_FOREACH(const Point& p, pts) {
        qpts.push_back(QPointF(p.x, p.y));
    }

    emit plotUpdated(QString(name.c_str()), qpts);
}

void DataWrapper::plotHistogram(string name, const Histogram1D &hist, Colour colour)
{
    errorlog << "plotHistogram - not implemented for NUView" << endl;
}

void DataWrapper::plotLineSegments(string name, vector<Point> pts)
{
    errorlog << "plotLineSegments - not implemented for NUView" << endl;
}

bool DataWrapper::updateFrame()
{
    // allow dynamic reloading of file values
    camera_data.LoadFromConfigFile((string(CONFIG_DIR) + string("CameraSpecs.cfg")).c_str());
    VisionConstants::loadFromFile(string(CONFIG_DIR) + string("VisionOptions.cfg"));
    //should check actions but for some reason it keeps coming in as null
    if (m_current_image == NULL || sensor_data == NULL || field_objects == NULL)
    {
        #if VISION_WRAPPER_VERBOSITY > 1
            debug << "DataWrapper::updateFrame(): null reference from BB" << endl;
        #endif
        // keep object times updated.
        if(field_objects && sensor_data)
        {
            field_objects->preProcess(sensor_data->GetTimestamp());
            field_objects->postProcess(sensor_data->GetTimestamp());
        }
        return false;
    }
    m_timestamp = m_current_image->GetTimestamp();
    //cout << "pre: " << m_timestamp << endl;
    field_objects->preProcess(m_timestamp);
    return true;
}

void DataWrapper::postProcess()
{
    if (m_current_image != NULL && field_objects != NULL)
    {
        //cout << "post: " << m_current_image->GetTimestamp() << endl;
        field_objects->postProcess(m_current_image->GetTimestamp());
    }
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
        debug << "DataWrapper::SaveAnImage(). Starting..." << endl;
    #endif

    if (!imagefile.is_open())
        imagefile.open((string(DATA_DIR) + string("image.strm")).c_str());
    if (!sensorfile.is_open())
        sensorfile.open((string(DATA_DIR) + string("sensor.strm")).c_str());

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
        debug << "DataWrapper::SaveAnImage(). Finished" << endl;
    #endif
}
