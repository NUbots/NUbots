#include <boost/foreach.hpp>
#include "datawrapperqt.h"
#include "Infrastructure/NUImage/ColorModelConversions.h"
#include "debug.h"
#include "debugverbosityvision.h"
#include "nubotdataconfig.h"

#include "Vision/VisionTypes/coloursegment.h"
#include "Vision/visionconstants.h"

#include <QMessageBox>
#include <qwt_symbol.h>

DataWrapper* DataWrapper::instance = 0;

DataWrapper::DataWrapper(MainWindow* ui, bool ok, INPUT_METHOD method, string istrm, string sstrm, string cfg, string lname)
{
    m_ok = ok;
    gui = ui;
    m_method = method;
    debug << "openning camera config: " << string(CONFIG_DIR) + string("CameraSpecs.cfg") << endl;
    if( ! m_camspecs.LoadFromConfigFile((string(CONFIG_DIR) + string("CameraSpecs.cfg")).c_str())) {
        errorlog << "DataWrapper::DataWrapper() - failed to load camera specifications: " << string(CONFIG_DIR) + string("CameraSpecs.cfg") << endl;
        ok = false;
    }

    kinematics_horizon.setLine(0, 1, 0);
    numFramesDropped = numFramesProcessed = 0;

    switch(method) {
    case STREAM:
        streamname = istrm;
        debug << "openning image stream: " << streamname << endl;
        imagestrm.open(streamname.c_str());

        using_sensors = !sstrm.empty();
        sensorstreamname = sstrm;
        if(m_ok && using_sensors) {
            debug << "openning sensor stream: " << sensorstreamname << endl;
            sensorstrm.open(sensorstreamname.c_str());
            if(!sensorstrm.is_open()) {
                QMessageBox::warning(NULL, "Error", QString("Failed to read sensors from: ") + QString(sensorstreamname.c_str()) + QString(" defaulting to sensors off."));
                using_sensors = false;
            }
        }

        if(!imagestrm.is_open()) {
            errorlog << "DataWrapper::DataWrapper() - failed to load stream: " << streamname << endl;
            m_ok = false;
        }
        break;
    case CAMERA:

        #ifdef TARGET_IS_MAC
        m_camera = new NUOpenCVCamera;
        #elif TARGET_IS_WINDOWS
        m_camera = new NUOpenCVCamera;
        #else
        m_camera = new PCCamera;
        #endif
    }

    configname = cfg;
    debug << "config: " << configname << endl;
    VisionConstants::loadFromFile(configname);

    LUTname = lname;
    if(!loadLUTFromFile(LUTname)){
        errorlog << "DataWrapper::DataWrapper() - failed to load LUT: " << LUTname << endl;
        ok = false;
    }

    //DEBUG
    ratio_hist.first = 0;
    ratio_hist.second = 0;
    ratio_r1.first = 0;
    ratio_r1.second = 0;
    ratio_r2.first = 0;
    ratio_r2.second = 0;
    //END DEBUG
}

DataWrapper::~DataWrapper()
{
}

DataWrapper* DataWrapper::getInstance()
{
    //if(!instance)
    //    instance = new DataWrapper();
    return instance;
}

/**
*   @brief Fetches the next frame from the webcam.
*/
NUImage* DataWrapper::getFrame()
{
    return &m_current_image;
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
    switch(m_method) {
    case CAMERA:
        return m_camera->getSettings();
    case STREAM:
        return CameraSettings();
    default:
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

void DataWrapper::debugPublish(const vector<Ball>& data) {
    BOOST_FOREACH(const Ball& b, data) {
        gui->addToLayer(DBID_BALLS, QCircle(QPointF(b.getLocationPixels().x, b.getLocationPixels().y), b.getRadius()), QColor(255, 160, 0));
    }
}

void DataWrapper::debugPublish(const vector<CentreCircle>& data)
{
    BOOST_FOREACH(const CentreCircle& c, data) {
        //need to change to display as ellipse - but for now just centre
        gui->addToLayer(DBID_CENTRE_CIRCLES, QPointF(c.getLocationPixels().x, c.getLocationPixels().y), QPen(Qt::magenta, 5));
    }
}

void DataWrapper::debugPublish(const vector<CornerPoint>& data)
{
    BOOST_FOREACH(const CornerPoint& c, data) {
        gui->addToLayer(DBID_CORNERS, QPointF(c.getLocationPixels().x, c.getLocationPixels().y), QPen(Qt::cyan, 5));
    }
}

//bool DataWrapper::debugPublish(const vector<Beacon>& data) {
//
//}

void DataWrapper::debugPublish(const vector<Goal>& data)
{
    BOOST_FOREACH(const Goal& g, data) {
        QPolygonF p;
        const Quad& q = g.getQuad();
        p.append(QPointF(q.getBottomLeft().x, q.getBottomLeft().y));
        p.append(QPointF(q.getTopLeft().x, q.getTopLeft().y));
        p.append(QPointF(q.getTopRight().x, q.getTopRight().y));
        p.append(QPointF(q.getBottomRight().x, q.getBottomRight().y));
        p.append(QPointF(q.getBottomLeft().x, q.getBottomLeft().y));

        gui->addToLayer(DBID_GOALS, Polygon(p, g.getID() != GOAL_U), QPen(Qt::yellow));
        gui->addToLayer(DBID_GOALS, QPointF(g.getLocationPixels().x, g.getLocationPixels().y), QPen(QColor(Qt::blue), 3));
    }
}

////DEBUG - FOR GOAL PAPER
//void DataWrapper::debugPublish(int i, const vector<Goal> &d)
//{
//    double dist = -1;
//    if(true_num_posts == 1) {
//        if(d.size() == 1) {
//            dist = d.front().width_dist;
//        }
//        else {
//            double maxarea = -1;
//            //find biggest
//            BOOST_FOREACH(Goal g, d) {
//                if(g.getQuad().area() > maxarea) {
//                    maxarea = g.getQuad().area();
//                    dist = g.width_dist;
//                }
//            }
//        }
//    }
//    else {
//        if(d.size() == 0) {
//            //missed both
//            switch(i) {
//            case 0:
//                ratio_hist.first++;
//                ratio_hist.second++;
//                break;
//            case 1:
//                ratio_r1.first++;
//                ratio_r1.second++;
//                break;
//            case 2:
//                ratio_r2.first++;
//                ratio_r2.second++;
//                break;
//            }
//        }
//        if(d.size() == 1) {
//            //missed one
//            switch(i) {
//            case 0:
//                ratio_hist.first++;
//                ratio_hist.second++;
//                break;
//            case 1:
//                ratio_r1.first++;
//                ratio_r1.second++;
//                break;
//            case 2:
//                ratio_r2.first++;
//                ratio_r2.second++;
//                break;
//            }
//            if(d.front().getLocationPixels().x > 150)
//                dist = d.front().width_dist;
//        }
//        else {
//            double maxx = -1;
//            //find rightmost
//            BOOST_FOREACH(Goal g, d) {
//                if(g.getLocationPixels().x > maxx) {
//                    maxx = g.getLocationPixels().x;
//                    dist = g.width_dist;
//                }
//            }
//        }
//    }


//    switch(i) {
//    case 0:
//        if(dist != -1)
//            acc_hist(dist);
//        else
//            ratio_hist.first++;
//        ratio_hist.second++;
//        break;
//    case 1:
//        if(dist != -1)
//            acc_r1(dist);
//        else
//            ratio_r1.first++;
//        ratio_r1.second++;
//        break;
//    case 2:
//        if(dist != -1)
//            acc_r2(dist);
//        else
//            ratio_r2.first++;
//        ratio_r2.second++;
//        break;
//    }
//}
////DEBUG

void DataWrapper::debugPublish(const vector<Obstacle>& data)
{
    BOOST_FOREACH(const Obstacle& o, data) {
        QPolygonF p;
        Point loc = o.getLocationPixels();
        Point size = o.getScreenSize();

        p.append(QPointF(loc.x - size.x*0.5, loc.y - size.y));
        p.append(QPointF(loc.x - size.x*0.5, loc.y));
        p.append(QPointF(loc.x + size.x*0.5, loc.y));
        p.append(QPointF(loc.x + size.x*0.5, loc.y - size.y));

        gui->addToLayer(DBID_OBSTACLES, Polygon(p, false), QColor(Qt::white));
    }
}

void DataWrapper::debugPublish(const vector<FieldLine> &data)
{
    BOOST_FOREACH(const FieldLine& l, data) {
        Vector2<NUPoint> endpts = l.getEndPoints();
        gui->addToLayer(DBID_LINES, QLineF( endpts[0].screen.x, endpts[0].screen.y, endpts[1].screen.x, endpts[1].screen.y ), QColor(Qt::red));
    }
}

void DataWrapper::debugPublish(DEBUG_ID id, const vector<Point> &data_points)
{
    int w = m_current_image.getWidth(),
        h = m_current_image.getHeight();
    switch(id) {
    case DBID_H_SCANS:
        BOOST_FOREACH(const Point& pt, data_points) {
            gui->addToLayer(id, QLineF(0, pt.y, w, pt.y), QColor(Qt::gray));
        }
        break;
    case DBID_V_SCANS:
        BOOST_FOREACH(const Point& pt, data_points) {
            gui->addToLayer(id, QLineF(pt.x, pt.y, pt.x, h), QColor(Qt::gray));
        }
        break;
    case DBID_HORIZON:
        gui->addToLayer(id, QLineF(data_points.front().x, data_points.front().y, data_points.back().x, data_points.back().y), QColor(Qt::blue));
        break;
    case DBID_GREENHORIZON_SCANS:
        BOOST_FOREACH(const Point& pt, data_points) {
            gui->addToLayer(id, QPointF(pt.x, pt.y), QPen(Qt::magenta, 3));
        }
        break;
    case DBID_GREENHORIZON_THROWN:
        BOOST_FOREACH(const Point& pt, data_points) {
            gui->addToLayer(id, QPointF(pt.x, pt.y), QPen(Qt::red, 3));
        }
        break;
    case DBID_GREENHORIZON_FINAL:
        for(vector< Point >::const_iterator it=data_points.begin(); it<data_points.end(); it++) {
            if (it > data_points.begin()) {
                gui->addToLayer(id, QLineF((it-1)->x, (it-1)->y, it->x, it->y), QColor(Qt::magenta));
            }
            //gui->addToLayer(id, QPointF(it->x, it->y), QColor(Qt::magenta));
        }
        break;
    case DBID_MATCHED_SEGMENTS:
        BOOST_FOREACH(const Point& pt, data_points) {
            gui->addToLayer(id, QPointF(pt.x, pt.y), QColor(Qt::cyan));
        }
        break;
    case DBID_OBSTACLE_POINTS:
        BOOST_FOREACH(const Point& pt, data_points) {
            gui->addToLayer(id, QPointF(pt.x, pt.y), QPen(Qt::cyan, 2));
        }
        break;
    default:
        errorlog << "DataWrapper::debugPublish - Called with invalid id" << endl;
        return;
    }
}

//! Outputs debug data to the appropriate external interface
void DataWrapper::debugPublish(DEBUG_ID id, const SegmentedRegion& region)
{
    unsigned char r, g, b;
    BOOST_FOREACH(const vector<ColourSegment>& line, region.getSegments()) {
        BOOST_FOREACH(const ColourSegment& seg, line) {
            getColourAsRGB(seg.getColour(), r, g, b);
            gui->addToLayer(id, QLineF(seg.getStart().x, seg.getStart().y, seg.getEnd().x, seg.getEnd().y), QColor(r, g, b));
        }
    }
}

void DataWrapper::debugPublish(DEBUG_ID id, NUImage const* const img)
{
    //for all images

    QImage qimg(img->getWidth(), img->getHeight(), QImage::Format_RGB888);
    unsigned char r, g, b;

    for(int y=0; y<img->getHeight(); y++) {
        for(int x=0; x<img->getWidth(); x++) {
            ColorModelConversions::fromYCbCrToRGB((*img)(x,y).y, (*img)(x,y).cb, (*img)(x,y).cr, r, g, b);
            qimg.setPixel(x, y, qRgb(r, g, b));
        }
    }
    gui->addToLayer(id, qimg, 1);
}

void DataWrapper::debugPublish(DEBUG_ID id, const vector<LSFittedLine>& data)
{
    QColor linecolour, pointcolour, endptcolour;

    switch(id) {
    case DBID_GOAL_LINES_START:
        linecolour = QColor(Qt::cyan);
        pointcolour = QColor(Qt::blue);
        endptcolour = QColor(Qt::green);
        break;
    case DBID_GOAL_LINES_END:
        linecolour = QColor(Qt::magenta);
        pointcolour = QColor(Qt::blue);
        endptcolour = QColor(Qt::green);
        break;
    default:
        errorlog << "DataWrapper::debugPublish - Called with invalid id" << endl;
        return;
    }

    BOOST_FOREACH(LSFittedLine l, data) {
        Point ep1, ep2;
        if(l.getEndPoints(ep1, ep2)) {
            gui->addToLayer(id, QLineF(ep1.x, ep1.y, ep2.x, ep2.y), linecolour);
            ep1 = l.projectOnto(ep1),
            ep2 = l.projectOnto(ep2);
            gui->addToLayer(id, QPointF(ep1.x, ep1.y), QPen(endptcolour, 3));
            gui->addToLayer(id, QPointF(ep2.x, ep2.y), QPen(endptcolour, 3));
            BOOST_FOREACH(Point p, l.getPoints()) {
                gui->addToLayer(id, QPointF(p.x, p.y), pointcolour);
            }
        }
        else {
            #if VISION_WRAPPER_VERBOSITY > 1
            debug << "DataWrapper::debugPublish called with invalid line: " << l << endl;
            #endif
        }
    }
}

void DataWrapper::debugPublish(DEBUG_ID id, const vector<Goal>& data)
{
    BOOST_FOREACH(const Goal& g, data) {
        QPolygonF p;
        const Quad& q = g.getQuad();
        p.append(QPointF(q.getBottomLeft().x, q.getBottomLeft().y));
        p.append(QPointF(q.getTopLeft().x, q.getTopLeft().y));
        p.append(QPointF(q.getTopRight().x, q.getTopRight().y));
        p.append(QPointF(q.getBottomRight().x, q.getBottomRight().y));
        p.append(QPointF(q.getBottomLeft().x, q.getBottomLeft().y));

        gui->addToLayer(id, Polygon(p, g.getID() != GOAL_U), QPen(Qt::yellow));
        gui->addToLayer(id, QPointF(g.getLocationPixels().x, g.getLocationPixels().y), QPen(QColor(Qt::blue), 3));
    }
}

void DataWrapper::plotCurve(string name, vector< Point > pts)
{
    QwtPlotCurve::CurveStyle style;
    MainWindow::PLOTWINDOW win;
    QColor colour;
    QwtSymbol symbol(QwtSymbol::NoSymbol);

    //hackalicious
    if(name.compare("Centre circle") == 0) {
        style = QwtPlotCurve::Lines;
        win = MainWindow::p1;
        colour = Qt::red;
    }
    else if(name.compare("Ground coords") == 0){
        style = QwtPlotCurve::NoCurve;
        win = MainWindow::p1;
        colour = Qt::white;
        symbol = QwtSymbol(QwtSymbol::Cross,
                           QBrush(Qt::green),
                           QPen(Qt::green),
                           QSize(3,3));
    }
    else if(name.compare("Corners") == 0) {
        //use a symbol and no line
        style = QwtPlotCurve::NoCurve;
        win = MainWindow::p1;
        colour = Qt::white;
        symbol = QwtSymbol(QwtSymbol::XCross,
                           QBrush(Qt::blue),
                           QPen(Qt::blue),
                           QSize(5,5));
    }
    else {
        win = MainWindow::p2;
        colour = Qt::black;
    }

    gui->setCurve(win, QString(name.c_str()), pts, colour, style, symbol);
}

void DataWrapper::plotLineSegments(string name, vector< Point > pts)
{
    gui->setDashedCurve(MainWindow::p1, QString(name.c_str()), pts, Qt::red, QwtPlotCurve::Lines, QwtSymbol(QwtSymbol::NoSymbol));
}

void DataWrapper::plotHistogram(string name, const Histogram1D& hist, Colour colour)
{
    QColor c;
    switch(colour) {
    case unclassified: c = Qt::black; break;
    case white: c = Qt::white; break;
    case green: c = Qt::green; break;
    case shadow_object: c = Qt::darkGray; break;
    case pink: c = Qt::magenta; break;
    case pink_orange: c = Qt::magenta; break;
    case orange: c = Qt::red; break;
    case yellow_orange: c = Qt::darkYellow; break;
    case yellow: c = Qt::yellow; break;
    case blue: c = Qt::blue; break;
    case shadow_blue: c = Qt::darkBlue; break;
    default: c = Qt::yellow; break;
    }


    if(name.compare("Before Merge2") == 0 || name.compare("After Merge2") == 0) {
        gui->setHistogram(MainWindow::p4, QString(name.c_str()), hist, c, QwtPlotHistogram::Columns);
    }
    else {
        gui->setHistogram(MainWindow::p3, QString(name.c_str()), hist, c, QwtPlotHistogram::Columns);
    }
}

bool DataWrapper::updateFrame()
{
    if(m_ok) {
        gui->clearLayers();

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
//                errorlog << "Image stream error - resetting: " << e.what() << endl;
//                imagestrm.clear() ;
//                imagestrm.seekg(0, ios::beg);
//                imagestrm >> m_current_image;
//                if(using_sensors) {
//                    sensorstrm.clear() ;
//                    sensorstrm.seekg(0, ios::beg);
//                }
                return false;
            }
            if(using_sensors) {
                try {
                    sensorstrm >> m_sensor_data;
                }
                catch(std::exception& e){
                    errorlog << "Sensor stream error: " << e.what() << endl;
                    return false;
                }
            }
            break;
        }

        //overwrite sensor horizon if using sensors
        vector<float> hor_data;
        if(using_sensors && m_sensor_data.getHorizon(hor_data)) {
            kinematics_horizon.setLine(hor_data.at(0), hor_data.at(1), hor_data.at(2));
        }

        //update kinematics snapshot
        if(using_sensors) {

            if(!m_sensor_data.getCameraHeight(m_camera_height))
                errorlog << "DataWrapperQt - updateFrame() - failed to get camera height from NUSensorsData" << endl;
            if(!m_sensor_data.getPosition(NUSensorsData::HeadPitch, m_head_pitch))
                errorlog << "DataWrapperQt - updateFrame() - failed to get head pitch from NUSensorsData" << endl;
            if(!m_sensor_data.getPosition(NUSensorsData::HeadYaw, m_head_yaw))
                errorlog << "DataWrapperQt - updateFrame() - failed to get head yaw from NUSensorsData" << endl;
            if(!m_sensor_data.get(NUSensorsData::Orientation, m_orientation))
                errorlog << "DataWrapperQt - updateFrame() - failed to get orientation from NUSensorsData" << endl;

            vector<float> left, right;
            if(m_sensor_data->get(NUSensorsData::LLegTransform, left) and m_sensor_data->get(NUSensorsData::RLegTransform, right))
            {
                m_neck_position = Kinematics::CalculateNeckPosition(Matrix4x4fromVector(left), Matrix4x4fromVector(right), m_calibration.m_calibration.m_neck_position_offset);
            }
            else
            {
                errorlog << "DataWrapperQt - updateFrame() - failed to get left or right leg transforms from NUSensorsData" << endl;
                // Default in case kinemtaics not available. Base height of darwin.
                m_neck_position = Vector3<double>(0.0, 0.0, 39.22);
            }
        }
        else {
            m_camera_height = m_head_pitch = m_head_yaw = 0;
            m_orientation = Vector3<float>(0,0,0);
            m_neck_position = Vector3<double>(0.0, 0.0, 39.22);
        }

        numFramesProcessed++;

        return m_current_image.getHeight() > 0 && m_current_image.getWidth() > 0;
    }
    return false;
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
