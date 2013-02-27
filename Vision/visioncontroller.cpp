#include "visioncontroller.h"
#include "debug.h"
#include "debugverbosityvision.h"
#include <time.h>

//#include "Infrastructure/Jobs/JobList.h"

#include "Tools/Profiling/Profiler.h"

#include "Vision/VisionTools/lookuptable.h"
#include "Vision/Modules/greenhorizonch.h"
#include "Vision/Modules/objectdetectionch.h"
#include "Vision/Modules/scanlines.h"
#include "Vision/visionconstants.h"
#include "Vision/Modules/LineDetectionAlgorithms/linedetectorsam.h"
#include "Vision/Modules/LineDetectionAlgorithms/linedetectorransac.h"
#include "Vision/Modules/GoalDetectionAlgorithms/goaldetectorhistogram.h"
#include "Vision/Modules/GoalDetectionAlgorithms/goaldetectorransacedges.h"
#include "Vision/Modules/GoalDetectionAlgorithms/goaldetectorransaccentres.h"

#include <boost/foreach.hpp>
#include <limits>

VisionController* VisionController::instance = 0;

VisionController::VisionController() : m_corner_detector(0.1), m_circle_detector(0.25)
{
    m_data_wrapper = DataWrapper::getInstance();
    m_blackboard = VisionBlackboard::getInstance();
    m_line_detector_ransac = new LineDetectorRANSAC();
    m_line_detector_sam = new LineDetectorSAM();
    m_goal_detector_hist = new GoalDetectorHistogram();
    m_goal_detector_ransac_edges = new GoalDetectorRANSACEdges();
    m_goal_detector_ransac_centres = new GoalDetectorRANSACCentres();

    //requires other detectors
    m_field_point_detector = new FieldPointDetector(m_line_detector_ransac, &m_circle_detector, &m_corner_detector);
}

VisionController::~VisionController()
{
    delete m_line_detector_ransac;
    delete m_line_detector_sam;
    delete m_goal_detector_hist;
    delete m_goal_detector_ransac_edges;
    delete m_goal_detector_ransac_centres;
}

VisionController* VisionController::getInstance()
{
    if(instance == 0)
        instance = new VisionController();
    return instance;
}

int VisionController::runFrame(bool lookForBall, bool lookForLandmarks)
{
    m_data_wrapper = DataWrapper::getInstance();
    #if VISION_CONTROLLER_VERBOSITY > 1
        debug << "VisionController::runFrame() - Begin" << endl;
    #endif
    //force blackboard to update from wrapper
    m_blackboard->update();
    #if VISION_CONTROLLER_VERBOSITY > 1
        debug << "VisionController::runFrame() - VisionBlackboard updated" << endl;
    #endif

    //! HORIZON

    GreenHorizonCH::calculateHorizon();
    #if VISION_CONTROLLER_VERBOSITY > 2
        debug << "VisionController::runFrame() - calculateHorizon done" << endl;
    #endif

    //! PRE-DETECTION PROCESSING

    ScanLines::generateScanLines();
    #if VISION_CONTROLLER_VERBOSITY > 2
        debug << "VisionController::runFrame() - generateScanLines done" << endl;
    #endif

    ScanLines::classifyHorizontalScanLines();
    #if VISION_CONTROLLER_VERBOSITY > 2
        debug << "VisionController::runFrame() - classifyHorizontalScanLines done" << endl;
    #endif

    ScanLines::classifyVerticalScanLines();
    #if VISION_CONTROLLER_VERBOSITY > 2
        debug << "VisionController::runFrame() - classifyVerticalScanLines done" << endl;
    #endif

    m_segment_filter.run();
    #if VISION_CONTROLLER_VERBOSITY > 2
        debug << "VisionController::runFrame() - segment filter done" << endl;
    #endif

    //! DETECTION MODULES

    if(lookForLandmarks) {
        vector<Goal> hist_goals = m_goal_detector_hist->run();   //POSTS

        //testing ransac for goals
        vector<Goal> ransac_goals_edges = m_goal_detector_ransac_edges->run();
        //vector<Goal> ransac_goals_centres = m_goal_detector_ransac_centres->run();

        m_data_wrapper->debugPublish(0, hist_goals);
        m_data_wrapper->debugPublish(1, ransac_goals_edges);
        //m_data_wrapper->debugPublish(2, ransac_goals_centres);
//        m_blackboard->addGoals(ransac_goals);

//        if(ransac_goals.size() == 2)
//            m_blackboard->addGoals(ransac_goals);
//        else
//            m_blackboard->addGoals(hist_goals);

        #if VISION_CONTROLLER_VERBOSITY > 2
            debug << "VisionController::runFrame() - goal detection done" << endl;
        #endif

        //PROFILING
        #ifdef VISION_PROFILER_ON
        static Profiler prof("field points");
        static ofstream profiling("Vision Profiling", ios_base::app);
        prof.start();
        #endif
        //FIELD POINTS

        m_field_point_detector->run();

        #ifdef VISION_PROFILER_ON
        prof.stop();
        profiling << prof << endl;
        #endif
        #if VISION_CONTROLLER_VERBOSITY > 2
            debug << "VisionController::runFrame() - centre circle, line and corner detection done" << endl;
        #endif
    }
    else {
        #if VISION_CONTROLLER_VERBOSITY > 2
            debug << "VisionController::runFrame() - not looking for landmarks" << endl;
        #endif
    }

    //find balls and publish to BB
    if(lookForBall) {
        m_blackboard->addBalls(m_ball_detector.run());
        #if VISION_CONTROLLER_VERBOSITY > 2
            debug << "VisionController::runFrame() - ball detection done" << endl;
        #endif
    }
    else {
        #if VISION_CONTROLLER_VERBOSITY > 2
            debug << "VisionController::runFrame() - not looking for ball" << endl;
        #endif
    }

    ObjectDetectionCH::detectObjects(); //OBSTACLES
    #if VISION_CONTROLLER_VERBOSITY > 2
        debug << "VisionController::runFrame() - detectObjects done" << endl;
    #endif

    //force blackboard to publish results through wrapper
    m_blackboard->publish();
    #if VISION_CONTROLLER_VERBOSITY > 1
        debug << "VisionController::runFrame() - Results published" << endl;
    #endif
    //publish debug information as well
    m_blackboard->debugPublish();
    #if VISION_CONTROLLER_VERBOSITY > 1
        debug << "VisionController::runFrame() - Debugging info published" << endl;
    #endif
    
    #if VISION_CONTROLLER_VERBOSITY > 1
        debug << "VisionController::runFrame() - Finish" << endl;
    #endif
    return 0;
}

//int VisionController::run()
//{
//    char c=0;

//    while(c!=27) {
//        DataWrapper::getInstance()->updateFrame();
//        instance->runFrame();
//        c = waitKey();
//    }

//    return 0;
//}

CameraSettings VisionController::getCurrentCameraSettings() const
{
    return m_blackboard->getCameraSettings();
}
