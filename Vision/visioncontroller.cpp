#include "visioncontroller.h"
#include "debug.h"
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

VisionController::VisionController() : m_corner_detector(0.1), m_circle_detector(0.25, 50, 100, 8.0, 3)
{
    m_data_wrapper = DataWrapper::getInstance();
    m_blackboard = VisionBlackboard::getInstance();
    m_line_detector_ransac = new LineDetectorRANSAC();
    m_line_detector_sam = new LineDetectorSAM();
    m_goal_detector_hist = new GoalDetectorHistogram();
    m_goal_detector_ransac_edges = new GoalDetectorRANSACEdges();

    //requires other detectors
    m_field_point_detector = new FieldPointDetector(m_line_detector_ransac, &m_circle_detector, &m_corner_detector);

#ifdef VISION_PROFILER_ON
    m_profiling_stream.open("VisionProfiling.txt");
#endif
}

VisionController::~VisionController()
{
#ifdef VISION_PROFILER_ON
    m_profiling_stream.close();
#endif
    delete m_line_detector_ransac;
    delete m_line_detector_sam;
    delete m_goal_detector_hist;
    delete m_goal_detector_ransac_edges;
}

VisionController* VisionController::getInstance()
{
    if(instance == 0)
        instance = new VisionController();
    return instance;
}

int VisionController::runFrame(bool lookForBall, bool lookForLandmarks)
{
#ifdef VISION_PROFILER_ON
    Profiler prof("Vision");
    prof.start();
#endif

    m_data_wrapper = DataWrapper::getInstance();
#if VISION_CONTROLLER_VERBOSITY > 1
    debug << "VisionController::runFrame() - Begin" << endl;
#endif
    //force blackboard to update from wrapper
    m_blackboard->update();
#if VISION_CONTROLLER_VERBOSITY > 1
    debug << "VisionController::runFrame() - VisionBlackboard updated" << endl;
#endif

#ifdef VISION_PROFILER_ON
    prof.split("Update");
#endif

    //! HORIZON

    GreenHorizonCH::calculateHorizon();
#if VISION_CONTROLLER_VERBOSITY > 2
    debug << "VisionController::runFrame() - calculateHorizon done" << endl;
#endif

#ifdef VISION_PROFILER_ON
    prof.split("Green Horizon");
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

#ifdef VISION_PROFILER_ON
    prof.split("Classify Scanlines");
#endif

    m_segment_filter.run();
#if VISION_CONTROLLER_VERBOSITY > 2
    debug << "VisionController::runFrame() - segment filter done" << endl;
#endif


#ifdef VISION_PROFILER_ON
    prof.split("Segment Filters");
#endif

    //! DETECTION MODULES

    if(lookForLandmarks) {
 //       vector<Goal> hist_goals = m_goal_detector_hist->run();   //POSTS

        //testing ransac for goals
        vector<Goal> ransac_goals_edges = m_goal_detector_ransac_edges->run();

//        m_data_wrapper->debugPublish(0, hist_goals);
        m_data_wrapper->debugPublish(1, ransac_goals_edges);
        m_blackboard->addGoals(ransac_goals_edges);

        #ifdef VISION_PROFILER_ON
        prof.split("Goals");
        #endif

        #if VISION_CONTROLLER_VERBOSITY > 2
        debug << "VisionController::runFrame() - goal detection done" << endl;
        #endif

        m_field_point_detector->run(true, true, true);

        #ifdef VISION_PROFILER_ON
        prof.split("Field Points");
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
        #ifdef VISION_PROFILER_ON
        prof.split("Ball");
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

    #ifdef VISION_PROFILER_ON
    prof.split("Obstacles");
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

    #ifdef VISION_PROFILER_ON
    prof.split("Publishing");
    #endif

    #ifdef VISION_PROFILER_ON
    prof.stop();
    m_profiling_stream << prof << endl;
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
