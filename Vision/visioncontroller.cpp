#include "visioncontroller.h"
#include "debug.h"
#include <time.h>

//#include "Infrastructure/Jobs/JobList.h"

#include "Tools/Profiling/Profiler.h"

#include "Vision/VisionTools/lookuptable.h"
#include "Vision/Modules/greenhorizonch.h"
#include "Vision/Modules/obstacledetectionch.h"
#include "Vision/Modules/scanlines.h"
#include "Vision/visionconstants.h"
#include "Vision/Modules/LineDetectionAlgorithms/linedetectorsam.h"
#include "Vision/Modules/LineDetectionAlgorithms/linedetectorransac.h"
#include "Vision/Modules/GoalDetectionAlgorithms/goaldetectorhistogram.h"
#include "Vision/Modules/GoalDetectionAlgorithms/goaldetectorransacedges.h"
#include "Vision/Modules/GoalDetectionAlgorithms/goaldetectorransaccentres.h"

#include <boost/foreach.hpp>
#include <limits>

VisionController::VisionController() : m_corner_detector(0.1), m_circle_detector(0.25, 50, 100, 8.0, 3)
{
    m_data_wrapper = DataWrapper::getInstance();
    m_blackboard = VisionBlackboard::getInstance();
    //m_line_detector_sam = new LineDetectorSAM();
    //m_goal_detector_hist = new GoalDetectorHistogram();
    m_line_detector_ransac = new LineDetectorRANSAC();
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
//    delete m_line_detector_sam;
//    delete m_goal_detector_hist;
    delete m_goal_detector_ransac_edges;
}

int VisionController::runFrame(bool lookForBall, bool lookForGoals, bool lookForFieldPoints, bool lookForObstacles)
{
#ifdef VISION_PROFILER_ON
    Profiler prof("Vision");
    prof.start();
#endif

    m_data_wrapper = DataWrapper::getInstance();
#if VISION_CONTROLLER_VERBOSITY > 1
    debug << "VisionController::runFrame()" << std::endl;
    debug << "\tBegin"
#endif
    //force blackboard to update from wrapper
    m_blackboard->update();
#if VISION_CONTROLLER_VERBOSITY > 1
    debug << "\tVisionBlackboard updated" << std::endl;
#endif

#ifdef VISION_PROFILER_ON
    prof.split("Update");
#endif

    //! HORIZON

    GreenHorizonCH::calculateHorizon();
#if VISION_CONTROLLER_VERBOSITY > 2
    debug << "\tcalculateHorizon done" << std::endl;
#endif

#ifdef VISION_PROFILER_ON
    prof.split("Green Horizon");
#endif

    //! PRE-DETECTION PROCESSING

    ScanLines::generateScanLines();
#if VISION_CONTROLLER_VERBOSITY > 2
    debug << "\tgenerateScanLines done" << std::endl;
#endif

    ScanLines::classifyHorizontalScanLines();
#if VISION_CONTROLLER_VERBOSITY > 2
    debug << "\tclassifyHorizontalScanLines done" << std::endl;
#endif

    ScanLines::classifyVerticalScanLines();
#if VISION_CONTROLLER_VERBOSITY > 2
    debug << "\tclassifyVerticalScanLines done" << std::endl;
#endif

#ifdef VISION_PROFILER_ON
    prof.split("Classify Scanlines");
#endif

    m_segment_filter.run();
#if VISION_CONTROLLER_VERBOSITY > 2
    debug << "\tsegment filter done" << std::endl;
#endif


#ifdef VISION_PROFILER_ON
    prof.split("Segment Filters");
#endif

    //! DETECTION MODULES
    std::vector<Goal> ransac_goals_edges;

    if(lookForGoals) {
//       std::vector<Goal> hist_goals = m_goal_detector_hist->run();   // histogram method
        ransac_goals_edges = m_goal_detector_ransac_edges->run();  //ransac method
    }
    else {
        #if VISION_CONTROLLER_VERBOSITY > 2
            debug << "\tnot looking for goals" << std::endl;
        #endif
    }

    #ifdef VISION_PROFILER_ON
    prof.split("Goals");
    #endif

    #if VISION_CONTROLLER_VERBOSITY > 2
    debug << "\tgoal detection done" << std::endl;
    #endif

    if(lookForFieldPoints) {
        // Edit here to change whether centre circles, lines or corners are found
        //      (note lines cannot be published yet)
        m_field_point_detector->run(true, true, true);
    }
    else {
        #if VISION_CONTROLLER_VERBOSITY > 2
            debug << "\tnot looking for lines, corners or the centre circle" << std::endl;
        #endif
    }

    #ifdef VISION_PROFILER_ON
    prof.split("Field Points");
    #endif

    #if VISION_CONTROLLER_VERBOSITY > 2
        debug << "\tcentre circle, line and corner detection done" << std::endl;
    #endif

    //find balls and publish to BB
    if(lookForBall) {
        m_blackboard->addBalls(m_ball_detector.run());
        #if VISION_CONTROLLER_VERBOSITY > 2
            debug << "\tball detection done" << std::endl;
        #endif
        #ifdef VISION_PROFILER_ON
        prof.split("Ball");
        #endif
    }
    else {
        #if VISION_CONTROLLER_VERBOSITY > 2
            debug << "\tnot looking for ball" << std::endl;
        #endif
    }

    //OBSTACLES
    std::vector<Obstacle> obstacles;
    if(lookForObstacles)
    {
        obstacles = ObstacleDetectionCH::run();
        m_blackboard->addObstacles(obstacles);
        #if VISION_CONTROLLER_VERBOSITY > 2
        debug << "\tdetectObstacles done" << std::endl;
        #endif
    }
    else
    {
        #if VISION_CONTROLLER_VERBOSITY > 2
        debug << "\tnot looking for obstacles" << std::endl;
        #endif
    }

    // ADD IN LABELLING OF GOALS BASED ON KEEPER COLOUR

    #ifdef VISION_PROFILER_ON
    prof.split("Obstacles");
    #endif

    //m_goal_detector_ransac_edges->relabel(ransac_goals_edges, obstacles);

    m_blackboard->addGoals(ransac_goals_edges);

    // publishing
    //force blackboard to publish results through wrapper
    m_blackboard->publish();
    #if VISION_CONTROLLER_VERBOSITY > 1
    debug << "\tResults published" << std::endl;
    #endif

    #ifdef VISION_PROFILER_ON
    prof.split("Publishing");
    #endif

    //publish debug information as well

    m_blackboard->debugPublish();   //only debug publish if some verbosity is on

    #if VISION_CONTROLLER_VERBOSITY > 1
    debug << "\tDebugging info published" << std::endl;
    debug << "\tFinish" << std::endl;
    #endif

    #ifdef VISION_PROFILER_ON
    prof.split("Debug publishing");
    prof.stop();
    m_profiling_stream << prof << std::endl;
    #endif

    return 0;
}
