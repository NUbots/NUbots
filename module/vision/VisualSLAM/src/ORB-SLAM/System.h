#ifndef SYSTEM_H
#define SYSTEM_H

#include <thread>
#include <opencv2/core/core.hpp>
#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"

namespace ORB_SLAM2
{
    class Viewer;
    class FrameDrawer;
    class Map;
    class Tracking;
    class LocalMapping;
    class LoopClosing;

    class System
    {
    public:
        // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
        System(const std::vector<std::pair<std::string, std::string>>& parameters, const bool bUseViewer);

        // Process the given monocular frame.
        // Input images: grayscale (CV_8U) only - must be converted before passing.
        // Returns the camera pose (empty if tracking fails).
        cv::Mat TrackMonocular(const cv::Mat& im, const double &timestamp);

        // This stops local mapping thread (map building) and performs only camera tracking.
        void ActivateLocalizationMode();
        // This resumes local mapping thread and performs SLAM again.
        void DeactivateLocalizationMode();

        // Returns true if there have been a big map change (loop closure, global BA)
        // since last call to this function
        bool MapChanged();

        // Reset the system (clear map)
        void Reset();

        // All threads will be requested to finish.
        // It waits until all threads have finished.
        // This function must be called before saving the trajectory.
        void Shutdown();

        // Save keyframe poses in the TUM RGB-D dataset format.
        // This method works for all sensor input.
        // Call first Shutdown()
        // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
        void SaveKeyFrameTrajectoryTUM(const string& filename);

        // Information from most recent processed frame
        // You can call this right after TrackMonocular.
        int GetTrackingState();
        std::vector<MapPoint*> GetTrackedMapPoints();
        std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

    private:

        // ORB vocabulary used for place recognition and feature matching.
        ORBVocabulary* mpVocabulary;

        // KeyFrame database for place recognition (re-localization and loop detection).
        KeyFrameDatabase* mpKeyFrameDatabase;

        // Map structure that stores the pointers to all KeyFrames and MapPoints.
        Map* mpMap;

        // Tracker. It receives a frame and computes the associated camera pose.
        // It also decides when to insert a new keyframe, create some new MapPoints and
        // performs re-localization if tracking fails.
        Tracking* mpTracker;

        // Local Mapper. It manages the local map and performs local bundle adjustment.
        LocalMapping* mpLocalMapper;

        // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
        // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
        LoopClosing* mpLoopCloser;

        // The viewer draws the map and the current camera pose. It uses Pangolin.
        Viewer* mpViewer;
        FrameDrawer* mpFrameDrawer;
        MapDrawer* mpMapDrawer;

        // System threads: Local Mapping, Loop Closing, Viewer.
        // The Tracking thread "lives" in the main execution thread that creates the System object.
        std::thread* mptLocalMapping;
        std::thread* mptLoopClosing;
        std::thread* mptViewer;

        // Change mode flags
        std::mutex mMutexMode;
        bool mbActivateLocalizationMode;
        bool mbDeactivateLocalizationMode;

        // Tracking state
        std::mutex mMutexState;
        int mTrackingState;
        std::vector<MapPoint*> mTrackedMapPoints;
        std::vector<cv::KeyPoint> mTrackedKeyPointsUn;

        // Reset flag
        std::mutex mMutexReset;
        bool mbReset;
    };

}// namespace ORB_SLAM

#endif // SYSTEM_H
