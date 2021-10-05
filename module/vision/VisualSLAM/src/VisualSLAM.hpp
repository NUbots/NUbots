#ifndef MODULE_VISION_VISUALSLAM_HPP
#define MODULE_VISION_VISUALSLAM_HPP

// System.
#include <fstream>
#include <mutex>
// NUBots.
#include <nuclear>
// NUBots.
#include "ORB-SLAM/System.h"


namespace module::vision
{
    class VisualSLAM : public NUClear::Reactor
    {
        private:
            /// The configuration variables for this reactor
            struct {
            } config;

        public:
            /// @brief Called by the powerplant to build and setup the VisualSLAM reactor.
            explicit VisualSLAM(std::unique_ptr<NUClear::Environment> environment);

            // Load image dataset from local disk.
            std::vector<std::string> LoadImageDatabase(const std::string& databaseDirectory);

        private:
            //////////////////////////////////////////////
            // SLAM components.
            std::unique_ptr<ORB_SLAM2::System> slamSystem;
            std::vector<std::pair<std::string, std::string>> slamSettings;
            std::mutex mutex;
            //////////////////////////////////////////////

            //////////////////////////////////////////////
            // VisualSLAM.yaml settings for data logging.
            bool saveImages;
            bool saveDataLog;
            std::ofstream outputFileStream;
            //////////////////////////////////////////////

             //////////////////////////////////////////////
            // VisualSLAM.yaml settings for database.
            bool useImageDatabase;
            std::string imageDatabaseDirectory;
            // Local image database - vector of sorted filepaths.
            std::vector<std::string> imageDataset;
            //////////////////////////////////////////////
    };
}

#endif  // MODULE_VISION_VISUALSLAM_HPP
