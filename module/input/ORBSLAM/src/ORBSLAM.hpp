#ifndef MODULE_INPUT_ORBSLAM_HPP
#define MODULE_INPUT_ORBSLAM_HPP

#include <nuclear>
#include <opencv2/core/core.hpp>

#include "utility/input/ORBSLAM3/src/System.h"

namespace module::input {

    class ORBSLAM : public NUClear::Reactor {
    private:
        /// @brief Stores configuration values
        struct Config {
        } cfg;

        //////////////////////////////////////////////
        // SLAM components.
        // const std::string vocabularyFile = "";
        // const std::string settingsFile   = "";
        // ORB_SLAM3::System slamSystem(vocabularyFile, settingsFile, ORB_SLAM3::System::eSensor::MONOCULAR, false);
        std::unique_ptr<ORB_SLAM3::System> slamSystem;
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

    public:
        /// @brief Called by the powerplant to build and setup the ORBSLAM reactor.
        explicit ORBSLAM(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::input

#endif  // MODULE_INPUT_ORBSLAM_HPP
