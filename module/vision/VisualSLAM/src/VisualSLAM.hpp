#ifndef MODULE_VISION_VISUALSLAM_HPP
#define MODULE_VISION_VISUALSLAM_HPP

#include <nuclear>


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

    };
}

#endif  // MODULE_VISION_VISUALSLAM_HPP
