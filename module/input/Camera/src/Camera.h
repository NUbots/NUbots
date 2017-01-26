#ifndef MODULE_INPUT_CAMERA_H
#define MODULE_INPUT_CAMERA_H

#include <nuclear>

#include "extension/Configuration.h"

#include "message/input/Image.h"
#include "message/input/CameraParameters.h"

#include "utility/vision/fourcc.h"

#include "V4L2Camera.h"

namespace module
{
	namespace input
	{

	    class Camera : public NUClear::Reactor {

	    public:
	        /// @brief Called by the powerplant to build and setup the Camera reactor.
	        explicit Camera(std::unique_ptr<NUClear::Environment> environment);

	    private:
	    	V4L2Camera initiateV4L2Camera(const ::extension::Configuration& config);
			void ShutdownV4L2Camera();

	        ReactionHandle V4L2FrameRateHandle; 
	        ReactionHandle V4L2SettingsHandle; 

	        std::map<std::string, V4L2Camera> V4L2Cameras;

	        static uint cameraCount;
	    };

	    uint Camera::cameraCount = 0;
	}
}

#endif  // MODULE_INPUT_CAMERA_H
