#ifndef MODULES_DARWINCAMERA_H
#define MODULES_DARWINCAMERA_H

#include <NUClear.h>

// Forward declarations
struct v4l2_buffer;
namespace messages {
	class NUImage;
	class CameraSettings;
}

namespace modules {

    class DarwinCamera : public NUClear::Reactor {
    public:
        DarwinCamera(NUClear::PowerPlant& plant);
		~DarwinCamera();

		messages::NUImage* grabNewImage();
		void applySettings(const messages::CameraSettings& newset);
		void forceApplySettings(const messages::CameraSettings& newset);

	private:
		//void loadCameraOffset();

		enum
		{
			frameBufferCount = 1, //!< Number of available frame buffers.
			WIDTH = 640,
			HEIGHT = 480,
			SIZE = WIDTH * HEIGHT * 2,
			FRAMERATE = 30
		};

		bool applySetting(unsigned int settingID, int value);
		int readSetting(unsigned int id);
		void initialiseCamera();
		void readCameraSettings();
		void openCameraDevice(const std::string& device_name);
		void setStreaming(bool streaming_on);

		int fd;                                             //!< The file descriptor for the video device.
		void* mem[frameBufferCount];                        //!< Frame buffer addresses.
		int memLength[frameBufferCount];                   	//!< The length of each frame buffer.
		std::unique_ptr<struct v4l2_buffer> buf;            //!< Reusable parameter struct for some ioctl calls.
		bool bufQueued;                                     //!< Whether 'buf' points to a currently queued buffer
		double timeStamp;                                   //!< Timestamp of the last captured image.
		double storedTimeStamp;                             //!< Timestamp when the next image recording starts.
		std::unique_ptr<messages::CameraSettings> settings; //!< The current camera settings

	    bool capturedNew();
		const unsigned char* getImage() const;
		double getTimeStamp() const;
	};
}
#endif

