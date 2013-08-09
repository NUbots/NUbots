#ifndef MODULES_DARWINCAMERA_H
#define MODULES_DARWINCAMERA_H

#include <NUClear.h>
#include "messages/NUImage.h"
#include "messages/CameraSettings.h"

struct v4l2_buffer;

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

		int fd;                             //!< The file descriptor for the video device.
		void* mem[frameBufferCount];        //!< Frame buffer addresses.
		int memLength[frameBufferCount]; 	//!< The length of each frame buffer.
		struct v4l2_buffer* buf;            //!< Reusable parameter struct for some ioctl calls.
		struct v4l2_buffer* currentBuf; 	//!< The last dequeued frame buffer.
		double timeStamp,                   //!< Timestamp of the last captured image.
			   storedTimeStamp;             //!< Timestamp when the next image recording starts.
		messages::NUImage currentBufferedImage;
		messages::CameraSettings settings;

	    bool capturedNew();
		const unsigned char* getImage() const;
		double getTimeStamp() const;
	};
}
#endif

