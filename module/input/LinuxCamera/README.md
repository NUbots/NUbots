Linux Camera Streamer
=====================

## Description

Streams video from a camera device, creating an image of each frame.

## Usage

The camera is initialised on startup using the settings from the configuration
file (see below). Once this has occurred, the module will emit a
`message::Image` for each frame of video it receives. To access the value
of a pixel in an `Image`, use its `(size_t x, size_t y)` operator.

Frames are retrieved in YUYV format with 4:2:2 chroma subsampling. This means
that for every two pixels horizontally of luma (brightness) there is one pixel
of chroma (colour) shared between them. For better performance and to simplify
its use, the `Image` class internally treats its image as if it were half the
width and half the height of the raw frame from the camera effectively ignoring
the subsampling. As such, If the camera is set to a resolution of 640x480 it
will emit 320x240 `Image`s.

Whenever a new configuration is loaded, the camera settings are re-applied. If
the resolution has changed the camera device must be re-created, this can take
a second or two in which time no images will be captured.

It is not possible to set the frame rate in the configuration file. It needs
to be known at compile time. The currently chosen value is 30fps.

## Consumes

* `message::Configuration<LinuxCameraStreamer>` from the config system to set
  camera parameters.

## Emits

* `message::Image` for each frame of video retrieved from the camera.

## Configuration

This module's configuration is stored in LinuxCameraStreamer.yaml. All of the
following settings must be present.

* "deviceID": the name of the camera device, e.g. /dev/video0
* "autoWhiteBalance": 0 = off, 1 = on
* "whiteBalanceTemperature": colour temperature in Kelvin, 0 to 10,000
* "brightness": black level, 0 to 255
* "contrast": luma gain, 0 to 255
* "saturation": chroma gain, 0 to 255
* "gain": ISO sensitivity, 0 to 255
* "autoExposure": 0 = auto exposure time and iris apterture, 1 = manual, 2 =
  manual exposure auto iris, 3 = auto exposure manual iris
* "autoExposurePriority": whether to dynamically adjust framerate in auto
  exposure mode, 0 = off, 1 = on
* "absoluteExposure": exposure time, measured in 1/10,000 second increments, 0
  to 10,000
* "powerLineFrequency": filter for flicker caused by power line frequency (0 =
  off, 1 = 50Hz, 2 = 60Hz)
* "sharpness": adjust camera sharpness filters, 0 to 255
* "imageWidth": camera horizontal resolution
* "imageHeight": camera vertical resolution

## Dependencies

* The ConfigSystem module is required to load all settings.
* Camera I/O is performed using the V4L2 (Video for Linux 2) API.

