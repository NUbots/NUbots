# Camera

Reads cameras that support GigE Vision or U3V (USB3 Vision) protocols.
This module interfaces with industrial cameras to capture images and emit them with appropriate timestamp synchronization and lens information.

The cameras must be properly connected via either Ethernet (for GigE) or USB3 (for U3V) ports with sufficient bandwidth.
Gigabit or faster Ethernet is required for GigE Vision devices.
For optimal performance, ensure the network interface is configured with jumbo frames enabled (MTU 9000).

For cameras that support IEEE 1588 Precision Time Protocol (PTP), configure the network to have a PTP master clock running.
The module will automatically configure supported cameras to operate in PTP slave mode for high-precision time synchronization.
For cameras without PTP support, the module will fall back to either timestamp latching (moderately accurate) or live estimation (least accurate) methods.
The time synchronization method is automatically selected based on camera capabilities.

For some camera models, an external power supply may be needed if the PoE or USB power is insufficient.

## Consumes

- `message.input.Sensors` Used to transform camera coordinates relative to the platform.

  The sensor data allows the module to track the camera's position and orientation in the world reference frame.
  When new sensor data arrives, the module updates its internal transformation matrices to ensure images have accurate spatial context.

## Emits

- `message.input.Image` Images captured from the camera with metadata.

  When a frame is captured from a camera, the module processes it and emits this message.
  The message provides all necessary information for downstream modules to interpret and process camera images with proper spatial and temporal context.
  The module applies appropriate lens corrections and timestamps based on camera configuration.

## Dependencies

- Aravis - An open-source library (glib-based) that provides access to industrial cameras supporting GenICam and GigE Vision protocols.
  Required for communicating with and controlling the camera devices.
