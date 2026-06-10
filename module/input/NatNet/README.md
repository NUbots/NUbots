# NatNet

## Description

Receives OptiTrack NatNet motion-capture packets and converts them into `message::input::MotionCapture`.

The module listens to NatNet data broadcasts, pings the discovered server, requests model definitions, and then parses
frame packets into NUbots mocap messages.

In the Motive software, ensure the advanced streaming settings are set to the following:

#### Motive Streaming Settings

**Broadcast Frame Data:** On
**Local Interface:** <motive_device_ip> # Set to the ip of the device running motive, sometimes toggling on/off needed

#### Marker Options

- **Labeled Markers:** Off
- **Unlabeled Markers:** Off
- **Asset Markers:** Off

#### Rigid Body Options

- **Rigid Bodies:** On
- **Skeletons:** Off
- **Skeleton Coordinates:** Global
- **Skeleton As Rigid Bodies:** Off
- **Bone Naming Convention:** Motive
- **Up Axis:** Z Up

#### Miscellaneous

- **Remote Trigger:** Off
- **Transmission Type:** Multicast
- **Subject Prefix:**
- **Visual3D Compatible:** Off
- **Scale:** 1

#### Networking

- **Command Port:** 1510
- **Data Port:** 1511
- **Multicast Interface:** `239.255.42.99`
- **Multicast as Broadcast:** On
- **Socket Size:** 1000000

## Usage

Install `module::input::NatNet` and configure `NatNet.yaml`.

Default configuration values:

- `multicast_address: 239.255.42.99`
- `command_port: 1510`
- `data_port: 1511`
- `dump_packets: false`
- `max_delay_samples: 100`

At runtime the module:

- Binds UDP sockets for NatNet command and data traffic
- Detects the active NatNet server from incoming broadcast packets
- Sends a NatNet ping and reads the remote protocol version
- Requests model definitions when needed
- Estimates and logs network delay statistics

## Consumes

- `extension::Configuration` from `NatNet.yaml`
- `NUClear::UDP::Packet` (NatNet command traffic)
- `NUClear::UDP::Packet` via `UDP::Broadcast` on `data_port` (NatNet frame traffic)

## Emits

- `message::input::MotionCapture`

## Dependencies

- An OptiTrack motion capture system sending packets over the network using the NatNet protocol.
