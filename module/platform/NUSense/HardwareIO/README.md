# NUSense HardwareIO

## Description

This module is responsible for communicating with the NUgus robot's NUSense controller.

## Usage

NUSense handles reading and writing to the Dynamixel devices, and reads its own accelerometer and gyroscope (IMU). This module sends target requests for Dynamixel devices and receive the NUSense data on Dynamixel device states and IMU state.

This module is built when the subcontroller CMake flag is set to `NUSense`. Using `platform::${SUBCONTROLLER}::HardwareIO` will use this module if the subcontroller CMake flag is set to `NUSense`.

## Consumes

- `message::actuation::ServoTarget` requesting a single servo command be performed
- `message::actuation::ServoTargets` requesting a batch of servo commands be performed

## Emits

- `message::platform::NUSense` with a `DIRECT` scope so that the message is picked up locally in this module to be converted to RawSensors.
- `message::input::RawSensors` containing the current NUgus sensor data from the NUSense device.

## Dependencies

## Notes

There are 2 ways to generate nanopb messages for NUSense. The generated files must then go into the NUSense NUController repo before building the binaries onto the board. This will be a temporary solution until a `./b install` pipeline for NUSense is set up. The guides below will assume that the OS is some linux based distro.

### b script

For example, if a user wants to generate nanopb messages and specify that certain properties of messages must have max counts, the command below should put the generated files in the `recordings` directory.

```bash
./b nusense generate_nanopb_message recordings/ServoTarget.proto \
    --selections ServoTargets:targets:max_count:20 \
                 SubcontrollerServoTargets:targets:max_count:20
```

The `recordings` directory is convenient as it is symlinked to the docker container. It is also imperative to specify maximum counts for repeated fields as encoding a message without it takes significantly longer and introduces a lot of overhead that is undesired in terms of performance.

### Manual (and lazy)

This way achieves the same outcome as the first one, but is objectively more tedious

```bash
cd ~
git clone https://github.com/nanopb/nanopb.git
cd nanopb/generator
cp ~/NUbots/shared/message/actuation/ServoTarget.proto .
```

Then parsing the `.options` file as required

```bash
cat << EOF > ServoTarget.options
message.actuation.ServoTargets.targets max_count:20
message.actuation.SubcontrollerServoTargets.targets max_count:20
EOF
```

Assuming that the guide was followed, the current working directory should be in `~/nanopb/generator`. To generate the files, run

```bash
python3 nanopb_generator.py ServoTarget.proto
```

The generated files should be in the same directory. Put these within `NUcontroller/NUSense/Core/Src/usb/protobuf` and replace the old files as required. The same logic goes with new messages.

See https://jpa.kapsi.fi/nanopb/docs/reference.html#defining-the-options-in-a-.options-file for more available options.
