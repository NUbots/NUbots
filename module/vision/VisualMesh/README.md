VisualMesh
==========

## Description

Runs a Visual Mesh over a given image and emits the result.

## Usage

This module requires the following configuration files to be set:
- `data/config/networks` should include a `yaml` file with Visual Mesh network configuration, which can be obtained from tools in the [Visual Mesh](https://github.com/Fastcode/VisualMesh) repository after training. A guide on updating this can be found on [NUbook](https://nubook.nubots.net/guides/main/maintaining-subsystems#vision).
- `data/config/VisualMesh.yaml` should include information on what cameras and network configuration to use.

Once these are set appropriately, this module will trigger on any `message::input::Image` emitted in the system.

The module uses the `./visualmesh/VisualMeshRunner` to run the Visual Mesh.

`VisualMeshRunner` gets a `LoadedModel` from `./visualmesh/load_model`, which uses the network configuration specified in `./data/config/VisualMesh.yaml` to create a network model through the `VisualMesh` library. This `LoadedModel` is the Visual Mesh model that the `Image` will be run on, and is only updated if the configuration changes.

This `LoadedModel` is then used in the `VisualMeshRunner` anytime an `Image` needs to be processed. The `VisualMeshRunner` takes the `Image` and `Hcw` [(world to camera homogenous transformation)](https://nubook.nubots.net/system/foundations/mathematics#homogeneous-transformations), gets the lens and height data from these and passes it through to the `VisualMesh` itself to get the results. The `VisualMeshRunner` then returns the results and the module emits those results as a `message::vision::VisualMesh` message. This will rerun every time a new `Image` message is received.

Note that this module does not receive a `message::input::Sensors` message. It uses the `Hcw` matrix from the `Image` message, as this will capture the robot's orientation at the time the `Image` is first processed to reduce error.

## Consumes

- `message::input::Image` the image to run the Visual Mesh on.

## Emits

- `message::vision::VisualMesh` the mesh results for the given image. The result is in the form of points on the field, arranged into multiple matrices that represent the coordinate values, neighbour points and classification for each point.

## Dependencies

- [Visual Mesh](https://github.com/Fastcode/VisualMesh)
