VisualMesh
==========

## Description

Runs the Visual Mesh over a given image and emits the result.

## Usage

This module requires on the following configuration files to be set:
- `data/config/networks` should include a `yaml` file with Visual Mesh network configuration, which can be obtained from tools in the [Visual Mesh](https://github.com/Fastcode/VisualMesh) repository after training. A guide on updating this can be found on [NUbook](https://nubook.nubots.net/guides/main/maintaining-subsystems#vision).
- `data/config/VisualMesh.yaml` should include information on what cameras and network configuration to use.

Once these are set appropriately, this module will trigger on any `Image`s emitted in the system and run the Visual Mesh as per the configuration.

## Consumes

- `message::input::Image` the image to process

## Emits

- `message::vision::VisualMesh` the mesh results for the given image

## Dependencies

- [Visual Mesh](https://github.com/Fastcode/VisualMesh)
