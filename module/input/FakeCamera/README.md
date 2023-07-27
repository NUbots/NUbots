# FakeCamera

## Description

Loads compressed images (JPEGs) and corresponding lens parameters (YAML files) from the specified folder and emits them as `CompressedImage` messages

Images are and lens files are expected to be named using a common theme (e.g. image000001.jpg and lens000001.yaml).

## Usage

In the config file set `image_folder` to be the path to the folder containing the image and lens files, `image_prefix` to the portion of the filename that is common to all image files and `lens_prefix` to the portion of the filename that is common to all lens files.

```yaml
image_folder: "images"
image_prefix: "image"
lens_prefix: "lens"
```

All images are assumed to be JPEGs with either a `.jpg` or `.jpeg` extension. All lens files are assumed to be YAML files with a `.yaml` extension.

Lens files should contain the following information

```yaml
projection: EQUISOLID # Either RECTILINEAR, EQUIDUSTANT, or EQUISOLID
focal_length: 420 # Un-normalised focal length in pixels
centre: [0, 0] # Distance between the camera centre axis and optical axis in pixels (measured from the centre of the image)
k: [0, 0] # Distortion coefficients
fov: 1.59286 # Field of view in radians
Hoc: # Camera to observation plane (world) homogeneous transformation
  - [-0.308872, 0.945477, 0.103304, 0]
  - [-0.84664, -0.223831, -0.482805, 0]
  - [-0.433359, -0.236586, 0.869613, 0.9]
  - [0, 0, 0, 1]
```

## Emits

`message::output::CompressedImage` with a `JPEG` FOURCC code

## Dependencies

Eigen
libjpeg-turbo
