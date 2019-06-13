/*
 * Copyright (C) 2017-2018 Trent Houliston <trent@houliston.me>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef VISUALMESH_LENS_HPP
#define VISUALMESH_LENS_HPP

#include <array>

namespace visualmesh {

enum LensProjection { RECTILINEAR, EQUISOLID, EQUIDISTANT };

template <typename Scalar>
struct Lens {

  // The projection that this image is using
  LensProjection projection;
  // The dimensions of the image
  std::array<int, 2> dimensions;
  /// The field of view of the camera measured in radians
  Scalar fov;
  /// The focal length of the camera, normalised to the image width
  Scalar focal_length;
  /// The pixel coordinates of the centre of the lens
  std::array<Scalar, 2> centre;
};

}  // namespace visualmesh

#endif  // VISUALMESH_MESH_LENS_HPP
