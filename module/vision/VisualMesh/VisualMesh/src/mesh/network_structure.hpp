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

#ifndef VISUALMESH_NETWORKSTRUCTURE_HPP
#define VISUALMESH_NETWORKSTRUCTURE_HPP

namespace visualmesh {

template <typename Scalar>
using weights_t = std::vector<std::vector<Scalar>>;
template <typename Scalar>
using biases_t = std::vector<Scalar>;

template <typename Scalar>
using layer_t = std::pair<weights_t<Scalar>, biases_t<Scalar>>;
template <typename Scalar>
using conv_layer_t = std::vector<layer_t<Scalar>>;
template <typename Scalar>
using network_structure_t = std::vector<conv_layer_t<Scalar>>;

}  // namespace visualmesh

#endif  // VISUALMESH_NETWORKSTRUCTURE_HPP
