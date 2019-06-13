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

#ifndef VISUALMESH_ENGINE_CPU_CLASSIFIER_HPP
#define VISUALMESH_ENGINE_CPU_CLASSIFIER_HPP

#include <cstdint>
#include <numeric>
#include "mesh/classified_mesh.hpp"
#include "mesh/mesh.hpp"
#include "mesh/network_structure.hpp"
#include "mesh/projected_mesh.hpp"
#include "util/fourcc.hpp"

namespace visualmesh {
namespace engine {
  namespace cpu {

    template <typename Scalar>
    class Engine;

    template <typename Scalar>
    class Classifier {
    public:
      Classifier(Engine<Scalar>* engine, const network_structure_t<Scalar>& structure)
        : engine(engine), structure(structure) {

        // Transpose all the weights matrices
        for (auto& conv : this->structure) {
          for (auto& layer : conv) {
            auto& w = layer.first;
            weights_t<Scalar> new_weights(w.front().size(), std::vector<Scalar>(w.size()));
            for (int i = 0; i < w.size(); ++i) {
              for (int j = 0; j < w[i].size(); ++j) {
                new_weights[j][i] = w[i][j];
              }
            }
            w = std::move(new_weights);
          }
        }
      }

      /**
       * Classify using the visual mesh network architecture provided.
       */
      ClassifiedMesh<Scalar> operator()(const Mesh<Scalar>& mesh,
                                        const void* image,
                                        const uint32_t& format,
                                        const mat4<Scalar>& Hoc,
                                        const Lens<Scalar>& lens) const {
        // Two buffers we can ping pong between
        std::vector<Scalar> input;
        std::vector<Scalar> output;

        // Get the parts of the mesh we need
        auto ranges = mesh.lookup(Hoc, lens);

        // Project the pixels to the display
        ProjectedMesh<Scalar> projected = engine->project(mesh, ranges, Hoc, lens);
        auto& neighbourhood             = projected.neighbourhood;
        int n_points                    = neighbourhood.size();

        // Based on the fourcc code, load the data from the image into input
        input.reserve(n_points * 4);
        const int R = ('R' == (format & 0xFF) ? 0 : 2);
        const int B = ('R' == (format & 0xFF) ? 2 : 0);
        switch (format) {
          case fourcc("RGB8"):
          case fourcc("RGB3"):
          case fourcc("BGR8"):
          case fourcc("BGR3"):

            for (const auto& px : projected.pixel_coordinates) {
              const uint8_t* const im = reinterpret_cast<const uint8_t*>(image);

              int c = (std::round(px[1]) * lens.dimensions[0] + std::round(px[0])) * 3;

              input.emplace_back(im[c + R] * Scalar(1.0 / 255.0));
              input.emplace_back(im[c + 1] * Scalar(1.0 / 255.0));
              input.emplace_back(im[c + B] * Scalar(1.0 / 255.0));
              input.emplace_back(0.0);
            }
            break;
          case fourcc("RGBA"):
          case fourcc("BGRA"):

            for (const auto& px : projected.pixel_coordinates) {
              auto im = reinterpret_cast<const uint8_t*>(image);

              int c = (std::round(px[1]) * lens.dimensions[0] + std::round(px[0])) * 4;

              input.emplace_back(im[c + R] * Scalar(1.0 / 255.0));
              input.emplace_back(im[c + 1] * Scalar(1.0 / 255.0));
              input.emplace_back(im[c + B] * Scalar(1.0 / 255.0));
              input.emplace_back(im[c + 3] * Scalar(1.0 / 255.0));
            }
            break;
          default: throw std::runtime_error("The CPU classifier is unable to decode the format " + fourcc_text(format));
        }
        // Four -1 values for the offscreen point
        input.insert(input.end(), {Scalar(-1.0), Scalar(-1.0), Scalar(-1.0), Scalar(-1.0)});

        // We start out with 4d input (RGBAesque)
        int ind  = 4;
        int outd = 0;

        // For each convolutional layer
        for (int conv_no = 0; conv_no < structure.size(); ++conv_no) {
          const auto& conv = structure[conv_no];

          // Ensure enough space for the convolutional gather
          output.resize(0);
          output.reserve(input.size() * 7);
          outd = ind * 7;

          // Gather over each of the neighbours
          for (int i = 0; i < neighbourhood.size(); ++i) {
            output.insert(output.end(), std::next(input.begin(), i * ind), std::next(input.begin(), (i + 1) * ind));
            for (const auto& n : neighbourhood[i]) {
              output.insert(output.end(), std::next(input.begin(), n * ind), std::next(input.begin(), (n + 1) * ind));
            }
          }

          // Output becomes input
          std::swap(input, output);
          ind = outd;

          // For each network layer
          for (int layer_no = 0; layer_no < conv.size(); ++layer_no) {
            const auto& weights = conv[layer_no].first;
            const auto& biases  = conv[layer_no].second;

            // Setup the shapes
            outd = biases.size();
            output.resize(0);
            output.reserve(n_points * outd);

            // Apply the weights and bias
            auto in_point = input.begin();
            for (int i = 0; i < n_points; ++i) {
              for (int j = 0; j < biases.size(); ++j) {
                output.emplace_back(std::inner_product(weights[j].begin(), weights[j].end(), in_point, biases[j]));
              }
              in_point += ind;
            }

            // If we are not on our last layer apply selu
            if (conv_no + 1 < structure.size() || layer_no + 1 < conv.size()) {
              std::transform(output.begin(), output.end(), output.begin(), [](const Scalar& s) {
                constexpr const Scalar lambda = 1.0507009873554804934193349852946;
                constexpr const Scalar alpha  = 1.6732632423543772848170429916717;
                return lambda * (s >= 0 ? s : alpha * std::exp(s) - alpha);
              });
            }

            // Swap our values over
            std::swap(input, output);
            ind = outd;
          }
        }

        // Apply softmax
        for (auto it = input.begin(); it < input.end(); std::advance(it, ind)) {
          const auto end = std::next(it, ind);
          std::transform(it, end, it, [](const Scalar& s) { return std::exp(s); });
          Scalar total = std::accumulate(it, end, static_cast<Scalar>(0.0));
          std::transform(it, end, it, [total](const Scalar& s) { return s / total; });
        }

        return ClassifiedMesh<Scalar>{std::move(projected.pixel_coordinates),
                                      std::move(projected.neighbourhood),
                                      std::move(projected.global_indices),
                                      std::move(input)};
      }

    private:
      Engine<Scalar>* engine;
      network_structure_t<Scalar> structure;
    };  // namespace cpu

  }  // namespace cpu
}  // namespace engine
}  // namespace visualmesh

#endif  // VISUALMESH_ENGINE_CPU_CLASSIFIER_HPP
