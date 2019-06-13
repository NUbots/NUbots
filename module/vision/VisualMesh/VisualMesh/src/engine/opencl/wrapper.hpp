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

#ifndef VISUALMESH_ENGINE_OPENCL_WRAPPER_HPP
#define VISUALMESH_ENGINE_OPENCL_WRAPPER_HPP

#define CL_USE_DEPRECATED_OPENCL_1_2_APIS
#if defined(__APPLE__) || defined(__MACOSX)
#  include <OpenCL/opencl.h>
#else
#  include <CL/opencl.h>
#endif  // !__APPLE__

#include <memory>
#include <string>
#include <type_traits>
#include "engine/opencl/opencl_error_category.hpp"

namespace visualmesh {
namespace engine {
  namespace opencl {

    void throw_cl_error(const cl_int& code, const std::string& msg) {
      if (code != CL_SUCCESS) { throw std::system_error(code, opencl_error_category(), msg); }
    }

    namespace cl {
      template <typename T>
      struct opencl_wrapper : public std::shared_ptr<std::remove_pointer_t<T>> {
        using std::shared_ptr<std::remove_pointer_t<T>>::shared_ptr;

        operator T() const {
          return this->get();
        }

      private:
        T ptr = nullptr;
      };

      using command_queue = opencl_wrapper<::cl_command_queue>;
      using context       = opencl_wrapper<::cl_context>;
      using event         = opencl_wrapper<::cl_event>;
      using kernel        = opencl_wrapper<::cl_kernel>;
      using mem           = opencl_wrapper<::cl_mem>;
      using program       = opencl_wrapper<::cl_program>;
    }  // namespace cl

  }  // namespace opencl
}  // namespace engine
}  // namespace visualmesh

#endif  // VISUALMESH_ENGINE_OPENCL_WRAPPER_HPP
