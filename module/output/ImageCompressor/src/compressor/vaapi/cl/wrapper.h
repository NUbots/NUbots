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

#ifndef MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_CL_WRAPPER_H
#define MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_CL_WRAPPER_H

#include <CL/opencl.h>

#include <memory>
#include <string>
#include <type_traits>

namespace module::output::compressor::vaapi::cl {

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

}  // namespace module::output::compressor::vaapi::cl

#endif  // MODULE_OUTPUT_IMAGECOMPRESSOR_COMPRESSOR_VAAPI_CL_WRAPPER_H
