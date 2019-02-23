/*
 * Copyright (C) 2013-2016 Trent Houliston <trent@houliston.me>
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

#ifndef EXTENSION_OPENCL_H
#define EXTENSION_OPENCL_H

#include <cstdlib>
#include <nuclear>

#include "FileWatch.h"

#include "utility/file/fileutil.h"
#include "utility/strutil/strutil.h"

namespace extension {

/**
 * TODO document
 *
 * @author Alex Biddulph
 */

struct OpenCLBuildContext {
    OpenCLBuildContext(const cl::Context& context, const std::string& build_flags)
        : context(context), build_flags(build_flags) {}
    cl::Context context;
    std::string build_flags;
};

struct OpenCL {
    cl::Program program;

    OpenCL(const std::string& file, OpenCLBuildContext& build_context) {
        // Create an OpenCL program from the provided context and source file.
        program = cl::Program(build_context.context, utility::file::loadFromFile(file));

        // Append an include search path directive to the build flags.
        std::string build_flags =
            fmt::format("{} -I{}", build_context.build_flags, utility::file::pathSplit(file).first);

        // Build the program.
        auto devices = build_context.context.getInfo<CL_CONTEXT_DEVICES>();

        if (program.build(devices, build_flags.c_str()) != CL_SUCCESS) {
            std::string err(
                fmt::format("Error building OpenCL kernel.\n"
                            "File: {}\n"
                            "Build Flags: {}\n"
                            "Compile Log:\n"
                            "{}",
                            file,
                            build_flags,
                            program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(devices.front())));
            NUClear::log<NUClear::ERROR>(err);
            throw std::runtime_error(err);
        }
    }
};

}  // namespace extension

// NUClear configuration extension
namespace NUClear {
namespace dsl {
    namespace operation {
        template <>
        struct DSLProxy<::extension::OpenCL> {
            template <typename DSL>
            static inline void bind(const std::shared_ptr<threading::Reaction>& reaction,
                                    const std::string& path,
                                    cl::Context& context           = cl::Context::getDefault(),
                                    const std::string& build_flags = std::string()) {
                auto flags = ::extension::FileWatch::ATTRIBUTE_MODIFIED | ::extension::FileWatch::CREATED
                             | ::extension::FileWatch::UPDATED | ::extension::FileWatch::MOVED_TO;

                // Set paths to the config files.
                auto kernel_path = "opencl/" + path;

                if (!utility::file::exists(kernel_path)) {
                    throw std::runtime_error(fmt::format("OpenCL file '{}' does not exist.", kernel_path));
                }

                // Bind our default path
                DSLProxy<::extension::FileWatch>::bind<DSL>(
                    reaction,
                    kernel_path,
                    flags,
                    std::static_pointer_cast<void>(
                        std::make_shared<::extension::OpenCLBuildContext>(context, build_flags)));
            }

            template <typename DSL>
            static inline std::shared_ptr<::extension::OpenCL> get(threading::Reaction& t) {

                // Get the file watch event
                ::extension::FileWatch watch = DSLProxy<::extension::FileWatch>::get<DSL>(t);

                // Check if the watch is valid
                if (watch && utility::strutil::endsWith(watch.path, ".cl")) {
                    // Return our OpenCL file
                    return std::make_shared<::extension::OpenCL>(
                        watch.path, *std::static_pointer_cast<::extension::OpenCLBuildContext>(watch.user_data));
                }
                else {
                    // Return an empty configuration (which will show up invalid)
                    return std::shared_ptr<::extension::OpenCL>(nullptr);
                }
            }
        };
    }  // namespace operation

    // OpenCL is transient
    namespace trait {
        template <>
        struct is_transient<std::shared_ptr<::extension::OpenCL>> : public std::true_type {};
    }  // namespace trait
}  // namespace dsl
}  // namespace NUClear

#endif  // EXTENSION_OPENCL_H
