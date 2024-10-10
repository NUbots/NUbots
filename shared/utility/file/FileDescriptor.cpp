/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "FileDescriptor.hpp"

#include <cerrno>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <utility>

namespace utility::file {

    FileDescriptor::FileDescriptor() = default;
    FileDescriptor::FileDescriptor(const int& fd_, std::function<void(int)> cleanup_)
        : fd(fd_), cleanup(std::move(cleanup_)) {}

    FileDescriptor::~FileDescriptor() {
        close();
    }

    void FileDescriptor::close() {
        if (valid()) {
            if (cleanup) {
                cleanup(fd);
            }
            ::close(fd);
            fd = -1;
        }
    }

    FileDescriptor::FileDescriptor(FileDescriptor&& rhs) noexcept : fd{rhs.fd} {
        if (this != &rhs) {
            rhs.fd = -1;
        }
    }

    FileDescriptor& FileDescriptor::operator=(FileDescriptor&& rhs) noexcept {
        if (this != &rhs) {
            close();
            fd = std::exchange(rhs.fd, -1);
        }
        return *this;
    }

    // No Lint: As we are giving access to a variable which can change state.
    // NOLINTNEXTLINE(readability-make-member-function-const)
    int FileDescriptor::get() {
        return fd;
    }

    bool FileDescriptor::valid() const {
        return ::fcntl(fd, F_GETFL) != -1 || errno != EBADF;
    }

    int FileDescriptor::available() const {
        if (valid()) {
            int nbytes = 0;
            ::ioctl(fd, FIONREAD, &nbytes);
            return nbytes;
        }
        return -2;
    }

}  // namespace utility::file
