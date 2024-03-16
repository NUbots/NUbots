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
