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
#ifndef UTILITY_FILEDESCRIPTOR_HPP
#define UTILITY_FILEDESCRIPTOR_HPP

#include <functional>

namespace utility::file {

    /**
     * @brief An RAII file descriptor.
     * @details This class represents an RAII file descriptor.
     *          It will close the file descriptor it holds on
     *          destruction.
     */
    class FileDescriptor {
    public:
        /**
         * @brief Constructs a new RAII file descriptor.
         *
         * @param fd [description]
         */
        FileDescriptor();
        FileDescriptor(const int& fd_, std::function<void(int)> cleanup_ = nullptr);

        // Don't allow copy construction or assignment
        FileDescriptor(const FileDescriptor&)            = delete;
        FileDescriptor& operator=(const FileDescriptor&) = delete;

        // Allow move construction or assignment
        FileDescriptor(FileDescriptor&& rhs) noexcept;
        FileDescriptor& operator=(FileDescriptor&& rhs) noexcept;

        /**
         * @brief Destruct the file descriptor, closes the held fd
         */
        ~FileDescriptor();

        /**
         * @brief Get the currently held file descriptor
         *
         * @return the file descriptor
         */
        // No Lint: As we are giving access to a variable which can change state.
        // NOLINTNEXTLINE(readability-make-member-function-const)
        [[nodiscard]] int get();

        /**
         * @brief Returns if the currently held file descriptor is valid
         *
         * @return true     if the file descriptor is valid
         * @return false    if the file descriptor is invalid
         */
        [[nodiscard]] bool valid() const;

        /**
         * @brief Close the currently held file descriptor
         */
        void close();

        /**
         * @brief Return the number of bytes available for reading
         *
         * @return -2 if no current connection, -1 for some other error. Check errno if -1 is returned.
         */
        [[nodiscard]] int available() const;

    private:
        /// @brief The held file descriptor
        int fd{-1};
        /// @brief An optional cleanup function to call on close
        std::function<void(int)> cleanup;
    };

}  // namespace utility::file


#endif  // UTILITY_FILEDESCRIPTOR_HPP
