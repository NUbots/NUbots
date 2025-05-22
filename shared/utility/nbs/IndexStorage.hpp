/*
 * MIT License
 *
 * Copyright (c) 2021 NUbots
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
#ifndef UTILITY_NBS_INDEXSTORAGE_HPP
#define UTILITY_NBS_INDEXSTORAGE_HPP

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <mio/mmap.hpp>
#include <vector>

namespace utility::nbs {

    // Make the packing match what it would be in the file.
    struct IndexItem {
        /// The hash type of the message
        uint64_t type;
        /// The id field of the message if it exists, else 0 (used for things like camera's id)
        uint32_t subtype;
        /// The timestamp from the protocol buffer if it exists, else the timestamp from the NBS file
        uint64_t timestamp;
        /// The offset of the message from the start of the file
        uint64_t offset;
        /// The length of the packet (including the header)
        uint32_t length;
    } __attribute__((packed));

    struct IndexItemFile {
        /// The actual index.
        IndexItem item;
        /// When we are opening multiple nbs files at once, this indicates which file it is from
        int fileno;
    };

    class IndexStorage {
    public:
        IndexStorage(bool big_index = false);

        // Delete copy constructor and operator
        IndexStorage(const IndexStorage&)            = delete;
        IndexStorage& operator=(const IndexStorage&) = delete;

        IndexStorage(IndexStorage&& other) noexcept;
        IndexStorage& operator=(IndexStorage&& other) noexcept;

        ~IndexStorage() noexcept;

        void push(const IndexItemFile& item);

        [[nodiscard]] IndexItemFile* begin();
        [[nodiscard]] IndexItemFile* end();
        [[nodiscard]] const IndexItemFile* begin() const;
        [[nodiscard]] const IndexItemFile* end() const;

        /**
         * Call this when we are finished adding elements to this index and get it ready for read/write access.
         *
         * After this function is called, you can manipulate elements that are already in the list, but you will
         * no longer be able to add or remove elements from the list
         */
        void finalise();

    private:
        bool big_index;
        bool finalised       = false;
        IndexItemFile* first = nullptr;
        IndexItemFile* last  = nullptr;

        /// Storage for the data if it is in memory as a vector
        std::vector<IndexItemFile> vector_storage;

        /// The file path that is used if the data is memory mapped
        std::filesystem::path mmap_path;
        /// The file writer used to add elements to the list before it is finalised
        std::ofstream mmap_out;
        /// Storage and for if the data is being memory mapped
        mio::mmap_sink mmap_storage;
    };
}  // namespace utility::nbs

#endif  // UTILITY_NBS_INDEXSTORAGE_HPP
