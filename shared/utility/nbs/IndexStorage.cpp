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
#include "IndexStorage.hpp"

#include <iostream>

#include "utility/random.hpp"

namespace utility::nbs {

    IndexStorage::IndexStorage(bool big_index) : big_index(big_index) {
        // If this is a big index, open a file
        if (big_index) {
            mmap_path = utility::random::generate_alphanum_string(5) + ".idx";
            mmap_out  = std::ofstream(mmap_path);
        }
    }

    IndexStorage::IndexStorage(IndexStorage&& other) noexcept
        : big_index(other.big_index)
        , finalised(other.finalised)
        , first(other.first)
        , last(other.last)
        , vector_storage(std::move(other.vector_storage))
        , mmap_path(std::move(other.mmap_path))
        , mmap_out(std::move(other.mmap_out))
        , mmap_storage(std::move(other.mmap_storage)) {

        // Swapping these ensures it doesn't delete anything it shouldn't
        other.finalised = true;
        other.big_index = false;
    }

    IndexStorage& IndexStorage::operator=(IndexStorage&& other) noexcept {

        if (big_index) {
            try {
                // Delete the file since we are replacing it with the other object
                if (!finalised) {
                    finalise();
                }
                mmap_storage.unmap();
                std::filesystem::remove(mmap_path);
            }
            catch (...) {
            }
            mmap_path    = std::move(other.mmap_path);
            mmap_out     = std::move(other.mmap_out);
            mmap_storage = std::move(other.mmap_storage);
        }
        else {
            vector_storage = std::move(other.vector_storage);
        }

        big_index = other.big_index;
        finalised = other.finalised;
        first     = other.first;
        last      = other.last;


        // Swapping these ensures it doesn't delete anything it shouldn't
        other.finalised = true;
        other.big_index = false;

        return *this;
    }

    IndexStorage::~IndexStorage() noexcept {
        if (big_index) {
            try {
                if (!finalised) {
                    finalise();
                }

                mmap_storage.unmap();
                std::filesystem::remove(mmap_path);
            }
            catch (...) {
            }
        }
    }

    void IndexStorage::push(const IndexItemFile& item) {
        if (finalised) {
            throw std::runtime_error("Index storage has already been finalised");
        }

        if (big_index) {
            mmap_out.write(reinterpret_cast<const char*>(&item), sizeof(IndexItemFile));
        }
        else {
            vector_storage.push_back(item);
        }
    }

    IndexItemFile* IndexStorage::begin() {
        return first;
    }
    IndexItemFile* IndexStorage::end() {
        return last;
    }

    const IndexItemFile* IndexStorage::begin() const {
        return first;
    }
    const IndexItemFile* IndexStorage::end() const {
        return last;
    }

    /**
     * Call this when we are finished adding elements to this index and get it ready for read/write access.
     *
     * After this function is called, you can manipulate elements that are already in the list, but you will
     * no longer be able to add or remove elements from the list
     */
    void IndexStorage::finalise() {
        if (!finalised) {
            if (big_index) {
                // Close the file and reload it as memory mapped data
                mmap_out.close();
                mmap_storage = mio::mmap_sink(mmap_path.string());

                first = reinterpret_cast<IndexItemFile*>(mmap_storage.data());
                last  = first + (mmap_storage.size() / sizeof(IndexItemFile));
            }
            else {
                // Store the data in a way we can access it
                vector_storage.shrink_to_fit();

                first = vector_storage.data();
                last  = vector_storage.data() + vector_storage.size();
            }
        }
        else {
            throw std::runtime_error("This storage is already finalised");
        }
    }

}  // namespace utility::nbs
