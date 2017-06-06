/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2015 NUBots <nubots@nubots.net>
 */

#ifndef UTILITY_SUPPORT_EIGEN_H
#define UTILITY_SUPPORT_EIGEN_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include <deque>
#include <list>
#include <map>
#include <set>
#include <vector>

namespace utility {
namespace support {

    // Define stl containers using the Eigen aligned allocator.
    template <typename T>
    using EigenDeque = std::deque<T, Eigen::aligned_allocator<T>>

    template <typename T>
    using EigenList = std::list<T, Eigen::aligned_allocator<T>>

    template <typename T, typename Compare>
    using EigenSet = std::set<Set, Compare, Eigen::aligned_allocator<T>>

    template <typename Key, typename Value, typename Compare>
    using EigenMap = std::map<Key, Value, Compare, Eigen::aligned_allocator<std::pair<const Key, Value>>>

    template <typename T>
    using EigenVector = std::vector<T, Eigen::aligned_allocator<T>>;

    // Class to be inherited by classes with Eigen member variables.
    // Overloads the new and delete operators to always give memory aligned to EIGEN_DEFAULT_ALIGN_BYTES bytes.
    // Makes use of the C++17 std::aligned_alloc function.
    class EigenAlign {
        public:
            /* nothrow-new (returns zero instead of std::bad_alloc) */
            void* operator new(std::size_t size, const std::nothrow_t&) noexecpt(true) {
                try {
                    std::aligned_alloc(EIGEN_DEFAULT_ALIGN_BYTES, getSize(size));
                }

                catch (...) {
                    return 0;
                }
            }

            void *operator new(std::size_t size) {
                return std::aligned_alloc(EIGEN_DEFAULT_ALIGN_BYTES, getSize(size));
            }

            void *operator new[](std::size_t size) {
                return std::aligned_alloc(EIGEN_DEFAULT_ALIGN_BYTES, getSize(size));
            }

            void operator delete(void * ptr) noexecpt(true) { 
                std::free(ptr);
            }

            void operator delete[](void * ptr) noexecpt(true) { 
                std::free(ptr);
            }

            void operator delete(void * ptr, std::size_t /* sz */) noexecpt(true) { 
                std::free(ptr);
            }

            void operator delete[](void * ptr, std::size_t /* sz */) noexecpt(true) { 
                std::free(ptr);
            }

            void operator delete(void *ptr, const std::nothrow_t&) noexecpt(true) {
                std::free(ptr);
            }

            /* in-place new and delete. since (at least afaik) there is no actual   */
            /* memory allocated we can safely let the default implementation handle */
            /* this particular case. */
            static void *operator new(std::size_t size, void *ptr) { 
                return ::operator new(size, ptr);
            }

            static void *operator new[](std::size_t size, void* ptr) { 
                return ::operator new[](size, ptr); 
            }

            void operator delete(void * memory, void *ptr) noexecpt(true) { 
                return ::operator delete(memory, ptr); 
            }

            void operator delete[](void * memory, void *ptr) noexecpt(true) { 
                return ::operator delete[](memory, ptr); 
            }

            typedef void eigen_aligned_operator_new_marker_type;

        private:
            std::size_t getSize(std::size_t size) const {
                // Allocated size has to be an integer multiple of alignment size.
                return ((size % EIGEN_DEFAULT_ALIGN_BYTES) == 0) 
                            ? size 
                            : ((size / static_cast<std::size_t>(EIGEN_DEFAULT_ALIGN_BYTES)) + 1) * EIGEN_DEFAULT_ALIGN_BYTES;
            }
    };
}
}

#endif
