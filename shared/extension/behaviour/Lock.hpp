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

#ifndef EXTENSION_BEHAVIOUR_LOCK_HPP
#define EXTENSION_BEHAVIOUR_LOCK_HPP

#include <memory>
#include <nuclear>

namespace extension::behaviour {

    class Lock {
    public:
        /**
         * Constructs a Lock with an optional on_destroy function.
         *
         * @param on_destroy A function to be called when the Lock is destroyed.
         */
        explicit Lock(std::function<void()> on_destroy = nullptr) : on_destroy(std::move(on_destroy)) {}

        /**
         * Move constructor.
         *
         * @param other The Lock to move from.
         */
        Lock(Lock&& other) noexcept : on_destroy(std::move(other.on_destroy)) {
            other.on_destroy = nullptr;
        }

        /**
         * Constructs a Lock from multiple locks.
         *
         * @tparam Lock1 The type of the first lock.
         * @tparam Lock2 The type of the second lock.
         * @tparam LockN The types of the remaining locks.
         *
         * @param lock1 The first lock.
         * @param lock2 The second lock.
         * @param lockN The remaining locks.
         */
        template <typename Lock1, typename Lock2, typename... LockN>
        Lock(Lock1&& lock1, Lock2&& lock2, LockN&&... lockN) {
            std::array<std::function<void()>, 2 + sizeof...(LockN)> on_destroy_functions = {
                std::exchange(lock1.on_destroy, nullptr),
                std::exchange(lock2.on_destroy, nullptr),
                std::exchange(lockN.on_destroy, nullptr)...,
            };

            on_destroy = [on_destroy_functions = std::move(on_destroy_functions)]() {
                for (auto& func : on_destroy_functions) {
                    if (func) {
                        func();
                    }
                }
            };
        }

        /**
         * Move assignment operator.
         *
         * @param other The Lock to move from.
         *
         * @return Lock& A reference to this Lock.
         */
        Lock& operator=(Lock&& other) noexcept {
            if (this != &other) {
                on_destroy       = std::move(other.on_destroy);
                other.on_destroy = nullptr;
            }
            return *this;
        }

        // Deleted copy constructor and copy assignment operator
        Lock(const Lock&)            = delete;
        Lock& operator=(const Lock&) = delete;

        /**
         * Destructor. Calls the on_destroy function if it exists.
         */
        ~Lock() {
            if (on_destroy) {
                on_destroy();
            }
        }

    private:
        std::function<void()> on_destroy;
    };

}  // namespace extension::behaviour

#endif  // EXTENSION_BEHAVIOUR_LOCK_HPP
