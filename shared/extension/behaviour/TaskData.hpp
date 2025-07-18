/*
 * MIT License
 *
 * Copyright (c) 2025 NUbots
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

#ifndef EXTENSION_BEHAVIOUR_TASKDATA_HPP
#define EXTENSION_BEHAVIOUR_TASKDATA_HPP

#include <memory>
#include <nuclear>

namespace extension::behaviour::information {

    /**
     * This class is used to store the task data for a provider.
     *
     * @tparam T the type of data that the provider using this store needs.
     */
    template <typename T>
    struct TaskDataStore {
    private:
        using TaskData    = std::pair<NUClear::id_t, std::shared_ptr<const T>>;
        using GlobalStore = NUClear::util::TypeMap<T, T, TaskData>;

    public:
        /**
         * Set the TaskData for this thread and return a lock that when will upgrade the data to the global store.
         *
         * @param allowed_id the ID of the provider that is allowed to access this data
         * @param raw_data   the data to be stored
         *
         * @return a lock object that will upgrade the data to the global store when it goes out of scope
         */
        static Lock set(const NUClear::id_t& allowed_id, const std::shared_ptr<const void>& raw_data) {
            auto data  = std::make_shared<TaskData>(allowed_id, std::static_pointer_cast<const T>(raw_data));
            auto lock  = Lock([data] {
                GlobalStore::set(data);
                local_data = nullptr;
            });
            local_data = data;

            return lock;
        }

        /**
         * Retrieves the TaskData for this provider.
         *
         * This function attempts to retrieve data from either the thread-local store or the global store,
         * depending on the current thread context. If the data is not found or the provider ID does not match,
         * it returns nullptr.
         *
         * @param provider_id The ID of the provider whose data is to be retrieved.
         *
         * @return A shared pointer to a constant object of type T if found, otherwise nullptr.
         */
        static std::shared_ptr<const T> get(const NUClear::id_t& provider_id) {
            auto d = local_data == nullptr ? GlobalStore::get() : local_data;
            if (d == nullptr || d->first != provider_id) {
                return nullptr;
            }
            return d->second;
        }

    private:
        static thread_local std::shared_ptr<TaskData> local_data;
    };

    template <typename T>
    thread_local std::shared_ptr<typename TaskDataStore<T>::TaskData> TaskDataStore<T>::local_data = nullptr;

}  // namespace extension::behaviour::information

#endif  // EXTENSION_BEHAVIOUR_TASKDATA_HPP
