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

#ifndef EXTENSION_BEHAVIOUR_TASKDATA_HPP
#define EXTENSION_BEHAVIOUR_TASKDATA_HPP

#include <memory>
#include <typeindex>

namespace extension::behaviour {

    namespace information {

        template <typename T>
        struct TaskDataStore {
        private:
            using DataThreadStore     = NUClear::dsl::store::ThreadStore<std::shared_ptr<void>>;
            using ProviderThreadStore = NUClear::dsl::store::ThreadStore<std::shared_ptr<uint64_t>>;

        public:
            using Lock = std::unique_ptr<void, std::function<void(void*)>>;

            /**
             * Set the TaskData for this thread and return a lock that when destroyed will default to OTHER_TRIGGER.
             *
             * @param info the TaskData to set
             *
             * @return a lock object that once destroyed will clear to nullptr
             */
            static Lock set(const std::shared_ptr<const TaskData<T>>& info,
                            const std::shared_ptr<uint64_t>& provider_id) {
                auto lock = std::unique_ptr<void, std::function<void(void*)>>(reinterpret_cast<void*>(0x1), [](void*) {
                    DataThreadStore::value     = nullptr;
                    ProviderThreadStore::value = nullptr;
                });
                DataThreadStore::value     = &info;
                ProviderThreadStore::value = &provider_id;
                return lock;
            }

            static std::shared_ptr<const TaskData<T>> get(const std::shared_ptr<uint64_t>& provider_id) {
                if (ProviderThreadStore::value == nullptr || *ProviderThreadStore::value != provider_id) {
                    return nullptr;
                }
                return std::make_shared<TaskData<T>>(*DataThreadStore::value);
            }
        }

    }  // namespace information

}  // namespace extension::behaviour

#endif  // EXTENSION_BEHAVIOUR_TASKDATA_HPP
