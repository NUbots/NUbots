/*
 * This file is part of NUbots Codebase.
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
 * Copyright 2021 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_SUPPORT_MODULETESTER
#define UTILITY_SUPPORT_MODULETESTER

#define CATCH_CONFIG_RUNNER

#include <catch.hpp>

#include "TestLogHandler.hpp"

namespace utility::support {

    /**
     * @brief Class for running discrete PowerPlants to test Reactors and Reactions
     *
     * @tparam ModuleToTest The module which is to be tested
     * @tparam timeout_ticks  The number of ticks of the TimeoutTimeUnit before the test times out and fails
     * @tparam TimeoutTimeUnit The unit of time which the timeout_ticks are
     */
    template <typename ModuleToTest, int timeout_ticks = 500, typename TimeoutTimeUnit = std::chrono::milliseconds>
    class [[nodiscard]] ModuleTester {
    public:
        /// @brief Instantiates the PowerPlant for the tests
        /// @details Installs a log handler for the tests, so that the module's logs are seen when tests fail
        /// @param num_threads The number of threads that the PowerPlant will use
        ModuleTester(const int& num_threads = 1) : plant(gen_config(num_threads)) {

            INFO("Installing TestLogHandler (automatic for all module tests)");
            plant.install<utility::support::TestLogHandler>();
        }

        /// @brief Installs modules to the test PowerPlant
        /// @details Modules which the test depends on should be installed with this function. This includes the
        ///          "intercepter" module, which saves the emissions from the module being tested
        /// @note Modules which the test depends on should be installed in the order they would be listed in as if they
        ///       were a part of a role
        /// @tparam Module The Reactor type to install now
        /// @param module_name The name of the module being installed. This name is logged before the PowerPlant
        ///                    installs the module
        template <typename Module>
        void install(const std::string& module_name) {
            INFO("Installing " << module_name);
            plant.install<Module>();
        }

        /// @brief Runs the test with the timeout condition
        /// @details The module being tested is installed, the timeout condition is set up, and and the PowerPlant is
        ///          started
        /// @attention After the last emission has been received by the intercepter, the PowerPlant should be shut down.
        ///            If it's not shut down, the timeout condition will stop and fail the test
        void run() {
            INFO("Installing the module being tested");
            plant.install<ModuleToTest>();

            INFO("Installing the Timeout Module")
            plant.install<TimeoutModule>();

            INFO("Starting PowerPlant");
            plant.start();
        }

    private:
        /// @brief The plant which the test runs on
        NUClear::PowerPlant plant;

        /// @brief Creates a config object to instantiate plant with
        NUClear::PowerPlant::Configuration gen_config(const int& num_threads) {
            NUClear::PowerPlant::Configuration config;
            config.thread_count = num_threads;
            return config;
        }

        /**
         * @brief Helper Reactor to timeout tests if they take too long
         * @details The Watchdog shuts down the PowerPlant and fails the test if the time limit is reached
         */
        class TimeoutModule : public NUClear::Reactor {
        public:
            /// @brief Registers the reactions to fail the test if it times out
            explicit TimeoutModule(std::unique_ptr<NUClear::Environment> environment)
                : Reactor(std::move(environment)) {

                // Start the timeout clock by servicing the watchdog
                on<Startup>().then([this] { emit<Scope::WATCHDOG>(ServiceWatchdog<TimeoutWatchdogGroup>()); });

                // Shutdown the powerplant if we're out of time
                on<Watchdog<TimeoutWatchdogGroup, timeout_ticks, TimeoutTimeUnit>>().then([this] {
                    log<NUClear::FATAL>("Test timeout reached. Shutting down PowerPlant.");
                    powerplant.shutdown();
                    FAIL(
                        "Test timeout was reached. If the test was still running as intended, increase the time limit "
                        "for the test by changing the `ModuleTest`'s template parameters.");
                });
            }

        private:
            struct TimeoutWatchdogGroup {};
        };
    };

}  // namespace utility::support

#endif  // UTILITY_SUPPORT_MODULETESTER
