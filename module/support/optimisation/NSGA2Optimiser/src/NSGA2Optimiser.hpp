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
#ifndef MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_HPP
#define MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_HPP

#include <nuclear>
#include <vector>

#include "nsga2/NSGA2.hpp"
#include "tasks/OptimiserTask.hpp"

#include "message/support/optimisation/NSGA2Evaluator.hpp"
#include "message/support/optimisation/NSGA2Optimiser.hpp"

namespace module::support::optimisation {

    using message::support::optimisation::NSGA2EvaluationRequest;

    class NSGA2Optimiser : public NUClear::Reactor {
    private:
        /// @brief Implementation of the NSGA II algorithm, holds the state of the entire optimisation,
        /// including the populations, scores, etc
        nsga2::NSGA2 nsga2_algorithm{};

        /// @brief A pointer to the specific type of task that is being optimised.
        std::unique_ptr<OptimiserTask> task = nullptr;

    public:
        /// @brief Called by the powerplant to build and setup the NSGA2Optimiser reactor.
        explicit NSGA2Optimiser(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace module::support::optimisation

#endif  // MODULE_SUPPORT_OPTIMISATION_NSGA2OPTIMISER_HPP
