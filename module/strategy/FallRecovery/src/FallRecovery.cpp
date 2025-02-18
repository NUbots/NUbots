/*
 * MIT License
 *
 * Copyright (c) 2023 NUbots
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
#include "FallRecovery.hpp"

#include "extension/Behaviour.hpp"

#include "message/planning/GetUpWhenFallen.hpp"
#include "message/planning/RelaxWhenFalling.hpp"
#include "message/strategy/FallRecovery.hpp"

namespace module::strategy {

    using message::planning::GetUpWhenFallen;
    using message::planning::RelaxWhenFalling;
    using FallRecoveryTask = message::strategy::FallRecovery;

    FallRecovery::FallRecovery(std::unique_ptr<NUClear::Environment> environment)
        : BehaviourReactor(std::move(environment)) {

        on<Provide<FallRecoveryTask>>().then([this] {
            // Plan to relax when falling and get up when on the ground
            // We set the get up priority higher than falling so that relax won't take over while we are getting up
            emit<Task>(std::make_unique<RelaxWhenFalling>(), 1);
            emit<Task>(std::make_unique<GetUpWhenFallen>(), 2);
        });
    }

}  // namespace module::strategy
