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
 * Copyright 2021 NUbots <nubots@nubots.net>
 */

#ifndef EXTENSION_MODULETEST_EXTENSIONCATCHER_HPP
#define EXTENSION_MODULETEST_EXTENSIONCATCHER_HPP

#include <nuclear>

namespace extension::moduletest {

    class EmissionCatcher : public NUClear::Reactor {
    public:
        explicit EmissionCatcher(std::unique_ptr<NUClear::Environment> environment);

        template <typename MessageType>
        void bind_catcher(std::shared_ptr<MessageType> message);

        void unbind_all();

    private:
        std::vector<NUClear::threading::ReactionHandle> handles{};
    };
}  // namespace extension::moduletest

#endif  // EXTENSION_MODULETEST_EMISSIONCATCHER
