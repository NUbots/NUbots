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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include "NUbugger.h"

#include "message/behaviour/Subsumption.h"

#include "utility/behaviour/Action.h"
#include "utility/input/LimbID.h"

namespace module {
namespace support {

    using message::behaviour::Subsumption;

    using utility::behaviour::ActionKill;
    using utility::behaviour::ActionPriorites;
    using utility::behaviour::ActionStart;
    using utility::behaviour::RegisterAction;
    using LimbID = utility::input::LimbID;

    void NUbugger::provideSubsumption() {

        handles["subsumption"].push_back(on<Trigger<ActionStart>>().then([this](const ActionStart& actionStart) {

            Subsumption subsumption;

            Subsumption::ActionStateChange actionStateChange;
            actionStateChange.state = Subsumption::ActionStateChange::State::Value::START;
            actionStateChange.name  = actionStart.name;

            for (auto& limbID : actionStart.limbs) {
                actionStateChange.limbs.push_back(limbID);
            }

            subsumption.action_state_change.push_back(actionStateChange);
            send(subsumption);
        }));

        handles["subsumption"].push_back(on<Trigger<ActionKill>>().then([this](const ActionKill& actionKill) {

            Subsumption subsumption;

            Subsumption::ActionStateChange actionStateChange;
            actionStateChange.state = Subsumption::ActionStateChange::State::Value::KILL;
            actionStateChange.name  = actionKill.name;

            for (auto& limbID : actionKill.limbs) {
                actionStateChange.limbs.push_back(limbID);
            }

            subsumption.action_state_change.push_back(actionStateChange);
            send(subsumption);
        }));

        handles["subsumption"].push_back(on<Trigger<RegisterAction>>().then([this](const RegisterAction& action) {

            Subsumption subsumption;

            Subsumption::ActionRegister actionRegister;
            actionRegister.id   = action.id;
            actionRegister.name = action.name;

            for (const auto& set : action.limbSet) {
                Subsumption::LimbSet limbSet;

                limbSet.priority = set.first;

                for (auto& limbID : set.second) {
                    limbSet.limbs.push_back(limbID);
                }

                actionRegister.limb_set.push_back(limbSet);
            }

            subsumption.action_register.push_back(actionRegister);
            actionRegisters.insert(std::make_pair(action.id, actionRegister));
            send(subsumption);
        }));

        handles["subsumption"].push_back(on<Trigger<ActionPriorites>>().then([this](const ActionPriorites& action) {

            Subsumption subsumption;

            Subsumption::ActionPriorites actionPriorityChange;
            actionPriorityChange.id = action.id;

            Subsumption::ActionRegister actionRegister = actionRegisters.find(action.id)->second;

            size_t index = 0;
            for (const auto& priority : action.priorities) {
                Subsumption::LimbSet limbSet = actionRegister.limb_set[index];
                actionPriorityChange.priorities.push_back(priority);
                limbSet.priority = priority;
                index++;
            }

            subsumption.action_priority_change.push_back(actionPriorityChange);
            send(subsumption);
        }));
    }

    void NUbugger::sendSubsumption() {

        Subsumption subsumption;

        for (const auto& actionRegister : actionRegisters) {
            subsumption.action_register.push_back(actionRegister.second);
        }

        send(subsumption);
    }
}  // namespace support
}  // namespace module
