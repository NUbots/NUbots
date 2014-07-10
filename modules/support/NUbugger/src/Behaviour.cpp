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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "NUbugger.h"

#include "messages/support/nubugger/proto/Message.pb.h"
#include "messages/behaviour/Action.h"

#include "utility/time/time.h"

namespace modules {
namespace support {
    using messages::support::nubugger::proto::Message;
    using utility::time::getUtcTimestamp;

    using messages::behaviour::ActionStart;
    using messages::behaviour::ActionKill;
    using messages::behaviour::RegisterAction;
    using messages::behaviour::proto::Behaviour;
    using messages::behaviour::proto::ActionStateChange;

    void NUbugger::provideBehaviour() {
        handles["behaviour"].push_back(on<Trigger<ActionStart>>([this](const ActionStart& actionStart) {
            Message message;
            message.set_type(Message::BEHAVIOUR);
            message.set_utc_timestamp(getUtcTimestamp());

            auto* behaviour = message.mutable_behaviour();
            behaviour->set_type(Behaviour::ACTION_STATE);
            auto* actionStateChange = behaviour->mutable_action_state_change();
            actionStateChange->set_state(ActionStateChange::START);
            actionStateChange->set_name(actionStart.name);
            for (auto& limbID : actionStart.limbs) {
                actionStateChange->add_limbs(static_cast<int>(limbID));
            }

            send(message);
        }));

        handles["behaviour"].push_back(on<Trigger<ActionKill>>([this](const ActionKill& actionKill) {
            Message message;
            message.set_type(Message::BEHAVIOUR);
            message.set_utc_timestamp(getUtcTimestamp());

            auto* behaviour = message.mutable_behaviour();
            behaviour->set_type(Behaviour::ACTION_STATE);
            auto* actionStateChange = behaviour->mutable_action_state_change();
            actionStateChange->set_state(ActionStateChange::KILL);
            actionStateChange->set_name(actionKill.name);
            for (auto& limbID : actionKill.limbs) {
                actionStateChange->add_limbs(static_cast<int>(limbID));
            }

            send(message);
        }));

        handles["behaviour"].push_back(on<Trigger<RegisterAction>>([this] (const RegisterAction& action) {
            Message message;
            message.set_type(Message::BEHAVIOUR);
            message.set_utc_timestamp(getUtcTimestamp());

            auto* behaviour = message.mutable_behaviour();
            behaviour->set_type(Behaviour::ACTION_REGISTER);
            auto* actionRegister = behaviour->mutable_action_register();
            actionRegister->set_id(action.id);
            actionRegister->set_name(action.name);
            for(const auto& set : action.limbSet) {
                auto* limbSet = actionRegister->add_limb_set();
                limbSet->set_priority(set.first);
                for (auto& limbID : set.second) {
                    limbSet->add_limbs(static_cast<int>(limbID));
                }
            }

            send(message);
        }));
    }
}
}