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
#include "messages/input/LimbID.h"

#include "utility/time/time.h"

namespace modules {
namespace support {
    using messages::support::nubugger::proto::Message;
    using utility::time::getUtcTimestamp;

    using messages::behaviour::ActionStart;
    using messages::behaviour::ActionKill;
    using messages::behaviour::RegisterAction;
    using messages::behaviour::proto::Subsumption;

    using messages::input::LimbID;

    inline Subsumption::Limb getLimb(const LimbID& limb) {
        switch (limb) {
            case LimbID::LEFT_LEG:
                return Subsumption::LEFT_LEG;
            case LimbID::RIGHT_LEG:
                return Subsumption::RIGHT_LEG;
            case LimbID::LEFT_ARM:
                return Subsumption::LEFT_ARM;
            case LimbID::RIGHT_ARM:
                return Subsumption::RIGHT_ARM;
            case LimbID::HEAD:
                return Subsumption::HEAD;
            default:
                throw std::runtime_error("Invalid Limb");
        }
    }

    void NUbugger::provideSubsumption() {
        handles["subsumption"].push_back(on<Trigger<ActionStart>>([this](const ActionStart& actionStart) {
            Message message;
            message.set_type(Message::SUBSUMPTION);
            message.set_filter_id(0);
            message.set_utc_timestamp(getUtcTimestamp());

            auto* subsumption = message.mutable_subsumption();
            subsumption->set_type(Subsumption::ACTION_STATE);
            auto* actionStateChange = subsumption->mutable_action_state_change();
            actionStateChange->set_state(Subsumption::ActionStateChange::START);
            actionStateChange->set_name(actionStart.name);
            for (auto& limbID : actionStart.limbs) {
                actionStateChange->add_limbs(getLimb(limbID));
            }

            send(message);
        }));

        handles["subsumption"].push_back(on<Trigger<ActionKill>>([this](const ActionKill& actionKill) {
            Message message;
            message.set_type(Message::SUBSUMPTION);
            message.set_filter_id(0);
            message.set_utc_timestamp(getUtcTimestamp());

            auto* subsumption = message.mutable_subsumption();
            subsumption->set_type(Subsumption::ACTION_STATE);
            auto* actionStateChange = subsumption->mutable_action_state_change();
            actionStateChange->set_state(Subsumption::ActionStateChange::KILL);
            actionStateChange->set_name(actionKill.name);
            for (auto& limbID : actionKill.limbs) {
                actionStateChange->add_limbs(getLimb(limbID));
            }

            send(message);
        }));

        handles["subsumption"].push_back(on<Trigger<RegisterAction>>([this] (const RegisterAction& action) {
            Message message;
            message.set_type(Message::SUBSUMPTION);
            message.set_filter_id(0);
            message.set_utc_timestamp(getUtcTimestamp());

            auto* subsumption = message.mutable_subsumption();
            subsumption->set_type(Subsumption::ACTION_REGISTER);
            auto* actionRegister = subsumption->mutable_action_register();
            actionRegister->set_id(action.id);
            actionRegister->set_name(action.name);
            for(const auto& set : action.limbSet) {
                auto* limbSet = actionRegister->add_limb_set();
                limbSet->set_priority(set.first);
                for (auto& limbID : set.second) {
                    limbSet->add_limbs(getLimb(limbID));
                }
            }

            send(message);
        }));
    }

}
}
