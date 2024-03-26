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
#include "MessageLogHandler.hpp"

#include "extension/Configuration.hpp"

#include "message/nuclear/LogMessage.hpp"
#include "message/nuclear/ReactionStatistics.hpp"

namespace module::support::logging {

    using extension::Configuration;

    namespace {
        message::nuclear::ReactionStatistics to_message(const NUClear::message::ReactionStatistics& in) {

            message::nuclear::ReactionStatistics out;

            out.identifiers.name     = in.identifiers.name;
            out.identifiers.reactor  = in.identifiers.reactor;
            out.identifiers.dsl      = in.identifiers.dsl;
            out.identifiers.function = in.identifiers.function;

            out.reaction_id       = uint64_t(in.reaction_id);
            out.task_id           = uint64_t(in.task_id);
            out.cause_reaction_id = uint64_t(in.cause_reaction_id);
            out.cause_task_id     = uint64_t(in.cause_task_id);
            out.emitted           = in.emitted;
            out.started           = in.started;
            out.finished          = in.finished;

            if (in.exception) {
                try {
                    std::rethrow_exception(in.exception);
                }
                catch (const std::exception& ex) {
                    out.exception = ex.what();
                }
                catch (...) {
                    // We don't actually want to crash
                }
            }

            return out;
        }

        message::nuclear::LogLevel to_message(const NUClear::LogLevel& level) {
            switch (level) {
                case NUClear::LogLevel::TRACE: return message::nuclear::LogLevel::TRACE;
                case NUClear::LogLevel::DEBUG: return message::nuclear::LogLevel::DEBUG;
                case NUClear::LogLevel::INFO: return message::nuclear::LogLevel::INFO;
                case NUClear::LogLevel::WARN: return message::nuclear::LogLevel::WARN;
                case NUClear::LogLevel::ERROR: return message::nuclear::LogLevel::ERROR;
                case NUClear::LogLevel::FATAL: return message::nuclear::LogLevel::FATAL;
                case NUClear::LogLevel::UNKNOWN:
                default: return message::nuclear::LogLevel::UNKNOWN;
            }
        }

    }  // namespace

    MessageLogHandler::MessageLogHandler(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), cfg{} {

        on<Configuration>("MessageLogHandler.yaml").then([this](const Configuration& config) {
            // Use configuration here from file MessageLogHandler.yaml
            this->log_level = config["log_level"].as<NUClear::LogLevel>();
            cfg.min_level   = config["log_settings"]["min_level"].as<NUClear::LogLevel>();
            cfg.log_all     = config["log_settings"]["log_all"].as<bool>();
        });

        on<Trigger<NUClear::message::LogMessage>>().then([this](const NUClear::message::LogMessage& log_msg) {
            // Skip if we are not logging this level
            if (log_msg.level < cfg.min_level || (!cfg.log_all && log_msg.level < log_msg.display_level)) {
                return;
            }

            auto msg           = std::make_unique<message::nuclear::LogMessage>();
            msg->timestamp     = NUClear::clock::now();
            msg->level         = to_message(log_msg.level);
            msg->display_level = to_message(log_msg.display_level);
            msg->message       = log_msg.message;
            if (log_msg.task) {
                msg->reaction_statistics = to_message(*log_msg.task);
            }
            emit(msg);
        });

        on<Trigger<NUClear::message::ReactionStatistics>>().then(
            [this](const NUClear::message::ReactionStatistics& stats) {
                auto msg = std::make_unique<message::nuclear::ReactionStatistics>(to_message(stats));
                emit(msg);
            });
    }

}  // namespace module::support::logging
