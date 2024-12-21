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

#include "message/nuclear/GroupDescriptor.hpp"
#include "message/nuclear/IDPair.hpp"
#include "message/nuclear/LogLevel.hpp"
#include "message/nuclear/LogMessage.hpp"
#include "message/nuclear/ReactionIdentifiers.hpp"
#include "message/nuclear/ReactionStatistics.hpp"
#include "message/nuclear/ThreadPoolDescriptor.hpp"

namespace module::support::logging {

    using extension::Configuration;

    using NUClearStatistics  = NUClear::message::ReactionStatistics;
    using MessageStatistics  = message::nuclear::ReactionStatistics;
    using NUClearGroup       = NUClear::util::GroupDescriptor;
    using MessageGroup       = message::nuclear::GroupDescriptor;
    using NUClearIdentifiers = NUClear::threading::ReactionIdentifiers;
    using MessageIdentifiers = message::nuclear::ReactionIdentifiers;
    using NUClearPool        = NUClear::util::ThreadPoolDescriptor;
    using MessagePool        = message::nuclear::ThreadPoolDescriptor;
    using MessageLogLevel    = message::nuclear::LogLevel;
    using NUClearLogLevel    = NUClear::LogLevel;
    using NUClearID          = NUClear::IDPair;
    using MessageID          = message::nuclear::IDPair;
    using NUClearLog         = NUClear::message::LogMessage;
    using MessageLog         = message::nuclear::LogMessage;
    using NUClearEvent       = NUClear::message::ReactionStatistics::Event;
    using MessageEvent       = message::nuclear::ReactionStatistics::Event;


    namespace {

        MessageID to_message(const NUClearID& in) {
            MessageID out;
            out.reaction_id = in.reaction_id;
            out.task_id     = in.task_id;
            return out;
        }

        MessageIdentifiers to_message(const NUClearIdentifiers& in) {
            MessageIdentifiers out;
            out.name     = in.name;
            out.reactor  = in.reactor;
            out.dsl      = in.dsl;
            out.function = in.function;
            return out;
        }

        MessagePool to_message(const NUClearPool& in) {
            MessagePool out;
            out.name            = in.name;
            out.concurrency     = in.concurrency;
            out.counts_for_idle = in.counts_for_idle;
            out.persistent      = in.persistent;
            return out;
        }


        MessageGroup to_message(const NUClearGroup& in) {
            MessageGroup out;
            out.name        = in.name;
            out.concurrency = in.concurrency;
            return out;
        }
        std::vector<MessageGroup> to_message(const std::set<std::shared_ptr<const NUClearGroup>>& in) {
            std::vector<MessageGroup> out;
            out.reserve(in.size());
            for (const auto& group : in) {
                out.push_back(to_message(*group));
            }
            return out;
        }

        std::string to_message(const std::exception_ptr& ptr) {
            if (ptr) {
                try {
                    std::rethrow_exception(ptr);
                }
                catch (const std::exception& ex) {
                    return ex.what();
                }
                catch (...) {
                    // We don't actually want to crash
                }
            }
            return {};
        }

        MessageEvent to_message(const NUClearEvent& in) {
            MessageEvent out;

            std::stringstream tid;
            tid << in.thread.thread_id;
            out.thread.thread_id = tid.str();

            if (in.thread.pool != nullptr) {
                out.thread.pool = to_message(*in.thread.pool);
            }
            out.nuclear_time = in.nuclear_time;
            out.real_time    = NUClear::clock::time_point(in.real_time.time_since_epoch());
            out.thread_time  = NUClear::clock::time_point(in.thread_time.time_since_epoch());
            return out;
        }

        MessageStatistics to_message(const NUClearStatistics& in) {

            MessageStatistics out;

            if (in.identifiers != nullptr) {
                out.identifiers = to_message(*in.identifiers);
            }

            out.cause  = to_message(in.cause);
            out.target = to_message(in.target);

            if (in.target_pool != nullptr) {
                out.target_pool = to_message(*in.target_pool);
            }
            out.target_groups = to_message(in.target_groups);

            out.created  = to_message(in.created);
            out.started  = to_message(in.started);
            out.finished = to_message(in.finished);

            out.exception = to_message(in.exception);

            return out;
        }

        MessageLogLevel to_message(const NUClearLogLevel& level) {
            switch (level) {
                case NUClear::LogLevel::TRACE: return MessageLogLevel::TRACE;
                case NUClear::LogLevel::DEBUG: return MessageLogLevel::DEBUG;
                case NUClear::LogLevel::INFO: return MessageLogLevel::INFO;
                case NUClear::LogLevel::WARN: return MessageLogLevel::WARN;
                case NUClear::LogLevel::ERROR: return MessageLogLevel::ERROR;
                case NUClear::LogLevel::FATAL: return MessageLogLevel::FATAL;
                case NUClear::LogLevel::UNKNOWN:
                default: return MessageLogLevel::UNKNOWN;
            }
        }

    }  // namespace

    MessageLogHandler::MessageLogHandler(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), cfg{} {

        on<Configuration>("MessageLogHandler.yaml").then([this](const Configuration& config) {
            // Use configuration here from file MessageLogHandler.yaml
            this->log_level     = config["log_level"].as<NUClear::LogLevel>();
            this->cfg.min_level = config["log_settings"]["min_level"].as<NUClear::LogLevel>();
            this->cfg.log_all   = config["log_settings"]["log_all"].as<bool>();
        });

        on<Trigger<NUClearLog>>().then([this](const NUClearLog& log_msg) {
            // Skip if we are not logging this level
            if (log_msg.level < cfg.min_level || (!cfg.log_all && log_msg.level < log_msg.display_level)) {
                return;
            }

            // Get the current reaction statistics to get the time the log message was created
            auto log_stats = NUClear::threading::ReactionTask::get_current_task()->statistics;

            auto msg           = std::make_unique<MessageLog>();
            msg->timestamp     = log_stats->created.nuclear_time;
            msg->level         = to_message(log_msg.level);
            msg->display_level = to_message(log_msg.display_level);
            msg->message       = log_msg.message;
            if (log_msg.statistics) {
                msg->reaction_statistics = to_message(*log_msg.statistics);
            }
            emit(msg);
        });

        on<Trigger<NUClearStatistics>>().then([this](const NUClearStatistics& stats) {  //
            emit(std::make_unique<MessageStatistics>(to_message(stats)));
        });
    }

}  // namespace module::support::logging
