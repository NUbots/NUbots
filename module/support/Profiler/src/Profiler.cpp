#include "Profiler.hpp"

#include "extension/Configuration.hpp"

#include "message/nuclear/LogMessage.hpp"
#include "message/nuclear/ReactionStatistics.hpp"

namespace module::support {

    using extension::Configuration;
    using NUClear::message::ReactionStatistics;

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

    Profiler::Profiler(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("Profiler.yaml").then([this](const Configuration& config) {
            log_level = config["log_level"].as<NUClear::LogLevel>();
        });

        on<Trigger<ReactionStatistics>>().then([this](const ReactionStatistics& stats) {
            auto msg = std::make_unique<message::nuclear::ReactionStatistics>(to_message(stats));
            emit(msg);
        });
    }
}  // namespace module::support
