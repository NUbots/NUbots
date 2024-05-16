#include "NBSPlayback.hpp"

#include "extension/Configuration.hpp"

namespace module::tools {

    using extension::Configuration;

    using message::nbs::player::Finished;
    using message::nbs::player::LoadRequest;
    using message::nbs::player::PauseRequest;
    using message::nbs::player::PlaybackState;
    using message::nbs::player::PlayRequest;
    using message::nbs::player::SetModeRequest;


    using message::nbs::player::PlaybackMode::FAST;
    using message::nbs::player::PlaybackMode::REALTIME;
    using message::nbs::player::PlaybackMode::SEQUENTIAL;

    using NUClear::message::CommandLineArguments;

    NBSPlayback::NBSPlayback(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), config{} {

        on<Configuration>("NBSPlayback.yaml").then([this](const Configuration& cfg) {
            this->log_level = cfg["log_level"].as<NUClear::LogLevel>();

            auto playback_mode = cfg["playback_mode"].as<std::string>();
            if (playback_mode == "FAST") {
                config.mode = FAST;
            }
            else if (playback_mode == "SEQUENTIAL") {
                config.mode = SEQUENTIAL;
            }
            else if (playback_mode == "REALTIME") {
                config.mode = REALTIME;
            }
            else {
                log<NUClear::ERROR>("Playback mode is invalid, stopping playback");
                powerplant.shutdown();
            }

            // Update which types we will be playing
            for (const auto& setting : cfg["messages"]) {
                auto name    = setting.first.as<std::string>();
                bool enabled = setting.second.as<bool>();
                if (enabled) {
                    config.messages.push_back(name);
                }
            }
        });

        on<Startup, With<CommandLineArguments>>().then([this](const CommandLineArguments& args) {
            // Set playback mode
            auto set_mode_request  = std::make_unique<SetModeRequest>();
            set_mode_request->mode = config.mode;
            emit<Scope::DIRECT>(set_mode_request);

            // Load the files
            auto load_request      = std::make_unique<LoadRequest>();
            load_request->files    = std::vector<std::string>(std::next(args.begin()), args.end());
            load_request->messages = config.messages;
            emit<Scope::DIRECT>(std::move(load_request));

            // Start playback
            emit<Scope::DIRECT>(std::make_unique<PlayRequest>());
        });

        on<Trigger<PlaybackState>>().then([this](const PlaybackState& playback_state) {
            // Update progress bar
            progress_bar.update(playback_state.current_message, playback_state.total_messages, "", "NBS Playback");
        });

        on<Trigger<Finished>>().then([this] {
            log<NUClear::INFO>("Finished playback");
            progress_bar.close();
            powerplant.shutdown();
        });
    }

}  // namespace module::tools
