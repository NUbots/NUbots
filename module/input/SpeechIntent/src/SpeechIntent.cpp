#include "SpeechIntent.hpp"

#include <alsa/asoundlib.h>
#include <fmt/format.h>
//#include <json.h>
#include <yaml-cpp/yaml.h>
#include <spawn.h>

#include "extension/Configuration.hpp"

namespace module::input {
    using NUClear::message::CommandLineArguments;

    using extension::Configuration;

    std::unique_ptr<SpeechIntentMsg> parse_voice2json_json(std::string  json_text) {
        std::unique_ptr<SpeechIntentMsg> intent = std::make_unique<SpeechIntentMsg>();
        
        YAML::Node root = YAML::Load(json_text);
        intent->text = root["text"].as<std::string>();
        YAML::Node intent_obj = root["intent"].as<YAML::Node>();
        intent->intent = intent_obj["name"].as<std::string>();
        intent->confidence = intent_obj["confidence"].as<float>();
        
        YAML::Node slots_obj = root["slots"].as<YAML::Node>();
        if(slots_obj.Type() != YAML::NodeType::Map) {
            NUClear::log<NUClear::FATAL>(fmt::format("invalid node type found = {}", slots_obj.Type()));
        }
        for (auto it = slots_obj.begin(); it != slots_obj.end(); it++) {
            auto slot_name = it->first.as<std::string>();
            auto slot_value = it->second.as<std::string>();
            
            intent->slots.push_back({slot_name, slot_value});
        }
        
        return intent;
    }

    // process: out parameter containing the information from the spawned process e.g.
    // stdin, stdout, stderr file handles, pid and process name
    // first char * in args array refers to the process name
    // args need to be null terminated. meaning that the last member of the should be null
    bool spawn_process(SpawnedProcess& process, const char** args) {
        bool success = false;

        pid_t pid          = 0;
        int stdout_pipe[2] = {};
        int stderr_pipe[2] = {};
        int stdin_pipe[2]  = {};

        char* path = getenv("PATH");

        size_t len     = strlen(path) + 5 + 1;
        char* env_path = (char*) malloc(len);
        int written    = sprintf(env_path, "PATH=%s", path);
        assert(written > 0);
        env_path[written] = 0;

        char* const envp[] = {(char*) "KALDI_DIR=",  // KALDI_DIR env var must be blank!
                              (char*) env_path,
                              NULL};

        if (!((pipe(stdout_pipe) == -1) || (pipe(stderr_pipe) == -1) || (pipe(stdin_pipe) == -1))) {
            posix_spawn_file_actions_t actions = {};
            posix_spawn_file_actions_init(&actions);

            posix_spawn_file_actions_addclose(&actions, stdout_pipe[0]);
            posix_spawn_file_actions_addclose(&actions, stderr_pipe[0]);
            posix_spawn_file_actions_addclose(&actions, stdin_pipe[1]);

            posix_spawn_file_actions_adddup2(&actions, stdout_pipe[1], STDOUT_FILENO);
            posix_spawn_file_actions_adddup2(&actions, stderr_pipe[1], STDERR_FILENO);
            posix_spawn_file_actions_adddup2(&actions, stdin_pipe[0], STDIN_FILENO);

            posix_spawn_file_actions_addclose(&actions, stdout_pipe[1]);
            posix_spawn_file_actions_addclose(&actions, stderr_pipe[1]);
            posix_spawn_file_actions_addclose(&actions, stdin_pipe[0]);

            success = posix_spawnp(&pid, (const char*) args[0], &actions, 0, (char* const*) args, envp) == 0;
            posix_spawn_file_actions_destroy(&actions);
            if (!success) {
                NUClear::log<NUClear::FATAL>(fmt::format("failed to spawn process, errno = {}", errno));
            }
        }
        else {
            NUClear::log<NUClear::FATAL>(fmt::format("failed to create pipes, errno = {}", errno));
        }
        if (stdout_pipe[1]) {
            close(stdout_pipe[1]);
        }
        if (stderr_pipe[1]) {
            close(stderr_pipe[1]);
        }
        if (stdin_pipe[0]) {
            close(stdin_pipe[0]);
        }
        process = {stdout_pipe[0], stderr_pipe[0], stdin_pipe[1], pid, std::string(args[0])};

        free(env_path);
        return success;
    }

    // python -m voice2json --base-directory /home/nubots/voice2json/ -p en transcribe-stream
    bool voice2json_transcribe_stream(SpawnedProcess& process) {
        const char* args[] = {"python",
                              "-m",
                              "voice2json",
                              "--base-directory",
                              "/home/nubots/voice2json/",
                              "-p",
                              "en",
                              "transcribe-stream",
                              NULL};

        return spawn_process(process, args);
    }

    // python -m voice2json --base-directory /home/nubots/voice2json/ -p en transcribe-wav --input-size
    bool voice2json_transcribe_wav(SpawnedProcess& process) {
        const char* args[] = {"python",
                              "-m",
                              "voice2json",
                              "--base-directory",
                              "/home/nubots/voice2json/",
                              "-p",
                              "en",
                              "transcribe-wav",
                              "--input-size",
                              NULL};
        return spawn_process(process, args);
    }

    // python -m voice2json --base-directory /home/nubots/voice2json/ -p en recognize-intent
    char* voice2json_recognize_intent(char* input) {
        char* output = 0;
        char buffer[read_buffer_size];
        ssize_t bytes_read = 0;

        const char* args[] = {"python",
                              "-m",
                              "voice2json",
                              "--base-directory",
                              "/home/nubots/voice2json/",
                              "-p",
                              "en",
                              "recognize-intent",
                              NULL};

        SpawnedProcess process = {};
        if (spawn_process(process, args)) {
            write(process.stdin, input, strlen(input));

            bytes_read = read(process.stdout, buffer, sizeof(buffer) - 1);
            if (bytes_read == -1) {
                NUClear::log<NUClear::FATAL>(
                    fmt::format("({}:{}) failed to read from spawned process "
                                "(voice2json recognize-intent) stdout - errno = {}",
                                __FILE__,
                                __LINE__,
                                errno));
            }
            buffer[bytes_read] = 0;

            output = strdup(buffer);
        }
        if (process.stdout) {
            close(process.stdout);
        }
        if (process.stderr) {
            close(process.stderr);
        }
        if (process.stdin) {
            close(process.stdin);
        }
        return output;
    }

    // write out the correct format for voice2json to consume
    // when voice2json is called using the "transcribe-wav --input-size" flags
    // it excepts the first line to be the file size (as ascii)
    // followed by the wav file after the newline
    bool write_audio_to_file(int fd, char* filename) {
        ssize_t write_res = 0;
        if (access(filename, F_OK) != 0) {
            return false;
        }

        FILE* file = fopen(filename, "rb");
        fseek(file, 0, SEEK_END);
        int file_size = ftell(file);
        fseek(file, 0, SEEK_SET);
        assert(file_size != -1);

        char* file_buffer = (char*) malloc(file_size);
        fread(file_buffer, file_size, 1, file);
        fclose(file);

        char file_size_buf[64];
        sprintf(file_size_buf, "%d\n", file_size);

        write_res = write(fd, file_size_buf, strlen(file_size_buf));
        if (write_res == -1) {
            NUClear::log<NUClear::FATAL>(fmt::format("({}:{}) write failed, errno = {}", __FILE__, __LINE__, errno));
            free(file_buffer);
            return false;
        }

        write_res = write(fd, file_buffer, file_size);
        if (write_res == -1) {
            NUClear::log<NUClear::FATAL>(fmt::format("({}:{}) write failed, errno = {}", __FILE__, __LINE__, errno));
            free(file_buffer);
            return false;
        }
        assert(write_res == file_size);

        free(file_buffer);

        return true;
    }


    void print_intent(const SpeechIntentMsg& msg) {
        NUClear::log(fmt::format("text = {}", msg.text.data()));
        NUClear::log(fmt::format("intent = {}", msg.intent.data()));
        NUClear::log("slots");
        for (Slot slot : msg.slots) {
            NUClear::log(fmt::format("{} = {}", slot.name.data(), slot.value.data()));
        }
    }

    // Common initialization function
    void SpeechIntent::init(SpeechIntentTranscribeMode transcribe_mode) {
        switch (transcribe_mode) {
            case TRANSCRIBE_MODE_STREAM:
                // Spawn voice2json process in transcribe-stream mode
                // This will accept audio from the microphone (if available)
                if (!voice2json_transcribe_stream(voice2json_proc)) {
                    NUClear::log<NUClear::FATAL>(
                        fmt::format("({}:{}) Failed to start voice2json, errno", __FILE__, __LINE__, errno));
                    return;
                }
                break;
            case TRANSCRIBE_MODE_FILE:
                // Spawn voice2json process in transcribe-wav mode
                // The process will accept a wav file reading from it's stdin
                if (!voice2json_transcribe_wav(voice2json_proc)) {
                    NUClear::log<NUClear::FATAL>(
                        fmt::format("({}:{}) Failed to start voice2json, errno", __FILE__, __LINE__, errno));
                    return;
                }
                break;
            default: assert(false); return;
        }

        // Read from the stdout of the voice2json process which is spawned in either "transcribe-stream" or
        // "transcribe-wav" modes.
        on<IO>(voice2json_proc.stdout, IO::READ).then([this] {
            char buffer[read_buffer_size];
            ssize_t bytes_read = read(voice2json_proc.stdout, buffer, sizeof(buffer) - 1);
            if (bytes_read == -1) {
                NUClear::log<NUClear::FATAL>(
                    fmt::format("({}:{}) failed to read from spawned process stdout, errno = {}",
                                __FILE__,
                                __LINE__,
                                errno));
                return;
            }
            buffer[bytes_read] = 0;

            if (output_enabled) {
                // parse and emit message
                char* intent_json = voice2json_recognize_intent(buffer);
                if (!intent_json) {
                    NUClear::log<NUClear::FATAL>(
                        fmt::format("({}:{}) failed to recognize intent, errno = {}", __FILE__, __LINE__, errno));
                    return;
                }

                std::unique_ptr<SpeechIntentMsg> intent = parse_voice2json_json(std::string(intent_json));
                emit(std::move(intent));

                free(intent_json);
            }
        });

        // Read any input from stderr and print it. We cannot proceed with normal operations so we just print
        // the error messages
        on<IO>(voice2json_proc.stderr, IO::READ).then([this] {
            char buffer[read_buffer_size];
            ssize_t bytes_read = read(voice2json_proc.stderr, buffer, sizeof(buffer) - 1);
            if (bytes_read == -1) {
                NUClear::log<NUClear::FATAL>(
                    fmt::format("({}:{}) Failed to read from stderr, errno = {}", __FILE__, __LINE__, errno));
                return;
            }
            buffer[bytes_read] = 0;
            NUClear::log<NUClear::FATAL>(fmt::format("({}:{})\nVOICE2JSON stderr, errno = {} ({} bytes)\n{}",
                                                     __FILE__,
                                                     __LINE__,
                                                     errno,
                                                     bytes_read,
                                                     buffer));
        });

        // This trigger allows the emission of a "SpeechIntentMsg" to be enabled/disabled.
        on<Trigger<SpeechInputSetOutputMsg>>().then(
            [this](const SpeechInputSetOutputMsg& msg) { output_enabled = msg.enabled; });

        // This trigger will recognize the intent of a wav file.
        on<Trigger<SpeechInputRecognizeWavFile>>().then([this](const SpeechInputRecognizeWavFile& msg) {
            if (!write_audio_to_file(voice2json_proc.stdin, (char*) msg.filename.data())) {
                NUClear::log<NUClear::FATAL>(
                    fmt::format("({}:{}) Failed to write to {}", __FILE__, __LINE__, msg.filename.data()));
            }
        });
    }

    // Convenience function for sending a "SpeechInputRecognizeWavFile" message to this module
    void SpeechIntent::recognize_wav(std::string wav_filename) {
        std::unique_ptr<SpeechInputRecognizeWavFile> msg =
            std::make_unique<SpeechInputRecognizeWavFile>(SpeechInputRecognizeWavFile{wav_filename});
        emit(std::move(msg));
    }

    SpeechIntent::SpeechIntent(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("SpeechIntent.yaml").then([this](const Configuration& cfg) {
            this->log_level = cfg["log_level"].as<NUClear::LogLevel>();
        });

        // Choose between 3 modes based on command line arguments. The "cli" and "file" arguments are for testing
        // purposes. In the case where no cmd line arguments are passed the module is initialized in
        // "TRANSCRIBE_MODE_STREAM" mode.
        on<Trigger<CommandLineArguments>>().then([this](const CommandLineArguments& args) {
            // For testing modes we initialize in "transcribe-wav" mode
            if (args.size() > 1 && args[1] == "file") {
                if (args.size() != 3) {
                    NUClear::log<NUClear::FATAL>(
                        fmt::format("invalid number of arguments, expected 3, got {}", args.size()));
                }
                std::string wav_filename = std::string(args[2]);

                init(TRANSCRIBE_MODE_FILE);

                // For testing we just want to print the message
                on<Trigger<SpeechIntentMsg>>().then([this](const SpeechIntentMsg& msg) { print_intent(msg); });
                // Immediately recognize a wav file
                recognize_wav(wav_filename);
            }
            else if (args.size() > 1 && args[1] == "cli") {
                // "Command line interface" mode. Used for testing purposes.
                // The module acts like a command line interface and accepts input from stdin.
                // Currently only accepts the "file" command, which can regonize the speech intention from a wav file
                init(TRANSCRIBE_MODE_FILE);

                // For testing we just want to print the message
                on<Trigger<SpeechIntentMsg>>().then([this](const SpeechIntentMsg& msg) { print_intent(msg); });
                on<IO>(STDIN_FILENO, IO::READ).then([this] {
                    std::string str = {};
                    getline(std::cin, str);

                    log(fmt::format("SpeechIntent STDIN = {}", str));

                    std::stringstream ss(str);
                    std::string cmd = {};
                    getline(ss, cmd, ' ');
                    if (cmd == "file") {
                        std::string filename = {};
                        getline(ss, filename);
                        recognize_wav(filename);
                    }
                });
            }
            else {
                // Use the microphone input via voice2json (which in turn uses an arecord subprocess).
                init(TRANSCRIBE_MODE_STREAM);
            }
        });
    }

    SpeechIntent::~SpeechIntent() {
        if (voice2json_proc.stdout) {
            close(voice2json_proc.stdout);
        }
        if (voice2json_proc.stderr) {
            close(voice2json_proc.stderr);
        }
        if (voice2json_proc.stdin) {
            close(voice2json_proc.stdin);
        }
    }


}  // namespace module::input
