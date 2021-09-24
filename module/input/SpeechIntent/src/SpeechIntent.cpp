#include "SpeechIntent.hpp"

#include <alsa/asoundlib.h>
#include <fmt/format.h>
#include <spawn.h>

#include "json.h"

#include "extension/Configuration.hpp"

namespace module::input {
    using NUClear::message::CommandLineArguments;

    using extension::Configuration;

    std::unique_ptr<SpeechIntentMsg> parse_voice2json_json(char* json_text, size_t size) {
        std::unique_ptr<SpeechIntentMsg> intent = std::make_unique<SpeechIntentMsg>();

        json_value_s* root = json_parse(json_text, size);

        json_object_s* root_object          = (json_object_s*) root->payload;
        json_object_element_s* root_element = root_object->start;
        while (root_element) {
            json_string_s* element_name = root_element->name;

            if (strcmp(element_name->string, "text") == 0) {
                json_string_s* text_str = json_value_as_string(root_element->value);
                intent->text            = std::string(text_str->string);
            }
            else if (strcmp(element_name->string, "intent") == 0) {
                json_object_s* intent_obj             = (json_object_s*) root_element->value->payload;
                json_object_element_s* intent_element = intent_obj->start;

                assert(strcmp(intent_element->name->string, "name") == 0);
                json_string_s* intent_name_str = json_value_as_string(intent_element->value);
                intent->intent                 = std::string(intent_name_str->string);

                intent_element = intent_element->next;

                assert(strcmp(intent_element->name->string, "confidence") == 0);
                json_number_s* confidence_num = json_value_as_number(intent_element->value);
                intent->confidence            = atof(confidence_num->number);
            }
            else if (strcmp(element_name->string, "slots") == 0) {
                json_object_s* slots_obj             = (json_object_s*) root_element->value->payload;
                json_object_element_s* slots_element = slots_obj->start;
                while (slots_element) {
                    json_string_s* slot_name  = slots_element->name;
                    json_string_s* slot_value = json_value_as_string(slots_element->value);

                    std::string slot_name_str  = std::string(slot_name->string);
                    std::string slot_value_str = std::string(slot_value->string);

                    intent->slots.push_back({slot_name_str, slot_value_str});

                    slots_element = slots_element->next;
                }
            }

            root_element = root_element->next;
        }
        free(root);

        return intent;
    }

    std::unique_ptr<SpeechIntentMsg> parse_voice2json_json(std::string str) {
        return parse_voice2json_json(str.data(), str.size());
    }

    // process: out parameter containing the information from the spawned process e.g.
    // stdin, stdout, stderr file handles, pid and process name
    // first char * in args array refers to the process name
    // args need to be null terminated. meaning that the last member of the should be null
    bool spawn_process(SpawnedProcess& process, const char** args) {
        bool success = false;

        pid_t pid                          = 0;
        int stdout_pipe[2]                 = {};
        int stderr_pipe[2]                 = {};
        int stdin_pipe[2]                  = {};
        posix_spawn_file_actions_t actions = {};
        bool actions_inited                = false;

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
            posix_spawn_file_actions_init(&actions);
            actions_inited = true;

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
            if (!success) {
                NUClear::log<NUClear::FATAL>(fmt::format("failed to spawn process, errno = {}", errno));
            }
        }
        else {
            NUClear::log<NUClear::FATAL>(fmt::format("failed to create pipes, errno = {}", errno));
        }
        if (stdout_pipe[1])
            close(stdout_pipe[1]);
        if (stderr_pipe[1])
            close(stderr_pipe[1]);
        if (stdin_pipe[0])
            close(stdin_pipe[0]);

        process = {stdout_pipe[0], stderr_pipe[0], stdin_pipe[1], pid, std::string(args[0])};

        if (actions_inited) {
            posix_spawn_file_actions_destroy(&actions);
        }
        if (env_path) {
            free(env_path);
        }

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
        char buffer[0x1000];
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
        if (process.stdout)
            close(process.stdout);
        if (process.stderr)
            close(process.stderr);
        if (process.stdin)
            close(process.stdin);

        return output;
    }

    // write out the correct format for voice2json to consume
    // when voice2json is called using the "transcribe-wav --input-size" flags
    // it excepts the first line to be the file size (as asii)
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

    void SpeechIntent::init() {
        switch (this->config.transcribe_mode) {
            case TRANSCRIBE_MODE_STREAM:
                log("transcribings stream");
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
        on<IO>(voice2json_proc.stdout, IO::READ).then([this] {
            char buffer[0x1000];
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
                char* intent_json = voice2json_recognize_intent(buffer);
                if (!intent_json) {
                    NUClear::log<NUClear::FATAL>(
                        fmt::format("({}:{}) failed to recognize intent, errno = {}", __FILE__, __LINE__, errno));
                    return;
                }

                std::unique_ptr<SpeechIntentMsg> intent = parse_voice2json_json(intent_json, strlen(intent_json));
                emit(std::move(intent));

                free(intent_json);
            }
        });

        on<IO>(voice2json_proc.stderr, IO::READ).then([this] {
            char buffer[0x1000];
            ssize_t bytes_read = read(voice2json_proc.stderr, buffer, sizeof(buffer) - 1);
            if (bytes_read == -1) {
                NUClear::log<NUClear::FATAL>(fmt::format("({}:{})\nVOICE2JSON stderr, errno = {} ({} bytes)\n{}",
                                                         __FILE__,
                                                         __LINE__,
                                                         errno,
                                                         bytes_read,
                                                         buffer));
                return;
            }
            buffer[bytes_read] = 0;
        });


        on<Trigger<SpeechInputSetOutputMsg>>().then(
            [this](const SpeechInputSetOutputMsg& msg) { output_enabled = msg.enabled; });

        on<Trigger<SpeechInputRecognizeWavFile>>().then([this](const SpeechInputRecognizeWavFile& msg) {
            if (!write_audio_to_file(voice2json_proc.stdin, (char*) msg.filename.data())) {
                NUClear::log<NUClear::FATAL>(
                    fmt::format("({}:{}) Failed to write to {}", __FILE__, __LINE__, msg.filename.data()));
            }
        });
    }

    void SpeechIntent::recognize_wav(std::string wav_filename) {
        std::unique_ptr<SpeechInputRecognizeWavFile> msg =
            std::make_unique<SpeechInputRecognizeWavFile>(SpeechInputRecognizeWavFile{wav_filename});
        emit(std::move(msg));
    }

    SpeechIntent::SpeechIntent(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), config{} {


        on<Configuration>("SpeechIntent.yaml").then([this](const Configuration& cfg) {
            this->log_level = cfg["log_level"].as<NUClear::LogLevel>();
        });

        on<Trigger<CommandLineArguments>>().then([this](const CommandLineArguments& args) {
            if (args.size() > 1 && args[1] == "file") {
                if (args.size() != 3) {
                    NUClear::log<NUClear::FATAL>(
                        fmt::format("invalid number of arguments, expected 3, got {}", args.size()));
                }
                this->config.transcribe_mode = TRANSCRIBE_MODE_FILE;
                std::string wav_filename     = std::string(args[2]);
                log(wav_filename);

                init();

                on<Trigger<SpeechIntentMsg>>().then([this](const SpeechIntentMsg& msg) { print_intent(msg); });
                recognize_wav(wav_filename);
            }
            else if (args.size() > 1 && args[1] == "cli") {
                this->config.transcribe_mode = TRANSCRIBE_MODE_FILE;
                init();

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
                init();
            }
        });
    }

    SpeechIntent::~SpeechIntent() {
        if (voice2json_proc.stdout)
            close(voice2json_proc.stdout);
        if (voice2json_proc.stderr)
            close(voice2json_proc.stderr);
        if (voice2json_proc.stdin)
            close(voice2json_proc.stdin);
    }


}  // namespace module::input
