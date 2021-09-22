#include <alsa/asoundlib.h>
#include <spawn.h>
#include <fmt/format.h>
#include "Microphone.hpp"

#include "extension/Configuration.hpp"
#include "json.h"

namespace module::input {

using extension::Configuration;
//scriptrunner.cpp


std::unique_ptr<SpeechIntentMessage> parse_voice2json_json(char *json_text, size_t size) {
    std::unique_ptr<SpeechIntentMessage> intent = std::make_unique<SpeechIntentMessage>();
    
    json_value_s *root = json_parse(json_text, size);
    
    json_object_s *root_object = (json_object_s*) root->payload;
    json_object_element_s *root_element = root_object->start;
    while(root_element) {
        json_string_s *element_name = root_element->name;
                
        if(strcmp(element_name->string, "text") == 0) {
            json_string_s *text_str = json_value_as_string(root_element->value);
            intent->text = std::string(text_str->string);
        } else if(strcmp(element_name->string, "intent") == 0) {
            json_object_s *intent_obj = (json_object_s*) root_element->value->payload;
            json_object_element_s *intent_element = intent_obj->start;
            
            assert(strcmp(intent_element->name->string, "name") == 0);
            json_string_s *intent_name_str = json_value_as_string(intent_element->value);
            intent->intent = std::string(intent_name_str->string);
            
            intent_element = intent_element->next;
            
            assert(strcmp(intent_element->name->string, "confidence") == 0);
            json_number_s *confidence_num = json_value_as_number(intent_element->value);
            intent->confidence = atof(confidence_num->number);
        } else if(strcmp(element_name->string, "slots") == 0) {
            json_object_s *slots_obj = (json_object_s*) root_element->value->payload;
            json_object_element_s *slots_element = slots_obj->start;
            while(slots_element) {
                json_string_s *slot_name = slots_element->name;
                json_string_s *slot_value = json_value_as_string(slots_element->value);
                            
                std::string slot_name_str = std::string(slot_name->string);
                std::string slot_value_str = std::string(slot_value->string);
                            
                intent->add_slot(slot_name_str, slot_value_str);
                            
                slots_element = slots_element->next;
            }
        }
        
        root_element = root_element->next;
    }
    free(root);
    
    return intent;
}

std::unique_ptr<SpeechIntentMessage> parse_voice2json_json(std::string str) {
    return parse_voice2json_json(str.data(), str.size());
}

//TODO: there no way of testing this for the moment.
//Use this reference example with "snd_pcm_poll_descriptors"
//Which returns a file handle we can wait on.
//https://gist.github.com/albanpeignier/104902
/*
bool start_microphone_capture() {
    const char *device_name = "default";

    int err = 0;
    snd_pcm_t *capture_handle;
    err = snd_pcm_open(&capture_handle, device_name, SND_PCM_STREAM_CAPTURE, 0);
    if(err < 0) {
        return false
    }
    
    
    
    return true;
}
*/

//handles: out parameter containing the stdin, stdout, stderr handles from the
//spawned process
//first char * in args array refers to the process name
//args need to be null terminated. meaning that the last member of the should be null
bool spawn_process(SpawnedProcess *process, char **args) {
    bool success = false;
    
    pid_t pid = 0;
    int stdout_pipe[2] = {};
    int stderr_pipe[2] = {};
    int stdin_pipe[2] = {};
    posix_spawn_file_actions_t actions = {};
    bool actions_inited = false;
    
    char *path = getenv("PATH");
    
    size_t len = strlen(path) + 5 + 1;
    char *env_path = (char*) malloc(len);
    int written = sprintf(env_path, "PATH=%s", path);
    assert(written > 0);
    env_path[written] = 0;
    
    char *const envp[] = { 
        (char*) "KALDI_DIR=", //KALDI_DIR env var must be blank!
        (char*) env_path, 
        NULL 
    };
    
    if(!((pipe(stdout_pipe) == -1) || (pipe(stderr_pipe) == -1) || (pipe(stdin_pipe) == -1))) {
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
            
        if(posix_spawnp(&pid, (char const *)args[0], &actions, 0, (char *const *)args, envp) == 0) {
            success = true;
        } else {
            NUClear::log<NUClear::FATAL>(fmt::format("failed to spawn process, errno = {}", errno));
        }
    } else {
        NUClear::log<NUClear::FATAL>(fmt::format("failed to create pipes, errno = {}", errno));
    }
    if(stdout_pipe[1]) close(stdout_pipe[1]);
    if(stderr_pipe[1]) close(stderr_pipe[1]);
    if(stdin_pipe[0]) close(stdin_pipe[0]);

    *process = {stdout_pipe[0], stderr_pipe[0], stdin_pipe[1], pid, std::string(args[0]) };
    
    if(actions_inited) {
        posix_spawn_file_actions_destroy(&actions);
    }
    if(env_path) {
        free(env_path);
    }
    
    return success;
}

//python -m voice2json --base-directory /home/nubots/voice2json/ -p en transcribe-wav
bool voice2json_transcribe_stream(SpawnedProcess *process) {
    char *args[] = {
        (char*) "python",     
        (char*) "-m",
        (char*) "voice2json",
        (char*) "--base-directory",
        (char*) "/home/nubots/voice2json/",
        (char*) "-p",
        (char*) "en",
        (char*) "transcribe-stream",
        NULL
    };
    
    return spawn_process(process, args);
}

//python -m voice2json --base-directory /home/nubots/voice2json/ -p en transcribe-wav --input-size
bool voice2json_transcribe_wav(SpawnedProcess *process) {
    char *args[] = {
        (char*) "python",     
        (char*) "-m",
        (char*) "voice2json",
        (char*) "--base-directory",
        (char*) "/home/nubots/voice2json/",
        (char*) "-p",
        (char*) "en",
        (char*) "transcribe-wav",
        (char*) "--input-size",
        NULL
    };
    
    return spawn_process(process, args);
}

//python -m voice2json --base-directory /home/nubots/voice2json/ -p en recognize-intent
char *voice2json_recognize_intent(char *input) {
    char *output = 0;
    char buffer[0x1000];
    ssize_t bytes_read = 0;
    
    char *args[] = {
        (char*) "python",     
        (char*) "-m",
        (char*) "voice2json",
        (char*) "--base-directory",
        (char*) "/home/nubots/voice2json/",
        (char*) "-p",
        (char*) "en",
        (char*) "recognize-intent",
        NULL
    };
    
    SpawnedProcess process = {};
    if(spawn_process(&process, args)) {
        write(process.stdin, input, strlen(input));
        
        bytes_read = read(process.stdout, buffer, sizeof(buffer)-1);
        if(bytes_read == -1) {
            NUClear::log<NUClear::FATAL>(
                fmt::format("failed to read from spawned process "
                    "(voice2json recognize-intent) stdout - error {}", errno));
        }
        buffer[bytes_read] = 0;
        
        output = strdup(buffer);
    }
    if(process.stdout) close(process.stdout);
    if(process.stderr) close(process.stderr);
    if(process.stdin) close(process.stdin);

    return output;
}

bool write_audio_to_file(int fd, char *filename) {
    if(access(filename, F_OK) != 0) {
        return false;
    }
    
    FILE *file = fopen(filename, "rb");
    fseek(file, 0, SEEK_END);
    int file_size = ftell(file);
    fseek(file, 0, SEEK_SET);
    assert(file_size != -1);
    
    char *file_buffer = (char*) malloc(file_size);
    fread(file_buffer, file_size, 1, file);
    fclose(file);
    
    char file_size_buf[64];
    sprintf(file_size_buf, "%d\n", file_size);
        
    write(fd, file_size_buf, strlen(file_size_buf));
    
    ssize_t write_res = write(fd, file_buffer, file_size);
    assert(write_res == file_size);

    free(file_buffer);
    
    return true;
}


Microphone::Microphone(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)), config{} {
    on<Configuration>("Microphone.yaml").then([this](const Configuration& cfg) {
        printf("wtf\n");
        fflush(stdout);
        // Use configuration here from file Microphone.yaml
        this->log_level = cfg["log_level"].as<NUClear::LogLevel>();
        //why does this not fire ????????
        this->config.debug_mode = cfg["debug_mode"].as<bool>();
        
    });
    //Spawn voice2json process in transcribe-wav mode 
    //This will accept a wav file in stdin
    if(!voice2json_transcribe_wav(&voice2json_proc)) {
        perror("failed to start voice2json");
        return;
    }
    
    on<Trigger<MicControlMsg>>().then([this](const MicControlMsg& msg) {
        switch(msg.type) {
            case MIC_MSG_ENABLE: 
                enabled = true;
            break;
            case MIC_MSG_DISABLE:
                enabled = false;
            break;
            case MIC_MSG_TEST_AUDIO:
                if(!write_audio_to_file(voice2json_proc.stdin, (char*) msg.filename.data())) {
                    log(fmt::format("Failed to write to {}", msg.filename.data()));
                }
            break;
        }
    });
    
    //THIS IS FOR TESTING
    printf("debug_mode = %d\n", this->config.debug_mode);
    //if(this->config.debug_mode)
    {
        on<Trigger<SpeechIntentMessage>>().then([this](const SpeechIntentMessage &msg) {
            //TODO needs to be freed...
            log(fmt::format("text = {}", msg.text.data()));
            log(fmt::format("intent = {}", msg.intent.data()));
            for(Slot slot : msg.slots) {
                log(fmt::format("\t{} = {}", slot.name.data(), slot.value.data()));
            }
#if 1
            printf("text = %s (%p)\n", msg.text.data(), msg.text.data());
            printf("intent = %s (%p)\n", msg.intent.data(), msg.intent.data());
            for(Slot slot : msg.slots) {
                printf("%s = %s\n", slot.name.data(), slot.value.data());
            }
#endif
        });
        //this is just for testing
        //we need this as we do not have an
        //environment for testing the microphone
        on<IO>(STDIN_FILENO, IO::READ).then([this] {
            std::string str = {};
            getline(std::cin, str);
            
            log(fmt::format("MICROPHONE STDIN - {}", str));
                        
            std::stringstream ss(str);
            std::string cmd = {};
            getline(ss, cmd, ' ');
            if(cmd == "file") {
                std::string filename = {};
                getline(ss, filename);
                std::unique_ptr<MicControlMsg> msg = std::make_unique<MicControlMsg>(
                    MicControlMsg { MIC_MSG_TEST_AUDIO, filename }
                );
                emit(std::move(msg));
            }
        }); 
    }
    
    
    on<IO>(voice2json_proc.stdout, IO::READ).then([this] {            
        //TODO: need to read and append to buffer until we reach a newline...
        char buffer[0x1000];
        ssize_t bytes_read = read(voice2json_proc.stdout, buffer, sizeof(buffer)-1);
        if(bytes_read == -1) {
            NUClear::log<NUClear::FATAL>(
                fmt::format("failed to read from spawned process stdout, errno = {}", errno));
            return;
        }
        buffer[bytes_read] = 0;
                        
        if(enabled) {            
            char *intent_json = voice2json_recognize_intent(buffer);
            if(!intent_json) {
                perror("error");
            }
            
            std::unique_ptr<SpeechIntentMessage> intent = parse_voice2json_json(intent_json, strlen(intent_json));
            emit(std::move(intent));

            free(intent_json);
        }
    });
    
    on<IO>(voice2json_proc.stderr, IO::READ).then([this] {            
        char buffer[0x1000];
        ssize_t bytes_read = read(voice2json_proc.stderr, buffer, sizeof(buffer)-1);
        if(bytes_read == -1) {
            NUClear::log<NUClear::FATAL>(
                fmt::format("failed to read from spawned process stderr, errno = {}", errno));
            return;
        }
        buffer[bytes_read] = 0;
        NUClear::log<NUClear::FATAL>(fmt::format("VOICE2JSON stderr ({} bytes) = {}", bytes_read, buffer));
    });
}

Microphone::~Microphone() {
    if(voice2json_proc.stdout) close(voice2json_proc.stdout);
    if(voice2json_proc.stderr) close(voice2json_proc.stderr);
    if(voice2json_proc.stdin) close(voice2json_proc.stdin);
}


}  // namespace module::input
