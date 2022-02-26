#include "nbs_player.hpp"

#include <iostream>

#include "nbs_play_action.hpp"

NBSPlayer::~NBSPlayer() {
    if (play_action) {
        std::lock_guard<std::mutex> lock(play_action->mutex);
        play_action->steps  = 0;
        play_action->exists = false;
        play_action->wait.notify_all();
    }
}

Packet NBSPlayer::read() {

    // Synchronize to a header
    int lock = 3;
    while (lock && pos < map.size()) {
        switch (lock) {
            case 3: lock = map[pos++] == 0xE2 ? 2 : 3; break;
            case 2: lock = map[pos++] == 0x98 ? 1 : 3; break;
            case 1: lock = map[pos++] == 0xA2 ? 0 : 3; break;
        }
    }

    // At the end of the file return an empty packet
    if (pos >= map.size()) {
        return Packet{0, 0, nullptr, 0};
    }

    // Read our size
    uint32_t size = *reinterpret_cast<uint32_t*>(&map[pos]);
    pos += sizeof(size);

    // Read the timestamp
    uint64_t timestamp = *reinterpret_cast<uint64_t*>(&map[pos]);
    pos += sizeof(timestamp);

    // Read the hash
    uint8_t* hash = &map[pos];
    pos += sizeof(hash);

    // Read the payload
    uint32_t payload_size = size - sizeof(timestamp) - sizeof(hash);
    uint8_t* payload      = &map[pos];
    pos += payload_size;

    // Return in an object
    return Packet{timestamp, hash, payload, payload_size};
}

void NBSPlayer::build_index() {

    // We need the file!
    std::lock_guard<std::mutex> lock(play_action->mutex);

    std::cerr << "Building nbs file index" << std::endl;
    index.clear();

    for (int64_t p = 0; p < map.size();) {

        // Synchronize to a header
        int lock = 3;
        while (lock && p < map.size()) {
            switch (lock) {
                case 3: lock = reinterpret_cast<uint8_t&>(map[p++]) == 0xE2 ? 2 : 3; break;
                case 2: lock = reinterpret_cast<uint8_t&>(map[p++]) == 0x98 ? 1 : 3; break;
                case 1: lock = reinterpret_cast<uint8_t&>(map[p++]) == 0xA2 ? 0 : 3; break;
            }
        }

        // 3 characters ago we started a header
        size_t idx = p - 3;

        // Read our size
        uint32_t size = *reinterpret_cast<uint32_t*>(&map[p]);
        p += sizeof(size);

        // Read the timestamp
        uint64_t timestamp = *reinterpret_cast<uint64_t*>(&map[p]);
        p += sizeof(timestamp);

        // Skip!
        p += size - sizeof(timestamp);

        // Add to the index
        index.emplace_back(idx, timestamp);
    }

    std::cerr << "Finished indexing" << std::endl;
}

void NBSPlayer::Play(const Nan::FunctionCallbackInfo<v8::Value>& info) {

    NBSPlayer* bind = ObjectWrap::Unwrap<NBSPlayer>(info.Holder());

    // -1 means play forever
    std::lock_guard<std::mutex> lock(bind->play_action->mutex);
    bind->play_action->start = true;
    bind->play_action->steps = -1;
    bind->play_action->wait.notify_all();
}

void NBSPlayer::Pause(const Nan::FunctionCallbackInfo<v8::Value>& info) {

    NBSPlayer* bind = ObjectWrap::Unwrap<NBSPlayer>(info.Holder());

    // 0 means stop playing
    std::lock_guard<std::mutex> lock(bind->play_action->mutex);
    bind->play_action->steps = 0;
    bind->play_action->wait.notify_all();
}

void NBSPlayer::Restart(const Nan::FunctionCallbackInfo<v8::Value>& info) {

    NBSPlayer* bind = ObjectWrap::Unwrap<NBSPlayer>(info.Holder());

    // Tell it that it needs to retake the start time
    std::lock_guard<std::mutex> lock(bind->play_action->mutex);
    bind->pos                = 0;
    bind->play_action->start = true;
    bind->play_action->wait.notify_all();
}

void NBSPlayer::Step(const Nan::FunctionCallbackInfo<v8::Value>& info) {

    NBSPlayer* bind = ObjectWrap::Unwrap<NBSPlayer>(info.Holder());

    // Get the mutex and then tell it it is playing
    std::lock_guard<std::mutex> lock(bind->play_action->mutex);
    bind->play_action->steps = Nan::To<int32_t>(info[0]).ToChecked();
    bind->play_action->wait.notify_all();
}

void NBSPlayer::Seek(const Nan::FunctionCallbackInfo<v8::Value>& info) {

    NBSPlayer* bind = ObjectWrap::Unwrap<NBSPlayer>(info.Holder());

    // Timestamp to seek to
    uint32_t timestamp = Nan::To<uint32_t>(info[0]).ToChecked();

    // Get the mutex and then tell it it is playing
    std::lock_guard<std::mutex> lock(bind->play_action->mutex);

    auto place = std::lower_bound(bind->index.begin(), bind->index.end(), std::make_pair(timestamp, size_t(0)));
    if (place != bind->index.end()) {
        bind->pos                = place->second;
        bind->play_action->start = true;
    }
    bind->play_action->wait.notify_all();
}

void NBSPlayer::New(const Nan::FunctionCallbackInfo<v8::Value>& info) {

    // Invoked as constructor: `new MyObject(...)`
    if (info.IsConstructCall()) {

        // Create our object and a player thread for it
        NBSPlayer* bind = new NBSPlayer();

        // This starts the player thread
        Nan::AsyncQueueWorker(new NBSPlayAction(bind));

        // Get the file name and callback function from the wrapper
        std::string filename  = *Nan::Utf8String(info[0]);
        bind->packet_callback = std::make_shared<Nan::Callback>(info[1].As<v8::Function>());

        // Check if the file is compressed
        if (filename.size() > 3
            && (0 == filename.compare(filename.length() - 2, 2, "gz")
                || 0 == filename.compare(filename.length() - 3, 3, "nbz"))) {

            // TODO find a good compression tool that can work with mmap
        }

        // Open the file
        std::error_code error;
        bind->map.map(filename, 0, mio::map_entire_file, error);

        // Index the file
        bind->build_index();

        bind->Wrap(info.This());
        info.GetReturnValue().Set(info.This());
    }
    // Invoked as function: `MyObject(...)` throw an error
    else {
        Nan::ThrowError("NBSPlayerAPI must be called as a constructor");
    }
}

void NBSPlayer::Init(v8::Local<v8::Object> exports, v8::Local<v8::Object> module) {
    Nan::HandleScope scope;

    // Prepare constructor template
    v8::Local<v8::FunctionTemplate> tpl = Nan::New<v8::FunctionTemplate>(New);
    tpl->SetClassName(Nan::New("NBSPlayer").ToLocalChecked());
    tpl->InstanceTemplate()->SetInternalFieldCount(1);

    // Prototype
    Nan::SetPrototypeMethod(tpl, "play", Play);
    Nan::SetPrototypeMethod(tpl, "pause", Pause);
    Nan::SetPrototypeMethod(tpl, "restart", Restart);
    Nan::SetPrototypeMethod(tpl, "step", Step);
    Nan::SetPrototypeMethod(tpl, "seek", Seek);

    constructor.Reset();
    auto ctx = Nan::GetCurrentContext();
    module->Set(ctx, Nan::New("exports").ToLocalChecked(), tpl->GetFunction(ctx).ToLocalChecked()).ToChecked();
}

Nan::Persistent<v8::Function> NBSPlayer::constructor;

void Init(v8::Local<v8::Object> exports, v8::Local<v8::Object> module) {
    NBSPlayer::Init(exports, module);
}

NODE_MODULE(nbs_frame_reader, Init)
