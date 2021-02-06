#ifndef NBS_PLAYER_H
#define NBS_PLAYER_H

#include <memory>
#include <nan.h>

#include "mio/mmap.hpp"
#include "packet.hpp"

// Forward declare
class NBSPlayAction;

class NBSPlayer : public Nan::ObjectWrap {
public:
    virtual ~NBSPlayer();

    Packet read();
    void build_index();

    static void Play(const Nan::FunctionCallbackInfo<v8::Value>& info);
    static void Pause(const Nan::FunctionCallbackInfo<v8::Value>& info);
    static void Step(const Nan::FunctionCallbackInfo<v8::Value>& info);
    static void Restart(const Nan::FunctionCallbackInfo<v8::Value>& info);
    static void Seek(const Nan::FunctionCallbackInfo<v8::Value>& info);

    static void New(const Nan::FunctionCallbackInfo<v8::Value>& info);

    static void Init(v8::Local<v8::Object> exports, v8::Local<v8::Object> module);

    static Nan::Persistent<v8::Function> constructor;

    NBSPlayAction* play_action = nullptr;
    mio::basic_mmap_source<uint8_t> map;
    int64_t pos = 0;
    std::shared_ptr<Nan::Callback> packet_callback;
    std::vector<std::pair<uint32_t, size_t>> index;
};

#endif
