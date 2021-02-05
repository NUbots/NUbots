#include "nbs_play_action.hpp"

#include <iostream>

void dont_delete(char*, void*){};

NBSPlayAction::NBSPlayAction(NBSPlayer* player)
    : Nan::AsyncProgressQueueWorker<Packet>(new Nan::Callback()), player(player) {

    // Register ourselves with our player
    player->play_action = this;
}

void NBSPlayAction::Execute(const ExecutionProgress& p) {
    using namespace std::chrono;

    while (exists) {
        // Obtain our lock so we can do work
        std::unique_lock<std::mutex> lock(mutex);

        // If steps is positive or -1
        if (steps) {
            auto packet = player->read();

            if (packet.payload == nullptr) {
                // Pause, we are done with the file, and send the packet so we can notify javascript
                steps = 0;
                p.Send(&packet, sizeof(packet));
            }
            else {
                auto now = duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();

                // If we are starting we need to record our offset
                if (start) {
                    start             = false;
                    file_start_time   = packet.timestamp;
                    player_start_time = now;
                }

                // When stepping we ignore wait times
                if (steps < 0) {
                    int wait_time = int(packet.timestamp - file_start_time) - int(now - player_start_time);
                    // Wait until it's time to send it
                    if (wait_time > 0) {
                        wait.wait_for(lock, microseconds(wait_time));
                    }
                }
                else {
                    --steps;
                }

                // Tell the progress callback that we have a packet!
                p.Send(&packet, sizeof(packet));
            }
        }
        else {
            wait.wait(lock);
        }
    }
}

void NBSPlayAction::HandleProgressCallback(const Packet* packet, size_t size) {
    Nan::HandleScope scope;

    // For some reason it throws nulls sometimes
    if (!packet)
        return;

    // This means we reached the end of the file
    if (packet->payload == nullptr) {
        Nan::Call(*player->packet_callback, 0, nullptr);
    }
    else {
        v8::Local<v8::Value> argv[3] = {
            Nan::New<v8::Number>(packet->timestamp).As<v8::Value>(),
            Nan::NewBuffer(reinterpret_cast<char*>(packet->hash), sizeof(uint64_t), dont_delete, nullptr)
                .ToLocalChecked()
                .As<v8::Value>(),
            // Since we are memory mapped, our deleter shouldn't do anything
            Nan::NewBuffer(reinterpret_cast<char*>(packet->payload), packet->size, dont_delete, nullptr)
                .ToLocalChecked()
                .As<v8::Value>()};

        Nan::Call(*player->packet_callback, 3, argv);
    }
}

void NBSPlayAction::HandleOKCallback() {}

void NBSPlayAction::HandleErrorCallback() {}
