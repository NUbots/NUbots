#ifndef NBS_PLAY_ACTION_H
#define NBS_PLAY_ACTION_H

#include <condition_variable>
#include <memory>
#include <mutex>
#include <nan.h>

#include "nbs_player.hpp"

class NBSPlayAction : public Nan::AsyncProgressQueueWorker<Packet> {
public:
    NBSPlayAction(NBSPlayer* player);

    void Execute(const ExecutionProgress& p);

    void HandleProgressCallback(const Packet*, size_t);

    void HandleOKCallback();

    void HandleErrorCallback();

    NBSPlayer* player;
    int steps = 0;
    uint64_t file_start_time;
    uint64_t player_start_time;
    bool start  = true;
    bool exists = true;
    std::mutex mutex;
    std::condition_variable wait;
};

#endif  // NBS_PLAY_ACTION_H
