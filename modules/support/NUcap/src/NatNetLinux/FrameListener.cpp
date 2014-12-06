/*
 * FrameListener.h is part of NatNetLinux, and is Copyright 2013-2014,
 * Philip G. Lee <rocketman768@gmail.com>
 *
 * NatNetLinux is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * NatNetLinux is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NatNetLinux.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "FrameListener.h"

#include <unistd.h>

FrameListener::FrameListener(int sd, unsigned char nnMajor, unsigned char nnMinor, size_t bufferSize)
  : _thread(0)
  , _sd(sd)
  , _nnMajor(nnMajor)
  , _nnMinor(nnMinor)
  , _framesMutex()
  , _frames(bufferSize)
  , _run(false) {}

FrameListener::~FrameListener()
{
    if(running()) {
        stop();
    }

    // I think this may be blocking unless the thread is stopped.
    delete _thread;
}

//! \brief Begin the listening in new thread. Non-blocking.
void FrameListener::start() {
    _run = true;
    _thread = new std::thread( &FrameListener::_work, this, _sd);
}

//! \brief Cause the thread to stop. Non-blocking.
void FrameListener::stop() {
    _run = false;
}

//! \brief Return true iff the listener thread is running. Non-blocking.
bool FrameListener::running() {
    return _run;
}

//! \brief Wait for the listening thread to stop. Blocking.
void FrameListener::join() {
    if(_thread) {
        _thread->join();
    }
}

std::pair<MocapFrame, struct timespec> FrameListener::pop(bool* success) {
    std::pair<MocapFrame, struct timespec> ret;
    bool retSuccess = false;

    _framesMutex.lock();

    if(!_frames.empty()) {
        retSuccess = true;
        ret = _frames.back();
        _frames.pop_back();
    }

    _framesMutex.unlock();

    if(success)
        *success = retSuccess;
    return ret;
}

std::pair<MocapFrame, struct timespec> FrameListener::tryPop(bool* success) {
    std::pair<MocapFrame, struct timespec> ret;
    bool retSuccess = false;

    if(_framesMutex.try_lock()) {
        if(!_frames.empty()) {
            retSuccess = true;
            ret = _frames.back();
            _frames.pop_back();
        }

        _framesMutex.unlock();
    }

    if(success) {
       *success = retSuccess;
    }
    return ret;
}

void FrameListener::_work(int sd) {

    NatNetPacket nnp;
    struct timespec ts;
    size_t dataBytes;

    fd_set rfds;
    struct timeval timeout;

    while(_run) {
        // Wait for at most 1 second until the socket has data (read()
        // will not block). Otherwise, continue. This gives outside threads
        // a chance to kill this thread every second.
        timeout.tv_sec = 1; timeout.tv_usec = 0;

        FD_ZERO(&rfds); FD_SET(sd, &rfds);
        if(!select(sd+1, &rfds, 0, 0, &timeout)) {
            continue;
        }

        clock_gettime(CLOCK_REALTIME, &ts);
        dataBytes = read( sd, nnp.rawPtr(), nnp.maxLength() );

        if(dataBytes > 0 && nnp.iMessage() == NatNetPacket::NAT_FRAMEOFDATA) {

            MocapFrame mFrame(_nnMajor,_nnMinor);
            mFrame.unpack(nnp.rawPayloadPtr());
            _framesMutex.lock();
               _frames.push_back(std::make_pair(mFrame,ts));
            _framesMutex.unlock();
        }
    }
}

