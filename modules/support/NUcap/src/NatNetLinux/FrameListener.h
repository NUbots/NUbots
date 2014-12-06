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

#ifndef FRAMELISTENER_H
#define FRAMELISTENER_H

#include <thread>
#include <mutex>
#include <deque>

#include "NatNet.h"
#include "MocapFrame.h"
#include "NatNetPacket.h"
#include "NatNetSender.h"

/*!
 * \brief Thread to listen for MocapFrame data.
 * \author Philip G. Lee
 *
 * This class listens for MocapFrame data on a given socket.
 * It uses a circular buffer to store the frame data, and provides a
 * thread-safe interface to query for the most recent frames.
 */
class FrameListener {
public:

    /*!
     * \brief Constructor
     *
     * \param sd The socket on which to listen.
     * \param nnMajor NatNet major version.
     * \param nnMinor NatNet minor version.
     * \param bufferSize number of frames in the \c frames() buffer.
     */
    FrameListener(int sd = -1, unsigned char nnMajor = 0, unsigned char nnMinor = 0, size_t bufferSize = 64);

    ~FrameListener();

    //! \brief Begin the listening in new thread. Non-blocking.
    void start();

    //! \brief Cause the thread to stop. Non-blocking.
    void stop();

    //! \brief Return true iff the listener thread is running. Non-blocking.
    bool running();

    //! \brief Wait for the listening thread to stop. Blocking.
    void join();

    // Data access =============================================================

    /*!
     * \brief Get the latest frame and remove it from the internal buffer. Thread-safe.
     *
     * This function may block while reading the buffer. \c success will be false only if there
     * is no more data in the internal buffer.
     *
     * \param success
     *    input parameter. If not null, its value is set to true if the return
     *    value is valid, and false otherwise.
     * \returns
     *    most recent frame/timestamp pair if the internal buffer has data.
     *    Otherwise, returns an invalid frame. The timestamp is the result of
     *    \c clock_gettime( \c CLOCK_REALTIME, ...) when the data is read
     *    from the UDP interface.
     *
     * \sa tryPop()
     */
    std::pair<MocapFrame, struct timespec> pop(bool* success = 0);

    /*!
     * \brief Get the latest frame and remove it from the internal buffer. Thread-safe *non-blocking*.
     *
     * This function is just like \c pop(), but never blocks. If you are willing
     * to wait to acquire the lock to read the internal data buffer, then use
     * the blocking version \c pop(). If you use \c tryPop(),
     * it will not block, but \c success will be false if either the
     * buffer is currently being written, *or* if there is no more data.
     *
     * \param success
     *    input parameter. If not null, its value is set to true if the return
     *    value is valid, and false otherwise.
     * \returns
     *    most recent frame/timestamp pair if the internal buffer has data
     *    *and* is available for reading immediately.
     *    Otherwise, returns an invalid frame. The timestamp is the result of
     *    \c clock_gettime( \c CLOCK_REALTIME, ...) when the data is read
     *    from the UDP interface.
     *
     * \sa pop()
     */
    std::pair<MocapFrame, struct timespec> tryPop(bool* success = 0);

    //--------------------------------------------------------------------------

private:

    std::thread* _thread;
    int _sd;
    unsigned char _nnMajor;
    unsigned char _nnMinor;
    mutable std::mutex _framesMutex;
    std::deque<std::pair<MocapFrame, struct timespec>> _frames;
    bool _run;

    void _work(int sd);\
};

#endif /*FRAMELISTENER_H*/
