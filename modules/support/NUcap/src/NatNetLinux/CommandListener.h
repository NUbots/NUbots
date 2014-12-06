/*
 * CommandListener.h is part of NatNetLinux, and is Copyright 2013-2014,
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

#ifndef COMMANDLISTENER_H
#define COMMANDLISTENER_H

#include <iostream>
#include <thread>
#include <mutex>

#include "NatNet.h"
#include "NatNetPacket.h"
#include "NatNetSender.h"

/*!
 * \brief Thread to listen for command responses.
 * \author Philip G. Lee
 *
 * This class spawns a new thread to listen for command responses. This class
 * is needed to retrieve the NatNet protocol version in use by the server.
 */
class CommandListener {
public:

    /*!
     * \brief Constructor
     *
     * \param sd The socket on which to listen.
     */
    CommandListener(int sd = -1);

    ~CommandListener();

    //! \brief Begin the listening in new thread. Non-blocking.
    void start();

     //! \brief Cause the thread to stop. Non-blocking.
    void stop();

    //! \brief Return true iff the listener thread is running. Non-blocking.
    bool running();

    //! \brief Wait for the listening thread to stop. Blocking.
    void join();

    /*!
     * \brief Get NatNet major and minor version numbers. Blocking.
     *
     * \warning
     *  this call blocks until the first ping response packet is heard,
     *  and afterwards does not block.
     *  The reason is that the ping response packet is the only one that
     *  contains the NatNet version string. So, you \b MUST send a ping to
     *  the server before calling this to avoid deadlock.
     *
     * \param major output NatNet major version
     * \param minor output NatNet minor version
     */
    void getNatNetVersion(unsigned char& major, unsigned char& minor);

private:
    bool _run;
    std::thread* _thread;
    int _sd;
    unsigned char _nnMajor;
    unsigned char _nnMinor;
    std::mutex _nnVersionMutex;

    void _work(int sd);
};

#endif /*COMMANDLISTENER_H*/
