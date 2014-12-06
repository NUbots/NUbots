/*
 * NatNet.h is part of NatNetLinux, and is Copyright 2013-2014,
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

#ifndef NATNET_H
#define NATNET_H

#include <cstdint>

/*!
 * \brief Encapsulates basic NatNet communication functionality
 * \author Philip G. Lee
 */
class NatNet {
public:

    //! \brief Default NatNet command port
    static const uint16_t COMMAND_PORT = 1510;
    //! \brief Default NatNet data port
    static const uint16_t DATA_PORT = 1511;

    /*!
     * \brief Create a socket IPv4 address structure.
     *
     * \param inAddr
     *    IPv4 address that the returned structure describes
     * \param port
     *    port that the returned structure describes
     * \returns
     *    an IPv4 socket address structure that describes a given address and
     *    port
     */
    static struct sockaddr_in createAddress(uint32_t inAddr, uint16_t port = COMMAND_PORT);

    /*!
     * \brief Creates a socket for receiving commands.
     *
     * To use this socket to send data, you must use \c sendto() with an
     * appropriate destination address.
     *
     * \param inAddr our local address
     * \param port command port, defaults to 1510
     * \returns socket descriptor bound to \c port and \c inAddr
     */
    static int createCommandSocket(uint32_t inAddr, uint16_t port = COMMAND_PORT);

    /*!
     * \brief Creates a socket to read data from the server.
     *
     * The socket returned from this function is bound to \c port and
     * \c INADDR_ANY, and is added to the multicast group given by
     * \c multicastAddr.
     *
     * \param inAddr our local address
     * \param port port to bind to, defaults to 1511
     * \param multicastAddr multicast address to subscribe to. Defaults to 239.255.42.99.
     * \returns socket bound as described above
     */
    static int createDataSocket(uint32_t inAddr, uint16_t port = DATA_PORT);

    /*!
     * \brief Creates a socket to read data from the server.
     *
     * The socket returned from this function is bound to \c port and
     * \c INADDR_ANY, and is added to the multicast group given by
     * \c multicastAddr.
     *
     * \param inAddr our local address
     * \param port port to bind to, defaults to 1511
     * \param multicastAddr multicast address to subscribe to. Defaults to 239.255.42.99.
     * \returns socket bound as described above
     */
    static int createDataSocket(uint32_t inAddr, uint16_t port, uint32_t multicastAddr);
};

#endif /*NATNET_H*/
