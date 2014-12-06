/*
 * NatNetPacket.h is part of NatNetLinux, and is Copyright 2013-2014,
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

#ifndef NATNETPACKET_H
#define NATNETPACKET_H

#include <netinet/in.h>

/*!
 * \brief Encapsulates NatNet packets.
 * \author Philip G. Lee
 */
class NatNetPacket {
public:

    static const int MAX_PACKETSIZE = 100000;

    //! \brief Message types
    enum NatNetMessageID {
        NAT_PING                 = 0,
        NAT_PINGRESPONSE         = 1,
        NAT_REQUEST              = 2,
        NAT_RESPONSE             = 3,
        NAT_REQUEST_MODELDEF     = 4,
        NAT_MODELDEF             = 5,
        NAT_REQUEST_FRAMEOFDATA  = 6,
        NAT_FRAMEOFDATA          = 7,
        NAT_MESSAGESTRING        = 8,
        NAT_UNRECOGNIZED_REQUEST = 100
    };

    //! \brief Default constructor.
    NatNetPacket();

    //! \brief Copy constructor.
    NatNetPacket(const NatNetPacket& other);

    ~NatNetPacket();

    //! \brief Assignment operator. Does deep copy.
    NatNetPacket& operator=(const NatNetPacket& other);

    //! \brief Construct a "ping" packet.
    static NatNetPacket pingPacket();

    /*!
     * \brief Send packet over the series of tubes.
     * \param sd Socket to use (already bound to an address)
     */
    int send(int sd) const;

    /*! \brief Send packet over the series of tubes.
     * \param sd Socket to use
     * \param destAddr Address to which to send the packet
     */
    int send(int sd, sockaddr_in destAddr) const;

    //! \brief Return a raw pointer to the packet data. Careful.
    char* rawPtr();

    const char* rawPtr() const;

    const char* rawPayloadPtr() const;

    /*!
     * \brief Maximum length of the underlying packet data.
     *
     * \c rawPtr()[\c maxLength()-1] should be a good dereference.
     */
    size_t maxLength() const;

    //! \brief Get the message type.
    NatNetMessageID iMessage() const;

    //! \brief Get the number of bytes in the payload.
    unsigned short nDataBytes() const;

    /*!
     * \brief Read payload data. Const version.
     * \param offset payload byte offset
     */
    template<typename T>
    const T* read(size_t offset) const {

        // NOTE: need to worry about network byte order?
        return reinterpret_cast<T*>(_data + 4 + offset);
    }

    /*!
     * \brief Read payload data.
     * \param offset payload byte offset
     */
    template<typename T>
    T* read(size_t offset) {

        // NOTE: need to worry about network byte order?
        return reinterpret_cast<T*>(_data + 4 + offset);
    }

private:

    char* _data;
    size_t _dataLen;
};

#endif /*NATNETPACKET_H*/
