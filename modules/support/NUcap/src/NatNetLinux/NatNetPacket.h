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

#include <stdint.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <string.h>

/*!
 * \brief Encapsulates NatNet packets.
 * \author Philip G. Lee
 */
class NatNetPacket
{
public:
   
#define MAX_PACKETSIZE 100000
   
   //! \brief Message types
   enum NatNetMessageID
   {
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
   NatNetPacket()
      : _data(new char[MAX_PACKETSIZE+4]), _dataLen(MAX_PACKETSIZE+4)
   {
   }
   
   //! \brief Copy constructor.
   NatNetPacket( NatNetPacket const& other ) :
      _data(new char[other._dataLen]), _dataLen(other._dataLen)
   {
      memcpy( _data, other._data, other._dataLen );
   }
   
   ~NatNetPacket()
   {
      delete[] _data;
   }
   
   //! \brief Assignment operator. Does deep copy.
   NatNetPacket& operator=( NatNetPacket const& other )
   {
      // Careful with self-assignment
      if( _dataLen < other._dataLen )
      {
         delete[] _data;
         _dataLen = other._dataLen;
         _data = new char[_dataLen];
         // Can do memcpy, because we know the other isn't us.
         memcpy( _data, other._data, _dataLen );
      }
      else
         memmove( _data, other._data, _dataLen );
      
      return *this;
   }
   
   //! \brief Construct a "ping" packet.
   static NatNetPacket pingPacket()
   {
      NatNetPacket packet;
      
      uint16_t m = NAT_PING;
      uint16_t len = 0;
      
      *reinterpret_cast<uint16_t*>(packet._data)   = m;
      *reinterpret_cast<uint16_t*>(packet._data+2) = len;
      
      return packet;
   }
   
   /*!
    * \brief Send packet over the series of tubes.
    * \param sd Socket to use (already bound to an address)
    */
   int send(int sd) const
   {
      // Have to prepend '::' to avoid conflicting with NatNetPacket::send().
      return ::send(sd, _data, 4+nDataBytes(), 0);
   }
   
   /*! \brief Send packet over the series of tubes.
    * \param sd Socket to use
    * \param destAddr Address to which to send the packet
    */
   int send(int sd, struct sockaddr_in destAddr) const
   {
      return sendto(sd, _data, 4+nDataBytes(), 0, (sockaddr*)&destAddr, sizeof(destAddr));
   }
   
   //! \brief Return a raw pointer to the packet data. Careful.
   char* rawPtr()
   {
      return _data;
   }
   
   const char* rawPtr() const
   {
      return _data;
   }
   
   const char* rawPayloadPtr() const
   {
      return _data+4;
   }
   
   /*!
    * \brief Maximum length of the underlying packet data.
    * 
    * \c rawPtr()[\c maxLength()-1] should be a good dereference.
    */
   size_t maxLength() const
   {
      return _dataLen;
   }
   
   //! \brief Get the message type.
   NatNetMessageID iMessage() const
   {
      unsigned short m = *reinterpret_cast<unsigned short*>(_data);
      return static_cast<NatNetMessageID>(m);
   }
   
   //! \brief Get the number of bytes in the payload.
   unsigned short nDataBytes() const
   {
      return *reinterpret_cast<unsigned short*>(_data+2);
   }
   
   /*!
    * \brief Read payload data. Const version.
    * \param offset payload byte offset
    */
   template<class T> T const* read( size_t offset ) const
   {
      // NOTE: need to worry about network byte order?
      return reinterpret_cast<T*>(_data+4+offset);
   }
   
   /*!
    * \brief Read payload data.
    * \param offset payload byte offset
    */
   template<class T> T* read( size_t offset )
   {
      // NOTE: need to worry about network byte order?
      return reinterpret_cast<T*>(_data+4+offset);
   }
   
private:
   
   char* _data;
   size_t _dataLen;

#undef MAX_PACKETSIZE
};

#endif /*NATNETPACKET_H*/
