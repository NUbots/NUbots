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

#include <NatNetLinux/NatNet.h>
#include <NatNetLinux/NatNetPacket.h>
#include <NatNetLinux/NatNetSender.h>
#include <boost/thread.hpp>
#include <boost/circular_buffer.hpp>
#include <utility>
#include <time.h>

/*!
 * \brief Thread to listen for MocapFrame data.
 * \author Philip G. Lee
 * 
 * This class listens for MocapFrame data on a given socket.
 * It uses a circular buffer to store the frame data, and provides a
 * thread-safe interface to query for the most recent frames.
 */
class FrameListener
{
public:
   
   /*!
    * \brief Constructor
    * 
    * \param sd The socket on which to listen.
    * \param nnMajor NatNet major version.
    * \param nnMinor NatNet minor version.
    * \param bufferSize number of frames in the \c frames() buffer.
    */
   FrameListener(int sd = -1, unsigned char nnMajor=0, unsigned char nnMinor=0, size_t bufferSize=64 ) :
      _thread(0),
      _sd(sd),
      _nnMajor(nnMajor),
      _nnMinor(nnMinor),
      _framesMutex(),
      _frames(bufferSize),
      _run(false)
   {
   }
   
   ~FrameListener()
   {
      if( running() )
         stop();
      // I think this may be blocking unless the thread is stopped.
      delete _thread;
   }
   
   //! \brief Begin the listening in new thread. Non-blocking.
   void start()
   {
      _run = true;
      _thread = new boost::thread( &FrameListener::_work, this, _sd);
   }
   
   //! \brief Cause the thread to stop. Non-blocking.
   void stop()
   {
      _run = false;
   }
   
   //! \brief Return true iff the listener thread is running. Non-blocking.
   bool running()
   {
      return _run;
   }
   
   //! \brief Wait for the listening thread to stop. Blocking.
   void join()
   {
      if(_thread)
         _thread->join();
   }
   
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
   std::pair<MocapFrame, struct timespec> pop(bool* success=0)
   {
      std::pair<MocapFrame, struct timespec> ret;
      bool retSuccess = false;
      
      _framesMutex.lock();
      if( !_frames.empty() )
      {
         retSuccess = true;
         ret = _frames.back();
         _frames.pop_back();
      }
      _framesMutex.unlock();
      
      if( success )
         *success = retSuccess;
      return ret;
   }
   
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
   std::pair<MocapFrame, struct timespec> tryPop(bool* success=0)
   {
      std::pair<MocapFrame, struct timespec> ret;
      bool retSuccess = false;
      
      if( _framesMutex.try_lock() )
      {
         if( !_frames.empty() )
         {
            retSuccess = true;
            ret = _frames.back();
            _frames.pop_back();
         }
         _framesMutex.unlock();
      }
      
      if( success )
         *success = retSuccess;
      return ret;
   }
   
   //--------------------------------------------------------------------------
   
private:
   
   boost::thread* _thread;
   int _sd;
   unsigned char _nnMajor;
   unsigned char _nnMinor;
   mutable boost::mutex _framesMutex;
   boost::circular_buffer< std::pair<MocapFrame, struct timespec> > _frames;
   bool _run;
   
   void _work(int sd)
   {
      NatNetPacket nnp;
      struct timespec ts;
      size_t dataBytes;
      
      fd_set rfds;
      struct timeval timeout;
      
      while(_run)
      {
         // Wait for at most 1 second until the socket has data (read()
         // will not block). Otherwise, continue. This gives outside threads
         // a chance to kill this thread every second.
         timeout.tv_sec = 1; timeout.tv_usec = 0;
         FD_ZERO(&rfds); FD_SET(sd, &rfds);
         if( !select(sd+1, &rfds, 0, 0, &timeout) )
            continue;
         
         clock_gettime( CLOCK_REALTIME, &ts );
         dataBytes = read( sd, nnp.rawPtr(), nnp.maxLength() );
         
         if( dataBytes > 0 && nnp.iMessage() == NatNetPacket::NAT_FRAMEOFDATA )
         {
            MocapFrame mFrame(_nnMajor,_nnMinor);
            mFrame.unpack(nnp.rawPayloadPtr());
            _framesMutex.lock();
               _frames.push_back(std::make_pair(mFrame,ts));
            _framesMutex.unlock();
         }
      }
   }
};

#endif /*FRAMELISTENER_H*/
