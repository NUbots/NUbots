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

#include <iostream>
#include <iomanip>
#include <ios>
#include <vector>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

/*!
 * \brief Encapsulates basic NatNet communication functionality
 * \author Philip G. Lee
 */
class NatNet
{
public:
   
   //! \brief Default NatNet command port
   static const uint16_t commandPort=1510;
   //! \brief Default NatNet data port
   static const uint16_t dataPort=1511;
   
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
   static struct sockaddr_in createAddress( uint32_t inAddr, uint16_t port=commandPort )
   {
      struct sockaddr_in ret;
      memset(&ret, 0, sizeof(ret));
      ret.sin_family = AF_INET;
      ret.sin_port = htons(port);
      ret.sin_addr.s_addr = inAddr;
      
      return ret;
   }
   
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
   static int createCommandSocket( uint32_t inAddr, uint16_t port=commandPort )
   {
      // Asking for a buffer of 1MB = 2^20 bytes. This is what NP does, but this
      // seems far too large on Linux systems where the max is usually something
      // like 256 kB.
      const int rcvBufSize = 0x100000;
      int sd;
      int tmp=0;
      socklen_t len=0;
      struct sockaddr_in sockAddr = createAddress(inAddr, port);
      
      sd = socket(AF_INET, SOCK_DGRAM, 0);
      if( sd < 0 )
      {
         std::cerr << "Could not open socket. Error: " << errno << std::endl;
         exit(1);
      }
      
      // Bind socket to the address.
      tmp = bind( sd, (struct sockaddr*)&sockAddr, sizeof(sockAddr) );
      if( tmp < 0 )
      {
         std::cerr << "Could not bind socket. Error: " << errno << std::endl;
         close(sd);
         exit(1);
      }
      
      int value = 1;
      tmp = setsockopt( sd, SOL_SOCKET, SO_BROADCAST, (char*)&value, sizeof(value) );
      if( tmp < 0 )
      {
         std::cerr << "Could not set socket to broadcast mode. Error: " << errno << std::endl;
         close(sd);
         exit(1);
      }
      
      setsockopt(sd, SOL_SOCKET, SO_RCVBUF, (char*)&rcvBufSize, sizeof(rcvBufSize));
      getsockopt(sd, SOL_SOCKET, SO_RCVBUF, (char*)&tmp, &len);
      if( tmp != rcvBufSize )
      {
         std::cerr << "WARNING: Could not set receive buffer size. Asked for "
            << rcvBufSize << "B got " << tmp << "B" << std::endl;
      }
      
      return sd;
   }
   
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
   static int createDataSocket( uint32_t inAddr, uint16_t port=dataPort, uint32_t multicastAddr=inet_addr("239.255.42.99") )
   {
      int sd;
      int value;
      int tmp;
      struct ip_mreq group;
      struct sockaddr_in localSock = createAddress(INADDR_ANY, port);
      
      sd = socket(AF_INET, SOCK_DGRAM, 0);
      value = 1;
      tmp = setsockopt(sd, SOL_SOCKET, SO_REUSEADDR, (char*)&value, sizeof(value));
      if( tmp < 0 )
      {
         std::cerr << "ERROR: Could not set socket option." << std::endl;
         close(sd);
         return -1;
      }
      
      // Bind the socket to a port.
      bind(sd, (struct sockaddr*)&localSock, sizeof(localSock));
      
      // Connect a local interface address to the multicast interface address.
      group.imr_multiaddr.s_addr = multicastAddr;
      group.imr_interface.s_addr = inAddr;
      tmp = setsockopt(sd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&group, sizeof(group));
      if( tmp < 0 )
      {
         std::cerr << "ERROR: Could not add the interface to the multicast group." << std::endl;
         close(sd);
         return -1;
      }
      
      return sd;
   }
};

/*!
 * \brief Simple 3D point
 * \author Philip G. Lee
 */
class Point3f
{
public:
   Point3f( float xx=0.f, float yy=0.f, float zz=0.f ) :
      x(xx), y(yy), z(zz)
   {
   }
   
   ~Point3f() {}
   
   Point3f( Point3f const& other ) :
      x(other.x),
      y(other.y),
      z(other.z)
   {
   }
   
   Point3f& operator=( Point3f const& other )
   {
      // Self-assignment no problem
      x = other.x;
      y = other.y;
      z = other.z;
      
      return *this;
   }
   
   float x;
   float y;
   float z;
};

//! \brief Output operator for Point3f.
std::ostream& operator<<( std::ostream& s, Point3f const& point )
{
   std::ios::fmtflags f(s.flags());
   
   s
   << std::fixed
   << "( "
   << std::setprecision(3) << point.x << ", "
   << std::setprecision(3) << point.y << ", "
   << std::setprecision(3) << point.z << " )" << std::endl;
   
   s.flags(f);
   
   return s;
}

/*!
 * \brief Quaternion for 3D rotations and orientation
 * \author Philip G. Lee
 * 
 * For more information, please see [Quaternions on Wikipedia](http://en.wikipedia.org/wiki/Quaternion).
 * A quaternion is a non-commutative group which describes 3D rotations, and whose group
 * operation (multiplication) represents rotation composition (A*B means rotate
 * by B, then by A). Since it is a proper group, there are no uninvertible
 * rotations (like with Euler angles). It is also parameterized by the smallest
 * number of parameters (4), unlike a 3x3 matrix (9).
 */
class Quaternion4f
{
public:
   float qx;
   float qy;
   float qz;
   float qw;
   
   //! \brief Default constructor. Without parameters, returns the identity (no rotation).
   Quaternion4f( float qx=0.f, float qy=0.f, float qz=0.f, float qw=1.f ) :
      qx(qx),
      qy(qy),
      qz(qz),
      qw(qw)
   {
      renormalize();
   }
   
   ~Quaternion4f(){}
   
   //! \brief Copy constructor
   Quaternion4f( Quaternion4f const& other ) :
      qx(other.qx),
      qy(other.qy),
      qz(other.qz),
      qw(other.qw)
   {
   }
   
   //! \brief Assignment operator
   Quaternion4f& operator=( Quaternion4f const& other )
   {
      qx = other.qx;
      qy = other.qy;
      qz = other.qz;
      qw = other.qw;
      
      return *this;
   }
   
   //! \brief Quaternion multiplication/assignment
   Quaternion4f& operator*=(Quaternion4f const& rhs)
   {
      float x,y,z,w;
      
      x = qw*rhs.qw - qx*rhs.qx - qy*rhs.qy - qz*rhs.qz;
      y = qw*rhs.qx + qx*rhs.qw + qy*rhs.qz - qz*rhs.qy;
      z = qw*rhs.qy - qx*rhs.qz + qy*rhs.qw + qz*rhs.qx;
      w = qw*rhs.qz + qx*rhs.qy - qy*rhs.qx + qz*rhs.qw;
      
      qx = x;
      qy = y;
      qz = z;
      qw = w;
      
      renormalize();
      
      return *this;
   }
   
   //! \brief Quaternion multiplication (rotation composition)
   Quaternion4f operator*(Quaternion4f const& rhs) const
   {
      Quaternion4f ret(*this);
      
      ret *= rhs;
      
      return ret;
   }
   
   //! \brief Quaternion division/assignment
   Quaternion4f& operator/=(Quaternion4f const& rhs)
   {
      // Create the conjugate and multiply.
      Quaternion4f rhsConj(-rhs.qx, -rhs.qy, -rhs.qy, rhs.qw);
      *this *= rhsConj;
      return *this;
   }
   
   //! \brief Quaternion division
   Quaternion4f operator/(Quaternion4f const& rhs) const
   {
      Quaternion4f ret(*this);
      ret /= rhs;
      return ret;
   }
   
   //! \brief Rotate a point using the quaternion.
   Point3f rotate(Point3f const& p) const
   {
      Point3f pout;
      
      pout.x = (1.f-2.f*qy*qy-2.f*qz*qz)*p.x + (2.f*qx*qy-2.f*qw*qz)*p.y + (2.f*qx*qz+2.f*qw*qy)*p.z;
      pout.y = (2.f*qx*qy+2.f*qw*qz)*p.x + (1.f-2.f*qx*qx-2.f*qz*qz)*p.y + (2.f*qy*qz+2.f*qw*qx)*p.z;
      pout.z = (2.f*qx*qz-2.f*qw*qy)*p.x + (2.f*qy*qz-2.f*qw*qx)*p.y + (1.f-2.f*qx*qx-2.f*qy*qy)*p.z;
      
      return pout;
   }
   
private:
   
   // If the magnitude of the quaternion exceeds a tolerance, renormalize it
   // to have magnitude of 1.
   void renormalize()
   {
      static const float tolHigh = 1.f+1e-6;
      static const float tolLow  = 1.f-1e-6;
      
      float mag = qx*qx + qy*qy + qw*qw + qz*qz;
      
      if( mag < tolLow || mag > tolHigh )
      {
         mag = sqrtf( mag );
         qx /= mag;
         qy /= mag;
         qz /= mag;
         qw /= mag;
      }
   }
};

//! \brief Output operator for Quaternion4f.
std::ostream& operator<<( std::ostream& s, Quaternion4f const& q )
{
   std::ios::fmtflags f(s.flags());
   
   s
   << std::fixed
   << "(qx,qy,qz,qw) = ( "
   << std::setprecision(3) << q.qx << ", "
   << std::setprecision(3) << q.qy << ", "
   << std::setprecision(3) << q.qz << ", "
   << std::setprecision(3) << q.qw << " )" << std::endl;
   
   s.flags(f);
   
   return s;
}

/*!
 * \brief Rigid body
 * \author Philip G. Lee
 * 
 * This class is a composition of markers that describe a rigid body. The basic
 * traits of the rigid body are its 3D location() and orientation(). Rigid
 * bodies can be created in Optitrack's Motive:Tracker software.
 */
class RigidBody
{
public:
   
   //! \brief Default constructor
   RigidBody() :
      _id(-1),
      _loc(),
      _ori(),
      _markers(),
      _mId(),
      _mSize(),
      _mErr(),
      _trackingValid(true)
   {
   }
   
   //! \brief Copy constructor
   RigidBody( RigidBody const& other ) :
      _id(other._id),
      _loc(other._loc),
      _ori(other._ori),
      _markers(other._markers),
      _mId(other._mId),
      _mSize(other._mSize),
      _mErr(other._mErr),
      _trackingValid(other._trackingValid)
   {
   }
   
   ~RigidBody(){}
   
   //! \brief Assignment operator
   RigidBody& operator=( RigidBody const& other )
   {
      _id = other._id;
      _loc = other._loc;
      _ori = other._ori;
      _markers = other._markers;
      _mId = other._mId;
      _mSize = other._mSize;
      _mErr = other._mErr;
      _trackingValid = other._trackingValid;
      
      return *this;
   }
   
   //! \brief ID of this RigidBody
   int id() const { return _id; }
   //! \brief Location of this RigidBody
   Point3f location() const { return _loc; }
   //! \brief Orientation of this RigidBody
   Quaternion4f orientation() const { return _ori; }
   //! \brief Vector of markers that make up this RigidBody
   std::vector<Point3f> const& markers() const { return _markers; }
   //! \brief True if the tracking is valid. Used in NatNet version >= 2.6.
   bool trackingValid() const { return _trackingValid; }
   
   /*!
    * \brief Unpack rigid body data from raw packed data.
    * 
    * \param data pointer to packed data representing a RigidBody
    * \param nnMajor major version of NatNet used to construct the packed data
    * \param nnMinor Minor version of NatNet packets used to read this frame
    * \returns pointer to data immediately following the RigidBody data
    */
   char const* unpack(char const* data, char nnMajor, char nnMinor)
   {
      int i;
      float x,y,z;
      
      // Rigid body ID
      memcpy(&_id,data,4); data += 4;
      
      // Location and orientation.
      memcpy(&_loc.x,data,4); data += 4;
      memcpy(&_loc.y,data,4); data += 4;
      memcpy(&_loc.z,data,4); data += 4;
      memcpy(&_ori.qx,data,4); data += 4;
      memcpy(&_ori.qy,data,4); data += 4;
      memcpy(&_ori.qz,data,4); data += 4;
      memcpy(&_ori.qw,data,4); data += 4;
      
      // Associated markers
      int nMarkers = 0;
      memcpy(&nMarkers,data,4); data += 4;
      for( i = 0; i < nMarkers; ++i )
      {
         memcpy(&x,data,4); data += 4;
         memcpy(&y,data,4); data += 4;
         memcpy(&z,data,4); data += 4;
         _markers.push_back(Point3f(x,y,z));
      }

      if( nnMajor >= 2 )
      {
         // Marker IDs
         uint32_t id = 0;
         for( i = 0; i < nMarkers; ++i )
         {
            memcpy(&id,data,4); data += 4;
            _mId.push_back(id);
         }

         // Marker sizes
         float size;
         for( i = 0; i < nMarkers; ++i )
         {
            memcpy(&size,data,4); data += 4;
            _mSize.push_back(size);
         }

         if( ((nnMajor==2) && (nnMinor >= 6)) || (nnMajor > 2) || (nnMajor == 0) )
         {
            uint16_t tmp;
            memcpy(&tmp, data, 2); data += 2;
            _trackingValid = tmp & 0x01;
         }
         // Mean marker error
         memcpy(&_mErr,data,4); data += 4;
      }
      
      return data;
   }
   
private:
   int _id;
   Point3f _loc;
   Quaternion4f _ori;
   // List of [x,y,z] positions of each marker.
   std::vector<Point3f> _markers;
   
   // NOTE: If NatNet.major >= 2
   // List of marker IDs (each uint32_t)
   std::vector<uint32_t> _mId;
   // List of marker sizes (each float)
   std::vector<float> _mSize;
   // Mean marker error
   float _mErr;

   // NOTE: If NatNet version >= 2.6
   bool _trackingValid;
};

//! \brief Output operator for RigidBody.
std::ostream& operator<<( std::ostream& s, RigidBody const& body )
{
   s
   << "    Rigid Body: " << body.id() << std::endl
   << "      loc: " << body.location()
   << "      ori: " << body.orientation() << std::endl;
   
   return s;
}

/*!
 * \brief A set of markers
 * \author Philip G. Lee
 */
class MarkerSet
{
public:
   //! \brief Default constructor
   MarkerSet() :
      _name(),
      _markers()
   {
   }
   
   ~MarkerSet(){}
   
   //! \brief Copy constructor
   MarkerSet( MarkerSet const& other ) :
      _name(other._name),
      _markers(other._markers)
   {
   }
   
   //! \brief Assignment operator
   MarkerSet& operator=( MarkerSet const& other )
   {
      _name = other._name;
      _markers = other._markers;
      return *this;
   }
   
   //! \brief The name of the set
   std::string const& name() const { return _name; }
   //! \brief Vector of markers making up the set
   std::vector<Point3f> const& markers() const { return _markers; }
   
   /*!
    * \brief Unpack the set from raw packed data
    * 
    * \param data pointer to packed data representing the MarkerSet
    * \returns pointer to data immediately following the MarkerSet data
    */
   char const* unpack(char const* data)
   {
      char n[256]; n[255] = '\0';
      int numMarkers;
      int i;
      float x,y,z;
      
      strncpy(n,data,sizeof(n)-1);
      _name = n;
      data += strlen(n)+1;
      
      memcpy(&numMarkers, data, 4); data += 4;
      for( i = 0; i < numMarkers; ++i )
      {
         memcpy(&x,data,4); data += 4;
         memcpy(&y,data,4); data += 4;
         memcpy(&z,data,4); data += 4;
         _markers.push_back(Point3f(x,y,z));
      }
      
      return data;
   }
   
private:
   
   std::string _name;
   std::vector<Point3f> _markers;
};

//! \brief Output operator to print human-readable text describin a MarkerSet
std::ostream& operator<<( std::ostream& s, MarkerSet const& set )
{
   size_t i, size;
   
   s
   << "    MarkerSet: '" << set.name() << "'" << std::endl;
   
   std::vector<Point3f> const& markers = set.markers();
   size = markers.size();
   for( i = 0; i < size; ++i )
      s << "      " << markers[i];
   s << std::endl;
   
   return s;
}

/*!
 * \brief A composition of rigid bodies
 * \author Philip G. Lee
 * 
 * A skeleton is simply a collection of RigidBody elements.
 */
class Skeleton
{
public:
   
   Skeleton() :
      _id(0),
      _rBodies()
   {
   }
   
   Skeleton( Skeleton const& other ) :
      _id(other._id),
      _rBodies(other._rBodies)
   {
   }
   
   ~Skeleton(){}
   
   //! \brief ID of this skeleton.
   int id() const { return _id; }
   //! \brief Vector of rigid bodies in this skeleton.
   std::vector<RigidBody> const& rigidBodies() const { return _rBodies; }
   
   /*!
    * \brief Unpack skeleton data from raw packed data.
    * 
    * \param data pointer to packed data representing a Skeleton
    * \param nnMajor major version of NatNet used to construct the packed data
    * \param nnMinor Minor version of NatNet packets used to read this frame
    * \returns pointer to data immediately following the Skeleton data
    */
   char const* unpack( char const* data, char nnMajor, char nnMinor )
   {
      int i;
      int numRigid = 0;
      
      memcpy(&_id,data,4); data += 4;
      memcpy(&numRigid,data,4); data += 4;
      for( i = 0; i < numRigid; ++i )
      {
         RigidBody b;
         data = b.unpack( data, nnMajor, nnMinor );
         _rBodies.push_back(b);
      }
      
      return data;
   }
   
private:
   int _id;
   std::vector<RigidBody> _rBodies;
};

/*!
 * \brief A labeled marker.
 * \author Philip G. Lee
 */
class LabeledMarker
{
public:
   
   //! \brief Default constructor.
   LabeledMarker() :
      _id(0),
      _p(),
      _size(0.f)
   {
   }
   
   ~LabeledMarker(){}
   
   //! \brief Copy constructor.
   LabeledMarker( LabeledMarker const& other ) :
      _id(other._id),
      _p(other._p),
      _size(other._size)
   {
   }
   
   //! \brief Assignment operator.
   LabeledMarker& operator=( LabeledMarker const& other )
   {
      _id = other._id;
      _p = other._p;
      _size = other._size;
      return *this;
   }
   
   //! \brief ID of this marker.
   int id() const { return _id; }
   //! \brief Location of this marker.
   Point3f location() const { return _p; }
   //! \brief Size of this marker.
   float size() const { return _size; }
   
   /*!
    * \brief Unpack the marker from packed data.
    * 
    * \param data pointer to packed data representing a labeled marker
    * \returns pointer to data immediately following the labeled marker data
    */
   char const* unpack( char const* data )
   {
      memcpy(&_id,data,4); data += 4;
      memcpy(&_p.x,data,4); data += 4;
      memcpy(&_p.y,data,4); data += 4;
      memcpy(&_p.z,data,4); data += 4;
      memcpy(&_size,data,4); data += 4;
      
      return data;
   }
   
private:
   int _id;
   Point3f _p;
   float _size;
};

/*!
 * \brief A complete frame of motion capture data.
 * \author Philip G. Lee
 */
class MocapFrame
{
public:
   
   /*!
    * \brief Constructor
    * 
    * Unless you want bad things to happen, specify the NatNet version
    * numbers correctly before you try to \c unpack().
    * 
    * \param nnMajor Major version of NatNet packets used to read this frame
    * \param nnMinor Minor version of NatNet packets used to read this frame
    */
   MocapFrame( unsigned char nnMajor=0, unsigned char nnMinor=0 ) :
      _nnMajor(nnMajor),
      _nnMinor(nnMinor),
      _frameNum(0),
      _numMarkerSets(0),
      _numRigidBodies(0)
   {
      
   }
   ~MocapFrame(){}
   
   //! \brief Copy constructor.
   MocapFrame( MocapFrame const& other ) :
      _nnMajor(other._nnMajor),
      _nnMinor(other._nnMinor),
      _frameNum(other._frameNum),
      _numMarkerSets(other._numMarkerSets),
      _markerSet(other._markerSet),
      _uidMarker(other._uidMarker),
      _numRigidBodies(other._numRigidBodies),
      _rBodies(other._rBodies),
      _skel(other._skel),
      _labeledMarkers(other._labeledMarkers),
      _latency(other._latency),
      _timecode(other._timecode),
      _subTimecode(other._subTimecode)
   {
      
   }
   
   //! \brief Assignment operator
   MocapFrame& operator=( MocapFrame const& other )
   {
      _nnMajor = other._nnMajor;
      _nnMinor = other._nnMinor;
      _frameNum = other._frameNum;
      _numMarkerSets = other._numMarkerSets;
      _markerSet = other._markerSet;
      _uidMarker = other._uidMarker;
      _numRigidBodies = other._numRigidBodies;
      _rBodies = other._rBodies;
      _skel = other._skel;
      _labeledMarkers = other._labeledMarkers;
      _latency = other._latency;
      _timecode = other._timecode;
      _subTimecode = other._subTimecode;
      
      return *this;
   }
   
   /*!
    * \brief Frame number.
    * 
    * Dustin Jakes at NaturalPoint says this is undefined in live capture mode,
    * and is the actual frame number in playback mode.
    */
   int frameNum() const { return _frameNum; }
   //! \brief All the sets of markers except unidentified ones.
   std::vector<MarkerSet> const& markerSets() const { return _markerSet; }
   //! \brief Set of unidentified markers.
   std::vector<Point3f> const& unIdMarkers() const { return _uidMarker; }
   //! \brief All the rigid bodies.
   std::vector<RigidBody> const& rigidBodies() const { return _rBodies; }
   /*!
    * \brief Either latency or timecode for the current frame.
    * 
    * Dustin Jakes at NaturalPoint says that this is an internal timecode from
    * Motive that represents the time at which the entire framegroup has
    * arrived from all the cameras.
    */
   float latency() const { return _latency; }
   /*!
    * \brief SMTPE timecode and sub-timecode.
    * 
    * \param timecode output timecode
    * \param subframe output subframe
    */
   void timecode( uint32_t& timecode, uint32_t& subframe ) const
   {
      timecode = _timecode;
      subframe = _subTimecode;
   }
   /*!
    * \brief Timecode decoded.
    * 
    * \param hour output timecode hour
    * \param minute output timecode minute
    * \param second output timecode second
    * \param frame output timecode frame
    * \param subFrame output timecode subframe
    */
   void timecode(
      int& hour,
      int& minute,
      int& second,
      int& frame,
      int& subFrame
   ) const
   {
      hour = (_timecode>>24)&0xFF;
      minute = (_timecode>>16)&0xFF;
      second = (_timecode>>8)&0xFF;
      frame = _timecode&0xFF;
      subFrame = _subTimecode;
   }
   
   /*!
    * \brief Unpack frame data from a packed buffer
    * 
    * \b WARNING: the NatNet version numbers must be correctly
    * specified in the constructor for this function to properly read the
    * data, as the data format depends on those version numbers.
    * 
    * \param data input data buffer
    * \returns pointer to data immediately following the frame data
    */
   char const* unpack(char const* data)
   {
      int i;
      int numUidMarkers;
      float x,y,z;
      
      //char const* const dataBeg = data;
      
      // NOTE: need to worry about network order here?
      
      // Get frame number.
      memcpy(&_frameNum, data, 4); data += 4;
      
      // Get marker sets.
      memcpy(&_numMarkerSets, data, 4); data += 4;
      for( i = 0; i < _numMarkerSets; ++i )
      {
         MarkerSet set;
         data = set.unpack(data);
         _markerSet.push_back(set);
      }
      
      // Get unidentified markers.
      memcpy(&numUidMarkers,data,4); data += 4;
      for( i = 0; i < numUidMarkers; ++i )
      {
         memcpy(&x,data,4); data += 4;
         memcpy(&y,data,4); data += 4;
         memcpy(&z,data,4); data += 4;
         _uidMarker.push_back(Point3f(x,y,z));
      }
      
      // Get rigid bodies
      _numRigidBodies = 0;
      memcpy(&_numRigidBodies,data,4); data += 4;
      for( i = 0; i < _numRigidBodies; ++i )
      {
         RigidBody b;
         data = b.unpack(data, _nnMajor, _nnMinor);
         _rBodies.push_back(b);
      }
      
      // Get skeletons (NatNet 2.1 and later)
      if( _nnMajor > 2 || (_nnMajor==2 && _nnMinor >= 1) )
      {
         int numSkel = 0;
         memcpy(&numSkel,data,4); data += 4;
         for( i = 0; i < numSkel; ++i )
         {
            Skeleton s;
            data = s.unpack( data, _nnMajor, _nnMinor );
            _skel.push_back(s);
         }
      }
      
      // Get labeled markers (NatNet 2.3 and later)
      if( _nnMajor > 2 || (_nnMajor==2 && _nnMinor >= 3) )
      {
         int numLabMark = 0;
         memcpy(&numLabMark,data,4); data += 4;
         for( i = 0; i < numLabMark; ++i )
         {
            LabeledMarker lm;
            data = lm.unpack(data);
            _labeledMarkers.push_back(lm);
         }
      }
      
      // Get latency/timecode
      memcpy(&_latency,data,4); data += 4;
      
      // Get timecode
      memcpy(&_timecode,data,4); data += 4;
      memcpy(&_subTimecode,data,4); data += 4;
      
      // Get "end of data" tag
      int eod = 0;
      memcpy(&eod,data,4); data += 4;
      
      return data;
   }
   
private:
   
   unsigned char _nnMajor;
   unsigned char _nnMinor;
   
   int _frameNum;
   int _numMarkerSets;
   // A list of marker sets. May subsume _numMarkerSets.
   std::vector<MarkerSet> _markerSet;
   // Set of unidentified markers.
   std::vector<Point3f> _uidMarker;
   int _numRigidBodies;
   // A list of rigid bodies.
   std::vector<RigidBody> _rBodies;
   // A list of skeletons.
   std::vector<Skeleton> _skel;
   // A list of labeled markers.
   std::vector<LabeledMarker> _labeledMarkers;
   // Latency
   float _latency;
   // Timestamp;
   uint32_t _timecode;
   uint32_t _subTimecode;
};

//! \brief For displaying human-readable MocapFrame data.
std::ostream& operator<<(std::ostream& s, MocapFrame const& frame)
{
   size_t i, size;
   std::ios::fmtflags flags = s.flags();
   
   s
   << "--Frame--" << std::endl
   << "  Frame #: " << frame.frameNum() << std::endl;
   
   std::vector<MarkerSet> const& markerSets = frame.markerSets();
   size = markerSets.size();
   
   s
   << "  Marker Sets: " << size << std::endl;
   for( i = 0; i < size; ++i )
      s << markerSets[i];
   
   s
   << "  Unidentified Markers: " << frame.unIdMarkers().size() << std::endl;
   
   std::vector<RigidBody> const& rBodies = frame.rigidBodies();
   size = rBodies.size();
   s
   << "  Rigid Bodies: " << size << std::endl;
   for( i = 0; i < size; ++i )
      s << rBodies[i];

   int hour,min,sec,fframe,subframe;
   frame.timecode(hour,min,sec,fframe,subframe);
   
   s.setf( s.fixed, s.floatfield );
   s.precision(4);
   s
   << "  Latency: " << frame.latency() << std::endl
   << "  Timecode: " << hour << ":" << min << ":" << sec << ":" << fframe << ":" << subframe << std::endl;
   
   s << "++Frame++" << std::endl;
   
   s.flags(flags);
   return s;
}

#endif /*NATNET_H*/
