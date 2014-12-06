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

#include "NatNet.h"

struct sockaddr_in NatNet::createAddress(uint32_t inAddr, uint16_t port) {
    struct sockaddr_in ret;
    memset(&ret, 0, sizeof(ret));
    ret.sin_family = AF_INET;
    ret.sin_port = htons(port);
    ret.sin_addr.s_addr = inAddr;

    return ret;
}

int NatNet::createCommandSocket(uint32_t inAddr, uint16_t port) {
    // Asking for a buffer of 1MB = 2^20 bytes. This is what NP does, but this
    // seems far too large on Linux systems where the max is usually something
    // like 256 kB.
    const int rcvBufSize = 0x100000;
    int sd;
    int tmp=0;
    socklen_t len=0;
    struct sockaddr_in sockAddr = createAddress(inAddr, port);

    sd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sd < 0) {
        std::cerr << "Could not open socket. Error: " << errno << std::endl;
        exit(1);
    }

    // Bind socket to the address.
    tmp = bind( sd, (struct sockaddr*)&sockAddr, sizeof(sockAddr) );
    if(tmp < 0) {
        std::cerr << "Could not bind socket. Error: " << errno << std::endl;
        close(sd);
        exit(1);
    }

    int value = 1;
    tmp = setsockopt( sd, SOL_SOCKET, SO_BROADCAST, (char*)&value, sizeof(value) );
    if(tmp < 0) {
        std::cerr << "Could not set socket to broadcast mode. Error: " << errno << std::endl;
        close(sd);
        exit(1);
    }

    setsockopt(sd, SOL_SOCKET, SO_RCVBUF, (char*)&rcvBufSize, sizeof(rcvBufSize));
    getsockopt(sd, SOL_SOCKET, SO_RCVBUF, (char*)&tmp, &len);
    if(tmp != rcvBufSize) {
        std::cerr << "WARNING: Could not set receive buffer size. Asked for "
                  << rcvBufSize << "B got " << tmp << "B" << std::endl;
    }

    return sd;
}

int NatNet::createDataSocket(uint32_t inAddr, uint16_t port, uint32_t multicastAddr) {
    int sd;
    int value;
    int tmp;
    struct ip_mreq group;
    struct sockaddr_in localSock = createAddress(INADDR_ANY, port);

    sd = socket(AF_INET, SOCK_DGRAM, 0);
    value = 1;
    tmp = setsockopt(sd, SOL_SOCKET, SO_REUSEADDR, (char*)&value, sizeof(value));
    if(tmp < 0) {
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
    if(tmp < 0) {
       std::cerr << "ERROR: Could not add the interface to the multicast group." << std::endl;
       close(sd);
       return -1;
    }

    return sd;
}

Point3f::Point3f(float xx, float yy, float zz)
  : x(xx), y(yy), z(zz) {}

Point3f::Point3f(Point3f const& other)
  : x(other.x)
  , y(other.y)
  , z(other.z) {}

Point3f::~Point3f() {}

Point3f& Point3f::operator=(const Point3f& other) {
    // Self-assignment no problem
    x = other.x;
    y = other.y;
    z = other.z;

    return *this;
}

std::ostream& operator<<(std::ostream& s, const Point3f& point) {
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

Quaternion4f::Quaternion4f(float qx, float qy, float qz, float qw)
  : qx(qx)
  , qy(qy)
  , qz(qz)
  , qw(qw) {

    renormalize();
}

Quaternion4f::Quaternion4f(const Quaternion4f& other)
  : qx(other.qx)
  , qy(other.qy)
  , qz(other.qz)
  , qw(other.qw) {}

Quaternion4f::~Quaternion4f(){}

Quaternion4f& Quaternion4f::operator=(const Quaternion4f& other) {
    qx = other.qx;
    qy = other.qy;
    qz = other.qz;
    qw = other.qw;

    return *this;
}

Quaternion4f& Quaternion4f::operator*=(const Quaternion4f& rhs) {
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

Quaternion4f Quaternion4f::operator*(const Quaternion4f& rhs) const {
    Quaternion4f ret(*this);

    ret *= rhs;

    return ret;
}

Quaternion4f& Quaternion4f::operator/=(Quaternion4f const& rhs) {
    // Create the conjugate and multiply.
    Quaternion4f rhsConj(-rhs.qx, -rhs.qy, -rhs.qy, rhs.qw);
    *this *= rhsConj;

    return *this;
}

Quaternion4f Quaternion4f::operator/(Quaternion4f const& rhs) const {
    Quaternion4f ret(*this);
    ret /= rhs;
    return ret;
}

Point3f Quaternion4f::rotate(Point3f const& p) const {
    Point3f pout;

    pout.x = (1.f-2.f*qy*qy-2.f*qz*qz)*p.x + (2.f*qx*qy-2.f*qw*qz)*p.y + (2.f*qx*qz+2.f*qw*qy)*p.z;
    pout.y = (2.f*qx*qy+2.f*qw*qz)*p.x + (1.f-2.f*qx*qx-2.f*qz*qz)*p.y + (2.f*qy*qz+2.f*qw*qx)*p.z;
    pout.z = (2.f*qx*qz-2.f*qw*qy)*p.x + (2.f*qy*qz-2.f*qw*qx)*p.y + (1.f-2.f*qx*qx-2.f*qy*qy)*p.z;

    return pout;
}

void Quaternion4f::renormalize() {
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

std::ostream& operator<<(std::ostream& s, const Quaternion4f& q) {

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

RigidBody::RigidBody()
  : _id(-1)
  , _loc()
  , _ori()
  , _markers()
  , _mId()
  , _mSize()
  , _mErr()
  , _trackingValid(true) {
}

RigidBody::RigidBody(const RigidBody & other)
  : _id(other._id)
  , _loc(other._loc)
  , _ori(other._ori)
  , _markers(other._markers)
  , _mId(other._mId)
  , _mSize(other._mSize)
  , _mErr(other._mErr)
  , _trackingValid(other._trackingValid) {
}

RigidBody::~RigidBody(){}

RigidBody& RigidBody::operator=(const RigidBody& other) {
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

int RigidBody::id() const {
    return _id;
}

Point3f RigidBody::location() const {
    return _loc;
}

Quaternion4f RigidBody::orientation() const {
    return _ori;
}

const std::vector<Point3f>& RigidBody::markers() const {
    return _markers;
}

bool RigidBody::trackingValid() const {
    return _trackingValid;
}

char const* RigidBody::unpack(const char* data, char nnMajor, char nnMinor) {
    int i;
    float x,y,z;

    // Rigid body ID
    memcpy(&_id,data,4); data += 4;

    // Location and orientation.
    memcpy(&_loc.x, data, 4); data += 4;
    memcpy(&_loc.y, data, 4); data += 4;
    memcpy(&_loc.z, data, 4); data += 4;
    memcpy(&_ori.qx, data, 4); data += 4;
    memcpy(&_ori.qy, data, 4); data += 4;
    memcpy(&_ori.qz, data, 4); data += 4;
    memcpy(&_ori.qw, data, 4); data += 4;

    // Associated markers
    int nMarkers = 0;
    memcpy(&nMarkers, data, 4); data += 4;
    for(i = 0; i < nMarkers; ++i) {
        memcpy(&x, data, 4); data += 4;
        memcpy(&y, data, 4); data += 4;
        memcpy(&z, data, 4); data += 4;
        _markers.push_back(Point3f(x,y,z));
    }

    if(nnMajor >= 2) {
        // Marker IDs
        uint32_t id = 0;
        for(i = 0; i < nMarkers; ++i) {
            memcpy(&id,data,4); data += 4;
            _mId.push_back(id);
        }

        // Marker sizes
        float size;
        for(i = 0; i < nMarkers; ++i) {
            memcpy(&size,data,4); data += 4;
            _mSize.push_back(size);
        }

        if(((nnMajor==2) && (nnMinor >= 6)) || (nnMajor > 2) || (nnMajor == 0)) {
            uint16_t tmp;
            memcpy(&tmp, data, 2); data += 2;
            _trackingValid = tmp & 0x01;
        }

        // Mean marker error
        memcpy(&_mErr,data,4); data += 4;
    }

    return data;
}

std::ostream& operator<<(std::ostream& s, const RigidBody& body) {
    s
    << "    Rigid Body: " << body.id() << std::endl
    << "      loc: " << body.location()
    << "      ori: " << body.orientation() << std::endl;

    return s;
}

MarkerSet::MarkerSet()
  : _name()
  , _markers() {}

MarkerSet::~MarkerSet(){}

MarkerSet::MarkerSet(const MarkerSet & other)
  : _name(other._name)
  , _markers(other._markers) {}

MarkerSet& MarkerSet::operator=(const MarkerSet& other) {
    _name = other._name;
    _markers = other._markers;
    return *this;
}

const std::string& MarkerSet::name() const {
    return _name;
}

const std::vector<Point3f>& MarkerSet::markers() const {
    return _markers;
}

char const* MarkerSet::unpack(char const* data) {
    char n[256];
    n[255] = '\0';

    int numMarkers;
    int i;
    float x, y, z;

    strncpy(n, data, sizeof(n)-1);
    _name = n;
    data += strlen(n) + 1;

    memcpy(&numMarkers, data, 4); data += 4;
    for(i = 0; i < numMarkers; ++i) {
        memcpy(&x, data, 4); data += 4;
        memcpy(&y, data, 4); data += 4;
        memcpy(&z, data, 4); data += 4;
        _markers.push_back(Point3f(x, y, z));
    }

    return data;
}

//! \brief Output operator to print human-readable text describin a MarkerSet
std::ostream& operator<<(std::ostream& s, MarkerSet const& set) {
    size_t i, size;

    s
    << "    MarkerSet: '" << set.name() << "'" << std::endl;

    std::vector<Point3f> const& markers = set.markers();
    size = markers.size();

    for(i = 0; i < size; ++i) {
        s << "      " << markers[i];
    }

    s << std::endl;

    return s;
}

Skeleton::Skeleton()
  : _id(0)
  , _rBodies() {}

Skeleton::Skeleton(const Skeleton & other)
  : _id(other._id)
  , _rBodies(other._rBodies) {}

Skeleton::~Skeleton() {}

int Skeleton::id() const {
    return _id;
}

const std::vector<RigidBody>& Skeleton::rigidBodies() const {
    return _rBodies;
}

char const* Skeleton::unpack(char const* data, char nnMajor, char nnMinor) {
    int i;
    int numRigid = 0;

    memcpy(&_id,data,4); data += 4;
    memcpy(&numRigid,data,4); data += 4;

    for(i = 0; i < numRigid; ++i) {
        RigidBody b;
        data = b.unpack( data, nnMajor, nnMinor );
        _rBodies.push_back(b);
    }

    return data;
}

LabeledMarker::LabeledMarker()
  : _id(0)
  , _p()
  , _size(0.f) {}

LabeledMarker::~LabeledMarker() {}

LabeledMarker::LabeledMarker(LabeledMarker const& other)
  : _id(other._id)
  , _p(other._p)
  , _size(other._size) {}

LabeledMarker& LabeledMarker::operator=(LabeledMarker const& other) {
    _id = other._id;
    _p = other._p;
    _size = other._size;
    return *this;
}

int LabeledMarker::id() const {
    return _id;
}

Point3f LabeledMarker::location() const {
    return _p;
}

float LabeledMarker::size() const {
    return _size;
}

char const* LabeledMarker::unpack( char const* data ) {
    memcpy(&_id, data, 4); data += 4;
    memcpy(&_p.x, data, 4); data += 4;
    memcpy(&_p.y, data, 4); data += 4;
    memcpy(&_p.z, data, 4); data += 4;
    memcpy(&_size, data, 4); data += 4;

    return data;
}

MocapFrame::MocapFrame(unsigned char nnMajor, unsigned char nnMinor)
  : _nnMajor(nnMajor)
  , _nnMinor(nnMinor)
  , _frameNum(0)
  , _numMarkerSets(0)
  , _numRigidBodies(0) {}


MocapFrame::MocapFrame(MocapFrame const& other)
  : _nnMajor(other._nnMajor)
  , _nnMinor(other._nnMinor)
  , _frameNum(other._frameNum)
  , _numMarkerSets(other._numMarkerSets)
  , _markerSet(other._markerSet)
  , _uidMarker(other._uidMarker)
  , _numRigidBodies(other._numRigidBodies)
  , _rBodies(other._rBodies)
  , _skel(other._skel)
  , _labeledMarkers(other._labeledMarkers)
  , _latency(other._latency)
  , _timecode(other._timecode)
  , _subTimecode(other._subTimecode) {}

MocapFrame::~MocapFrame() {}

MocapFrame& MocapFrame::operator=(MocapFrame const& other) {
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

int MocapFrame::frameNum() const {
    return _frameNum;
}

std::vector<MarkerSet> const& MocapFrame::markerSets() const {
    return _markerSet;
}

std::vector<Point3f> const& MocapFrame::unIdMarkers() const {
    return _uidMarker;
}

std::vector<RigidBody> const& MocapFrame::rigidBodies() const {
    return _rBodies;
}

float MocapFrame::latency() const {
    return _latency;
}

void MocapFrame::timecode(uint32_t& timecode, uint32_t& subframe) const {
    timecode = _timecode;
    subframe = _subTimecode;
}

void MocapFrame::timecode(int& hour, int& minute, int& second, int& frame, int& subFrame) const {
    hour = (_timecode>>24)&0xFF;
    minute = (_timecode>>16)&0xFF;
    second = (_timecode>>8)&0xFF;
    frame = _timecode&0xFF;
    subFrame = _subTimecode;
}

char const* MocapFrame::unpack(char const* data)
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
    for(i = 0; i < _numMarkerSets; ++i) {
        MarkerSet set;
        data = set.unpack(data);
        _markerSet.push_back(set);
    }

    // Get unidentified markers.
    memcpy(&numUidMarkers,data,4); data += 4;
    for(i = 0; i < numUidMarkers; ++i) {
        memcpy(&x,data,4); data += 4;
        memcpy(&y,data,4); data += 4;
        memcpy(&z,data,4); data += 4;
        _uidMarker.push_back(Point3f(x,y,z));
    }

    // Get rigid bodies
    _numRigidBodies = 0;
    memcpy(&_numRigidBodies,data,4); data += 4;
    for(i = 0; i < _numRigidBodies; ++i) {
        RigidBody b;
        data = b.unpack(data, _nnMajor, _nnMinor);
        _rBodies.push_back(b);
    }

    // Get skeletons (NatNet 2.1 and later)
    if(_nnMajor > 2 || (_nnMajor==2 && _nnMinor >= 1)) {
        int numSkel = 0;
        memcpy(&numSkel,data,4); data += 4;

        for(i = 0; i < numSkel; ++i) {
            Skeleton s;
            data = s.unpack( data, _nnMajor, _nnMinor );
            _skel.push_back(s);
        }
    }

    // Get labeled markers (NatNet 2.3 and later)
    if(_nnMajor > 2 || (_nnMajor==2 && _nnMinor >= 3)) {

        int numLabMark = 0;
        memcpy(&numLabMark,data,4); data += 4;
        for(i = 0; i < numLabMark; ++i) {
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


std::ostream& operator<<(std::ostream& s, MocapFrame const& frame) {

    size_t i;
    size_t size;

    std::ios::fmtflags flags = s.flags();

    s
    << "--Frame--" << std::endl
    << "  Frame #: " << frame.frameNum() << std::endl;

    std::vector<MarkerSet> const& markerSets = frame.markerSets();
    size = markerSets.size();

    s
    << "  Marker Sets: " << size << std::endl;
    for(i = 0; i < size; ++i) {
        s << markerSets[i];
    }

    s
    << "  Unidentified Markers: " << frame.unIdMarkers().size() << std::endl;

    std::vector<RigidBody> const& rBodies = frame.rigidBodies();
    size = rBodies.size();
    s
    << "  Rigid Bodies: " << size << std::endl;
    for(i = 0; i < size; ++i) {
       s << rBodies[i];
    }

    int hour,min,sec,fframe,subframe;

    frame.timecode(hour,min,sec,fframe,subframe);

    s.setf(s.fixed, s.floatfield);

    s.precision(4);
    s
    << "  Latency: " << frame.latency() << std::endl
    << "  Timecode: " << hour << ":" << min << ":" << sec << ":" << fframe << ":" << subframe << std::endl;

    s << "++Frame++" << std::endl;

    s.flags(flags);
    return s;
}
