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

#include "MocapFrame.h"

#include <cstring>

MocapFrame::MocapFrame(unsigned char nnMajor, unsigned char nnMinor)
  : _nnMajor(nnMajor)
  , _nnMinor(nnMinor)
  , _frameNum(0)
  , _numMarkerSets(0)
  , _numRigidBodies(0) {}

MocapFrame::MocapFrame(const MocapFrame& other)
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

MocapFrame& MocapFrame::operator=(const MocapFrame& other) {
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

const std::vector<MarkerSet>& MocapFrame::markerSets() const {
    return _markerSet;
}

const std::vector<Point3f>& MocapFrame::unIdMarkers() const {
    return _uidMarker;
}

const std::vector<RigidBody>& MocapFrame::rigidBodies() const {
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

const char* MocapFrame::unpack(const char* data)
{
    int i;
    int numUidMarkers;
    float x,y,z;

    //const char* const dataBeg = data;

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
        _uidMarker.push_back(Point3f{x, y, z});
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


std::ostream& operator<<(std::ostream& s, const MocapFrame& frame) {

    size_t i;
    size_t size;

    std::ios::fmtflags flags = s.flags();

    s
    << "--Frame--" << std::endl
    << "  Frame #: " << frame.frameNum() << std::endl;

    const std::vector<MarkerSet>& markerSets = frame.markerSets();
    size = markerSets.size();

    s
    << "  Marker Sets: " << size << std::endl;
    for(i = 0; i < size; ++i) {
        s << markerSets[i];
    }

    s
    << "  Unidentified Markers: " << frame.unIdMarkers().size() << std::endl;

    const std::vector<RigidBody>& rBodies = frame.rigidBodies();
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
