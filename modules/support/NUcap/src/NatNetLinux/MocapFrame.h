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

#ifndef MOCAPFRAME_H
#define MOCAPFRAME_H

#include "Point3f.h"
#include "LabeledMarker.h"
#include "RigidBody.h"
#include "MarkerSet.h"
#include "Skeleton.h"

#include <cstdint>
#include <iostream>
#include <vector>

/*!
 * \brief A complete frame of motion capture data.
 * \author Philip G. Lee
 */
class MocapFrame {
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
    MocapFrame(unsigned char nnMajor = 0, unsigned char nnMinor = 0);

    ~MocapFrame();

    //! \brief Copy constructor.
    MocapFrame(const MocapFrame& other);

    //! \brief Assignment operator
    MocapFrame& operator=(const MocapFrame& other);

    /*!
     * \brief Frame number.
     *
     * Dustin Jakes at NaturalPoint says this is undefined in live capture mode,
     * and is the actual frame number in playback mode.
     */
    int frameNum() const;
    //! \brief All the sets of markers except unidentified ones.
    const std::vector<MarkerSet>& markerSets() const;
    //! \brief Set of unidentified markers.
    const std::vector<Point3f>& unIdMarkers() const;
    //! \brief All the rigid bodies.
    const std::vector<RigidBody>& rigidBodies() const;

    /*!
     * \brief Either latency or timecode for the current frame.
     *
     * Dustin Jakes at NaturalPoint says that this is an internal timecode from
     * Motive that represents the time at which the entire framegroup has
     * arrived from all the cameras.
     */
    float latency() const;

    /*!
     * \brief SMTPE timecode and sub-timecode.
     *
     * \param timecode output timecode
     * \param subframe output subframe
     */
    void timecode(uint32_t& timecode, uint32_t& subframe) const;

    /*!
     * \brief Timecode decoded.
     *
     * \param hour output timecode hour
     * \param minute output timecode minute
     * \param second output timecode second
     * \param frame output timecode frame
     * \param subFrame output timecode subframe
     */
    void timecode(int& hour, int& minute, int& second, int& frame, int& subFrame ) const;

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
    const char* unpack(const char* data);

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
std::ostream& operator<<(std::ostream& s, const MocapFrame& frame);

#endif /*MOCAPFRAME_H*/
