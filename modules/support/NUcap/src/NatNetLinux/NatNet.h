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
#include <iostream>
#include <vector>

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

/*!
 * \brief Simple 3D point
 * \author Philip G. Lee
 */
class Point3f {
public:
    Point3f(float xx = 0.f, float yy = 0.f, float zz = 0.f);

    ~Point3f();

    Point3f(Point3f const& other);

    Point3f& operator=(Point3f const& other);

    float x;
    float y;
    float z;
};

//! \brief Output operator for Point3f.
std::ostream& operator<<(std::ostream& s, Point3f const& point);

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
class Quaternion4f {
public:
    float qx;
    float qy;
    float qz;
    float qw;

    //! \brief Default constructor. Without parameters, returns the identity (no rotation).
    Quaternion4f(float qx = 0.f, float qy = 0.f, float qz = 0.f, float qw = 1.f);

    ~Quaternion4f();

    //! \brief Copy constructor
    Quaternion4f(Quaternion4f const& other);

    //! \brief Assignment operator
    Quaternion4f& operator=(Quaternion4f const& other);

    //! \brief Quaternion multiplication/assignment
    Quaternion4f& operator*=(Quaternion4f const& rhs);

    //! \brief Quaternion multiplication (rotation composition)
    Quaternion4f operator*(Quaternion4f const& rhs) const;

    //! \brief Quaternion division/assignment
    Quaternion4f& operator/=(Quaternion4f const& rhs);

    //! \brief Quaternion division
    Quaternion4f operator/(Quaternion4f const& rhs) const;

    //! \brief Rotate a point using the quaternion.
    Point3f rotate(Point3f const& p) const;

private:

    // If the magnitude of the quaternion exceeds a tolerance, renormalize it
    // to have magnitude of 1.
    void renormalize();
};

//! \brief Output operator for Quaternion4f.
std::ostream& operator<<(std::ostream& s, Quaternion4f const& q);

/*!
 * \brief Rigid body
 * \author Philip G. Lee
 *
 * This class is a composition of markers that describe a rigid body. The basic
 * traits of the rigid body are its 3D location() and orientation(). Rigid
 * bodies can be created in Optitrack's Motive:Tracker software.
 */
class RigidBody {
public:

    //! \brief Default constructor
    RigidBody();

    //! \brief Copy constructor
    RigidBody(RigidBody const& other);

    ~RigidBody();

    //! \brief Assignment operator
    RigidBody& operator=(RigidBody const& other);

    //! \brief ID of this RigidBody
    int id() const;
    //! \brief Location of this RigidBody
    Point3f location() const;
    //! \brief Orientation of this RigidBody
    Quaternion4f orientation() const;
    //! \brief Vector of markers that make up this RigidBody
    std::vector<Point3f> const& markers() const;
    //! \brief True if the tracking is valid. Used in NatNet version >= 2.6.
    bool trackingValid() const;

    /*!
     * \brief Unpack rigid body data from raw packed data.
     *
     * \param data pointer to packed data representing a RigidBody
     * \param nnMajor major version of NatNet used to construct the packed data
     * \param nnMinor Minor version of NatNet packets used to read this frame
     * \returns pointer to data immediately following the RigidBody data
     */
    char const* unpack(char const* data, char nnMajor, char nnMinor);

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
std::ostream& operator<<(std::ostream& s, RigidBody const& body);

/*!
 * \brief A set of markers
 * \author Philip G. Lee
 */
class MarkerSet {
public:
    //! \brief Default constructor
    MarkerSet();

    ~MarkerSet();

    //! \brief Copy constructor
    MarkerSet(MarkerSet const& other);

    //! \brief Assignment operator
    MarkerSet& operator=(MarkerSet const& other);

    //! \brief The name of the set
    std::string const& name() const;
    //! \brief Vector of markers making up the set
    std::vector<Point3f> const& markers() const;

    /*!
     * \brief Unpack the set from raw packed data
     *
     * \param data pointer to packed data representing the MarkerSet
     * \returns pointer to data immediately following the MarkerSet data
     */
    char const* unpack(char const* data);

private:

    std::string _name;
    std::vector<Point3f> _markers;
};

//! \brief Output operator to print human-readable text describin a MarkerSet
std::ostream& operator<<(std::ostream& s, MarkerSet const& set);

/*!
 * \brief A composition of rigid bodies
 * \author Philip G. Lee
 *
 * A skeleton is simply a collection of RigidBody elements.
 */
class Skeleton {
public:

    Skeleton();

    Skeleton(Skeleton const& other);

    ~Skeleton();

    //! \brief ID of this skeleton.
    int id() const;
    //! \brief Vector of rigid bodies in this skeleton.
    std::vector<RigidBody> const& rigidBodies() const;

    /*!
     * \brief Unpack skeleton data from raw packed data.
     *
     * \param data pointer to packed data representing a Skeleton
     * \param nnMajor major version of NatNet used to construct the packed data
     * \param nnMinor Minor version of NatNet packets used to read this frame
     * \returns pointer to data immediately following the Skeleton data
     */
    char const* unpack(char const* data, char nnMajor, char nnMinor);

private:
    int _id;
    std::vector<RigidBody> _rBodies;
};

/*!
 * \brief A labeled marker.
 * \author Philip G. Lee
 */
class LabeledMarker {
public:

    //! \brief Default constructor.
    LabeledMarker();

    ~LabeledMarker();

    //! \brief Copy constructor.
    LabeledMarker(LabeledMarker const& other);

    //! \brief Assignment operator.
    LabeledMarker& operator=(LabeledMarker const& other);

    //! \brief ID of this marker.
    int id() const;
    //! \brief Location of this marker.
    Point3f location() const;
    //! \brief Size of this marker.
    float size() const;

    /*!
     * \brief Unpack the marker from packed data.
     *
     * \param data pointer to packed data representing a labeled marker
     * \returns pointer to data immediately following the labeled marker data
     */
    char const* unpack(char const* data);

private:
    int _id;
    Point3f _p;
    float _size;
};

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
    MocapFrame(MocapFrame const& other);

    //! \brief Assignment operator
    MocapFrame& operator=(MocapFrame const& other);

    /*!
     * \brief Frame number.
     *
     * Dustin Jakes at NaturalPoint says this is undefined in live capture mode,
     * and is the actual frame number in playback mode.
     */
    int frameNum() const;
    //! \brief All the sets of markers except unidentified ones.
    std::vector<MarkerSet> const& markerSets() const;
    //! \brief Set of unidentified markers.
    std::vector<Point3f> const& unIdMarkers() const;
    //! \brief All the rigid bodies.
    std::vector<RigidBody> const& rigidBodies() const;

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
    char const* unpack(char const* data);

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
std::ostream& operator<<(std::ostream& s, MocapFrame const& frame);

#endif /*NATNET_H*/
