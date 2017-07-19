/****************************************************************************

 Copyright (C) 2002-2014 Gilles Debunne. All rights reserved.

 This file is part of the QGLViewer library version 2.7.0.

 http://www.libqglviewer.com - contact@libqglviewer.com

 This file may be used under the terms of the GNU General Public License
 versions 2.0 or 3.0 as published by the Free Software Foundation and
 appearing in the LICENSE file included in the packaging of this file.
 In addition, as a special exception, Gilles Debunne gives you certain
 additional rights, described in the file GPL_EXCEPTION in this package.

 libQGLViewer uses dual licensing. Commercial/proprietary software must
 purchase a libQGLViewer Commercial License.

 This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.

*****************************************************************************/

#ifndef UTILITY_MATH_GEOMETRY_FRAME_H
#define UTILITY_MATH_GEOMETRY_FRAME_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace utility {
namespace math {
    namespace geometry {

        /*! \brief The Frame class represents a coordinate system, defined by a position
          and an orientation. \class Frame frame.h QGLViewer/frame.h

          A Frame is a 3D coordinate system, represented by a position() and an
          orientation(). The order of these transformations is important: the Frame is
          first translated \e and \e then rotated around the new translated origin.

          A Frame is useful to define the position and orientation of a 3D rigid object,
          using its matrix() method, as shown below: \code
          // Builds a Frame at position (0.5,0,0) and oriented such that its Y axis is
          along the (1,1,1)
          // direction. One could also have used setPosition() and setOrientation().
          Frame fr(Eigen::Vector3d(0.5,0,0), Eigen::Quaterniond(Eigen::Vector3d(0,1,0), Eigen::Vector3d(1,1,1)));
          glPushMatrix();
          glMultMatrixd(fr.matrix());
          // Draw your object here, in the local fr coordinate system.
          glPopMatrix();
          \endcode

          Many functions are provided to transform a 3D point from one coordinate system
          (Frame) to an other: see coordinatesOf(), inverseCoordinatesOf(),
          coordinatesOfIn(), coordinatesOfFrom()...

          You may also want to transform a 3D vector (such as a normal), which
          corresponds to applying only the rotational part of the frame transformation:
          see transformOf() and inverseTransformOf(). See the <a
          href="../examples/frameTransform.html">frameTransform example</a> for an
          illustration.

          The translation() and the rotation() that are encapsulated in a Frame can also
          be used to represent a \e rigid \e transformation of space. Such a
          transformation can also be interpreted as a change of coordinate system, and
          the coordinate system conversion functions actually allow you to use a Frame
          as a rigid transformation. Use inverseCoordinatesOf() (resp. coordinatesOf())
          to apply the transformation (resp. its inverse). Note the inversion.

          <h3>Hierarchy of Frames</h3>

          The position and the orientation of a Frame are actually defined with respect
          to a referenceFrame(). The default referenceFrame() is the world coordinate
          system (represented by a \c NULL referenceFrame()). If you setReferenceFrame()
          to a different Frame, you must then differentiate:

          \arg the \e local translation() and rotation(), defined with respect to the
          referenceFrame(),

          \arg the \e global position() and orientation(), always defined with respect
          to the world coordinate system.

          A Frame is actually defined by its translation() with respect to its
          referenceFrame(), and then by a rotation() of the coordinate system around the
          new translated origin.

          This terminology for \e local (translation() and rotation()) and \e global
          (position() and orientation()) definitions is used in all the methods' names
          and should be sufficient to prevent ambiguities. These notions are obviously
          identical when the referenceFrame() is \c NULL, i.e. when the Frame is defined
          in the world coordinate system (the one you are in at the beginning of the
          QGLViewer::draw() method, see the <a href="../introduction.html">introduction
          page</a>).

          Frames can hence easily be organized in a tree hierarchy, which root is the
          world coordinate system. A loop in the hierarchy would result in an
          inconsistent (multiple) Frame definition.
          settingAsReferenceFrameWillCreateALoop() checks this and prevents
          setReferenceFrame() from creating such a loop.

          This frame hierarchy is used in methods like coordinatesOfIn(),
          coordinatesOfFrom()... which allow coordinates (or vector) conversions from a
          Frame to any other one (including the world coordinate system).

          However, one must note that this hierarchical representation is internal to
          the Frame classes. When the Frames represent OpenGL coordinates system, one
          should map this hierarchical representation to the OpenGL GL_MODELVIEW matrix
          stack. See the matrix() documentation for details.

          <h3>Derived classes</h3>

          The ManipulatedFrame class inherits Frame and implements a mouse motion
          convertion, so that a Frame (and hence an object) can be manipulated in the
          scene with the mouse.

          \nosubgrouping */
        class Frame {
        public:
            /*! Creates a default Frame.

              Its position() is (0,0,0) and it has an identity orientation() Quaternion. The
              referenceFrame() and the constraint() are \c NULL. */
            Frame() : referenceFrame_(NULL) {}

            /*! Virtual destructor. Empty. */
            virtual ~Frame() {}

            Frame(const Frame& frame);
            Frame& operator=(const Frame& frame);

        public:
            /*! @name World coordinates position and orientation */
            //@{
            /*! Creates a Frame with a position() and an orientation().

             See the Eigen::Vector3d and Quaternion documentations for convenient constructors and
             methods.

             The Frame is defined in the world coordinate system (its referenceFrame() is \c
             NULL). It has a \c NULL associated constraint(). */
            Frame(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation)
                : t_(position), q_(orientation), referenceFrame_(NULL) {}

            void setPosition(const Eigen::Vector3d& position);
            void setPosition(double x, double y, double z);
            void setPositionWithConstraint(Eigen::Vector3d& position);

            void setOrientation(const Eigen::Quaterniond& orientation);
            void setOrientation(double q0, double q1, double q2, double q3);
            void setOrientationWithConstraint(Eigen::Quaterniond& orientation);

            void setPositionAndOrientation(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);
            void setPositionAndOrientationWithConstraint(Eigen::Vector3d& position, Eigen::Quaterniond& orientation);

            Eigen::Vector3d position() const;
            Eigen::Quaterniond orientation() const;

            void getPosition(double& x, double& y, double& z) const;
            void getOrientation(double& q0, double& q1, double& q2, double& q3) const;
            //@}

        public:
            /*! @name Local translation and rotation w/r reference Frame */
            //@{
            /*! Sets the translation() of the frame, locally defined with respect to the
            referenceFrame(). Emits the modified() signal.

            Use setPosition() to define the world coordinates position(). Use
            setTranslationWithConstraint() to take into account the potential constraint()
            of the Frame. */
            void setTranslation(const Eigen::Vector3d& translation) {
                t_ = translation;
            }

            void setTranslation(double x, double y, double z);
            void setTranslationWithConstraint(Eigen::Vector3d& translation);

            /*! Set the current rotation Eigen::Quaterniond. See rotation() and the different
            Eigen::Quaterniond constructors. Emits the modified() signal. See also
            setTranslation() and setRotationWithConstraint(). */

            /*! Sets the rotation() of the Frame, locally defined with respect to the
             referenceFrame(). Emits the modified() signal.

             Use setOrientation() to define the world coordinates orientation(). The
             potential constraint() of the Frame is not taken into account, use
             setRotationWithConstraint() instead. */
            void setRotation(const Eigen::Quaterniond& rotation) {
                q_ = rotation;
            }

            void setRotation(double q0, double q1, double q2, double q3);
            void setRotationWithConstraint(Eigen::Quaterniond& rotation);

            void setTranslationAndRotation(const Eigen::Vector3d& translation, const Eigen::Quaterniond& rotation);
            void setTranslationAndRotationWithConstraint(Eigen::Vector3d& translation, Eigen::Quaterniond& rotation);

            /*! Returns the Frame translation, defined with respect to the
            referenceFrame().

            Use position() to get the result in the world coordinates. These two values
            are identical when the referenceFrame() is \c NULL (default).

            See also setTranslation() and setTranslationWithConstraint(). */
            Eigen::Vector3d translation() const {
                return t_;
            }

            /*! Returns the Frame rotation, defined with respect to the referenceFrame().

            Use orientation() to get the result in the world coordinates. These two values
            are identical when the referenceFrame() is \c NULL (default).

            See also setRotation() and setRotationWithConstraint(). */

            /*! Returns the current Eigen::Quaterniond orientation. See setRotation(). */
            Eigen::Quaterniond rotation() const {
                return q_;
            }

            void getTranslation(double& x, double& y, double& z) const;
            void getRotation(double& q0, double& q1, double& q2, double& q3) const;
            //@}

        public:
            /*! @name Frame hierarchy */
            //@{
            /*! Returns the reference Frame, in which coordinates system the Frame is
            defined.

            The translation() and rotation() of the Frame are defined with respect to the
            referenceFrame() coordinate system. A \c NULL referenceFrame() (default value)
            means that the Frame is defined in the world coordinate system.

            Use position() and orientation() to recursively convert values along the
            referenceFrame() chain and to get values expressed in the world coordinate
            system. The values match when the referenceFrame() is \c NULL.

            Use setReferenceFrame() to set this value and create a Frame hierarchy.
            Convenient functions allow you to convert 3D coordinates from one Frame to an
            other: see coordinatesOf(), localCoordinatesOf(), coordinatesOfIn() and their
            inverse functions.

            Vectors can also be converted using transformOf(), transformOfIn,
            localTransformOf() and their inverse functions. */
            const Frame* referenceFrame() const {
                return referenceFrame_;
            }

            void setReferenceFrame(const Frame* const refFrame);
            bool settingAsReferenceFrameWillCreateALoop(const Frame* const frame);
            //@}

            /*! @name Frame modification */
            //@{
            void translate(Eigen::Vector3d& t);
            void translate(const Eigen::Vector3d& t);
            // Some compilers complain about "overloading cannot distinguish from previous
            // declaration" Simply comment out the following method and its associated
            // implementation
            void translate(double x, double y, double z);
            void translate(double& x, double& y, double& z);

            void rotate(Eigen::Quaterniond& q);
            void rotate(const Eigen::Quaterniond& q);
            // Some compilers complain about "overloading cannot distinguish from previous
            // declaration" Simply comment out the following method and its associated
            // implementation
            void rotate(double q0, double q1, double q2, double q3);
            void rotate(double& q0, double& q1, double& q2, double& q3);

            void rotateAroundPoint(Eigen::Quaterniond& rotation, const Eigen::Vector3d& point);
            void rotateAroundPoint(const Eigen::Quaterniond& rotation, const Eigen::Vector3d& point);

            void alignWithFrame(const Frame* const frame, bool move = false, double threshold = 0.0);
            void projectOnLine(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction);
            //@}

            /*! @name Coordinate system transformation of 3D coordinates */
            //@{
            Eigen::Vector3d coordinatesOf(const Eigen::Vector3d& src) const;
            Eigen::Vector3d inverseCoordinatesOf(const Eigen::Vector3d& src) const;
            Eigen::Vector3d localCoordinatesOf(const Eigen::Vector3d& src) const;
            Eigen::Vector3d localInverseCoordinatesOf(const Eigen::Vector3d& src) const;
            Eigen::Vector3d coordinatesOfIn(const Eigen::Vector3d& src, const Frame* const in) const;
            Eigen::Vector3d coordinatesOfFrom(const Eigen::Vector3d& src, const Frame* const from) const;

            void getCoordinatesOf(const double src[3], double res[3]) const;
            void getInverseCoordinatesOf(const double src[3], double res[3]) const;
            void getLocalCoordinatesOf(const double src[3], double res[3]) const;
            void getLocalInverseCoordinatesOf(const double src[3], double res[3]) const;
            void getCoordinatesOfIn(const double src[3], double res[3], const Frame* const in) const;
            void getCoordinatesOfFrom(const double src[3], double res[3], const Frame* const from) const;
            //@}

            /*! @name Coordinate system transformation of vectors */
            // A frame is as a new coordinate system, defined with respect to a reference
            // frame (the world coordinate system by default, see the "Composition of
            // frame" section).

            // The transformOf() (resp. inverseTransformOf()) functions transform a 3D
            // vector from (resp. to) the world coordinates system. This section defines
            // the 3D vector transformation functions. See the Coordinate system
            // transformation of 3D points above for the transformation of 3D points. The
            // difference between the two sets of functions is simple: for vectors, only
            // the rotational part of the transformations is taken into account, while
            // translation is also considered for 3D points.

            // The length of the resulting transformed vector is identical to the one of
            // the source vector for all the described functions.

            // When local is prepended to the names of the functions, the functions simply
            // transform from (and to) the reference frame.

            // When In (resp. From) is appended to the names, the functions transform from
            // (resp. To) the frame that is given as an argument. The frame does not need
            // to be in the same branch or the hierarchical tree, and can be \c NULL (the
            // world coordinates system).

            // Combining any of these functions with its inverse (in any order) leads to
            // the identity.
            //@{
            Eigen::Vector3d transformOf(const Eigen::Vector3d& src) const;
            Eigen::Vector3d inverseTransformOf(const Eigen::Vector3d& src) const;
            Eigen::Vector3d localTransformOf(const Eigen::Vector3d& src) const;
            Eigen::Vector3d localInverseTransformOf(const Eigen::Vector3d& src) const;
            Eigen::Vector3d transformOfIn(const Eigen::Vector3d& src, const Frame* const in) const;
            Eigen::Vector3d transformOfFrom(const Eigen::Vector3d& src, const Frame* const from) const;

            void getTransformOf(const double src[3], double res[3]) const;
            void getInverseTransformOf(const double src[3], double res[3]) const;
            void getLocalTransformOf(const double src[3], double res[3]) const;
            void getLocalInverseTransformOf(const double src[3], double res[3]) const;
            void getTransformOfIn(const double src[3], double res[3], const Frame* const in) const;
            void getTransformOfFrom(const double src[3], double res[3], const Frame* const from) const;
            //@}

            /*! @name Associated matrices */
            //@{
        public:
            /*! @name Inversion of the transformation */
            //@{
            Frame inverse() const;
            /*! Returns the inverse() of the Frame world transformation.

            The orientation() of the new Frame is the Eigen::Quaterniond::inverse() of the
            original orientation. Its position() is the negated and inverse rotated image
            of the original position.

            The result Frame has a \c NULL referenceFrame() and a \c NULL constraint().

            Use inverse() for a local (i.e. with respect to referenceFrame())
            transformation inverse. */
            Frame worldInverse() const {
                return Frame(-(orientation().inverse() * position()), orientation().inverse());
            }

        private:
            // Position and orientation
            Eigen::Vector3d t_;
            Eigen::Quaterniond q_;

            // Frame composition
            const Frame* referenceFrame_;
        };
    }
}
}

#endif  // UTILITY_MATH_GEOMETRY_FRAME_H
