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

#include "Frame.h"

#include <cmath>
#include <iostream>

namespace utility {
namespace math {
    namespace geometry {

        /*! Equal operator.

          The referenceFrame() and constraint() pointers are copied.

          \attention Signal and slot connections are not copied. */
        Frame& Frame::operator=(const Frame& frame) {
            // Automatic compiler generated version would not emit the modified() signals
            // as is done in setTranslationAndRotation.
            setTranslationAndRotation(frame.translation(), frame.rotation());
            setReferenceFrame(frame.referenceFrame());
            return *this;
        }

        /*! Copy constructor.

          The translation() and rotation() as well as constraint() and referenceFrame()
          pointers are copied. */
        Frame::Frame(const Frame& frame) {
            (*this) = frame;
        }

        /////////////////////////////// MATRICES //////////////////////////////////////

        /*! Returns a Frame representing the inverse of the Frame space transformation.

          The rotation() of the new Frame is the Eigen::Quaterniond::inverse() of the original
          rotation. Its translation() is the negated inverse rotated image of the
          original translation.

          If a Frame is considered as a space rigid transformation (translation and
          rotation), the inverse() Frame performs the inverse transformation.

          Only the local Frame transformation (i.e. defined with respect to the
          referenceFrame()) is inverted. Use worldInverse() for a global inverse.

          The resulting Frame has the same referenceFrame() as the Frame and a \c NULL
          constraint().

          \note The scaling factor of the 4x4 matrix is 1.0. */
        Frame Frame::inverse() const {
            Frame fr(-(q_.inverse() * t_), q_.inverse());
            fr.setReferenceFrame(referenceFrame());
            return fr;
        }

        //////////////////// SET AND GET LOCAL TRANSLATION AND ROTATION
        //////////////////////////////////

        /*! Same as setTranslation(), but with \p double parameters. */
        void Frame::setTranslation(double x, double y, double z) {
            setTranslation(Eigen::Vector3d(x, y, z));
        }

        /*! Fill \c x, \c y and \c z with the translation() of the Frame. */
        void Frame::getTranslation(double& x, double& y, double& z) const {
            const Eigen::Vector3d t = translation();
            x                       = t[0];
            y                       = t[1];
            z                       = t[2];
        }

        /*! Same as setRotation() but with \c double Eigen::Quaterniond parameters. */
        void Frame::setRotation(double q0, double q1, double q2, double q3) {
            setRotation(Eigen::Quaterniond(q0, q1, q2, q3));
        }

        /*! The \p q are set to the rotation() of the Frame.

        See Eigen::Quaterniond::Eigen::Quaterniond(double, double, double, double) for details on \c q. */
        void Frame::getRotation(double& q0, double& q1, double& q2, double& q3) const {
            const Eigen::Quaterniond q = rotation();
            q0                         = q.w();
            q1                         = q.x();
            q2                         = q.y();
            q3                         = q.z();
        }

        ////////////////////////////////////////////////////////////////////////////////

        /*! Translates the Frame of \p t (defined in the Frame coordinate system).

          The translation actually applied to the Frame may differ from \p t since it
          can be filtered by the constraint(). Use translate(Eigen::Vector3d&) or
          setTranslationWithConstraint() to retrieve the filtered translation value. Use
          setTranslation() to directly translate the Frame without taking the
          constraint() into account.

          See also rotate(const Eigen::Quaterniond&). Emits the modified() signal. */
        void Frame::translate(const Eigen::Vector3d& t) {
            Eigen::Vector3d tbis = t;
            translate(tbis);
        }

        /*! Same as translate(const Eigen::Vector3d&) but with \c double parameters. */
        void Frame::translate(double x, double y, double z) {
            Eigen::Vector3d t(x, y, z);
            translate(t);
        }

        /*! Same as translate(Eigen::Vector3d&) but with \c double parameters. */
        void Frame::translate(double& x, double& y, double& z) {
            Eigen::Vector3d t(x, y, z);
            translate(t);
            x = t[0];
            y = t[1];
            z = t[2];
        }

        /*! Rotates the Frame by \p q (defined in the Frame coordinate system): R = R*q.

          The rotation actually applied to the Frame may differ from \p q since it can
          be filtered by the constraint(). Use rotate(Eigen::Quaterniond&) or
          setRotationWithConstraint() to retrieve the filtered rotation value. Use
          setRotation() to directly rotate the Frame without taking the constraint()
          into account.

          See also translate(const Eigen::Vector3d&). Emits the modified() signal. */
        void Frame::rotate(const Eigen::Quaterniond& q) {
            Eigen::Quaterniond qbis = q;
            rotate(qbis);
        }

        /*! Same as rotate(Eigen::Quaterniond&) but with \c double Eigen::Quaterniond parameters. */
        void Frame::rotate(double& q0, double& q1, double& q2, double& q3) {
            Eigen::Quaterniond q(q0, q1, q2, q3);
            rotate(q);
            q0 = q.w();
            q1 = q.x();
            q2 = q.y();
            q3 = q.z();
        }

        /*! Same as rotate(const Eigen::Quaterniond&) but with \c double Eigen::Quaterniond parameters.
         */
        void Frame::rotate(double q0, double q1, double q2, double q3) {
            Eigen::Quaterniond q(q0, q1, q2, q3);
            rotate(q);
        }

        /*! Makes the Frame rotate() by \p rotation around \p point.

          \p point is defined in the world coordinate system, while the \p rotation axis
          is defined in the Frame coordinate system.

          If the Frame has a constraint(), \p rotation is first constrained using
          Constraint::constrainRotation(). The translation which results from the
          filtered rotation around \p point is then computed and filtered using
          Constraint::constrainTranslation(). The new \p rotation value corresponds to
          the rotation that has actually been applied to the Frame.

          Emits the modified() signal. */
        void Frame::rotateAroundPoint(Eigen::Quaterniond& rotation, const Eigen::Vector3d& point) {
            q_ *= rotation;
            q_.normalize();  // Prevents numerical drift
            Eigen::AngleAxisd aa(rotation);

            Eigen::Vector3d trans = point
                                    + Eigen::Quaterniond(Eigen::AngleAxisd(aa.angle(), inverseTransformOf(aa.axis())))
                                          * (position() - point)
                                    - t_;
            t_ += trans;
        }

        /*! Same as rotateAroundPoint(), but with a \c const \p rotation Eigen::Quaterniond.
          Note that the actual rotation may differ since it can be filtered by the
          constraint(). */
        void Frame::rotateAroundPoint(const Eigen::Quaterniond& rotation, const Eigen::Vector3d& point) {
            Eigen::Quaterniond rot = rotation;
            rotateAroundPoint(rot, point);
        }

        //////////////////// SET AND GET WORLD POSITION AND ORIENTATION
        //////////////////////////////////

        /*! Sets the position() of the Frame, defined in the world coordinate system.
        Emits the modified() signal.

        Use setTranslation() to define the \e local frame translation (with respect to
        the referenceFrame()). The potential constraint() of the Frame is not taken into
        account, use setPositionWithConstraint() instead. */
        void Frame::setPosition(const Eigen::Vector3d& position) {
            if (referenceFrame()) {
                setTranslation(referenceFrame()->coordinatesOf(position));
            }
            else {
                setTranslation(position);
            }
        }

        /*! Same as setPosition(), but with \c double parameters. */
        void Frame::setPosition(double x, double y, double z) {
            setPosition(Eigen::Vector3d(x, y, z));
        }

        /*! Same as successive calls to setPosition() and then setOrientation().

        Only one modified() signal is emitted, which is convenient if this signal is
        connected to a QGLViewer::update() slot. See also setTranslationAndRotation()
        and setPositionAndOrientationWithConstraint(). */
        void Frame::setPositionAndOrientation(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
            if (referenceFrame()) {
                t_ = referenceFrame()->coordinatesOf(position);
                q_ = referenceFrame()->orientation().inverse() * orientation;
            }
            else {
                t_ = position;
                q_ = orientation;
            }
        }

        /*! Same as successive calls to setTranslation() and then setRotation().

        Only one modified() signal is emitted, which is convenient if this signal is
        connected to a QGLViewer::update() slot. See also setPositionAndOrientation()
        and setTranslationAndRotationWithConstraint(). */
        void Frame::setTranslationAndRotation(const Eigen::Vector3d& translation, const Eigen::Quaterniond& rotation) {
            t_ = translation;
            q_ = rotation;
        }

        /*! \p x, \p y and \p z are set to the position() of the Frame. */
        void Frame::getPosition(double& x, double& y, double& z) const {
            Eigen::Vector3d p = position();
            x                 = p.x();
            y                 = p.y();
            z                 = p.z();
        }

        /*! Sets the orientation() of the Frame, defined in the world coordinate system.
        Emits the modified() signal.

        Use setRotation() to define the \e local frame rotation (with respect to the
        referenceFrame()). The potential constraint() of the Frame is not taken into
        account, use setOrientationWithConstraint() instead. */
        void Frame::setOrientation(const Eigen::Quaterniond& orientation) {
            if (referenceFrame()) {
                setRotation(referenceFrame()->orientation().inverse() * orientation);
            }
            else {
                setRotation(orientation);
            }
        }

        /*! Same as setOrientation(), but with \c double parameters. */
        void Frame::setOrientation(double q0, double q1, double q2, double q3) {
            setOrientation(Eigen::Quaterniond(q0, q1, q2, q3));
        }

        /*! Get the current orientation of the frame (same as orientation()).
          Parameters are the orientation Eigen::Quaterniond values.
          See also setOrientation(). */

        /*! The \p q are set to the orientation() of the Frame.

        See Eigen::Quaterniond::Eigen::Quaterniond(double, double, double, double) for details on \c q. */
        void Frame::getOrientation(double& q0, double& q1, double& q2, double& q3) const {
            Eigen::Quaterniond o = orientation();
            q0                   = o.w();
            q1                   = o.x();
            q2                   = o.y();
            q3                   = o.z();
        }

        /*! Returns the position of the Frame, defined in the world coordinate system.
           See also orientation(), setPosition() and translation(). */
        Eigen::Vector3d Frame::position() const {
            if (referenceFrame_) {
                return inverseCoordinatesOf(Eigen::Vector3d(0.0, 0.0, 0.0));
            }
            else {
                return t_;
            }
        }

        /*! Returns the orientation of the Frame, defined in the world coordinate
          system. See also position(), setOrientation() and rotation(). */
        Eigen::Quaterniond Frame::orientation() const {
            Eigen::Quaterniond res = rotation();
            const Frame* fr        = referenceFrame();
            while (fr != NULL) {
                res = fr->rotation() * res;
                fr  = fr->referenceFrame();
            }
            return res;
        }

        ////////////////////// C o n s t r a i n t   V e r s i o n s
        /////////////////////////////

        /*! Same as setTranslation(), but \p translation is modified so that the
          potential constraint() of the Frame is satisfied.

          Emits the modified() signal. See also setRotationWithConstraint() and
          setPositionWithConstraint(). */
        void Frame::setTranslationWithConstraint(Eigen::Vector3d& translation) {
            Eigen::Vector3d deltaT = translation - this->translation();
            setTranslation(this->translation() + deltaT);
            translation = this->translation();
        }

        /*! Same as setRotation(), but \p rotation is modified so that the potential
          constraint() of the Frame is satisfied.

          Emits the modified() signal. See also setTranslationWithConstraint() and
          setOrientationWithConstraint(). */
        void Frame::setRotationWithConstraint(Eigen::Quaterniond& rotation) {
            Eigen::Quaterniond deltaQ = this->rotation().inverse() * rotation;

            // Prevent numerical drift
            deltaQ.normalize();

            setRotation(this->rotation() * deltaQ);
            q_.normalize();
            rotation = this->rotation();
        }

        /*! Same as setTranslationAndRotation(), but \p translation and \p orientation
          are modified to satisfy the constraint(). Emits the modified() signal. */
        void Frame::setTranslationAndRotationWithConstraint(Eigen::Vector3d& translation,
                                                            Eigen::Quaterniond& rotation) {
            Eigen::Vector3d deltaT    = translation - this->translation();
            Eigen::Quaterniond deltaQ = this->rotation().inverse() * rotation;

            // Prevent numerical drift
            deltaQ.normalize();

            t_ += deltaT;
            q_ *= deltaQ;
            q_.normalize();

            translation = this->translation();
            rotation    = this->rotation();
        }

        /*! Same as setPosition(), but \p position is modified so that the potential
          constraint() of the Frame is satisfied. See also
          setOrientationWithConstraint() and setTranslationWithConstraint(). */
        void Frame::setPositionWithConstraint(Eigen::Vector3d& position) {
            if (referenceFrame()) {
                position = referenceFrame()->coordinatesOf(position);
            }

            setTranslationWithConstraint(position);
        }

        /*! Same as setOrientation(), but \p orientation is modified so that the
          potential constraint() of the Frame is satisfied. See also
          setPositionWithConstraint() and setRotationWithConstraint(). */
        void Frame::setOrientationWithConstraint(Eigen::Quaterniond& orientation) {
            if (referenceFrame()) {
                orientation = referenceFrame()->orientation().inverse() * orientation;
            }

            setRotationWithConstraint(orientation);
        }

        /*! Same as setPositionAndOrientation() but \p position and \p orientation are
        modified to satisfy the constraint. Emits the modified() signal. */
        void Frame::setPositionAndOrientationWithConstraint(Eigen::Vector3d& position,
                                                            Eigen::Quaterniond& orientation) {
            if (referenceFrame()) {
                position    = referenceFrame()->coordinatesOf(position);
                orientation = referenceFrame()->orientation().inverse() * orientation;
            }

            setTranslationAndRotationWithConstraint(position, orientation);
        }

        ///////////////////////////// REFERENCE FRAMES
        //////////////////////////////////////////

        /*! Sets the referenceFrame() of the Frame.

        The Frame translation() and rotation() are then defined in the referenceFrame()
        coordinate system. Use position() and orientation() to express these in the
        world coordinate system.

        Emits the modified() signal if \p refFrame differs from the current
        referenceFrame().

        Using this method, you can create a hierarchy of Frames. This hierarchy needs to
        be a tree, which root is the world coordinate system (i.e. a \c NULL
        referenceFrame()). A warning is printed and no action is performed if setting \p
        refFrame as the referenceFrame() would create a loop in the Frame hierarchy (see
        settingAsReferenceFrameWillCreateALoop()). */
        void Frame::setReferenceFrame(const Frame* const refFrame) {
            if (settingAsReferenceFrameWillCreateALoop(refFrame)) {
                std::cerr << "utility::math::geometry::Frame::setReferenceFrame would create a loop in Frame hierarchy"
                          << std::endl;
            }
            else {
                referenceFrame_ = refFrame;
            }
        }

        /*! Returns \c true if setting \p frame as the Frame's referenceFrame() would
          create a loop in the Frame hierarchy. */
        bool Frame::settingAsReferenceFrameWillCreateALoop(const Frame* const frame) {
            const Frame* f = frame;
            while (f != NULL) {
                if (f == this) {
                    return true;
                }
                f = f->referenceFrame();
            }
            return false;
        }

        ///////////////////////// FRAME TRANSFORMATIONS OF 3D POINTS
        /////////////////////////////////

        /*! Returns the Frame coordinates of a point \p src defined in the world
         coordinate system (converts from world to Frame).

         inverseCoordinatesOf() performs the inverse convertion. transformOf() converts
         3D vectors instead of 3D coordinates.

         See the <a href="../examples/frameTransform.html">frameTransform example</a>
         for an illustration. */
        Eigen::Vector3d Frame::coordinatesOf(const Eigen::Vector3d& src) const {
            if (referenceFrame()) {
                return localCoordinatesOf(referenceFrame()->coordinatesOf(src));
            }
            else {
                return localCoordinatesOf(src);
            }
        }

        /*! Returns the world coordinates of the point whose position in the Frame
          coordinate system is \p src (converts from Frame to world).

          coordinatesOf() performs the inverse convertion. Use inverseTransformOf() to
          transform 3D vectors instead of 3D coordinates. */
        Eigen::Vector3d Frame::inverseCoordinatesOf(const Eigen::Vector3d& src) const {
            const Frame* fr     = this;
            Eigen::Vector3d res = src;
            while (fr != NULL) {
                res = fr->localInverseCoordinatesOf(res);
                fr  = fr->referenceFrame();
            }
            return res;
        }

        /*! Returns the Frame coordinates of a point \p src defined in the
          referenceFrame() coordinate system (converts from referenceFrame() to Frame).

          localInverseCoordinatesOf() performs the inverse convertion. See also
          localTransformOf(). */
        Eigen::Vector3d Frame::localCoordinatesOf(const Eigen::Vector3d& src) const {
            return rotation().inverse() * (src - translation());
        }

        /*! Returns the referenceFrame() coordinates of a point \p src defined in the
         Frame coordinate system (converts from Frame to referenceFrame()).

         localCoordinatesOf() performs the inverse convertion. See also
         localInverseTransformOf(). */
        Eigen::Vector3d Frame::localInverseCoordinatesOf(const Eigen::Vector3d& src) const {
            return (rotation() * src) + translation();
        }

        /*! Returns the Frame coordinates of the point whose position in the \p from
          coordinate system is \p src (converts from \p from to Frame).

          coordinatesOfIn() performs the inverse transformation. */
        Eigen::Vector3d Frame::coordinatesOfFrom(const Eigen::Vector3d& src, const Frame* const from) const {
            if (this == from) {
                return src;
            }
            else if (referenceFrame()) {
                return localCoordinatesOf(referenceFrame()->coordinatesOfFrom(src, from));
            }
            else {
                return localCoordinatesOf(from->inverseCoordinatesOf(src));
            }
        }

        /*! Returns the \p in coordinates of the point whose position in the Frame
          coordinate system is \p src (converts from Frame to \p in).

          coordinatesOfFrom() performs the inverse transformation. */
        Eigen::Vector3d Frame::coordinatesOfIn(const Eigen::Vector3d& src, const Frame* const in) const {
            const Frame* fr     = this;
            Eigen::Vector3d res = src;
            while ((fr != NULL) && (fr != in)) {
                res = fr->localInverseCoordinatesOf(res);
                fr  = fr->referenceFrame();
            }

            if (fr != in) {
                // in was not found in the branch of this, res is now expressed in the world
                // coordinate system. Simply convert to in coordinate system.
                res = in->coordinatesOf(res);
            }

            return res;
        }

        ////// double[3] versions

        /*! Same as coordinatesOf(), but with \c double parameters. */
        void Frame::getCoordinatesOf(const double src[3], double res[3]) const {
            const Eigen::Vector3d r = coordinatesOf(Eigen::Vector3d(src));
            for (int i = 0; i < 3; ++i) {
                res[i] = r[i];
            }
        }

        /*! Same as inverseCoordinatesOf(), but with \c double parameters. */
        void Frame::getInverseCoordinatesOf(const double src[3], double res[3]) const {
            const Eigen::Vector3d r = inverseCoordinatesOf(Eigen::Vector3d(src));
            for (int i = 0; i < 3; ++i) {
                res[i] = r[i];
            }
        }

        /*! Same as localCoordinatesOf(), but with \c double parameters. */
        void Frame::getLocalCoordinatesOf(const double src[3], double res[3]) const {
            const Eigen::Vector3d r = localCoordinatesOf(Eigen::Vector3d(src));
            for (int i = 0; i < 3; ++i) {
                res[i] = r[i];
            }
        }

        /*! Same as localInverseCoordinatesOf(), but with \c double parameters. */
        void Frame::getLocalInverseCoordinatesOf(const double src[3], double res[3]) const {
            const Eigen::Vector3d r = localInverseCoordinatesOf(Eigen::Vector3d(src));
            for (int i = 0; i < 3; ++i) {
                res[i] = r[i];
            }
        }

        /*! Same as coordinatesOfIn(), but with \c double parameters. */
        void Frame::getCoordinatesOfIn(const double src[3], double res[3], const Frame* const in) const {
            const Eigen::Vector3d r = coordinatesOfIn(Eigen::Vector3d(src), in);
            for (int i = 0; i < 3; ++i) {
                res[i] = r[i];
            }
        }

        /*! Same as coordinatesOfFrom(), but with \c double parameters. */
        void Frame::getCoordinatesOfFrom(const double src[3], double res[3], const Frame* const from) const {
            const Eigen::Vector3d r = coordinatesOfFrom(Eigen::Vector3d(src), from);
            for (int i = 0; i < 3; ++i) {
                res[i] = r[i];
            }
        }

        ///////////////////////// FRAME TRANSFORMATIONS OF VECTORS
        /////////////////////////////////

        /*! Returns the Frame transform of a vector \p src defined in the world
         coordinate system (converts vectors from world to Frame).

         inverseTransformOf() performs the inverse transformation. coordinatesOf()
         converts 3D coordinates instead of 3D vectors (here only the rotational part of
         the transformation is taken into account).

         See the <a href="../examples/frameTransform.html">frameTransform example</a>
         for an illustration. */
        Eigen::Vector3d Frame::transformOf(const Eigen::Vector3d& src) const {
            if (referenceFrame()) {
                return localTransformOf(referenceFrame()->transformOf(src));
            }
            else {
                return localTransformOf(src);
            }
        }

        /*! Returns the world transform of the vector whose coordinates in the Frame
          coordinate system is \p src (converts vectors from Frame to world).

          transformOf() performs the inverse transformation. Use inverseCoordinatesOf()
          to transform 3D coordinates instead of 3D vectors. */
        Eigen::Vector3d Frame::inverseTransformOf(const Eigen::Vector3d& src) const {
            const Frame* fr     = this;
            Eigen::Vector3d res = src;
            while (fr != NULL) {
                res = fr->localInverseTransformOf(res);
                fr  = fr->referenceFrame();
            }
            return res;
        }

        /*! Returns the Frame transform of a vector \p src defined in the
          referenceFrame() coordinate system (converts vectors from referenceFrame() to
          Frame).

          localInverseTransformOf() performs the inverse transformation. See also
          localCoordinatesOf(). */
        Eigen::Vector3d Frame::localTransformOf(const Eigen::Vector3d& src) const {
            return rotation().inverse() * src;
        }

        /*! Returns the referenceFrame() transform of a vector \p src defined in the
         Frame coordinate system (converts vectors from Frame to referenceFrame()).

         localTransformOf() performs the inverse transformation. See also
         localInverseCoordinatesOf(). */
        Eigen::Vector3d Frame::localInverseTransformOf(const Eigen::Vector3d& src) const {
            return rotation() * src;
        }

        /*! Returns the Frame transform of the vector whose coordinates in the \p from
          coordinate system is \p src (converts vectors from \p from to Frame).

          transformOfIn() performs the inverse transformation. */
        Eigen::Vector3d Frame::transformOfFrom(const Eigen::Vector3d& src, const Frame* const from) const {
            if (this == from) {
                return src;
            }
            else if (referenceFrame()) {
                return localTransformOf(referenceFrame()->transformOfFrom(src, from));
            }
            else {
                return localTransformOf(from->inverseTransformOf(src));
            }
        }

        /*! Returns the \p in transform of the vector whose coordinates in the Frame
          coordinate system is \p src (converts vectors from Frame to \p in).

          transformOfFrom() performs the inverse transformation. */
        Eigen::Vector3d Frame::transformOfIn(const Eigen::Vector3d& src, const Frame* const in) const {
            const Frame* fr     = this;
            Eigen::Vector3d res = src;
            while ((fr != NULL) && (fr != in)) {
                res = fr->localInverseTransformOf(res);
                fr  = fr->referenceFrame();
            }

            if (fr != in) {
                // in was not found in the branch of this, res is now expressed in the world
                // coordinate system. Simply convert to in coordinate system.
                res = in->transformOf(res);
            }

            return res;
        }

        /////////////////  double[3] versions  //////////////////////

        /*! Same as transformOf(), but with \c double parameters. */
        void Frame::getTransformOf(const double src[3], double res[3]) const {
            Eigen::Vector3d r = transformOf(Eigen::Vector3d(src));
            for (int i = 0; i < 3; ++i) {
                res[i] = r[i];
            }
        }

        /*! Same as inverseTransformOf(), but with \c double parameters. */
        void Frame::getInverseTransformOf(const double src[3], double res[3]) const {
            Eigen::Vector3d r = inverseTransformOf(Eigen::Vector3d(src));
            for (int i = 0; i < 3; ++i) {
                res[i] = r[i];
            }
        }

        /*! Same as localTransformOf(), but with \c double parameters. */
        void Frame::getLocalTransformOf(const double src[3], double res[3]) const {
            Eigen::Vector3d r = localTransformOf(Eigen::Vector3d(src));
            for (int i = 0; i < 3; ++i) {
                res[i] = r[i];
            }
        }

        /*! Same as localInverseTransformOf(), but with \c double parameters. */
        void Frame::getLocalInverseTransformOf(const double src[3], double res[3]) const {
            Eigen::Vector3d r = localInverseTransformOf(Eigen::Vector3d(src));
            for (int i = 0; i < 3; ++i) {
                res[i] = r[i];
            }
        }

        /*! Same as transformOfIn(), but with \c double parameters. */
        void Frame::getTransformOfIn(const double src[3], double res[3], const Frame* const in) const {
            Eigen::Vector3d r = transformOfIn(Eigen::Vector3d(src), in);
            for (int i = 0; i < 3; ++i) {
                res[i] = r[i];
            }
        }

        /*! Same as transformOfFrom(), but with \c double parameters. */
        void Frame::getTransformOfFrom(const double src[3], double res[3], const Frame* const from) const {
            Eigen::Vector3d r = transformOfFrom(Eigen::Vector3d(src), from);
            for (int i = 0; i < 3; ++i) {
                res[i] = r[i];
            }
        }

        /////////////////////////////////   ALIGN   /////////////////////////////////

        /*! Aligns the Frame with \p frame, so that two of their axis are parallel.

        If one of the X, Y and Z axis of the Frame is almost parallel to any of the X,
        Y, or Z axis of \p frame, the Frame is rotated so that these two axis actually
        become parallel.

        If, after this first rotation, two other axis are also almost parallel, a second
        alignment is performed. The two frames then have identical orientations, up to
        90 degrees rotations.

        \p threshold measures how close two axis must be to be considered parallel. It
        is compared with the absolute values of the dot product of the normalized axis.
        As a result, useful range is sqrt(2)/2 (systematic alignment) to 1 (no
        alignment).

        When \p move is set to \c true, the Frame position() is also affected by the
        alignment. The new Frame's position() is such that the \p frame position
        (computed with coordinatesOf(), in the Frame coordinates system) does not
        change.

        \p frame may be \c NULL and then represents the world coordinate system (same
        convention than for the referenceFrame()).

        The rotation (and translation when \p move is \c true) applied to the Frame are
        filtered by the possible constraint(). */
        void Frame::alignWithFrame(const Frame* const frame, bool move, double threshold) {
            Eigen::Vector3d directions[2][3];
            for (unsigned short d = 0; d < 3; ++d) {
                Eigen::Vector3d dir((d == 0) ? 1.0 : 0.0, (d == 1) ? 1.0 : 0.0, (d == 2) ? 1.0 : 0.0);
                if (frame) {
                    directions[0][d] = frame->inverseTransformOf(dir);
                }
                else {
                    directions[0][d] = dir;
                }
                directions[1][d] = inverseTransformOf(dir);
            }

            double maxProj = 0.0;
            double proj;
            unsigned short index[2];
            index[0] = index[1] = 0;
            for (unsigned short i = 0; i < 3; ++i) {
                for (unsigned short j = 0; j < 3; ++j) {
                    if ((proj = std::abs(directions[0][i].dot(directions[1][j]))) >= maxProj) {
                        index[0] = i;
                        index[1] = j;
                        maxProj  = proj;
                    }
                }
            }

            Frame old;
            old = *this;

            double coef = directions[0][index[0]].dot(directions[1][index[1]]);
            if (fabs(coef) >= threshold) {
                const Eigen::Vector3d axis = (directions[0][index[0]].cross(directions[1][index[1]]));
                double angle               = std::asin(axis.norm());
                if (coef >= 0.0) {
                    angle = -angle;
                }
                rotate(rotation().inverse() * Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis)) * orientation());

                // Try to align an other axis direction
                unsigned short d = (index[1] + 1) % 3;
                Eigen::Vector3d dir((d == 0) ? 1.0 : 0.0, (d == 1) ? 1.0 : 0.0, (d == 2) ? 1.0 : 0.0);
                dir = inverseTransformOf(dir);

                double max = 0.0;
                for (unsigned short i = 0; i < 3; ++i) {
                    double proj = std::abs(directions[0][i].dot(dir));
                    if (proj > max) {
                        index[0] = i;
                        max      = proj;
                    }
                }

                if (max >= threshold) {
                    const Eigen::Vector3d axis = directions[0][index[0]].cross(dir);
                    double angle               = std::asin(axis.norm());
                    if (directions[0][index[0]].dot(dir) >= 0.0) {
                        angle = -angle;
                    }
                    rotate(rotation().inverse() * Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis)) * orientation());
                }
            }

            if (move) {
                Eigen::Vector3d center;
                if (frame) {
                    center = frame->position();
                }

                translate(center - (orientation() * old.coordinatesOf(center)) - translation());
            }
        }

        /*! Translates the Frame so that its position() lies on the line defined by \p
        origin and \p direction (defined in the world coordinate system).

        Simply uses an orthogonal projection. \p direction does not need to be
        normalized. */
        void Frame::projectOnLine(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction) {
            // If you are trying to find a bug here, because of memory problems, you waste
            // your time. This is a bug in the gcc 3.3 compiler. Compile the library in
            // debug mode and test. Uncommenting this line also seems to solve the
            // problem. Horrible. cout << "position = " << position() << endl; If you
            // found a problem or are using a different compiler, please let me know.
            const Eigen::Vector3d shift = origin - position();
            Eigen::Vector3d proj        = (shift.dot(direction) / direction.squaredNorm()) * direction;
            translate(shift - proj);
        }
    }
}
}
