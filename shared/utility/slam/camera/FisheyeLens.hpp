/**
 * @file FisheyeLens.hpp
 * @brief NUbots-compatible fisheye camera projection (ray <-> pixel).
 *
 * A pixel-space wrapper over the NUbots vision pipeline's projection
 * (shared/utility/vision/projection.hpp), so that unit rays in the camera frame
 * {c} recorded in the log can be drawn back onto the source video, and pixels
 * can be unprojected to rays (for future out-of-field landmark work).
 *
 * Camera frame convention (NUbots): x is the optical axis (viewing direction),
 * y points to the left of the image, z points up. Pixel coordinates have (0,0)
 * at the top-left, x to the right, y down.
 *
 * The lens parameters (focal length, centre offset and distortion coefficients)
 * are normalised by the image width, exactly as in the NUbots camera configs
 * (module/input/Camera/data/config/<robot>/Cameras/Left.yaml) and in
 * message::input::Image::Lens. Prefer fromLens() to take them from the Image
 * message, which is correct for whichever robot is running. The compiled-in
 * defaults are sarah's calibration. Using another robots calibration (e.g.
 * kevin's) shifts and scales the projection so re-projected detections no longer
 * line up with the frame.
 */
#ifndef FISHEYELENS_HPP
#define FISHEYELENS_HPP

#include <Eigen/Core>
#include <algorithm>
#include <cmath>

#include "message/input/Image.hpp"

#include "utility/vision/projection.hpp"

namespace utility::slam::camera {

    /**
     * @brief Equidistant fisheye lens model with radial distortion (NUbots-compatible).
     *
     * @note This is *not* covered by Camera.hpp/cpp: `utility::slam::camera::Camera` is the
     * MCHA4400 OpenCV pinhole model (cameraMatrix fx/fy/cx/cy with rational + thin-prism
     * distortion, calibrated from camera.xml), which is a different projection entirely.
     *
     * The projection this class implements *is* already in the tree, as
     * `utility::vision::project`/`unproject` (shared/utility/vision/projection.hpp) — the
     * canonical NUbots implementation. The radial distortion polynomials below therefore
     * delegate to it rather than restating the coefficients. What this struct adds over
     * calling `utility::vision` directly is a pixel-in/pixel-out API (that utility works in
     * width-normalised units) and a lens fixed to the equidistant projection.
     */
    struct FisheyeLens {
        double width  = 1280.0;  ///< Image width [px]
        double height = 1024.0;  ///< Image height [px]

        // All of the following are normalised by the image width, per the NUbots convention.
        // These defaults are sarah's Left.yaml calibration (the robot that made the data2
        // recording) and exist so offline replay of that recording works with no
        // configuration. Live code should prefer fromLens(), which takes the calibration
        // from the Image message and so is correct for whichever robot is running.
        double focalLength = 0.34690945742400775;                             ///< Normalised focal length
        Eigen::Vector2d centre{0.02072339174622414, -0.0011612242293956145};  ///< Normalised optical-centre offset
        Eigen::Vector2d k{0.38553542593448015, 0.1498415334589703};  ///< Radial distortion coefficients [k1, k2]

        /**
         * @brief Build a lens from a live Image message's calibration.
         *
         * `Image::Lens` already stores focal length and centre normalised by image width,
         * so the fields transfer directly.
         *
         * @param lens The lens block of a message::input::Image
         * @param dimensions The image dimensions in pixels {width, height}
         */
        static FisheyeLens fromLens(const message::input::Image::Lens& lens,
                                    const Eigen::Matrix<unsigned int, 2, 1>& dimensions) {
            FisheyeLens l;
            l.width       = double(dimensions.x());
            l.height      = double(dimensions.y());
            l.focalLength = double(lens.focal_length);
            l.centre      = lens.centre.cast<double>();
            l.k           = lens.k.cast<double>();
            return l;
        }

        /// @brief Map an ideal (undistorted) radius to the distorted radius (used when projecting).
        double distort(double r) const {
            return utility::vision::distort<double>(r, *this);
        }

        /// @brief Map a distorted radius back to the ideal radius (used when unprojecting).
        double undistort(double r) const {
            return utility::vision::undistort<double>(r, *this);
        }

        /**
         * @brief Project a unit ray in {c} to a pixel coordinate (x right, y down).
         * @param ray Unit vector in the camera frame (x optical axis, y left, z up)
         * @return Pixel coordinate; check inFrontOfCamera()/inImage() for validity
         */
        Eigen::Vector2d project(const Eigen::Vector3d& ray) const {
            const double x        = std::clamp(ray.x(), -1.0, 1.0);
            const double theta    = std::acos(x);
            const double sinTheta = std::sqrt(std::max(1.0 - x * x, 1e-12));

            const double rUndist = focalLength * theta;  // Equidistant projection
            const double rDist   = distort(rUndist);     // Normalised distorted radius

            // Screen offset (normalised by width) in left/up axes, then to pixels.
            const double scale      = (sinTheta > 1e-9 ? rDist / sinTheta : 0.0) * width;
            const double screenLeft = scale * ray.y();
            const double screenUp   = scale * ray.z();

            return Eigen::Vector2d(width * 0.5 - screenLeft - centre.x() * width,
                                   height * 0.5 - screenUp - centre.y() * width);
        }

        /**
         * @brief Unproject a pixel coordinate (x right, y down) to a unit ray in {c}.
         */
        Eigen::Vector3d unproject(const Eigen::Vector2d& px) const {
            const double screenLeft = width * 0.5 - px.x() - centre.x() * width;
            const double screenUp   = height * 0.5 - px.y() - centre.y() * width;
            const double rDist      = std::sqrt(screenLeft * screenLeft + screenUp * screenUp) / width;
            if (rDist <= 0.0) {
                return Eigen::Vector3d::UnitX();
            }
            const double rUndist  = undistort(rDist);
            const double theta    = rUndist / focalLength;  // Equidistant inverse
            const double sinTheta = std::sin(theta);
            const double norm     = std::sqrt(screenLeft * screenLeft + screenUp * screenUp);
            return Eigen::Vector3d(std::cos(theta), sinTheta * screenLeft / norm, sinTheta * screenUp / norm);
        }

        /// @brief True if the ray points into the camera's forward hemisphere.
        static bool inFrontOfCamera(const Eigen::Vector3d& ray) {
            return ray.x() > 1e-3;
        }

        /// @brief True if a pixel lies within the image bounds.
        bool inImage(const Eigen::Vector2d& px) const {
            return px.x() >= 0.0 && px.x() < width && px.y() >= 0.0 && px.y() < height;
        }
    };

}  // namespace utility::slam::camera

#endif
