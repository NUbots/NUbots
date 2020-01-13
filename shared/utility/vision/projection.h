#ifndef UTILITY_VISION_PROJECTION_H
#define UTILITY_VISION_PROJECTION_H

// https://wiki.panotools.org/Fisheye_Projection

#include <Eigen/Core>
#include <cmath>

namespace utility {
namespace vision {

    //  Unit Vector to Screen

    /**
     * @brief Projects the camera ray measured in coordinate system where x is forward down the camera axis, y is to the
     * left and z is up. The output coordinate system is one where the origin is the centre of the image, x is to the
     * left and y is up.
     *
     * @param ray   the ray to project to pixel coordinates
     * @param f     the focal length measured as a ratio of the width of the image
     *
     * @return  the position of the pixel measured as a fraction of the image width
     */
    template <typename T, int options, int max_rows_at_compile_time, int max_cols_at_compile_time>
    Eigen::Matrix<T, 2, 1> project_equidistant(
        const Eigen::Matrix<T, 3, 1, options, max_rows_at_compile_time, max_cols_at_compile_time>& ray,
        const T& f) {

        const T theta     = std::cos(ray.x());
        const T sin_theta = std::sin(theta);
        const T r         = f * theta;
        return Eigen::Matrix<T, 2, 1>(r * ray.y() / sin_theta, r * ray.z() / sin_theta);
    }

    template <typename T, int options, int max_rows_at_compile_time, int max_cols_at_compile_time>
    Eigen::Matrix<T, 2, 1> project_equisolid(
        const Eigen::Matrix<T, 3, 1, options, max_rows_at_compile_time, max_cols_at_compile_time>& ray,
        const T& f) {
        const T theta     = std::cos(ray.x());
        const T sin_theta = std::sin(theta);
        const T r         = T(2.0) * f * std::sin(theta * T(0.5));

        return Eigen::Matrix<T, 2, 1>(r * ray.y() / sin_theta, r * ray.z() / sin_theta);
    }

    template <typename T, int options, int max_rows_at_compile_time, int max_cols_at_compile_time>
    Eigen::Matrix<T, 2, 1> project_rectilinear(
        const Eigen::Matrix<T, 3, 1, options, max_rows_at_compile_time, max_cols_at_compile_time>& ray,
        const T& f) {
        return Eigen::Matrix<T, 2, 1>(f * (ray.y() / ray.x()), f * (ray.z() / ray.x()));
    }

    template <typename T, int options, int max_rows_at_compile_time, int max_cols_at_compile_time>
    Eigen::Matrix<T, 2, 1> project(
        const Eigen::Matrix<T, 3, 1, options, max_rows_at_compile_time, max_cols_at_compile_time>& ray,
        const message::input::Image::Lens& lens,
        const Eigen::Matrix<T, 2, 1>& dimensions) {

        Eigen::Matrix<T, 2, 1> projected;
        float fl = float(lens.focal_length * dimensions.x());

        switch (lens.projection.value) {
            case message::input::Image::Lens::Projection::RECTILINEAR: projected = project_rectilinear(ray, fl); break;
            case message::input::Image::Lens::Projection::EQUISOLID: projected = project_equisolid(ray, fl); break;
            case message::input::Image::Lens::Projection::EQUIDISTANT: projected = project_equidistant(ray, fl); break;
            default: throw std::runtime_error("Cannot project: Unknown lens type"); break;
        }
        projected.x() = -projected.x() + dimensions.x() / 2;
        projected.y() = projected.y() + dimensions.y() / 2;
        return projected;
    }

    /**
     * @brief Projects the pixel coordinates measured in coordinate system with origin in the centre of the image where
     * x is left and y is up. And projects to a coordinate system where x is forward, y is to the left and z is up.
     *
     * @param px    the coordinates of the pixel measured as a fraction of the width
     * @param f     the focal length measured as a ratio of the width of the image
     *
     * @return  the unit vector in the direction of this pixel
     */
    template <typename T, int options, int max_rows_at_compile_time, int max_cols_at_compile_time>
    Eigen::Matrix<T, 3, 1> unproject_equidistant(
        const Eigen::Matrix<T, 2, 1, options, max_rows_at_compile_time, max_cols_at_compile_time>& px,
        const T& f) {
        const T r = px.norm();
        // This calculation returns a number greater than 1, causing inf/nans in further trig functions:
        // Restricting theta to be max of 1.
        const T theta = std::max(std::min(r / f, T(1.0)), T(-1.0));

        const T sin_theta = std::sin(theta);
        Eigen::Matrix<T, 3, 1> A =
            Eigen::Matrix<T, 3, 1>(std::cos(theta), sin_theta * px.x() / r, sin_theta * px.y() / r);
        return A;
    }

    template <typename T, int options, int max_rows_at_compile_time, int max_cols_at_compile_time>
    Eigen::Matrix<T, 3, 1> unproject_equisolid(
        const Eigen::Matrix<T, 2, 1, options, max_rows_at_compile_time, max_cols_at_compile_time>& px,
        const T& f) {
        const T r         = px.norm();
        const T theta     = T(2.0) * std::asin(r / (T(2.0) * f));
        const T sin_theta = std::sin(theta);
        Eigen::Matrix<T, 3, 1> A =
            Eigen::Matrix<T, 3, 1>(std::cos(theta), sin_theta * px.x() / r, sin_theta * px.y() / r);

        return A;
    }

    template <typename T, int options, int max_rows_at_compile_time, int max_cols_at_compile_time>
    Eigen::Matrix<T, 3, 1> unproject_rectilinear(
        const Eigen::Matrix<T, 2, 1, options, max_rows_at_compile_time, max_cols_at_compile_time>& px,
        const T& f) {
        return Eigen::Matrix<T, 3, 1>(f, px.x(), px.y()).normalized();
    }

    template <typename T, int options, int max_rows_at_compile_time, int max_cols_at_compile_time>
    Eigen::Matrix<T, 3, 1> unproject(
        const Eigen::Matrix<T, 2, 1>& px,
        const message::input::Image::Lens& lens,
        const Eigen::Matrix<T, 2, 1, options, max_rows_at_compile_time, max_cols_at_compile_time>& dimensions) {
        // Transform to centre of the screen:
        Eigen::Matrix<T, 2, 1, options, max_rows_at_compile_time, max_cols_at_compile_time> transformed_px = px;

        transformed_px.x() = -(px.x() - dimensions.x() / 2);
        transformed_px.y() = px.y() - dimensions.y() / 2;

        float fl = float(lens.focal_length * dimensions.x());
        switch (lens.projection.value) {
            case message::input::Image::Lens::Projection::RECTILINEAR: return unproject_rectilinear(transformed_px, fl);
            case message::input::Image::Lens::Projection::EQUISOLID: return unproject_equisolid(transformed_px, fl);
            case message::input::Image::Lens::Projection::EQUIDISTANT: return unproject_equidistant(transformed_px, fl);

            default: throw std::runtime_error("Cannot project: Unknown lens type"); break;
        }
    }

}  // namespace vision
}  // namespace utility

#endif  // UTILITY_VISION_PROJECTION_H
