#ifndef UTILITY_VISION_PROJECTION_HPP
#define UTILITY_VISION_PROJECTION_HPP

// https://wiki.panotools.org/Fisheye_Projection

#include <Eigen/Core>
#include <cmath>

namespace utility::vision {

    template <typename T>
    inline T equidistant_theta(const T& r, const T& f) {
        return r / f;
    }

    template <typename T>
    inline T equidistant_r(const T& theta, const T& f) {
        return f * theta;
    }

    template <typename T>
    inline T equisolid_theta(const T& r, const T& f) {
        return T(2.0) * std::asin(r / (T(2.0) * f));
    }

    template <typename T>
    inline T equisolid_r(const T& theta, const T& f) {
        return T(2.0) * f * std::sin(theta * T(0.5));
    }

    template <typename T>
    inline T rectilinear_theta(const T& r, const T& f) {
        return std::atan(r / f);
    }

    template <typename T>
    inline T rectilinear_r(const T& theta, const T& f) {
        return f * std::tan(theta);
    }

    template <typename T>
    inline __attribute__((optimize("-ffast-math"))) T distort(const T& r, const message::input::Image::Lens& lens) {
        // Uses the math from the paper
        // An Exact Formula for Calculating Inverse Radial Lens Distortions
        // https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4934233/pdf/sensors-16-00807.pdf

        // Extract constants
        const T& k1 = lens.k[0];
        const T& k2 = lens.k[1];

        // Compute powers
        const T k1_2 = k1 * k1;
        const T k1_3 = k1_2 * k1;
        const T k1_4 = k1_3 * k1;
        const T k1_5 = k1_4 * k1;
        const T k1_6 = k1_5 * k1;
        const T k1_7 = k1_6 * k1;
        const T k1_8 = k1_7 * k1;
        const T k1_9 = k1_8 * k1;

        const T k2_2 = k2 * k2;
        const T k2_3 = k2_2 * k2;
        const T k2_4 = k2_3 * k2;

        // These terms have been stripped back to only include k1 and k2
        // if more are needed in the future go and get them from the original paper
        // TODO(VisionTeam): if performance ever becomes an issue, this can be precomputed for the same k values
        // clang-format off
        const T b1 = -k1;
        const T b2 = 3.0*k1_2 - k2;
        const T b3 = -12.0*k1_3 + 8.0*k1*k2;
        const T b4 = 55.0*k1_4 - 55.0*k1_2*k2 + 5.0*k2_2;
        const T b5 = -273.0*k1_5 + 364.0*k1_3*k2 - 78.0*k1*k2_2;
        const T b6 = 1428.0*k1_6 - 2380.0*k1_4*k2 + 840.0*k1_2*k2_2 - 35.0*k2_3;
        const T b7 = -7752.0*k1_7 + 15504.0*k1_5*k2 - 7752.0*k1_3*k2_2 + 816.0*k1*k2_3;
        const T b8 = 43263.0*k1_8 - 100947.0*k1_6*k2 + 65835.0*k1_4*k2_2 - 11970.0*k1_2*k2_3 + 285.0*k2_4;
        const T b9 = -246675.0*k1_9 + 657800.0*k1_7*k2 - 531300.0*k1_5*k2_2 + 141680.0*k1_3*k2_3 - 8855.0*k1*k2_4;
        // clang-format on

        // Perform the undistortion
        const T r2  = r * r;
        const T r4  = r2 * r2;
        const T r8  = r4 * r4;
        const T r16 = r8 * r8;
        return r
               * (1.0                  //
                  + b1 * r2            //
                  + b2 * r4            //
                  + b3 * r4 * r2       //
                  + b4 * r8            //
                  + b5 * r8 * r2       //
                  + b6 * r8 * r4       //
                  + b7 * r8 * r4 * r2  //
                  + b8 * r16           //
                  + b9 * r16 * r2      //
               );
    }

    template <typename T>
    inline T __attribute__((optimize("-ffast-math"))) undistort(const T& r, const message::input::Image::Lens& lens) {
        const auto& k = lens.k;
        // These parenthesis are important as they allow the compiler to optimise further
        return r * (1.0 + k[0] * (r * r) + k[1] * (r * r) * (r * r) + k[2] * (r * r) * (r * r) * (r * r));
    }

    template <typename T, int options, int max_rows_at_compile_time, int max_cols_at_compile_time>
    Eigen::Matrix<T, 2, 1> project(
        const Eigen::Matrix<T, 3, 1, options, max_rows_at_compile_time, max_cols_at_compile_time>& ray,
        const message::input::Image::Lens& lens,
        const Eigen::Matrix<T, 2, 1>& dimensions) {

        // Perform the projection math
        const T f         = lens.focal_length;
        const T theta     = std::acos(ray.x());
        const T sin_theta = std::sin(theta);
        T r_u;
        switch (lens.projection.value) {
            case message::input::Image::Lens::Projection::RECTILINEAR: r_u = rectilinear_r(theta, f); break;
            case message::input::Image::Lens::Projection::EQUISOLID: r_u = equisolid_r(theta, f); break;
            case message::input::Image::Lens::Projection::EQUIDISTANT: r_u = equidistant_r(theta, f); break;
            default: throw std::runtime_error("Cannot project: Unknown lens type"); break;
        }
        const T r_d = distort(r_u, lens);

        Eigen::Matrix<T, 2, 1> projected(r_d * ray.y() / sin_theta, r_d * ray.z() / sin_theta);

        // Back to pixel coordinates
        projected.x() = -projected.x() + dimensions.x() / 2;
        projected.y() = projected.y() + dimensions.y() / 2;
        return projected;
    }

    /**
     * @brief Projects the pixel coordinates measured in coordinate system with origin in the centre of the image
     * where x is left and y is up. And projects to a coordinate system where x is forward, y is to the left and z
     * is up.
     *
     * @param px    the coordinates of the pixel measured as a fraction of the width
     * @param f     the focal length measured as a ratio of the width of the image
     *
     * @return  the unit vector in the direction of this pixel
     */
    template <typename T, int options, int max_rows_at_compile_time, int max_cols_at_compile_time>
    Eigen::Matrix<T, 3, 1> unproject(
        const Eigen::Matrix<T, 2, 1>& px,
        const message::input::Image::Lens& lens,
        const Eigen::Matrix<T, 2, 1, options, max_rows_at_compile_time, max_cols_at_compile_time>& dimensions) {

        // Transform to centre of the screen:
        Eigen::Matrix<T, 2, 1, options, max_rows_at_compile_time, max_cols_at_compile_time> transformed_px = px;
        transformed_px.x() = -(px.x() - dimensions.x() / 2);
        transformed_px.y() = px.y() - dimensions.y() / 2;

        // Perform the unprojection math
        const T f   = lens.focal_length;
        const T r_d = transformed_px.norm();
        const T r_u = undistort(r_d, lens);
        T theta;
        switch (lens.projection.value) {
            case message::input::Image::Lens::Projection::RECTILINEAR: theta = rectilinear_theta(r_u, f); break;
            case message::input::Image::Lens::Projection::EQUISOLID: theta = equisolid_theta(r_u, f); break;
            case message::input::Image::Lens::Projection::EQUIDISTANT: theta = equidistant_theta(r_u, f); break;
            default: throw std::runtime_error("Cannot project: Unknown lens type"); break;
        }
        const T sin_theta = std::sin(theta);

        return Eigen::Matrix<T, 3, 1>(std::cos(theta), sin_theta * px.x() / r_d, sin_theta * px.y() / r_d);
    }

}  // namespace utility::vision

#endif  // UTILITY_VISION_PROJECTION_HPP
