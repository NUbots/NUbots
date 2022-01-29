/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_MATH_GEOMETRY_QUAD_HPP
#define UTILITY_MATH_GEOMETRY_QUAD_HPP

#include <Eigen/Core>
#include <ostream>
#include <utility>
#include <vector>

namespace utility::math::geometry {

    template <typename Scalar, int R, int C>
    class Quad {
    public:
        using T = typename Eigen::Matrix<Scalar, R, C>;
        Quad() : bl(T::Zero()), br(T::Zero()), tr(T::Zero()), tl(T::Zero()) {}
        Quad(const Quad& other) : bl(other.bl), br(other.br), tr(other.tr), tl(other.tl) {}
        Quad(const Quad&& other) noexcept : bl(other.bl), br(other.br), tr(other.tr), tl(other.tl) {}
        [[nodiscard]] Quad& operator=(const Quad& other) {
            bl = other.bl;
            br = other.br;
            tr = other.tr;
            tl = other.tl;
        }
        [[nodiscard]] Quad& operator=(const Quad&& other) noexcept {
            bl = other.bl;
            br = other.br;
            tr = other.tr;
            tl = other.tl;
        }
        Quad(T bottomLeft, T topLeft, T topRight, T bottomRight)
            : bl(std::move(bottomLeft)), br(std::move(bottomRight)), tr(std::move(topRight)), tl(std::move(topLeft)) {}
        Quad(const Scalar& left, const Scalar& top, const Scalar& right, const Scalar& bottom)
            : bl(left, bottom), br(right, bottom), tr(right, top), tl(left, top) {}

        /**
         * Sets the Quad as a screen aligned rectangle given the specified positions.
         * @param left     The left x pixel value.
         * @param top      The top y pixel value.
         * @param right    The right x pixel value.
         * @param bottom   The bottom y pixel value.
         */
        void set(const Scalar& left, const Scalar& top, const Scalar& right, const Scalar& bottom) {
            bl.x() = left;
            bl.y() = bottom;
            br.x() = right;
            br.y() = bottom;
            tl.x() = left;
            tl.y() = top;
            tr.x() = right;
            tr.y() = top;
        }

        /**
         * Sets the Quad given the specified corners.
         * @param bottomLeft  The bottom left corner.
         * @param topLeft     The top left corner.
         * @param topRight    The top right corner.
         * @param bottomRight The bottom right corner.
         */
        void set(const T& bottomLeft, const T& topLeft, const T& topRight, const T& bottomRight) {
            bl = bottomLeft;
            tl = topLeft;
            tr = topRight;
            br = bottomRight;
        }

        //! Returns the bottom centre pixel location of the Quad.
        [[nodiscard]] T getTopCentre() const {
            return ((tl + tr) * 0.5);
        }
        //! Returns the bottom centre pixel location of the Quad.
        [[nodiscard]] T getBottomCentre() const {
            return ((bl + br) * 0.5);
        }
        [[nodiscard]] T getRightCentre() const {
            return ((br + tr) * 0.5);
        }
        [[nodiscard]] T getLeftCentre() const {
            return ((bl + tl) * 0.5);
        }

        //! Returns the centre pixel location  of the Quad.
        [[nodiscard]] T getCentre() const {
            return ((bl + tl + tr + br) * 0.25);
        }

        //! Returns the bottom left pixel location  of the Quad.
        [[nodiscard]] T getBottomLeft() const {
            return bl;
        }
        //! Returns the bottom right pixel location  of the Quad.
        [[nodiscard]] T getBottomRight() const {
            return br;
        }
        //! Returns the top left pixel location  of the Quad.
        [[nodiscard]] T getTopLeft() const {
            return tl;
        }
        //! Returns the top right pixel location  of the Quad.
        [[nodiscard]] T getTopRight() const {
            return tr;
        }

        // Returns the bounding box width and height
        [[nodiscard]] T getSize() const {
            Quad boundingBox = getBoundingBox(getVertices());
            return T(boundingBox.getAverageWidth(), boundingBox.getAverageHeight());
        }

        [[nodiscard]] Scalar getLeft() const {
            return (0.5 * (bl.x() + tl.x()));
        }
        [[nodiscard]] Scalar getRight() const {
            return (0.5 * (br.x() + tr.x()));
        }
        [[nodiscard]] Scalar getTop() const {
            return (0.5 * (tl.y() + tr.y()));
        }
        [[nodiscard]] Scalar getBottom() const {
            return (0.5 * (bl.y() + br.y()));
        }

        // //! Returns the base width of the Quad in pixels.
        // int getBaseWidth() const;
        // //! Returns the top width of the Quad in pixels.
        // int getTopWidth() const;

        // //! Returns the left height of the Quad in pixels.
        // int getLeftHeight() const;
        // //! Returns the right height of the Quad in pixels.
        // int getRightHeight() const;

        //! Returns the average width of the Quad in pixels.
        [[nodiscard]] Scalar getAverageWidth() const {
            return ((0.5 * ((br - bl).norm() + (tr - tl).norm())));
        }

        //! Returns the average height of the Quad in pixels.
        [[nodiscard]] Scalar getAverageHeight() const {
            return ((0.5 * ((br - tr).norm() + (bl - tl).norm())));
        }

        [[nodiscard]] Scalar area() const {
            // Area of a quadrilateral: A = 0.5 * |diag1 X diag2|
            // In two dimensions, this equates to: A = 0.5 * |(diag1.x)(diag2.y) - (diag2.x)(diag2.y)|
            T diag1 = bl - tr;
            T diag2 = tl - br;
            return std::abs(0.5 * ((diag1.x() * diag2.y()) - (diag1.y() * diag2.x())));
        }
        [[nodiscard]] Scalar aspectRatio() const {
            return (((br - tr).norm() + (bl - tl).norm() + 2) / ((br - bl).norm() + (tr - tl).norm() + 2));
        }

        [[nodiscard]] std::vector<T> getVertices() const {
            std::vector<T> vert = {tr, br, bl, tl};
            return vert;
        }

        [[nodiscard]] bool overlapsHorizontally(const Quad& other) const {
            // Rough for now.
            Scalar farRight   = std::max(tr.x(), br.x());
            Scalar farLeft    = std::min(tl.x(), bl.x());
            Scalar o_farRight = std::max(other.tr.x(), other.br.x());
            Scalar o_farLeft  = std::min(other.tl.x(), other.bl.x());

            return !((farRight < o_farLeft) || (o_farRight < farLeft));
        }

        [[nodiscard]] bool checkCornersValid() const {
            return br.innerSize() == 2 && bl.innerSize() == 2 && tr.innerSize() == 2 && tl.innerSize() == 2;
        }

        static Quad getBoundingBox(const std::vector<T>& points) {
            // Check for
            if (points.empty()) {
                throw std::domain_error("Request made for bounding box for empty list of points!");
            }

            Scalar min_x = points[0].x();
            Scalar max_x = points[0].x();
            Scalar min_y = points[0].y();
            Scalar max_y = points[0].y();
            for (uint i = 1; i < points.size(); i++) {
                auto& p = points[i];
                max_x   = std::max(max_x, p.x());
                min_x   = std::min(min_x, p.x());
                max_y   = std::max(max_y, p.y());
                min_y   = std::min(min_y, p.y());
            }
            return Quad(T(min_x, min_y), T(min_x, max_y), T(max_x, max_y), T(max_x, min_y));
        }

    private:
        T bl;  //! @variable The bottom-left of the Quad.
        T br;  //! @variable The bottom-right of the Quad.
        T tr;  //! @variable The top-right of the Quad.
        T tl;  //! @variable The top-left of the Quad.
    };


    /// @brief Stream insertion operator for a single Quad.
    /// @relates Quad
    template <typename Scalar, int R, int C>
    inline std::ostream& operator<<(std::ostream& output, const Quad<Scalar, R, C>& quad) {
        output << "(" << quad.getBottomLeft().x() << ", " << quad.getBottomLeft().y() << ") (" << quad.getTopLeft().x()
               << ", " << quad.getTopLeft().y() << ") (" << quad.getTopRight().x() << ", " << quad.getTopRight().y()
               << ") (" << quad.getBottomRight().x() << ", " << quad.getBottomRight().y() << ")";

        return output;
    }

    /// @brief Stream insertion operator for a std::vector of Quads.
    /// @relates Quad
    template <typename Scalar, int R, int C>
    inline std::ostream& operator<<(std::ostream& output, const std::vector<Quad<Scalar, R, C>>& quads) {
        output << "[";

        for (const auto& quad : quads) {
            output << quad << ", ";
        }

        output << "]";

        return output;
    }

}  // namespace utility::math::geometry

#endif
