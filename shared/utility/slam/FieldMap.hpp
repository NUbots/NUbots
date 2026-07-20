/**
 * @file FieldMap.hpp
 * @brief Defines a static map of known landmark positions on a RoboCup humanoid soccer field
 *
 * This file provides #FieldDimensions (the physical measurements of the field, matching the
 * layout used by the NUbots humanoid robot soccer codebase, see
 * NUbots/module/support/configuration/SoccerConfig/data/config/FieldDescription.yaml and
 * NUbots/shared/message/support/FieldDescription.proto) and #FieldMap, which builds the set of
 * field-line intersection landmarks and goal-post landmarks in the field coordinate frame
 * {f} for use as a known map in robot localisation (e.g. an EKF/particle filter observing
 * field lines and goal posts).
 *
 * The landmark layout and field frame convention are derived directly from the NUbots
 * reference implementation:
 *  - NUbots/shared/utility/localisation/FieldLineOccupanyMap.hpp,
 *    function setup_field_landmarks() (L/T/X field-line intersections)
 *  - NUbots/module/localisation/FieldLocalisationNLopt/src/FieldLocalisationNLopt.cpp,
 *    lines 143-154 (goal post positions)
 *
 * @section field_frame Field coordinate frame {f}
 * The field frame {f} is a right-handed frame with:
 *  - Origin at the centre of the field, i.e. the centre of the centre circle / the midpoint
 *    of the halfway line (see setup_field_landmarks(): the field corners are placed at
 *    (\f$\pm\f$half_length, \f$\pm\f$half_width) and the halfway-line/centre-circle
 *    intersection landmark is placed at the origin,
 *    FieldLineOccupanyMap.hpp:182-203).
 *  - +x axis directed along the long axis of the field (the direction from one goal line to
 *    the other, i.e. parallel to the sidelines/touchlines), such that the two goals sit at
 *    x = -field_length/2 and x = +field_length/2
 *    (FieldLocalisationNLopt.cpp:143-150, where the goal posts are placed at
 *    x = \f$\pm\f$field_length/2).
 *  - +y axis directed along the short axis of the field (parallel to the goal lines,
 *    i.e. from one touchline to the other), such that the two goal posts of either goal sit
 *    at y = -goal_width/2 and y = +goal_width/2
 *    (FieldLocalisationNLopt.cpp:143-150).
 *  - +z axis normal to the field surface, pointing up out of the ground (right-handed frame);
 *    all landmarks in this file lie on the ground plane, so z = 0 for every landmark.
 *
 * Note: which physical goal is labelled "own" vs "opposition" is not part of the map itself
 * (NUbots's own two modules disagree on this convention -- compare
 * FieldLocalisationNLopt.cpp:143-150, which places "own" goal posts at +half_length, against
 * SoccerConfig.cpp:53-56, which places "own" goal posts at -half_length). #FieldMap therefore
 * simply reports all four goal posts at x = \f$\pm\f$fieldLength/2, y = \f$\pm\f$goalWidth/2,
 * matching the FieldLocalisationNLopt.cpp convention, and leaves any "own"/"opposition"
 * labelling to the caller.
 *
 * @section landmark_types Landmark types and counts (default dimensions)
 * NUbots classifies field-line landmarks using
 * message::vision::FieldIntersection::IntersectionType, which has three useful values:
 * L_INTERSECTION, T_INTERSECTION and X_INTERSECTION (plus UNKNOWN, which is not used for map
 * landmarks). Goal posts are not part of that enum in NUbots -- they are tracked separately
 * (own_goal_posts / opp_goal_posts in FieldLocalisationNLopt.cpp) -- so this module adds a
 * fourth #LandmarkType, GOAL_POST, to hold them alongside the field-line landmarks.
 *
 * For the default FieldDimensions given in this file (which match the field this data set was
 * recorded on: field_length = 6.8 m, field_width = 5.0 m, goal_width = 1.95 m,
 * goal_area_length = 1.05 m, goal_area_width = 2.62 m, penalty_area_length = 1.55 m,
 * penalty_area_width = 4.05 m, penalty_mark_distance = 1.27 m,
 * centre_circle_diameter = 1.5 m), replicating setup_field_landmarks() gives:
 *  - L_INTERSECTION: 12 landmarks (4 field corners + 4 penalty-area corners nearest the goal
 *    line + 4 goal-area corners nearest the goal line).
 *  - T_INTERSECTION: 10 landmarks (2 halfway-line/touchline junctions + 4 penalty-area-line/
 *    goal-line junctions + 4 goal-area-line/goal-line junctions).
 *  - X_INTERSECTION: 5 landmarks (the centre-circle/halfway-line centre point, the 2 points
 *    where the centre circle crosses the halfway line, and the 2 penalty marks). NUbots
 *    classifies the penalty marks as X_INTERSECTION landmarks alongside the centre-circle
 *    crossings (FieldLineOccupanyMap.hpp:260-266) -- there is no separate "penalty mark"
 *    landmark type. NUbots' setup_field_landmarks() also re-pushes the centre point
 *    (0, 0, 0) a second time at FieldLineOccupanyMap.hpp:265-266, which is an exact duplicate
 *    of the point already added at FieldLineOccupanyMap.hpp:202-203; this module de-duplicates
 *    that repeated entry (judgement call: a duplicate landmark position carries no extra
 *    information for a localisation map and would only cost extra distance computations).
 *  - GOAL_POST: 4 landmarks (2 posts per goal, 2 goals).
 *
 * Total: 31 landmarks for the default field dimensions.
 */

#ifndef FIELDMAP_HPP
#define FIELDMAP_HPP

#include <Eigen/Core>
#include <vector>

namespace utility::slam {

    /**
     * @brief Physical measurements of a RoboCup humanoid soccer field, in metres
     *
     * Field names and defaults mirror
     * NUbots/module/support/configuration/SoccerConfig/data/config/FieldDescription.yaml and
     * NUbots/shared/message/support/FieldDescription.proto (message FieldDescription.FieldDimensions).
     */
    struct FieldDimensions {
        // These defaults match the field this data set was recorded on; they stand in for unit tests and
        // offline replay. At runtime, build them from a message::support::FieldDescription via
        // utility::slam::field_dimensions() (FieldMapFromDescription.hpp), which keeps this struct — and
        // the estimator core that includes it — free of the message library.
        double lineWidth            = 0.05;  ///< Width of field lines
        double fieldLength          = 6.8;   ///< Touchline (sideline) length
        double fieldWidth           = 5.0;   ///< Goal line (baseline) length
        double goalDepth            = 0.4;   ///< Distance behind the goal line to the back of the net
        double goalWidth            = 1.95;  ///< Distance between the inner edges of the goal posts
        double goalAreaLength       = 1.05;  ///< Goal area (6-yard box) length, from the goal line
        double goalAreaWidth        = 2.62;  ///< Goal area (6-yard box) width
        double penaltyMarkDistance  = 1.27;  ///< Distance from the goal line to the penalty mark
        double centreCircleDiameter = 1.5;   ///< Diameter of the centre circle
        double penaltyAreaLength    = 1.55;  ///< Penalty area (18-yard box) length, from the goal line
        double penaltyAreaWidth     = 4.05;  ///< Penalty area (18-yard box) width
        double goalpostWidth        = 0.10;  ///< Diameter of a (circular) goal post
        double borderStripMinWidth  = 0.38;  ///< Minimum width of the border strip around the field
    };

    /**
     * @brief Classification of a field landmark
     *
     * L_INTERSECTION, T_INTERSECTION and X_INTERSECTION mirror
     * message::vision::FieldIntersection::IntersectionType from the NUbots codebase
     * (NUbots/shared/message/vision/FieldIntersections.proto). GOAL_POST is an addition used to
     * hold the four goal-post positions, which NUbots tracks separately from field-line
     * intersections (see FieldLocalisationNLopt.cpp own_goal_posts/opp_goal_posts).
     */
    enum class LandmarkType { L_INTERSECTION, T_INTERSECTION, X_INTERSECTION, GOAL_POST };

    /**
     * @brief A static map of known field landmark positions in the field frame {f}
     *
     * See @ref field_frame "the file-level documentation" for the field frame convention and
     * @ref landmark_types "landmark type documentation" for the landmark layout and counts.
     */
    class FieldMap {
    public:
        /**
         * @brief Construct the field map from a set of field dimensions
         * @param dims Field dimensions to build the landmark map from (defaults to the field this
         *             data set was recorded on)
         */
        explicit FieldMap(const FieldDimensions& dims = {});

        /**
         * @brief A painted line segment on the field (centreline coordinates, ground plane)
         */
        struct LineSegment {
            Eigen::Vector2d a;  ///< Segment start (x, y) in {f}
            Eigen::Vector2d b;  ///< Segment end (x, y) in {f}
        };

        /**
         * @brief A painted circle on the field (centreline coordinates, ground plane)
         */
        struct Circle {
            Eigen::Vector2d centre;  ///< Circle centre (x, y) in {f}
            double radius;           ///< Circle radius
        };

        /**
         * @brief Get all known landmark positions of a given type
         * @param type Landmark type to retrieve
         * @return Landmark positions rLFf (landmark relative to field origin, expressed in the
         *         field frame {f}), each with z = 0 (ground plane)
         */
        const std::vector<Eigen::Vector3d>& landmarks(LandmarkType type) const;

        /// @brief Painted line segments (boundary, halfway line, goal/penalty areas)
        const std::vector<LineSegment>& lineSegments() const {
            return lineSegments_;
        }

        /// @brief Painted circles (centre circle)
        const std::vector<Circle>& lineCircles() const {
            return lineCircles_;
        }

        /**
         * @brief Squared distance from a ground-plane point to the nearest painted line.
         *
         * Analytic minimum over all line segments and circles. Working with the
         * squared distance keeps the function smooth through zero (points exactly
         * on a line), which matters for derivative-based optimisation.
         *
         * @param p Point (x, y) in the field frame {f}
         * @return Squared distance to the nearest line centreline [m^2]
         */
        template <typename Scalar>
        Scalar distanceSquaredToNearestLine(const Eigen::Vector2<Scalar>& p) const {
            using std::sqrt;
            Scalar best = Scalar(1e12);
            for (const LineSegment& seg : lineSegments_) {
                const Eigen::Vector2<Scalar> a  = seg.a.cast<Scalar>();
                const Eigen::Vector2<Scalar> ab = (seg.b - seg.a).cast<Scalar>();
                const double len2               = (seg.b - seg.a).squaredNorm();
                Scalar t                        = (p - a).dot(ab) / Scalar(len2);
                if (t < Scalar(0))
                    t = Scalar(0);
                if (t > Scalar(1))
                    t = Scalar(1);
                Scalar d2 = (p - (a + t * ab)).squaredNorm();
                if (d2 < best)
                    best = d2;
            }
            for (const Circle& c : lineCircles_) {
                const Eigen::Vector2<Scalar> pc = p - c.centre.cast<Scalar>();
                Scalar n2                       = pc.squaredNorm();
                if (n2 < Scalar(1e-12))
                    n2 = Scalar(1e-12);  // Guard sqrt at the circle centre
                Scalar n  = sqrt(n2);
                Scalar d  = n - Scalar(c.radius);
                Scalar d2 = d * d;
                if (d2 < best)
                    best = d2;
            }
            return best;
        }

        FieldDimensions dims;  ///< Field dimensions used to build this map

    private:
        /// @brief Populate the landmarks and line primitives from #dims
        void build();

        std::vector<Eigen::Vector3d> landmarksL_;         ///< L-intersection landmarks rLFf
        std::vector<Eigen::Vector3d> landmarksT_;         ///< T-intersection landmarks rLFf
        std::vector<Eigen::Vector3d> landmarksX_;         ///< X-intersection landmarks rLFf
        std::vector<Eigen::Vector3d> landmarksGoalPost_;  ///< Goal post landmarks rLFf
        std::vector<LineSegment> lineSegments_;           ///< Painted line segments
        std::vector<Circle> lineCircles_;                 ///< Painted circles
    };
}  // namespace utility::slam

#endif  // FIELDMAP_H
