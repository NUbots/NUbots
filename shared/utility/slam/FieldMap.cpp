#include "FieldMap.hpp"

#include <stdexcept>

namespace utility::slam {

    FieldMap::FieldMap(const FieldDimensions& dims) : dims(dims) {
        build();
    }

    const std::vector<Eigen::Vector3d>& FieldMap::landmarks(LandmarkType type) const {
        switch (type) {
            case LandmarkType::L_INTERSECTION: return landmarksL_;
            case LandmarkType::T_INTERSECTION: return landmarksT_;
            case LandmarkType::X_INTERSECTION: return landmarksX_;
            case LandmarkType::GOAL_POST: return landmarksGoalPost_;
        }
        throw std::invalid_argument("FieldMap::landmarks: unknown LandmarkType");
    }

    void FieldMap::build() {
        landmarksL_.clear();
        landmarksT_.clear();
        landmarksX_.clear();
        landmarksGoalPost_.clear();

        const double halfLength = dims.fieldLength / 2;
        const double halfWidth  = dims.fieldWidth / 2;

        // --- L intersections -----------------------------------------------------------------
        // Four corners of the field (outer boundary of the field of play)
        landmarksL_.emplace_back(-halfLength, halfWidth, 0);
        landmarksL_.emplace_back(-halfLength, -halfWidth, 0);
        landmarksL_.emplace_back(halfLength, halfWidth, 0);
        landmarksL_.emplace_back(halfLength, -halfWidth, 0);

        if (dims.penaltyAreaLength != 0 && dims.penaltyAreaWidth != 0) {
            // L intersections at the corners of the penalty areas nearest the centre of the field
            landmarksL_.emplace_back(halfLength - dims.penaltyAreaLength, dims.penaltyAreaWidth / 2, 0);
            landmarksL_.emplace_back(halfLength - dims.penaltyAreaLength, -dims.penaltyAreaWidth / 2, 0);
            landmarksL_.emplace_back(-halfLength + dims.penaltyAreaLength, dims.penaltyAreaWidth / 2, 0);
            landmarksL_.emplace_back(-halfLength + dims.penaltyAreaLength, -dims.penaltyAreaWidth / 2, 0);
        }

        // L intersections at the corners of the goal areas nearest the centre of the field
        landmarksL_.emplace_back(halfLength - dims.goalAreaLength, dims.goalAreaWidth / 2, 0);
        landmarksL_.emplace_back(halfLength - dims.goalAreaLength, -dims.goalAreaWidth / 2, 0);
        landmarksL_.emplace_back(-halfLength + dims.goalAreaLength, dims.goalAreaWidth / 2, 0);
        landmarksL_.emplace_back(-halfLength + dims.goalAreaLength, -dims.goalAreaWidth / 2, 0);

        // --- T intersections -------------------------------------------------------------------
        // Mid-points of each sideline, where the halfway line meets the touchlines
        landmarksT_.emplace_back(0, halfWidth, 0);
        landmarksT_.emplace_back(0, -halfWidth, 0);

        if (dims.penaltyAreaLength != 0 && dims.penaltyAreaWidth != 0) {
            // T intersections where the penalty area lines meet the goal lines
            landmarksT_.emplace_back(halfLength, dims.penaltyAreaWidth / 2, 0);
            landmarksT_.emplace_back(halfLength, -dims.penaltyAreaWidth / 2, 0);
            landmarksT_.emplace_back(-halfLength, dims.penaltyAreaWidth / 2, 0);
            landmarksT_.emplace_back(-halfLength, -dims.penaltyAreaWidth / 2, 0);
        }

        // T intersections where the goal area lines meet the goal lines
        landmarksT_.emplace_back(halfLength, dims.goalAreaWidth / 2, 0);
        landmarksT_.emplace_back(halfLength, -dims.goalAreaWidth / 2, 0);
        landmarksT_.emplace_back(-halfLength, dims.goalAreaWidth / 2, 0);
        landmarksT_.emplace_back(-halfLength, -dims.goalAreaWidth / 2, 0);

        // --- X intersections ---------------------------------------------------------------------
        // Centre of the field (centre-circle / halfway-line crossing)
        landmarksX_.emplace_back(0, 0, 0);
        // Points where the centre circle crosses the halfway line
        landmarksX_.emplace_back(0, dims.centreCircleDiameter / 2, 0);
        landmarksX_.emplace_back(0, -dims.centreCircleDiameter / 2, 0);

        if (dims.penaltyMarkDistance != 0) {
            // Penalty marks are classified as X intersections in NUbots (see FieldLineOccupanyMap.hpp)
            landmarksX_.emplace_back(halfLength - dims.penaltyMarkDistance, 0, 0);
            landmarksX_.emplace_back(-halfLength + dims.penaltyMarkDistance, 0, 0);
        }

        // --- Goal posts --------------------------------------------------------------------------
        // Base centre of each of the 4 goal posts (2 per goal), at ground level
        landmarksGoalPost_.emplace_back(halfLength, dims.goalWidth / 2, 0);
        landmarksGoalPost_.emplace_back(halfLength, -dims.goalWidth / 2, 0);
        landmarksGoalPost_.emplace_back(-halfLength, dims.goalWidth / 2, 0);
        landmarksGoalPost_.emplace_back(-halfLength, -dims.goalWidth / 2, 0);

        // --- Painted lines -----------------------------------------------------------------------
        auto seg = [this](double x0, double y0, double x1, double y1) {
            lineSegments_.push_back({Eigen::Vector2d(x0, y0), Eigen::Vector2d(x1, y1)});
        };

        // Boundary
        seg(-halfLength, -halfWidth, halfLength, -halfWidth);
        seg(-halfLength, halfWidth, halfLength, halfWidth);
        seg(-halfLength, -halfWidth, -halfLength, halfWidth);
        seg(halfLength, -halfWidth, halfLength, halfWidth);

        // Halfway line
        seg(0, -halfWidth, 0, halfWidth);

        // Goal and penalty areas at both ends
        for (int s : {-1, 1}) {
            const double xGoal = s * halfLength;
            const double xGA   = s * (halfLength - dims.goalAreaLength);
            seg(xGoal, dims.goalAreaWidth / 2, xGA, dims.goalAreaWidth / 2);
            seg(xGoal, -dims.goalAreaWidth / 2, xGA, -dims.goalAreaWidth / 2);
            seg(xGA, -dims.goalAreaWidth / 2, xGA, dims.goalAreaWidth / 2);

            const double xPA = s * (halfLength - dims.penaltyAreaLength);
            seg(xGoal, dims.penaltyAreaWidth / 2, xPA, dims.penaltyAreaWidth / 2);
            seg(xGoal, -dims.penaltyAreaWidth / 2, xPA, -dims.penaltyAreaWidth / 2);
            seg(xPA, -dims.penaltyAreaWidth / 2, xPA, dims.penaltyAreaWidth / 2);
        }

        // Centre circle
        lineCircles_.push_back({Eigen::Vector2d::Zero(), dims.centreCircleDiameter / 2});
    }
}  // namespace utility::slam
