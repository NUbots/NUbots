#ifndef MESSAGES_BEHAVIOUR_WALKSTRATEGY_H
#define MESSAGES_BEHAVIOUR_WALKSTRATEGY_H


namespace messages {
    namespace behaviour {
        
        enum class WalkTarget {
            WayPoint = 0,
            Robot = 1,
            Ball = 2
        };
        
        enum class WalkApproach {
            StandStill = 0,
            ApproachFromDirection = 1,
            WalkToPoint = 2,
            OmnidirectionalReposition = 3,
        };
        
        struct WalkStrategy {
            arma::vec2 target;
            arma::vec2 heading;
            WalkTarget targetPositionType;
            WalkTarget targetHeadingType;
            WalkApproach walkMovementType;
        };
    }
}

#endif
