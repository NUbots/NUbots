#ifndef MESSAGES_LOCALISATIONFIELDOBJECT
#define MESSAGES_LOCALISATIONFIELDOBJECT

#include <armadillo>

namespace messages {
    namespace localisation {

        // The FieldObject message that the NUbugger reactor listens for
        struct FieldObject {
            std::string name;

            struct Model {
                double wm_x;
                double wm_y;
                double heading;
                double sd_heading;
                double sd_x;
                double sd_y;
                double sr_xx;
                double sr_xy;
                double sr_yy;
                bool lost;
            };

            std::vector<Model> models;
        };

        // Temporary test input for localisation
        struct FakeOdometry {
            arma::vec2 torso_displacement;
            double torso_rotation;
        };

        class LocalisationObject {
        public:
            LocalisationObject() {}

            arma::vec2 position;
            double sr_xx;
            double sr_xy;
            double sr_yy;
        };

        class Ball : public LocalisationObject {
        public:
            Ball() : LocalisationObject() {}
            arma::vec2 velocity;
        };

        class Self : public LocalisationObject {
        public:
            Self() : LocalisationObject() {}
            arma::vec2 heading;
        };
    }
}

#endif
