#ifndef MESSAGES_LOCALISATIONFIELDOBJECT
#define MESSAGES_LOCALISATIONFIELDOBJECT

namespace messages {
    namespace localisation {
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
    }
}

#endif
