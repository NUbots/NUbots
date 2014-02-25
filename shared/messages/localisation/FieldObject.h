#ifndef MESSAGES_LOCALISATIONFIELDOBJECT
#define    MESSAGES_LOCALISATIONFIELDOBJECT

namespace messages {
    namespace localisation {
        struct FieldObject {
            std::string name;
            double wm_x;
            double wm_y;
            double heading;
            double sd_x;
            double sd_y;
            double sr_xx;
            double sr_xy;
            double sr_yy;
            bool lost;
        };
    }
}

#endif
