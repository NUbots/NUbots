#ifndef KFBUILDER_H
#define KFBUILDER_H
#include "IKalmanFilter.h"

class KFBuilder
{
public:
    enum Model
    {
        kmobile_object,
        krobot,
        ktotal_models
    };
    enum Filter
    {
        kseq_ukf,
        ksr_seq_ukf,
        kukf,
        ksr_ukf,
        ktotal_filters
    };
    KFBuilder();
    static IKalmanFilter* getNewFilter(Filter filter_type, Model model_type);
};

#endif // KFBUILDER_H
