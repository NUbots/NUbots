#ifndef KFBUILDER_H
#define KFBUILDER_H
#include "IKalmanFilter.h"

class KFBuilder
{
public:
    enum Model
    {
        kmobile_object_model,
        krobot_model,
        ktotal_models
    };
    enum Filter
    {
        kseq_ukf_filter,
        ksr_seq_ukf_filter,
        kukf_filter,
        ksr_ukf_filter,
        ktotal_filters
    };
    KFBuilder();
    static IKalmanFilter* getNewFilter(Filter filter_type, Model model_type);
};

#endif // KFBUILDER_H
