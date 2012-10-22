#include "KFBuilder.h"
#include "IKFModel.h"

// Include different models.
#include "MobileObjectModel.h"
#include "RobotModel.h"

// Include different filters.
#include "SeqUKF.h"

KFBuilder::KFBuilder()
{
}

IKalmanFilter* KFBuilder::getNewFilter(Filter filter_type, Model model_type)
{
    IKalmanFilter* filter = NULL;
    IKFModel* model = NULL;

    // First get the correct model to use for our filter.
    switch (model_type)
    {
        case kmobile_object:
            model = new MobileObjectModel();
            break;
        case krobot:
            model = new RobotModel();
            break;
        default:
            model = NULL;
    }

    // Reurn null if a valid filter was not found.
    if(model == NULL) return NULL;

    // Now make the filter type we want using the model.
    switch (filter_type)
    {
        case kukf:
            break;
        case ksr_ukf:
            break;
        case kseq_ukf:
            filter = new SeqUKF(model);
            break;
        case ksr_seq_ukf:
            break;
        default:
            filter = NULL;
    }

    // If the correct filter was not created, delete the model since it cannot be used.
    // Otherwise the filter is responsible for deleting the model.
    if(filter == NULL and model != NULL)
    {
        delete model;
    }

    // Return the new filter.
    return filter;
}
