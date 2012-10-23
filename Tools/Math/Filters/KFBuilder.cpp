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
        case kmobile_object_model:
            model = new MobileObjectModel();
            break;
        case krobot_model:
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
        case kukf_filter:
            break;
        case ksr_ukf_filter:
            break;
        case kseq_ukf_filter:
            filter = new SeqUKF(model);
            break;
        case ksr_seq_ukf_filter:
            break;
        default:
            filter = NULL;
    }

    // Set the filter for the model.
    switch (model_type)
    {
        case kmobile_object_model:
            break;
        case krobot_model:
            filter->enableOutlierFiltering();
            filter->enableWeighting();
            break;
        default:
            break;
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
