#include "LocalisationSettings.h"

LocalisationSettings::LocalisationSettings()
{
    m_self_loc_model = KFBuilder::krobot_model;
    m_self_loc_filter = KFBuilder::kbasic_ukf_filter;
    m_ball_loc_model = KFBuilder::kmobile_object_model;
    m_ball_loc_filter = KFBuilder::kseq_ukf_filter;
}

LocalisationSettings::LocalisationSettings(const LocalisationSettings& source)
{
    m_branching_method = source.m_branching_method;
    m_pruning_method = source.m_pruning_method;
    m_self_loc_model = source.m_self_loc_model;
    m_self_loc_filter = source.m_self_loc_filter;
    m_ball_loc_model = source.m_ball_loc_model;
    m_ball_loc_filter = source.m_ball_loc_filter;
    return;
}

std::string LocalisationSettings::branchMethodString(BranchMethod method) const
{
    std::string result;
    switch(method)
    {
    case branch_none:
        result = "none";
        break;
    case branch_exhaustive:
        result = "exhaustive";
        break;
    case branch_selective:
        result = "selective";
        break;
    case branch_constraint:
        result = "constraint";
        break;
    case branch_probDataAssoc:
        result = "probabalistic data association";
        break;
    default:
        result = "unknown";
        break;
    }
    return result;
}

std::string LocalisationSettings::pruneMethodString(PruneMethod method) const
{
    std::string result;

    switch(method)
    {
    case prune_none:
        result = "none";
        break;
    case prune_merge:
        result = "merge";
        break;
    case prune_max_likelyhood:
        result = "maximum likelyhood";
        break;
    case prune_viterbi:
        result = "viterbi";
        break;
    case prune_nscan:
        result = "N-scan";
        break;
    default:
        result = "unknown";
        break;
    }
    return result;
}

std::string LocalisationSettings::filterString(KFBuilder::Filter filter) const
{
    std::string result;

    switch(filter)
    {
    case KFBuilder::kbasic_ukf_filter:
        result = "Basic UKF";
        break;
    case KFBuilder::ksr_basic_ukf_filter:
        result = "Square Root UKF";
        break;
    case KFBuilder::kseq_ukf_filter:
        result = "Sequential UKF";
        break;
    case KFBuilder::ksr_seq_ukf_filter:
        result = "Square Root Sequential UKF";
        break;
    default:
        result = "unknown";
        break;
    }
    return result;
}

std::string LocalisationSettings::modelString(KFBuilder::Model model) const
{
    std::string result;

    switch(model)
    {
    case KFBuilder::krobot_model:
        result = "Robot Model";
        break;
    case KFBuilder::kmobile_object_model:
        result = "Mobile object model";
        break;
    default:
        result = "unknown";
        break;
    }
    return result;
}
