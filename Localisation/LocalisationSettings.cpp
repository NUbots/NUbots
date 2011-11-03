#include "LocalisationSettings.h"

LocalisationSettings::LocalisationSettings()
{
}

LocalisationSettings::LocalisationSettings(const LocalisationSettings& source)
{
    m_branching_method = source.m_branching_method;
    m_pruning_method = source.m_pruning_method;
    return;
}

std::string LocalisationSettings::branchMethodString(BranchMethod method)
{
    std::string result;
    switch(method)
    {
    case branch_exhaustive:
        result = "exhaustive";
        break;
    case branch_selective:
        result = "selective";
        break;
    default:
        result = "unknown";
        break;
    }
    return result;
}

std::string LocalisationSettings::pruneMethodString(PruneMethod method)
{
    std::string result;

    switch(method)
    {
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
