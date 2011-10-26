#ifndef LOCALISATIONSETTINGS_H
#define LOCALISATIONSETTINGS_H
#include <string>


class LocalisationSettings
{
public:
    enum PruneMethod
    {
        prune_unknown,
        prune_merge,
        prune_max_likelyhood,
        prune_viterbi,
        prune_nscan
    };

    enum BranchMethod
    {
        branch_unknown,
        branch_exhaustive,
        branch_selective
    };

    LocalisationSettings();
    LocalisationSettings(const LocalisationSettings& source);
    PruneMethod pruneMethod() const {return m_pruning_method;}
    BranchMethod branchMethod() const {return m_branching_method;}
    void setPruneMethod(PruneMethod newMethod){m_pruning_method = newMethod;}
    void setBranchMethod(BranchMethod newMethod){m_branching_method = newMethod;}
protected:
    PruneMethod m_pruning_method;
    BranchMethod m_branching_method;
};

#endif // LOCALISATIONSETTINGS_H
