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
        branch_selective,
        branch_constraint,
        branch_probDataAssoc
    };

    LocalisationSettings();
    LocalisationSettings(const LocalisationSettings& source);
    PruneMethod pruneMethod() const {return m_pruning_method;}
    BranchMethod branchMethod() const {return m_branching_method;}
    void setPruneMethod(PruneMethod newMethod){m_pruning_method = newMethod;}
    void setBranchMethod(BranchMethod newMethod){m_branching_method = newMethod;}
    std::string branchMethodString() const {return branchMethodString(m_branching_method);}
    std::string pruneMethodString() const {return pruneMethodString(m_pruning_method);}
    std::string branchMethodString(BranchMethod) const;
    std::string pruneMethodString(PruneMethod) const;
protected:
    PruneMethod m_pruning_method;
    BranchMethod m_branching_method;
};

#endif // LOCALISATIONSETTINGS_H
