#ifndef LOCALISATIONSETTINGS_H
#define LOCALISATIONSETTINGS_H
#include <string>
#include <iostream>

/*!
@class The LocalisationSettings class is used to define settings for the loaclisation system.
*/
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

    /*!
    @brief Default constructor, loads using default settings.
    */
    LocalisationSettings();

    /*!
    @brief Copy constructor, loads settings from source.
    */
    LocalisationSettings(const LocalisationSettings& source);

    /*!
    @brief Returns the identifier for the current pruning method.
    @return The ID of the pruning method.
    */
    PruneMethod pruneMethod() const {return m_pruning_method;}

    /*!
    @brief Returns the identifier for the current branching method.
    @return The ID of the branching method.
    */
    BranchMethod branchMethod() const {return m_branching_method;}

    /*!
    @brief Sets the current pruning method.
    @param newMethod The ID of the new pruning method.
    */
    void setPruneMethod(PruneMethod newMethod){m_pruning_method = newMethod;}

    /*!
    @brief Sets the current branching method.
    @param newMethod The ID of the new branching method.
    */
    void setBranchMethod(BranchMethod newMethod){m_branching_method = newMethod;}

    /*!
    @brief Retrieve the name of the current branching method.
    @return A string containing the name of the current branching method.
    */
    std::string branchMethodString() const {return branchMethodString(m_branching_method);}

    /*!
    @brief Retrieve the name of the current pruning method.
    @return A string containing the name of the current pruning method.
    */
    std::string pruneMethodString() const {return pruneMethodString(m_pruning_method);}

    /*!
    @brief Retrieve the name of a given branching method.
    @param The ID of the branching method.
    @return A string containing the name of the branching method.
    */
    std::string branchMethodString(BranchMethod) const;

    /*!
    @brief Retrieve the name of a given pruning method.
    @param The ID of the pruning method.
    @return A string containing the name of the pruning method.
    */
    std::string pruneMethodString(PruneMethod) const;

    /*!
    @brief Outputs a binary representation of the loaclisationSettings object to a stream.
    @param output The output stream.
    @return The output stream.
    */
    std::ostream& writeStreamBinary (std::ostream& output) const
    {
        output.write(reinterpret_cast<const char*>(&m_pruning_method), sizeof(m_pruning_method));
        output.write(reinterpret_cast<const char*>(&m_branching_method), sizeof(m_branching_method));
        return output;
    }

    /*!
    @brief Reads in a LocalisationSettings object from the input stream.
    @param input The input stream.
    @return The input stream.
    */
    std::istream& readStreamBinary (std::istream& input)
    {
        input.read(reinterpret_cast<char*>(&m_pruning_method), sizeof(m_pruning_method));
        input.read(reinterpret_cast<char*>(&m_branching_method), sizeof(m_branching_method));
        return input;
    }

protected:
    PruneMethod m_pruning_method;
    BranchMethod m_branching_method;
};

#endif // LOCALISATIONSETTINGS_H
