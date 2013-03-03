#ifndef LOCALISATIONSETTINGS_H
#define LOCALISATIONSETTINGS_H
#include <string>
#include <iostream>

#include "Tools/Math/Filters/KFBuilder.h"

/*!
@class The LocalisationSettings class is used to define settings for the loaclisation system.
*/
class LocalisationSettings
{
public:
    enum PruneMethod
    {
        prune_unknown,
        prune_none,
        prune_merge,
        prune_max_likelyhood,
        prune_viterbi,
        prune_nscan
    };

    enum BranchMethod
    {
        branch_unknown,
        branch_none,
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
    @brief Returns the identifier for the current self localisation model.
    @return The ID of the self localisation model.
    */
    KFBuilder::Model selfLocModel() const {return m_self_loc_model;}

    /*!
    @brief Returns the identifier for the current filter type for self localisation.
    @return The ID of the filter type.
    */
    KFBuilder::Filter selfLocFilter() const {return m_self_loc_filter;}

    /*!
    @brief Returns the identifier for the current ball localisation model.
    @return The ID of the model.
    */
    KFBuilder::Model ballLocModel() const {return m_ball_loc_model;}

    /*!
    @brief Returns the identifier for the current filter type for ball localisation.
    @return The ID of the filter type.
    */
    KFBuilder::Filter ballLocFilter() const {return m_ball_loc_filter;}

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
    @brief Sets the current self localisation model.
    @param newModel The ID of the new model.
    */
    void setSelfLocModel(KFBuilder::Model newModel) {m_self_loc_model = newModel;}

    /*!
    @brief Sets the current self localisation filter.
    @param newFilter The ID of the new filter.
    */
    void setSelfLocFilter(KFBuilder::Filter newFilter) {m_self_loc_filter = newFilter;}

    /*!
    @brief Sets the current ball localisation model.
    @param newModel The ID of the new model.
    */
    void setBallLocModel(KFBuilder::Model newModel) {m_ball_loc_model = newModel;}

    /*!
    @brief Sets the current ball localisation filter.
    @param newFilter The ID of the new filter.
    */
    void setBallLocFilter(KFBuilder::Filter newFilter) {m_ball_loc_filter = newFilter;}

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
    @brief Retrieve the name of a given filter type.
    @param The ID of the filter.
    @return A string containing the name of the filter.
    */
    std::string filterString(KFBuilder::Filter filter) const;

    /*!
    @brief Retrieve the name of a given model type.
    @param The ID of the model.
    @return A string containing the name of the model.
    */
    std::string modelString(KFBuilder::Model model) const;


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
    KFBuilder::Model m_self_loc_model;
    KFBuilder::Model m_ball_loc_model;
    KFBuilder::Filter m_self_loc_filter;
    KFBuilder::Filter m_ball_loc_filter;
};

#endif // LOCALISATIONSETTINGS_H
