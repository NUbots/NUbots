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
