#include "Configurable.h"


void Configurable::setConfigBasePath(std::string configBasePath)
{
    _configBasePath = configBasePath;
}

std::string Configurable::getConfigBasePath()
{
    return _configBasePath;
}

bool Configurable::isConfigOutdated()
{
    return _configModified;
}

void Configurable::setConfigAsRecent()
{
    _configModified = false;
}

void Configurable::setConfigAsOutdated()
{
    _configModified = true;
}