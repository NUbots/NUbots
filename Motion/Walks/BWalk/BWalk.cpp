#include "BWalk.h"

BWalk::BWalk(NUSensorsData* data, NUActionatorsData* actions): NUWalk(data, actions)
{
    m_walk_parameters.load("BWalkDefault");
    return;
}

BWalk::~BWalk()
{

}
