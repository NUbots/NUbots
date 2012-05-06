#include "goal.h"

Goal::Goal()
{
    Goal(Invalid);
}

Goal::Goal(ID id)
{
    Goal(id, Quad(0,0,0,0));
}

Goal::Goal(ID id, const Quad &corners)
{
    m_id = id;
    m_corners = corners;
    //SET WIDTH
}
