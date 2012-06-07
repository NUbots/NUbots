#include "objectcandidate.h"

 /*
        unclassified,   //!< Colour has not be given a category.
        white,          //!< Colour is in the White region.
        green,          //!< Colour is in the Green region.
        shadow_object,  //!< Colour is part of a shadowed area.
        red,            //!< Colour is in the Red region.
        red_orange,     //!< Colour is in the region of overlap between Red and Orange.
        orange,         //!< Colour is in the Orange region.
        yellow_orange,  //!< Colour is in the region of overlap between Yellow and Orange.
        yellow,         //!< Colour is in the Yellow region.
        blue,           //!< Colour is in the Sky Blue region.
        shadow_blue,    //!< Colour is in the Dark Blue region.
    //*/

ObjectCandidate::ObjectCandidate()
{
    ObjectCandidate(PointType(0,0), PointType(0,0));
}

ObjectCandidate::ObjectCandidate(const PointType& top_left, const PointType& bottom_right)
{
    ObjectCandidate(top_left, bottom_right, ClassIndex::unclassified);
}

ObjectCandidate::ObjectCandidate(const PointType& top_left, const PointType& bottom_right, ClassIndex::Colour colour): colour(colour)
{
    topLeft = top_left;
    bottomRight = bottom_right;
    recalculate();
}

ObjectCandidate::ObjectCandidate(const PointType& top_left, const PointType& bottom_right, ClassIndex::Colour colour, const vector<Transition>& candidate_segments): colour(colour)
{
    topLeft = top_left;
    bottomRight = bottom_right;    
    recalculate();
    
    transitions = candidate_segments;
}

const vector<Transition>& ObjectCandidate::getTransitions() const
{
    return transitions;
}

void ObjectCandidate::addTransitions(const vector<Transition>& new_transitions)
{
    transitions.insert(transitions.end(), new_transitions.begin(), new_transitions.end());
    return;
}

void ObjectCandidate::addTransition(const Transition& new_transition)
{
    transitions.push_back(new_transition);
    return;
}

ObjectCandidate::~ObjectCandidate()
{
    return;
}

const PointType& ObjectCandidate::getTopLeft() const
{
    return topLeft;
}

const PointType& ObjectCandidate::getBottomRight() const
{
    return bottomRight;
}

int ObjectCandidate::getWidth() const
{
    return width;
}

int ObjectCandidate::getHeight() const
{
    return height;
}

float ObjectCandidate::getAspect() const
{
    return aspect;
}

ClassIndex::Colour ObjectCandidate::getColour() const
{
    return colour;
}

void ObjectCandidate::setColour(ClassIndex::Colour c)
{
    colour = c;
}
void ObjectCandidate::setTopLeft(const PointType& point)
{
    topLeft = point;
    recalculate();
}

void ObjectCandidate::setBottomRight(const PointType& point)
{
    bottomRight = point;
    recalculate();
}

const PointType& ObjectCandidate::getCentre() const
{
    return centre;
}

void ObjectCandidate::recalculate()
{
    centre = (topLeft + bottomRight) * 0.5;
    width = (bottomRight.x - topLeft.x);
    height = (bottomRight.y - topLeft.y);
    aspect = (float)(bottomRight.x - topLeft.x) / (float)(bottomRight.y - topLeft.y);
}

