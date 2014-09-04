#include "colourtransitionrule.h"
#include <boost/algorithm/string.hpp>
#include <algorithm>
#include "debug.h"

ColourSegment ColourTransitionRule::nomatch(Point(0,0), Point(0,0), invalid);

ColourTransitionRule::ColourTransitionRule()
{
}

bool ColourTransitionRule::match(const ColourSegment &before, const ColourSegment& middle, const ColourSegment &after) const
{
    return oneWayMatch(before, middle, after) || oneWayMatch(after, middle, before); //test both directions
}

//! @brief Returns the ID of the VFO this rule is related to.
COLOUR_CLASS ColourTransitionRule::getColourClass() const
{
    return m_colour_class;
}


/*! @brief Stream insertion operator for a single ColourTransitionRule
 */
std::ostream& operator<< (std::ostream& output, const ColourTransitionRule& c)
{
    std::vector<Colour>::const_iterator it;

    output << getColourClassName(c.m_colour_class) << ":\n";

    //before
    output << "before: (" << c.m_before_min << ", " << c.m_before_max << ") [";
    for(it = c.m_before.begin(); it != c.m_before.end(); it++) {
        output << getColourName(*it) << ", ";
    }
    output << "]\t// (min, max) [colourlist]\n";

    //this
    output << "middle: (" << c.m_min << ", " << c.m_max << ") [";
    for(it = c.m_middle.begin(); it != c.m_middle.end(); it++) {
        output << getColourName(*it) << ", ";
    }
    output << "]\t// (min, max) [colourlist]\n";

    //after
    output << "after: (" << c.m_after_min << ", " << c.m_after_max << ") [";
    for(it = c.m_after.begin(); it != c.m_after.end(); it++) {
        output << getColourName(*it) << ", ";
    }
    output << "]\t// (min, max) [colourlist]" << std::endl;

    return output;
}

/*! @brief Stream insertion operator for a vector of ColourTransitionRule.
 *  @relates ColourRule
 */
std::ostream& operator<< (std::ostream& output, const std::vector<ColourTransitionRule>& v)
{
    for (size_t i=0; i<v.size(); i++)
        output << v.at(i);
    return output;
}

/*! @brief Stream extraction operator for a ColourTransitionRule.
 *  @relates ColourRule
 */
std::istream& operator>> (std::istream& input, ColourTransitionRule& c)
{
    std::stringstream colour_stream;
    std::string next, colour_str;
    std::string id_str;

    // read in the rule name
    getline(input, id_str, ':');
    boost::trim(id_str);
    c.m_colour_class = getColourClassFromName(id_str);
//! DO MORE HERE

    //BEFORE
    //reset colour list
    c.m_before.clear();
    // read in the before: (min, max)
    input.ignore(30, '(');
    input >> c.m_before_min;
    input.ignore(10, ',');
    input >> c.m_before_max;
    input.ignore(10, ')');

    input.ignore(10, '[');

    //get colour list
    getline(input, colour_str, ']');
    colour_str.erase(remove(colour_str.begin(), colour_str.end(), ' '), colour_str.end());  //remove whitespace
    if(!colour_str.empty()) {
        colour_stream.str(colour_str);
        while(colour_stream.good()) {
            getline(colour_stream, next, ',');
            c.m_before.push_back(getColourFromName(next));
        }
    }

    //MIDDLE
    //reset colour list
    c.m_middle.clear();
    // read in the before: (min, max)
    input.ignore(30, '(');
    input >> c.m_min;
    input.ignore(10, ',');
    input >> c.m_max;
    input.ignore(10, ')');

    input.ignore(10, '[');

    //get colour list
    getline(input, colour_str, ']');
    colour_str.erase(remove(colour_str.begin(), colour_str.end(), ' '), colour_str.end());  //remove whitespace
    if(!colour_str.empty()) {
        colour_stream.str(colour_str);
        while(colour_stream.good()) {
            getline(colour_stream, next, ',');
            c.m_middle.push_back(getColourFromName(next));
        }
    }

    //AFTER
    //reset colour list
    c.m_after.clear();
    // read in the before: (min, max)
    input.ignore(30, '(');
    input >> c.m_after_min;
    input.ignore(10, ',');
    input >> c.m_after_max;
    input.ignore(10, ')');

    input.ignore(10, '[');

    //get colour list
    getline(input, colour_str, ']');
    colour_str.erase(remove(colour_str.begin(), colour_str.end(), ' '), colour_str.end());  //remove whitespace
    if(!colour_str.empty()) {
        colour_stream.clear();
        colour_stream.str(colour_str);
        while(colour_stream.good()) {
            getline(colour_stream, next, ',');
            c.m_after.push_back(getColourFromName(next));
        }
    }

    // ignore the rest of the line
    input.ignore(128, '\n');
    input.peek();               //trigger eofbit being set in the case of this being the last rule

    return input;
}

/*! @brief Stream extraction operator for a vector of ColourTransitionRule.
 *  @relates ColourTransitionRule
 */
std::istream& operator>> (std::istream& input, std::vector<ColourTransitionRule>& v)
{
    ColourTransitionRule temp;
    v.clear();
    while(input.good())
    {
        input >> temp;
        if(temp.getColourClass() != UNKNOWN_COLOUR) {
            v.push_back(temp);
        }
        else {
            errorlog << "ColourTransitionRule istream operator: UNKOWN_COLOUR match ignored." << std::endl;
        }
    }

    return input;
}

bool ColourTransitionRule::oneWayMatch(const ColourSegment &before, const ColourSegment &middle, const ColourSegment &after) const
{
    //check lengths first to save iterating over colour vectors pointlessly as this method is majority false
    if(!(m_min <= middle.getLength() && m_max >= middle.getLength() &&
         m_before_min <= before.getLength() && m_before_max >= before.getLength() &&
         m_after_min <= after.getLength() && m_after_max >= after.getLength())) {
        //did not match size requirements
        return false;
    }

    bool valid;
    std::vector<Colour>::const_iterator it;

    if(!m_middle.empty()) {
        if(middle.getColour() == invalid)
            return false;   //there is a before set, but no before colour
        valid = false;
        for(it = m_middle.begin(); it != m_middle.end(); it++) {
            if(*it == middle.getColour())
                valid = true;   //a match has been found
        }
        if(!valid)
            return false;   //did not match before set
    }

    if(!m_before.empty()) {
        if(before.getColour() == invalid)
            return false;   //there is a before set, but no before colour
        valid = false;
        for(it = m_before.begin(); it != m_before.end(); it++) {
            if(*it == before.getColour())
                valid = true;   //a match has been found
        }
        if(!valid)
            return false;   //did not match before set
    }

    if(!m_after.empty()) {
        if(after.getColour() == invalid)
            return false;   //there is an after set, but no after colour
        valid = false;
        for(it = m_after.begin(); it != m_after.end(); it++) {
            if(*it == after.getColour())
                valid = true;   //a match has been found
        }
        if(!valid)
            return false;   //did not match after set
    }

    return true;    //passed all checks
}
