#include "colourreplacementrule.h"

ColourSegment ColourReplacementRule::nomatch(Point(0,0), Point(0,0), invalid);

string ColourReplacementRule::getMethodName(ReplacementMethod method)
{
    switch(method)
    {
        case BEFORE:    return "before";
        case AFTER:     return "after";
        case SPLIT:     return "green";
        default:        return "unknown method";
    };
}

/*!
  Gets the method matching the given string.
  @param name String name of the method.
  @return The method desired.
  */
ColourReplacementRule::ReplacementMethod ColourReplacementRule::getMethodFromName(const string& name)
{
    if(name.compare("before") == 0)
        return BEFORE;
    else if(name.compare("after") == 0)
        return AFTER;
    else if(name.compare("split") == 0)
        return SPLIT;
    else
        return INVALID;
}

ColourReplacementRule::ColourReplacementRule()
{
}

bool ColourReplacementRule::match(const ColourSegment& before, const ColourSegment& middle, const ColourSegment& after) const
{
//    int multiplier;
//    switch(dir) {
//    case HORIZONTAL:
//        multiplier = VisionBlackboard::getInstance()->getImageWidth();
//        break;
//    case VERTICAL:
//        multiplier = VisionBlackboard::getInstance()->getImageHeight();
//        break;
//    }
//    if(!(m_middle_min*multiplier <= middle.getLengthPixels() && m_middle_max*multiplier >= middle.getLengthPixels() &&
//         m_before_min*multiplier <= before.getLengthPixels() && m_before_max*multiplier >= before.getLengthPixels() &&
//         m_after_min*multiplier <= after.getLengthPixels() && m_after_max*multiplier >= after.getLengthPixels()))
//    {
//        return false;   //did not match size requirements
//    }

//check lengths first to save iterating over colour vectors pointlessly as this method is majority false
    if(!(m_middle_min <= middle.getLength() && m_middle_max >= middle.getLength() &&
         m_before_min <= before.getLength() && m_before_max >= before.getLength() &&
         m_after_min <= after.getLength() && m_after_max >= after.getLength())) {
        //did not match size requirements
        return false;
    }

    bool valid;
    vector<Colour>::const_iterator it;
    if(!m_middle.empty()) {
        valid = false;
        for(it = m_middle.begin(); it != m_middle.end(); it++) {
            if(*it == middle.getColour())
                valid = true;   //a match has been found
        }
        if(!valid)
            return false; //did not match middle set
    }
    else
        return false;	//if middle is empty the rule matches nothing

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

ColourReplacementRule::ReplacementMethod ColourReplacementRule::getMethod() const
{
    return m_method;
}

/*! @brief Stream insertion operator for a single ColourReplacementRule
 */
ostream& operator<< (ostream& output, const ColourReplacementRule& c)
{
    vector<Colour>::const_iterator it;

    output << c.m_name << ":" << endl;

    //before
    output << "\tbefore: (" << c.m_before_min << ", " << c.m_before_max << ") [";
    for(it = c.m_before.begin(); it != c.m_before.end(); it++) {
        output << getColourName(*it) << ", ";
    }
    output << "]\t\t// (min, max) [colourlist]" << endl;

    //middle
    output << "\tmiddle: (" << c.m_middle_min << ", " << c.m_middle_max << ") [";
    for(it = c.m_middle.begin(); it != c.m_middle.end(); it++) {
        output << getColourName(*it) << ", ";
    }
    output << "]\t\t// (min, max) [colourlist]" << endl;

    //after
    output << "\tafter(" << c.m_after_min << ", " << c.m_after_max << ") [";
    for(it = c.m_after.begin(); it != c.m_after.end(); it++) {
        output << getColourName(*it) << ", ";
    }
    output << "]\t\t// (min, max) [colourlist]" << endl;

    //replacement method
    output << "\treplacement: " << ColourReplacementRule::getMethodName(c.m_method) << "\t\t// [colourlist]" << endl;

    return output;
}

/*! @brief Stream insertion operator for a vector of ColourReplacementRule.
 *  @relates ColourReplacementRule
 */
ostream& operator<< (ostream& output, const vector<ColourReplacementRule>& v)
{
    for (size_t i=0; i<v.size(); i++)
        output << v.at(i);
    return output;
}

/*! @brief Stream extraction operator for a ColourReplacementRule.
 *  @relates ColourReplacementRule
 */
istream& operator>> (istream& input, ColourReplacementRule& c)
{
    stringstream colour_stream;
    string next, colour_str;

    // read in the rule name
    getline(input, c.m_name, ':');

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

    // ignore the rest of the line
    input.ignore(128, '\n');

    //middle
    //reset colour list
    c.m_middle.clear();
    // read in the middle: (min, max)
    input.ignore(30, '(');
    input >> c.m_middle_min;
    input.ignore(10, ',');
    input >> c.m_middle_max;
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
            c.m_middle.push_back(getColourFromName(next));
        }
    }
    // ignore the rest of the line
    input.ignore(128, '\n');

    //AFTER
    //reset colour list
    c.m_after.clear();
    // read in the after: (min, max)
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
    
    //REPLACEMENT
    //get method
    input.ignore(20, ':');
    getline(input, colour_str, '/');
    colour_str.erase(remove(colour_str.begin(), colour_str.end(), ' '), colour_str.end());  //remove whitespace
    if(!colour_str.empty()) {
        c.m_method = ColourReplacementRule::getMethodFromName(colour_str);
    }
    // ignore the rest of the line - potentially holds comment
    input.ignore(128, '\n');
    
    //force eofbit in the case of last rule
    input.peek();

    return input;
}

/*! @brief Stream extraction operator for a vector of ColourReplacementRules.
 *  @relates ColourReplacementRule
 */
istream& operator>> (istream& input, vector<ColourReplacementRule>& v)
{
    ColourReplacementRule temp;
    v.clear();
    while(input.good())
    {
        input >> temp;
        v.push_back(temp);
    }

    return input;
}
