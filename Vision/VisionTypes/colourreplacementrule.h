#ifndef COLOURREPLACEMENTRULE_H
#define COLOURREPLACEMENTRULE_H


#include "Vision/visionblackboard.h"

#include "Vision/VisionTools/classificationcolours.h"
#include "Vision/VisionTypes/coloursegment.h"

class ColourReplacementRule
{
//METHOD DEFINITION
public:
    enum ReplacementMethod {
        BEFORE,
        AFTER,
        SPLIT,
        INVALID
    };
    
    /*!
      Gets the name of the given method.
      @param method The method name desired.
      @return String name of the method.
      */
    static string getMethodName(ReplacementMethod method)
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
    static ReplacementMethod getMethodFromName(const string& name)
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
    
//ACTUAL CLASS DEFINITION
public:
    static ColourSegment nomatch;   //! @variable a static segment used to represent one that cannot be matched to any rule.
    
    ColourReplacementRule();
    
    /*!
      Checks if the given segment triplet matches this rule.
      @param before the first segment.
      @param middle the second segment.
      @param after the last segment.
      @param dir The scan direction (vertical or horizontal).
      @return Whether it is a match.
      */
    bool match(const ColourSegment& before, const ColourSegment& middle, const ColourSegment& after, ScanDirection dir) const;
    
    /*!
      Returns the replacement method (before, after or split) for this rule.
       - before - the middle segment is given the colour of the first.
       - after - the middle segment is given the colour of the last.
       - split - the middle segment is split into two, each given the colour of the adjacent segment.
      @return An enum for the method.
      */
    ReplacementMethod getMethod() const;
    
    //! output stream operator.
    friend ostream& operator<< (ostream& output, const ColourReplacementRule& c);
    //! output stream operator for a vector of rules.
    friend ostream& operator<< (ostream& output, const vector<ColourReplacementRule>& v);

    //! input stream operator.
    friend istream& operator>> (istream& input, ColourReplacementRule& c);
    //! input stream operator for a vector of rules.
    friend istream& operator>> (istream& input, vector<ColourReplacementRule>& v);
    
private:
    string m_name;  //! @variable the name of the rule.
    
    int m_middle_min,   //! @variable the minimum length of the middle segment for a match.
        m_middle_max,   //! @variable the maximum length of the middle segment for a match.
        m_before_min,   //! @variable the minimum length of the first segment for a match.
        m_before_max,   //! @variable the maximum length of the first segment for a match.
        m_after_min,    //! @variable the minimum length of the last segment for a match.
        m_after_max;    //! @variable the maximum length of the last segment for a match.
    
    vector<ClassIndex::Colour>  m_before,   //! @variable The colour that the first segment must be.
                                m_middle,   //! @variable The colour that the middle segment must be.
                                m_after;    //! @variable The colour that the last segment must be.
    
    ReplacementMethod m_method;  //! @variable The replacement method for this rule.
};

#endif // COLOURREPLACEMENTRULE_H
