#ifndef COLOURREPLACEMENTRULE_H
#define COLOURREPLACEMENTRULE_H


#include "visionblackboard.h"

#include "VisionTools/classificationcolours.h"
#include "VisionTypes/coloursegment.h"

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
    static ColourSegment nomatch;
    
    ColourReplacementRule();
    
    bool match(const ColourSegment& before, const ColourSegment& middle, const ColourSegment& after, ScanDirection dir) const;
    
    ReplacementMethod getMethod() const;
    
    friend ostream& operator<< (ostream& output, const ColourReplacementRule& c);
    friend ostream& operator<< (ostream& output, const vector<ColourReplacementRule>& v);
    
    friend istream& operator>> (istream& input, ColourReplacementRule& c);
    friend istream& operator>> (istream& input, vector<ColourReplacementRule>& v);
    
private:
    string m_name;
    
    float m_middle_min, m_middle_max,
          m_before_min, m_before_max,
          m_after_min,  m_after_max;
    
    vector<ClassIndex::Colour>  m_before, 
                                m_middle, 
                                m_after;
    
    ReplacementMethod m_method;
};

#endif // COLOURREPLACEMENTRULE_H
