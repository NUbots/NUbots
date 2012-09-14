/*!
  * @file colourrule.h
  * @class ColourRule
  * @author Shannon Fenn
  * @date 23-03-12
  *
  * @brief Class to use for simplified but flexible segment rule matching.
  *
  */

#ifndef COLOURTRANSITIONRULE_H
#define COLOURTRANSITIONRULE_H

#include <vector>
#include <iostream>

#include "Vision/visionblackboard.h"
#include "Vision/VisionTools/classificationcolours.h"
#include "Vision/VisionTypes/coloursegment.h"
#include "Vision/VisionTypes/VisionFieldObjects/visionfieldobject.h"

using namespace std;

class ColourTransitionRule
{
public:
    static ColourSegment nomatch;   //! @variable A static segment used to represent one that cannot be matched to any rule.

    ColourTransitionRule();
    /*!
      Checks if the given segment pair matches this rule.
      @param before the first segment.
      @param after the second segment.
      @param dir The scan direction (vertical or horizontal).
      @return Whether it is a match.
      */
    bool match(const ColourSegment& before, const ColourSegment& after, ScanDirection dir) const;
    //! Returns the ID of the field object that this rule is for.
    VisionFieldObject::VFO_ID getVFO_ID() const;

    //! output stream operator.
    friend ostream& operator<< (ostream& output, const ColourTransitionRule& c);
    //! output stream operator for a vector of rules.
    friend ostream& operator<< (ostream& output, const vector<ColourTransitionRule>& v);

    //! input stream operator.
    friend istream& operator>> (istream& input, ColourTransitionRule& c);
    //! input stream operator for a vector of rules.
    friend istream& operator>> (istream& input, vector<ColourTransitionRule>& v);

private:
    //vector<VisionFieldObject::VFO_ID> m_potential_vfo_list;
    VisionFieldObject::VFO_ID m_vfo_id;     //! @variable The ID of the field object that this rule is for.

    vector<ClassIndex::Colour>  m_before,   //! @variable The colour that the first segment must be.
                                m_after;    //! @variable The colour that the second segment must be.

    int m_before_min,   //! @variable the minimum length of the first segment for a match.
        m_before_max,   //! @variable the maximum length of the first segment for a match.
        m_after_min,    //! @variable the minimum length of the second segment for a match.
        m_after_max;    //! @variable the maximum length of the second segment for a match.
};

#endif // COLOURTRANSITIONRULE_H
