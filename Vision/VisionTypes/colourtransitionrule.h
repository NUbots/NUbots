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


using namespace Vision;

class ColourTransitionRule
{
public:
    static ColourSegment nomatch;   //! @variable A static segment used to represent one that cannot be matched to any rule.

    ColourTransitionRule();
    /*!
      Checks if the given segment pair matches this rule (forward and reverse).
      @param before the preceeding segment.
      @param middle the middle segment.
      @param after the following segment.
      @return Whether it is a match in either direction.
      */
    bool match(const ColourSegment& before, const ColourSegment& middle, const ColourSegment& after) const;
    //! Returns the ID of the field object that this rule is for.
    COLOUR_CLASS getColourClass() const;

    //! output stream operator.
    friend std::ostream& operator<< (std::ostream& output, const ColourTransitionRule& c);
    //! output stream operator for a vector of rules.
    friend std::ostream& operator<< (std::ostream& output, const std::vector<ColourTransitionRule>& v);

    //! input stream operator.
    friend std::istream& operator>> (std::istream& input, ColourTransitionRule& c);
    //! input stream operator for a vector of rules.
    friend std::istream& operator>> (std::istream& input, std::vector<ColourTransitionRule>& v);

private:
    //std::vector<COLOUR_CLASS> m_potential_vfo_list;
    COLOUR_CLASS m_colour_class;     //! @variable The ID of the field object that this rule is for.

    std::vector<Colour>  m_before,   //! @variable The colour that the previous segment must be.
                    m_middle,   //! @variable The colour that this segment must be
                    m_after;    //! @variable The colour that the following segment must be.

    unsigned int m_before_min,   //! @variable the minimum length of the previous segment for a match.
        m_before_max,   //! @variable the maximum length of the previous segment for a match.
        m_min,          //! @variable the minimum length of the segment for a match.
        m_max,          //! @variable the maximum length of the segment for a match.
        m_after_min,    //! @variable the minimum length of the following segment for a match.
        m_after_max;    //! @variable the maximum length of the following segment for a match.

    /*!
      Checks if the given segment triplet matches this rule in one direction.
      @param before the preceeding segment.
      @param middle the middle segment.
      @param after the following segment.
      @return Whether it is a match.
      */
    bool oneWayMatch(const ColourSegment& before, const ColourSegment& middle, const ColourSegment& after) const;
};

#endif // COLOURTRANSITIONRULE_H
