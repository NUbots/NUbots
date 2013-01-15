#include <boost/foreach.hpp>
#include <vector>

#ifndef CONFIG_RANGE_H
#define CONFIG_RANGE_H

namespace ConfigSystem
{
    // enum representing the possible types of boundaries of an interval.
    // open   <==> (_min, _max) <==> _min <  x >  _max
    // closed <==> [_min, _max] <==> _min <= x >= _max
    // none   <==> (-inf, +inf) <==> unrestricted x
    enum BoundType { NONE, OPEN, CLOSED };

    /*! 
     * This class represents a range, and provides 
     * methods to check whether a value falls within it.
     */
    template <typename T>
    class ConfigRange
    {
    private:
        //! Minimum and maximum values in the range
        T _min, _max;

        //! Type of the lower and upper bounds (closed (<=), open (<), or none)
        BoundType _lBound, _uBound;

        //! If true, test() will pass iff the value is not 
        //! between _min and _max.
        //! Ex. _outside = false; ==> [_min, _max]
        //! Ex. _outside = true ; ==> (-inf, _min] U [_max, +inf)
        bool _outside;
        
    public:
        // ConfigRange();
        // ConfigRange(T min, T max);
        // ConfigRange(T min, T max, BoundType lBound, BoundType uBound );
        ConfigRange(
            T min = 0, 
            T max = 1, 
            bool outside = false
            )
        {
            _min     = min    ;
            _max     = max    ;
            _outside = outside;
            _lBound  = CLOSED ;
            _uBound  = CLOSED ;
        };

        ConfigRange(
            T min, 
            T max, 
            bool outside,
            BoundType lBound,
            BoundType uBound
            )
        {
            _min     = min    ;
            _max     = max    ;
            _outside = outside;
            _lBound  = lBound ;
            _uBound  = uBound ;
        };

        bool test(std::vector<T> values)
        {
            BOOST_FOREACH(T &val, values)
            {
                if(!test(val)) return false;
            }

            return true;
        };

        //! Returns whether the given value satisfies the constraints specified
        //! by this range object.
        bool test(T value)
        {
            // Comparisons (unnecessary comparisons, etc. will hopefully be 
            // optimised away by the compiler).

            bool valLU  = value <  _max;
            bool valLEU = value <= _max;
            bool valGU  = !valLEU      ;
            bool valGEU = !valLU       ;
            bool valLL  = value <  _min;
            bool valLEL = value <= _min;
            bool valGL  = !valLEL      ;
            bool valGEL = !valLL       ;

            bool lbO = _lBound == OPEN  ;
            bool lbC = _lBound == CLOSED;
            bool lbN = _lBound == NONE  ;
            bool ubO = _uBound == OPEN  ;
            bool ubC = _uBound == CLOSED;
            bool ubN = _uBound == NONE  ;

            // The following tests could be optimised a little.
            if(!_outside) // [...] || (...)
            {
                // If value must be less than max and greater than min

                if(valLEL && lbO) return false; //  x <= (...
                if(valGEU && ubO) return false; //  ...) >= x
                if(valLL  && lbC) return false; // x < [...
                if(valGU  && ubC) return false; // ...] < x
            }
            else // ...) (... || ...] [...
            {
                // If value must be either greater than max or less than min

                if(lbO   && valGL  && valLU) return false; // ...) >  x <  ?...
                if(lbO   && valGEL && valLU) return false; // ...) >= x <  ?...
                if(lbC   && valGL  && valLU) return false; // ...] >  x <  ?...
                if(lbC   && valGEL && valLU) return false; // ...] >= x <  ?...
                
                if(valGL && valLU  && ubO  ) return false; // ...? >  x <  )...
                if(valGL && valLEU && ubO  ) return false; // ...? >  x <= )...
                if(valGL && valLU  && ubC  ) return false; // ...? >  x <  ]...
                if(valGL && valLEU && ubC  ) return false; // ...? >  x <= ]...
            }

            return true;
        };
    };
}

#endif

