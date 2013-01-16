/*! 
	@file 	ConfigRange.h
    @brief 	Header file for the ConfigRange class. 
 
    @class 	ConfigRange
    @brief 	Templated class used for storing ranges.

    @author Sophie Calland, Mitchell Metcalfe 
 
  Copyright (c) 2012 Sophie Calland, Mitchell Metcalfe 
 
    This file is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This file is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CONFIG_RANGE_H
#define CONFIG_RANGE_H

#include <boost/foreach.hpp>
#include <vector>

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
		public:
		    // ConfigRange();
		    // ConfigRange(T min, T max);
		    // ConfigRange(T min, T max, BoundType lBound, BoundType uBound );
		    ConfigRange(T min = 0, T max = 1, bool outside = false);
            
		    ConfigRange(T min, T max, bool outside, BoundType lBound, BoundType uBound);
            
			//destructor:
			~ConfigRange();



			
			/*! @brief Retrieves the "_max" private member variable.
 
   			@param N/A.
    		@return Returns a pointer to the "_max" private member variable.
 			*/
			T* getMax();

			/*! @brief Retrieves the "_max" private member variable.
 
   			@param N/A.
    		@return Returns a const pointer to the "_max" private member variable.
 			*/
			const T* getMax() const;
			
			/*! @brief Retrieves the "_min" private member variable.
 
   			@param N/A.
    		@return Returns a pointer to the "_min" private member variable.
 			*/
			T* getMin();
			
			/*! @brief Retrieves the "_min" private member variable.
 
   			@param N/A.
    		@return Returns a const pointer to the "_min" private member variable.
 			*/
			const T* getMin() const;
			
			/*! @brief Retrieves the upper bound type of the range.
 
   			@param N/A.
    		@return Returns the BoundType corresponding to the upper bound of the ConfigRange object.
 			*/
			BoundType getUpperBoundType();
			
			/*! @brief Retrieves the lower bound type of the range.
 
   			@param N/A.
    		@return Returns the BoundType corresponding to the lower bound of the ConfigRange object.
 			*/
			BoundType getLowerBoundType();
			
			

=======
            
            
            
            
>>>>>>> 9bba4a0c29c3bd2838db5047bba3e00f910cb6f9
		    bool test(std::vector<T> values);
            
		    //! Returns whether the given value satisfies the constraints specified
		    //! by this range object.
		    bool test(T value);
		    
		    
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
    };
}

#include "ConfigRange.template"
#endif

