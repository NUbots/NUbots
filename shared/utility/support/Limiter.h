
/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2016 NUBots <nubots@nubots.net>
 */

#ifndef UTILITY_SUPPORT_LIMITER_H
#define UTILITY_SUPPORT_LIMITER_H

#include <map>

namespace utility{
	namespace support{

		template <typename Index, typename T>
		class Limiter{
		private:
			struct Range {
				T min;
				T max;
			};

			static T min(const T& x, const T& y){
				return x > y ? y : x;
			}

			static T max(const T& x, const T& y){
				return x < y ? y : x;
			}

			std::map<Index, Range> limits;
			std::map<Index, float> alpha;
			std::map<Index, T> lastValue;

		public:
			void addLimit(Index i, T min, T max){
				limits[i] = Range({min,max});
			}

			void addSmoothing(Index i, float smoothing){
				alpha[i] = std::fmax(std::fmin(smoothing,1),0);
			}

			T clamp(Index i, T x){
				if(limits.count(i) > 0){
					Range& range = limits[i];
					return min(max(x,range.min),range.max);
				}
				return x;
			}

			T clampAndSmooth(Index i, T x){
				//Todo: optimise these if-statements
				T result = x;
				if(limits.count(i) > 0){
					Range& range = limits[i];
					result = min(max(x,range.min),range.max);
				}
				if(alpha.count(i) > 0){
					if(lastValue.count(i) > 0){
						result = alpha[i] * result + (1-alpha[i]) * lastValue[i];						
					}
					lastValue[i] = result;
				}
				return result;
			}
		};
	}
}

#endif