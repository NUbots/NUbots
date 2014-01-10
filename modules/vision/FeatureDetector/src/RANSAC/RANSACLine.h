
#ifndef MODULES_VISION_FEATUREDETECTOR_RANSACLINE_H
#define MODULES_VISION_FEATUREDETECTOR_RANSACLINE_H

#include "utility/math/Line.h"

#include "NUPoint.h"
namespace modules{
	namespace vision{

		template<typename T>
		class RANSACLine : public utility::math::Line
		{
		public:
		    RANSACLine() {}

		    bool regenerate(const std::vector<T>& pts) {
		        if(pts.size() == minPointsForFit()) {
		            setLineFromPoints(pts.at(0), pts.at(1));
		            return true;
		        }
		        else {
		            return false;
		        }
		    }

		    inline size_t minPointsForFit() const {return 2;}

		    double calculateError(T p) const { return getLinePointDistance(p); }
		};


		template<>
		class RANSACLine<NUPoint> : public utility::math::Line
		{
		public:
		    RANSACLine() {}

		    bool regenerate(const std::vector<NUPoint> &pts) {
		        if(pts.size() == minPointsForFit()) {
		            setLineFromPoints(pts.at(0).groundCartesian, pts.at(1).groundCartesian);
		            return true;
		        }
		        else {
		            return false;
		        }
		    }

		    inline size_t minPointsForFit() const { return 3; }

		    double calculateError(NUPoint p) const { return getLinePointDistance(p.groundCartesian); }
		};

		
	}
}

#endif