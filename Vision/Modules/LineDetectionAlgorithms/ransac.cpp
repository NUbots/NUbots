//#include "ransac.h"

//RANSAC::RANSAC()
//{
//}

//bool FieldEdgeDetection::ransacLine(
//      const vector<Point>& points, Line& result,
//      Point& resultp1, Point& resultp2,
//      bool **con, unsigned int k, float e, unsigned int n, unsigned int seed)
//{

//   if (points.size() < n) {
//      return false;
//   }

//   // error of best line found so far
//   float minerr = numeric_limits<float>::max();
//   // arrays for storing concensus sets
//   bool *best_concensus;
//   bool *this_concensus;
//   static bool c1[IMAGE_COLS/SALIENCY_DENSITY];
//   static bool c2[IMAGE_COLS/SALIENCY_DENSITY];
//   best_concensus = c1;

//   for (unsigned int i = 0; i < k; ++i) {
//      // randomly select 2 distinct points
//      unsigned int p1 = rand_r(&seed) % points.size();
//      unsigned int p2 = p1;
//      while (p1 == p2) {
//         p2 = rand_r(&seed) % points.size();
//      }
//      // generate the line between those points
//      Line l(points[p1].x, points[p1].y,
//             points[p2].x, points[p2].y);
//      l.var = 0;
//      // figure out the variance (sum of distances of points from the line)
//      // could use dist() here, but since the denominator is consistent, we
//      // save time and implement it again here.
//      float denom = sqrt(l.t1*l.t1 + l.t2*l.t2);
//      float newe = e*denom;
//      unsigned int concensus = 0;
//      unsigned int q;
//      vector<LinePoint>::const_iterator j;
//      // store the concensus set in a bit-array
//      // don't overwrite the current best one
//      if (c1 == best_concensus) {
//         this_concensus = c2;
//      } else {
//         this_concensus = c1;
//      }
//      for (j = points.begin(), q = 0; j != points.end(); ++j, ++q) {
//         float dist = (l.t1*((*j).first) + l.t2*((*j).second) + l.t3);
//         if (dist < 0) {
//            dist *= -1;
//         }
//         if (dist < newe) {
//            l.var += dist;
//            ++concensus;
//            this_concensus[q] = true;
//         } else {
//            this_concensus[q] = false;
//         }
//      }
//      l.var /= denom;
//      static float k = 0.2;
//      l.var = k*l.var - concensus;
//      if (l.var < minerr && concensus >= n) {
//         minerr = l.var;
//         l.var = l.var/(points.size()*e);
//         result = l;
//         best_concensus = this_concensus;
//         resultp1 = points[p1];
//         resultp2 = points[p2];
//      }
//   }

//   if (minerr < numeric_limits<float>::max()) {
//      *con = best_concensus;
//      return true;
//   } else {
//      return false;
//   }
//}
