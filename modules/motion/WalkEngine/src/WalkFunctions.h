/* WalkFunction contains the set of functions used for the walk engine
 *
 * This file is part of WalkEngine. (based largely off Team Darwin's lua code)
 * it provieds utility functions for the walkengine.
 *
 * WalkEngine is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * WalkFunction is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with WalkEngine.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

namespace modules {
	namespace motion {
		double procFunc(double a, double deadband, double maxvalue); //TODO: move documentation from .cpp to .h file
		double modAngle(double a);
		arma::vec poseGlobal(arma::vec pRelative, arma::vec pose);
		arma::vec poseRelative(arma::vec pGlobal, arma::vec pose);
		arma::vec se2Interpolate(double t, arma::vec u1, arma::vec u2);
	}
}