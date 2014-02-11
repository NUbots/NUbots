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

#include <cmath> //aka <math.h>
#include <algorithm> //uknown reason, if this is'nt included compiler reports min() & max() as not being part of the standard namespace..
#include <armadillo>

//util.lua------------------------
double procFunc(double a, double deadband, double maxvalue) { //a function for IMU feedback (originally from teamdarwin2013release/player/util/util.lua)
	double   ret  = std::min( std::max(0., std::abs(a)-deadband), maxvalue);
	if(a<=0) ret *= -1.;
	return   ret;
}

double mod_angle(double a) { // reduce an angle to [-pi, pi)
	if(a==0) return 0.;
	a = std::fmod(a, (2. * M_PI)); //fmod(a,b) the same as a%b, but for doubles (fmod() defined in math.h)
	if(a >= M_PI)
		a -= 2. * M_PI;
	return a;
}

arma::vec pose_global(arma::vec pRelative, arma::vec pose) { //TEAMDARWIN LUA VECs START INDEXING @ 1 not 0 !!
	double ca = std::cos(pose(2));
	double sa = std::sin(pose(2));

	arma::vec ret(3);
	ret(0) = pose(0) + ca * pRelative(0) - sa * pRelative(1);
	ret(1) = pose(1) + sa * pRelative(0) + ca * pRelative(1);
	ret(2) = pose(2) + pRelative(2);
	return ret;
}

arma::vec pose_relative(arma::vec pGlobal, arma::vec pose) {
	double ca = std::cos(pose(2));
	double sa = std::sin(pose(2));
	double px = pGlobal(0) - pose(0);
	double py = pGlobal(1) - pose(1);
	double pa = pGlobal(2) - pose(2);

	arma::vec ret(3); 
	ret(0) =  ca * px + sa * py;
	ret(1) = -sa * px + ca * py;
	ret(2) = mod_angle(pa);
	return ret;
}

//should t be an integer???
arma::vec se2_interpolate(double t, arma::vec u1, arma::vec u2) { //helps smooth out the motions using a weighted average

	arma::vec ret(3);
	ret(0) = u1(0) + t*(u2(0) - u1(0));
	ret(1) = u1(1) + t*(u2(1) - u1(1));
	ret(2) = u1(2) + t*mod_angle(u2(2) - u1(2));
	return ret;
}

/*/testing --NEVER include a main function in the actual build ;)
#include <iostream>
int main() { 
	arma::vec a; a << 123 << 223 << 3232;
	arma::vec b; b << 0 << 0 << 0;
	std::cout << "test " << procFunc(-1,2,2.6) << std::endl << pose_global(a,b) << std::endl << pose_relative(a,b) << std::endl; 
	std::cout << M_PI << std::endl;
	std::cout << se2_interpolate(3.14, a, b) << std::endl;
	return 0; 
} //*/

/*TODO: MAKE A .H FILE, NAMESPACE, GUARD etc and finish the following dependancy functions for WalkEngine...
Config.Walk.stancelimitX,StancelimitY,....mostionDef[motionname], use_alternative trajectory 
// basically, define a bunch of parameters and store them somewhere (seem to already be partially ported already)

Body.get_time()
Body.set_larm_command(a)
Body.set_rarm_command(a)
Body.set_larm_hardness(a,b,c)
Body.set_lleg_hardness(a,b,c)
Body.set_lleg_hardness(a)
Body.get_sensor_imuGyrRPY()
Body.set_lleg_command(a)

Kinematics.inverse_legs(a,b,c,d)
Vector.new({0,0,0})

mcm.get_footX()
mcm.set_walk_isMoving(bool)
mcm.set_walk_isStepping(bool)
*/