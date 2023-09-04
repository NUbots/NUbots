#ifndef PROJECTION_GLSL
#define PROJECTION_GLSL

#include "math_constants.glsl"

#define RECTILINEAR_PROJECTION 1
#define EQUIDISTANT_PROJECTION 2
#define EQUISOLID_PROJECTION   3

/**
 * Takes an undistorted radial distance from the optical axis and computes and applies an inverse distortion
 *
 * @param r the radius to undistort
 * @param k the undistortion coefficients
 *
 * @return an distortion radius
 */
float distort(float r, vec2 k) {
    // Uses the math from the paper
    // An Exact Formula for Calculating Inverse Radial Lens Distortions
    // https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4934233/pdf/sensors-16-00807.pdf
    // These terms have been stripped back to only include k1 and k2 and only uses the first 4 terms
    // if more are needed in the future go and get them from the original paper
    // TODO if performance ever becomes an issue, this can be precomputed for the same k values
    float b1 = -k.x;
    float b2 = 3.0 * (k.x * k.x) - k.y;
    float b3 = -12.0 * (k.x * k.x) * k.x + 8.0 * k.x * k.y;
    float b4 = 55.0 * (k.x * k.x) * (k.x * k.x) - 55.0 * (k.x * k.x) * k.y + 5.0 * (k.y * k.y);

    // These parenthesis are important as they allow the compiler to optimise further
    // Since floating point multiplication is not commutative r * r * r * r != (r * r) * (r * r)
    // This means that the first needs 3 multiplication operations while the second needs only 2
    return r
           * (1.0                                               //
              + b1 * (r * r)                                    //
              + b2 * ((r * r) * (r * r))                        //
              + b3 * ((r * r) * (r * r)) * (r * r)              //
              + b4 * ((r * r) * (r * r)) * ((r * r) * (r * r))  //
           );
}

/**
 * Takes a distorted radial distance from the optical axis and applies the polynomial distortion coefficients to
 * undistort it
 *
 * @param r the radius to undistort
 * @param k the undistortion coefficients
 *
 * @return an undistorted radius
 */
float undistort(float r, vec2 k) {
    // These parenthesis are important as they allow the compiler to optimise further
    return r * (1.0 + k.x * (r * r) + k.y * (r * r) * (r * r));
}

/**
 * Given a radius from the optical centre of a projection, calculate the angle that is made from the optical axis to the
 * point using an equidistant projection.
 *
 * @param r the radius from the optical centre of the image
 * @param f the focal length of the lens
 *
 * @return the angle from the optical axis to this point
 */
float equidistantTheta(float r, float f) {
    return r / f;
}

/**
 * Given an angle from the optical axis of the lens, calculate the distance from the optical centre of the lens using
 * an equidistant projection.
 *
 * @param theta the angle between the optical axis to the point
 * @param f     the focal length of the lens
 *
 * @return the distance from the optical centre when the point is projected onto the screen
 */
float equidistantR(float theta, float f) {
    return f * theta;
}

/**
 * Given a radius from the optical centre of a projection, calculate the angle that is made from the optical axis to the
 * point using an equisolid projection.
 *
 * @param r the radius from the optical centre of the image
 * @param f the focal length of the lens
 *
 * @return the angle from the optical axis to this point
 */
float equisolidTheta(float r, float f) {
    return 2.0 * asin(r / (2.0 * f));
}

/**
 * Given an angle from the optical axis of the lens, calculate the distance from the optical centre of the lens using
 * an equisolid projection.
 *
 * @param theta the angle between the optical axis to the point
 * @param f     the focal length of the lens
 *
 * @return the distance from the optical centre when the point is projected onto the screen
 */
float equisolidR(float theta, float f) {
    return 2.0 * f * sin(theta * 0.5);
}

/**
 * Given a radius from the optical centre of a projection, calculate the angle that is made from the optical axis to the
 * point using a rectilinear projection.
 *
 * @param r the radius from the optical centre of the image
 * @param f the focal length of the lens
 *
 * @return the angle from the optical axis to this point
 */
float rectilinearTheta(float r, float f) {
    return atan(r / f);
}

/**
 * Given an angle from the optical axis of the lens, calculate the distance from the optical centre of the lens using
 * a rectilinear projection.
 *
 * @param theta the angle between the optical axis to the point
 * @param f     the focal length of the lens
 *
 * @return the distance from the optical centre when the point is projected onto the screen
 */
float rectilinearR(float theta, float f) {
    return f * tan(clamp(theta, 0.0, M_PI_2));
}

/**
 * Unprojects a pixel coordinate into a unit vector. The input coordinate system is one where the origin is the centre
 * of the image, x is to the left and y is up. The output unit vector is measured as a ray measured in coordinate system
 * where x is forward down the camera axis, y is to the left and z is up
 *
 * @param point      the pixel coordinate to unproject back to a unit vector
 * @param f          the focal length to use in the projection
 * @param c          the offset from the centre of the image to the optical centre lens
 * @param k          the distortion coefficients for the lens model
 * @param projection the type of projection to use
 *
 * @return the unit vector that corresponds to the provided pixel
 */
vec3 unproject(vec2 point, float f, vec2 c, vec2 k, int projection) {
    vec2 p      = point - c;
    float rD    = length(p);
    float rU    = undistort(rD, k);
    float theta = 0.0;
    if (projection == RECTILINEAR_PROJECTION)
        theta = rectilinearTheta(rU, f);
    else if (projection == EQUIDISTANT_PROJECTION)
        theta = equidistantTheta(rU, f);
    else if (projection == EQUISOLID_PROJECTION)
        theta = equisolidTheta(rU, f);
    return vec3(cos(theta), sin(theta) * p / rD);
}

/**
 * Projects the camera ray measured in coordinate system where x is forward down the camera axis, y is to the
 * left and z is up. The output coordinate system is one where the origin is the centre of the image, x is to the
 * left and y is up.
 *
 * @param ray        the ray to project to pixel coordinates
 * @param f          the focal length to use in the projection
 * @param c          the offset from the centre of the image to the optical centre lens
 * @param k          the distortion coefficients for the lens model
 * @param projection the type of projection to use
 *
 * @return the position of the pixel measured as a fraction of the image width
 */
vec2 project(vec3 ray, float f, vec2 c, vec2 k, int projection) {
    float theta     = acos(ray.x);
    float rSinTheta = 1.0 / sqrt(1.0 - ray.x * ray.x);
    float rU        = 0.0;
    if (projection == RECTILINEAR_PROJECTION)
        rU = rectilinearR(theta, f);
    else if (projection == EQUIDISTANT_PROJECTION)
        rU = equidistantR(theta, f);
    else if (projection == EQUISOLID_PROJECTION)
        rU = equisolidR(theta, f);

    // This is an approximation. This value just needs to be high enough to be outside the image view, and low
    // enough to be before the turning point of the distortion function for the lens parameters.
    // `sqrt(2)` has been chosen here since it is 2 times the diagonal image distance:
    // 2 * sqrt(0.5^2 + 0.5^2)
    float criticalRU = sqrt(2.0);

    // If the input value is outside the range we care about, set the output radius to a large value to draw the
    // value far off the screen. Otherwise, distort it normally.
    float rD = rU > criticalRU ? 1.0e9 : distort(rU, k);

    vec2 p = ray.x >= 1.0 ? vec2(0) : vec2(rD * ray.y * rSinTheta, rD * ray.z * rSinTheta);
    return p + c;
}

#endif  // PROJECTION_GLSL
