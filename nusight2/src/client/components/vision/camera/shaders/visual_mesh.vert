precision lowp float;
precision lowp int;

uniform mat4 projectionMatrix;
uniform mat4 modelViewMatrix;
uniform vec2 viewSize;
uniform mat4 Hcw;
uniform float focalLength;
uniform vec2 centre;
uniform vec2 k;
uniform int projection;

attribute vec3 position;

attribute float ball;
attribute float goal;
attribute float field;
attribute float fieldLine;
attribute float environment;

varying float vBall;
varying float vGoal;
varying float vFieldLine;
varying float vField;
varying float vEnvironment;

#define RECTILINEAR_PROJECTION 1
#define EQUIDISTANT_PROJECTION 2
#define EQUISOLID_PROJECTION 3

#define M_PI_2 1.57079632679489661923

// TODO(trent) these should be moved into a separate GLSL file once there is a decent #include system

/**
 * Takes an undistorted radial distance from the optical axis and computes and applies an inverse distortion
 *
 * @param r the radius to undistort
 * @param k the undistortion coefficents
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
 * Given an angle from the optical axis of the lens, calcululate the distance from the optical centre of the lens using
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
 * Given an angle from the optical axis of the lens, calcululate the distance from the optical centre of the lens using
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
 * Given an angle from the optical axis of the lens, calcululate the distance from the optical centre of the lens using
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
 * Projects the camera ray measured in coordinate system where x is forward down the camera axis, y is to the
 * left and z is up. The output coordinate system is one where the origin is the centre of the image, x is to the
 * left and y is up.
 *
 * @param ray        the ray to project to pixel coordinates
 * @param f          the focal length to use in the projection
 * @param c          the offset from the centre of the image to the optical centre lens
 * @param k          the distortion coefficents for the lens model
 * @param projection the type of projection to use
 *
 * @return the position of the pixel measured as a fraction of the image width
 */
vec2 project(vec3 ray, float f, vec2 c, vec2 k, int projection) {
  float theta     = acos(ray.x);
  float rSinTheta = 1.0 / sqrt(1.0 - ray.x * ray.x);
  float rU         = 0.0;
  if (projection == RECTILINEAR_PROJECTION) rU = rectilinearR(theta, f);
  else if (projection == EQUIDISTANT_PROJECTION) rU = equidistantR(theta, f);
  else if (projection == EQUISOLID_PROJECTION) rU = equisolidR(theta, f);
  float rD = distort(rU, k);
  vec2 p   = ray.x >= 1.0 ? vec2(0) : vec2(rD * ray.y * rSinTheta, rD * ray.z * rSinTheta);
  return p - c;
}

void main() {
  // Rotate vector into camera space and project into image space
  // Correct for OpenGL coordinate system and aspect ratio
  // Multiply by 2.0 to get a coordinate since the width of the "image (the camera planes)" is -1 to 1 (width of 2.0)
  vec2 pos = project((Hcw * vec4(position, 0)).xyz, focalLength, centre, k, projection)
             * vec2(-1.0, viewSize.x / viewSize.y) * 2.0;

  vBall        = ball;
  vGoal        = goal;
  vFieldLine   = fieldLine;
  vField       = field;
  vEnvironment = environment;

  gl_Position = projectionMatrix * modelViewMatrix * vec4(pos, 0.0, 1.0);
}
