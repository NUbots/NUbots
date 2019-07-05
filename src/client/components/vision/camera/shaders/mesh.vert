precision lowp float;
precision lowp int;

uniform mat4 projectionMatrix;
uniform mat4 modelViewMatrix;
uniform vec2 viewSize;
uniform mat4 Hcw;
uniform float focalLength;
uniform vec2 centre;
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

// TODO(trent) these should be moved into a separate GLSL file once there is a decent #include system
vec2 projectEquidistant(vec3 ray, float f, vec2 c) {
  // Calculate some intermediates
  float theta     = acos(ray.x);
  float r         = f * theta;
  float rSinTheta = 1.0 / sqrt(1.0 - ray.x * ray.x);

  // Work out our pixel coordinates as a 0 centred image with x to the left and y up (screen space)
  vec2 screen = ray.x >= 1.0 ? vec2(0) : vec2(r * ray.y * rSinTheta, r * ray.z * rSinTheta);

  // Then apply the offset to the centre of our lens
  return screen - c;
}

vec2 projectEquisolid(vec3 ray, float f, vec2 c) {
  // Calculate some intermediates
  float theta     = acos(ray.x);
  float r         = 2.0 * f * sin(theta * 0.5);
  float rSinTheta = 1.0 / sqrt(1.0 - ray.x * ray.x);

  // Work out our pixel coordinates as a 0 centred image with x to the left and y up (screen space)
  vec2 screen = ray.x >= 1.0 ? vec2(0) : vec2(r * ray.y * rSinTheta, r * ray.z * rSinTheta);

  // Then apply the offset to the centre of our lens
  return screen - c;
}

vec2 projectRectilinear(vec3 ray, float f, vec2 c) {
  float rx = 1.0 / ray.x;
  return vec2(f * ray.y * rx, f * ray.z * rx) - c;
}

vec2 project(vec3 ray, float f, vec2 c, int projection) {
  if (projection == RECTILINEAR_PROJECTION) return projectRectilinear(ray, f, c);
  if (projection == EQUIDISTANT_PROJECTION) return projectEquidistant(ray, f, c);
  if (projection == EQUISOLID_PROJECTION) return projectEquisolid(ray, f, c);
  return vec2(0);
}

void main() {
  // Rotate vector into camera space and project into image space
  // Correct for OpenGL coordinate system and aspect ratio
  // Focal length is * 2 since the width of the "image" is -1 to 1 (width of 2.0)
  vec2 pos = project((Hcw * vec4(position, 0)).xyz, 2.0 * focalLength, centre, projection)
             * vec2(-1.0, viewSize.x / viewSize.y);

  vBall        = ball;
  vGoal        = goal;
  vFieldLine   = fieldLine;
  vField       = field;
  vEnvironment = environment;

  gl_Position = projectionMatrix * modelViewMatrix * vec4(pos, 0.0, 1.0);
}
