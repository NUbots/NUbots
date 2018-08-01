precision lowp float;

#define M_PI 3.1415926535897932384626433832795

// Lens/projection parameters
uniform vec2 viewSize;
uniform float focalLength;
uniform int projection;
uniform vec2 centre;

// Line parameters
uniform vec3 axis;
uniform vec3 start;
uniform vec3 end;

// Style
uniform vec4 colour;
uniform float lineWidth;

varying vec2 vUv;

#define RECTILINEAR_PROJECTION 1
#define EQUIDISTANT_PROJECTION 2
#define EQUISOLID_PROJECTION 3

// TODO(trent) these should be moved into a separate GLSL file once there is a decent #include system
vec3 unprojectEquidistant(vec2 point, float f, vec2 c) {
  float r = length(point + c);
  float theta = r / f;
  vec3 s = vec3(
    cos(theta),
    sin(theta) * point.x / r,
    sin(theta) * point.y / r
  );
  return normalize(s);
}

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

vec3 unprojectEquisolid(vec2 point, float f, vec2 c) {
  float r = length(point + c);
  float theta = 2.0 * asin(r / (2.0 * f));
  vec3 s = vec3(
    cos(theta),
    sin(theta) * point.x / r,
    sin(theta) * point.y / r
  );
  return normalize(s);
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

vec3 unprojectRectilinear(vec2 point, float f, vec2 c) {
  return normalize(vec3(f, point + c));
}

vec2 projectRectilinear(vec3 ray, float f, vec2 c) {
  float rx = 1.0 / ray.x;
  return vec2(f * ray.y * rx, f * ray.z * rx) - c;
}

vec3 unproject(vec2 point, float f, vec2 c, int projection) {
  if (projection == RECTILINEAR_PROJECTION) return unprojectRectilinear(point, f, c);
  if (projection == EQUIDISTANT_PROJECTION) return unprojectEquidistant(point, f, c);
  if (projection == EQUISOLID_PROJECTION) return unprojectEquisolid(point, f, c);
  return vec3(0);
}

vec2 project(vec3 ray, float f, vec2 c, int projection) {
  if (projection == RECTILINEAR_PROJECTION) return projectRectilinear(ray, f, c);
  if (projection == EQUIDISTANT_PROJECTION) return projectEquidistant(ray, f, c);
  if (projection == EQUISOLID_PROJECTION) return projectEquisolid(ray, f, c);
  return vec2(0);
}

/**
 * Rotate v about e by theta
 */
vec3 rotateByAxisAngle(vec3 v, vec3 e, float theta) {
  return cos(theta) * v + sin(theta) * cross(e, v) + (1.0 - cos(theta)) * (dot(e, v)) * e;
}

float angleAround(vec3 axis, vec3 start, vec3 vec) {

  // Put start and vec in the plane of axis
  vec3 aStart = normalize(cross(axis, start));
  vec3 aVec = normalize(cross(axis, vec));

  float x = dot(aStart, aVec); // cos(theta)
  float y = dot(axis, cross(aStart, aVec)); // sin(theta)

  // sin(theta)/cos(theta) = tan(theta)
  float theta = atan(y, x);

  // We want 0 -> 2pi not -pi -> pi
  theta += theta < 0.0 ? (M_PI * 2.0) : 0.0;
  return theta;
}

void main() {

  // Get our position on the screen in pixel coordinates
  vec2 screenPoint = vec2(0.5 - vUv.x, vUv.y - 0.5) * viewSize;

  // Get the gradient of the curve we are drawing
  float gradient = dot(axis, start);

  // Project it into the world space
  vec3 cam = unproject(screenPoint, focalLength * viewSize.x, centre, projection);

  // Rotate the axis vector towards the screen point by the angle to gradient
  // This gives the closest point on the curve
  vec3 nearestPoint = rotateByAxisAngle(axis, normalize(cross(axis, cam)), acos(gradient));

  // Work out if we are in range
  float range = angleAround(axis, start, end);
  float value = angleAround(axis, start, nearestPoint);

  // start == end means do the whole circle
  range = all(equal(start, end)) ? 2.0 * M_PI : range;

  // If we are past the start or end, snap to start/end
  nearestPoint = value > range && value - range > (M_PI * 2.0 - range) * 0.5 ? start : nearestPoint;
  nearestPoint = value > range && value - range < (M_PI * 2.0 - range) * 0.5 ? end : nearestPoint;

  // When we project this back onto the image we get the nearest pixel
  vec2 nearestPixel = project(nearestPoint, focalLength * viewSize.x, centre, projection);

  // We get the distance from us to the nearest pixel and smoothstep to make a line
  float pixelDistance = length(screenPoint - nearestPixel);
  float alpha = smoothstep(0.0, lineWidth * 0.5, lineWidth * 0.5 - pixelDistance);

  gl_FragColor = vec4(colour.rgb, colour.a * alpha);
}
