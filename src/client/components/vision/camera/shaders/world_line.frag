precision lowp float;

#define M_PI 3.1415926535897932384626433832795

uniform vec2 viewSize;
uniform float focalLength;
uniform vec3 axis;
uniform vec3 start;
uniform vec3 end;
uniform vec4 colour;
uniform float lineWidth;

varying vec2 vUv;

// TODO(trent) these should be moved into a separate GLSL file once there is a decent #include system
vec3 unprojectEquidistant(vec2 point, float focalLength) {
  float r = length(point);
  vec3 s = vec3(
    cos(r / focalLength),
    sin(r / focalLength) * point.x / r,
    sin(r / focalLength) * point.y / r
  );
  return normalize(s);
}

vec2 projectEquidistant(vec3 ray, float focalLength) {
  float theta = acos(ray.x);
  float r = focalLength * theta;
  float sinTheta = sin(theta);
  return vec2(
    r * ray.y / sinTheta,
    r * ray.z / sinTheta
  );
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
  // TODO(trent) this can't handle different lens types, in a future PR fix this
  vec3 cam = unprojectEquidistant(screenPoint, focalLength * viewSize.x);

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
  vec2 nearestPixel = projectEquidistant(nearestPoint, focalLength * viewSize.x);

  // We get the distance from us to the nearest pixel and smoothstep to make a line
  float pixelDistance = length(screenPoint - nearestPixel);
  float alpha = smoothstep(0.0, lineWidth * 0.5, lineWidth * 0.5 - pixelDistance);

  gl_FragColor = vec4(colour.rgb, colour.a * alpha);
}

