precision highp float;

#include "math_constants.glsl"
#include "projection.glsl"

attribute vec3 position;
attribute vec2 uv;

// Lens/projection parameters
uniform vec2 viewSize;
uniform float focalLength;
uniform int projection;
uniform vec2 centre;
uniform vec2 k;

uniform float imageAspectRatio;

// Line parameters
uniform vec3 axis;
uniform vec3 start;
uniform vec3 end;
uniform float lineWidth;

uniform mat4 projectionMatrix;
uniform mat4 modelViewMatrix;

varying vec2 vUv;

/**
 * Rotate a vector v about the axis e by the angle theta
 *
 * @param v       the vector to rotate
 * @param axis    the axis to rotate around
 * @param theta   the angle to rotate by
 *
 * @return the input vector rotated around e by theta
 */
vec3 rotateByAxisAngle(vec3 v, vec3 axis, float theta) {
    return cos(theta) * v + sin(theta) * cross(axis, v) + (1.0 - cos(theta)) * (dot(axis, v)) * axis;
}

/**
 * Measures the angle that start must be rotated around axis to reach end within the 2d plane defined by axis
 *
 * @param axis    the axis that defines the plane to measure the angle in
 * @param start   the vector to measure the angle from
 * @param end     the vector to measure the angle to
 *
 * @return the angle between the two vectors from start to end
 */
float angleAround(vec3 axis, vec3 start, vec3 end) {

    // Put start and end in the plane of axis
    vec3 aStart = normalize(cross(axis, start));
    vec3 aEnd   = normalize(cross(axis, end));

    float x = dot(aStart, aEnd);               // cos(theta)
    float y = dot(axis, cross(aStart, aEnd));  // sin(theta)

    // sin(theta)/cos(theta) = tan(theta)
    float theta = atan(y, x);

    // We want 0 -> 2pi not -pi -> pi
    theta += theta <= 0.0 ? (M_PI * 2.0) : 0.0;
    theta = all(equal(start, end)) ? 2.0 * M_PI : theta;

    return theta;
}

#define dTheta 1e-3 * M_PI / 180.0

void main() {
    vUv = uv;

    // Aspect ratio of GL view
    float viewRatio = viewSize.x / viewSize.y;

    // In the case where the GL view's aspect ratio is wider than the image's, the line must
    // be shrunk to align with the image.
    float scale = (viewRatio > imageAspectRatio) ? (imageAspectRatio / viewRatio) : 1.0;

    // Work out how far along in the curve we want to draw this vertex
    float totalAngle = angleAround(axis, start, end);
    float angle      = totalAngle * uv.x;
    vec3 lineCentre  = rotateByAxisAngle(start, axis, angle);

    // Calculate the tangential direction at this point in the line (the "line width" direction)
    vec3 tangentAxis = normalize(cross(axis, lineCentre));

    // Project our centre point, as well as a point that is very close in the tangential direction
    // This lets us work out a locally linear approximation for line width in pixels to angle
    vec3 testEdge      = rotateByAxisAngle(lineCentre, tangentAxis, dTheta);
    vec2 centrePixel   = project(lineCentre, focalLength, centre, k, projection);
    vec2 testEdgePixel = project(testEdge, focalLength, centre, k, projection);
    float angleScale   = lineWidth / (scale * (viewSize.x * length(centrePixel - testEdgePixel)));

    // Rotate around by our calculated scale to get the correct pixel width line and project
    vec3 lineEdge = rotateByAxisAngle(lineCentre, tangentAxis, (uv.y - 0.5) * angleScale * dTheta);
    vec2 point    = project(lineEdge, focalLength, centre, k, projection);

    // Convert the point from screen space from projection to GLSL coordinates for our view
    vec2 aspect = vec2(-2.0, 2.0 * viewSize.x / viewSize.y);
    gl_Position = projectionMatrix * modelViewMatrix * vec4(point * aspect * scale, 0.0, 1.0);
}
