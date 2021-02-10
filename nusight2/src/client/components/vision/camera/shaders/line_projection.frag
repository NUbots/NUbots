precision lowp float;

#include "math.glsl"
#include "projection.glsl"

// Lens/projection parameters
uniform vec2 viewSize;
uniform float focalLength;
uniform int projection;
uniform vec2 centre;
uniform vec2 k;

// Line parameters
uniform vec3 axis;
uniform vec3 start;
uniform vec3 end;

// Style
uniform vec4 color;
uniform float lineWidth;

varying vec2 vUv;

/**
 * Rotate a vector v about the axis e by the angle theta
 *
 * @param v       the vector to rotate
 * @param e       the axis to rotate around
 * @param theta   the angle to rotate by
 *
 * @return the input vector rotated around e by theta
 */
vec3 rotateByAxisAngle(vec3 v, vec3 e, float theta) {
    return cos(theta) * v + sin(theta) * cross(e, v) + (1.0 - cos(theta)) * (dot(e, v)) * e;
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
    theta += theta < 0.0 ? (M_PI * 2.0) : 0.0;
    return theta;
}

void main() {

    // Get our position on the screen in normalised coordinates
    vec2 point = vec2(0.5 - vUv.x, vUv.y - 0.5) * vec2(1.0, viewSize.y / viewSize.x);

    // Get the gradient of the curve we are drawing
    float gradient = dot(axis, start);

    // Project it into the world space
    vec3 cam = unproject(point, focalLength, centre, k, projection);

    // Rotate the axis vector towards the screen point by the angle to gradient
    // This gives the closest point on the curve
    vec3 nearestRay = rotateByAxisAngle(axis, normalize(cross(axis, cam)), acos(gradient));

    // Work out if we are in range
    float range = angleAround(axis, start, end);
    float value = angleAround(axis, start, nearestRay);

    // start == end means do the whole circle
    range = all(equal(start, end)) ? 2.0 * M_PI : range;

    // If we are past the start or end, snap to start/end
    nearestRay = value > range && value - range > (M_PI * 2.0 - range) * 0.5 ? start : nearestRay;
    nearestRay = value > range && value - range < (M_PI * 2.0 - range) * 0.5 ? end : nearestRay;

    // When we project this back onto the image we get the nearest pixel
    vec2 closestPoint = project(nearestRay, focalLength, centre, k, projection);
    vec2 unPoint      = project(cam, focalLength, centre, k, projection);

    // We get the distance from us to the nearest pixel and smoothstep to make a line
    // For all the previous calculations we are using normalised pixel coordinates where the coordinate is divided by
    // the width of the image. This ensures that any calculation we do is independent of the resolution that we are
    // displaying the image at. However when we actually want to calculate a distance that is in pixels, we must
    // multiply the coordinates by our horizontal resolution. Once we have multiplied by this resolution we will get a
    // value in pixels relative to the resolution that we are displaying at.
    float pixelDistance = length(unPoint - closestPoint) * viewSize.x;
    float alpha         = smoothstep(0.0, lineWidth * 0.5, lineWidth * 0.5 - pixelDistance);

    gl_FragColor = vec4(color.rgb, color.a * alpha);
}
