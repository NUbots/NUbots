precision highp float;
precision lowp int;

uniform mat4 projectionMatrix;
uniform mat4 modelViewMatrix;
uniform vec2 viewSize;
uniform mat4 Hcw;
uniform float focalLength;
uniform vec2 centre;
uniform vec2 k;
uniform int projection;
uniform float imageAspectRatio;

// Current vertex and UV coord of the line segment geometry
attribute vec3 position;
attribute vec2 uv;

// Start/End ray of the current line segment, with the color of
// the segment.
attribute vec3 startPoint;
attribute vec3 endPoint;
attribute vec4 startColor;
attribute vec4 endColor;
attribute float width;

varying vec4 vColorA;
varying vec4 vColorB;
varying vec2 vUv;

#include "../../shaders/projection.glsl"
#include "segment.glsl"

void main() {

    // Aspect ratio of GL view
    float viewRatio            = viewSize.x / viewSize.y;
    vec2 aspectRatioCorrection = vec2(-min(imageAspectRatio / viewRatio, 1.0), min(imageAspectRatio, viewRatio));

    vec2 lineStart = project(startPoint, focalLength, centre, k, projection) * aspectRatioCorrection * 2.0;
    vec2 lineEnd   = project(endPoint, focalLength, centre, k, projection) * aspectRatioCorrection * 2.0;

    vec2 point = getSegmentVertex(lineStart, lineEnd, position.xy, width);

    vColorA = startColor;
    vColorB = endColor;
    vUv     = uv;

    gl_Position = projectionMatrix * modelViewMatrix * vec4(point, 0.0, 1.0);
}
