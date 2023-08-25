precision highp float;

uniform mat4 projectionMatrix;
uniform mat4 modelViewMatrix;
uniform vec2 viewSize;
uniform vec2 imageSize;
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

#include "segment.glsl"

void main() {

    // Aspect ratio of GL view
    float viewRatio = viewSize.x / viewSize.y;
    vec2 topLeft    = vec2(-min(imageAspectRatio / viewRatio, 1.0), min(viewRatio / imageAspectRatio, 1.0));

    vec2 lineStart = topLeft + (startPoint.xy / imageSize) * topLeft * -2.0;
    vec2 lineEnd   = topLeft + (endPoint.xy / imageSize) * topLeft * -2.0;

    vec2 point = getSegmentVertex(lineStart, lineEnd, position.xy, width);

    vColorA = startColor;
    vColorB = endColor;
    vUv     = uv;

    gl_Position = projectionMatrix * modelViewMatrix * vec4(point, 0.0, 1.0);
}
