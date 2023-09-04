precision highp float;

uniform mat4 projectionMatrix;
uniform mat4 modelViewMatrix;
uniform vec2 viewSize;
uniform vec2 imageSize;
uniform float imageAspectRatio;

// Current vertex and UV coord of the line segment geometry
attribute vec2 position;
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

vec2 getSegmentVertex(vec2 start, vec2 end, vec2 position, float width) {

    // Current xy zoom level extracted from the scale part of the mv matrix
    vec2 zoom = vec2(length(modelViewMatrix[0].xyz), length(modelViewMatrix[1].xyz));

    // The GL direction and length of the current line segment.
    vec2 xBasis = end - start;

    // The orthogonal direction of the line segment to move in to create the width of the line.
    vec2 yBasis = normalize(vec2(-xBasis.y / viewSize.x, xBasis.x / viewSize.y)) / viewSize;

    // Position of the line segment vertex
    return start + (xBasis * position.x) + (yBasis * 2.0 * width * position.y * (1.0 / zoom));
}

void main() {

    // Aspect ratio of GL view
    float viewRatio = viewSize.x / viewSize.y;
    vec2 topLeft    = vec2(-min(imageAspectRatio / viewRatio, 1.0), min(viewRatio / imageAspectRatio, 1.0));

    vec2 lineStart = topLeft + (startPoint.xy / imageSize) * topLeft * -2.0;
    vec2 lineEnd   = topLeft + (endPoint.xy / imageSize) * topLeft * -2.0;

    vec2 point = getSegmentVertex(lineStart, lineEnd, position, width);

    vColorA = startColor;
    vColorB = endColor;
    vUv     = uv;

    gl_Position = projectionMatrix * modelViewMatrix * vec4(point, 0.0, 1.0);
}
