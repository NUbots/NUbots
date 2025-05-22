precision highp float;
precision lowp int;

#include "../../shaders/math_constants.glsl"

uniform mat4 projectionMatrix;
uniform mat4 modelViewMatrix;
uniform vec2 viewSize;
uniform mat4 Hcw;
uniform float focalLength;
uniform vec2 centre;
uniform vec2 k;
uniform int projection;
uniform vec2 imageSize;
uniform float imageAspectRatio;
uniform vec3 ray;
uniform vec2 pixel;
uniform float textHeight;

// 0: The text is positioned by a ray.
// 1: The text is positioned by an image pixel.
uniform int positionMode;

attribute vec2 uv;
attribute vec3 position;

varying vec2 vUv;

#include "../../shaders/projection.glsl"

/**
 * Calculates the anchor point of the text geometry.
 * This is a constant value for a single draw, since all values used are uniforms.
 */
vec2 getTextAnchorPoint() {
    float viewRatio            = viewSize.x / viewSize.y;
    vec2 zoom                  = vec2(length(modelViewMatrix[0].xyz), length(modelViewMatrix[1].xyz));
    vec2 projectionScaleFactor = vec2(-min(imageAspectRatio / viewRatio, 1.0), min(imageAspectRatio, viewRatio));

    if (positionMode == 0) {
        vec2 pixelOffset = ((pixel / viewSize) * 2.0) * (1.0 / zoom);
        return project(ray, focalLength, centre, k, projection) * projectionScaleFactor * 2.0 + pixelOffset;
    }
    else if (positionMode == 1) {
        vec2 topLeft = vec2(-min(imageAspectRatio / viewRatio, 1.0), min(viewRatio / imageAspectRatio, 1.0));
        return topLeft + (pixel.xy / imageSize) * topLeft * -2.0;
    }
}

void main() {
    vec2 textAnchor = getTextAnchorPoint();

    // Offset of this vertex of the geometry from the anchor point
    vec2 vertexOffset = (2.0 * position.xy * textHeight) / viewSize;

    // Current xy zoom level from the scale part of the mv matrix
    vec2 zoom = vec2(length(modelViewMatrix[0].xyz), length(modelViewMatrix[1].xyz));

    // Combine coordinates, stating independent of zoom level
    vec2 point = textAnchor + (vertexOffset / zoom);

    vUv         = uv;
    gl_Position = projectionMatrix * modelViewMatrix * vec4(point, 0.0, 1.0);
}
