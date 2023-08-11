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

// Current vertex and UV coord of the line join geometry
attribute vec3 position;
attribute vec2 uv;

// The ray of the line join and color of the join
attribute vec3 lineJoin;
attribute vec4 color;
attribute float radius;

#include "../../shaders/projection.glsl"
#include "join.glsl"

varying vec2 vUv;
varying vec4 vColor;

void main() {

    // Aspect ratio of GL view
    float viewRatio            = viewSize.x / viewSize.y;
    vec2 aspectRatioCorrection = vec2(-min(imageAspectRatio / viewRatio, 1.0), min(imageAspectRatio, viewRatio));
    vec2 lineJoinCentre        = project(lineJoin, focalLength, centre, k, projection) * aspectRatioCorrection * 2.0;

    // Get the vertex position for the join
    vec2 point = getJoinVertex(lineJoinCentre, position.xy, radius);

    vUv    = uv;
    vColor = color;

    gl_Position = projectionMatrix * modelViewMatrix * vec4(point, 0.0, 1.0);
}
