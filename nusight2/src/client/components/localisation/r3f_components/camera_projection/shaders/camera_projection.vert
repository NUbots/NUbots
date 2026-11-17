precision highp float;

attribute vec3 position;

uniform mat4 projectionMatrix;
uniform mat4 modelViewMatrix;

// Position of this fragment on the field plane, in field space
varying vec3 vrPFf;

void main() {
    // The geometry is defined directly in field space, on the ground plane
    vrPFf       = vec3(position.xy, 0.0);
    gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
}
