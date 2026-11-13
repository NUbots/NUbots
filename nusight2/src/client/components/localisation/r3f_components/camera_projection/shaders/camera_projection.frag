precision highp float;

#include "../../../../camera/objects/shaders/math_constants.glsl"
#include "../../../../camera/objects/shaders/projection.glsl"

// The camera image
uniform sampler2D image;
// Image width / height
uniform float imageAspectRatio;

// Field to camera transform
uniform mat4 Hcf;

// Lens/projection parameters
uniform float focalLength;
uniform int projection;
uniform vec2 centre;
uniform vec2 k;

// Points further than this from the camera are not drawn (metres)
uniform float maxDistance;
uniform float opacity;

varying vec3 vrPFf;

void main() {
    // Vector from the camera to this point on the field, in camera space (x forward, y left, z up)
    vec3 rPCc      = (Hcf * vec4(vrPFf, 1.0)).xyz;
    float distance = length(rPCc);

    if (distance > maxDistance) {
        discard;
    }

    // A rectilinear lens cannot see anything behind the image plane
    if (projection == RECTILINEAR_PROJECTION && rPCc.x <= 0.0) {
        discard;
    }

    // Project into the image. The result is measured as a fraction of the image width, from the centre of the image
    // with x to the left and y up.
    vec2 p = project(rPCc / distance, focalLength, centre, k, projection);

    // Convert to texture coordinates, with the origin at the top left of the image, x to the right and y down
    vec2 uv = vec2(0.5 - p.x, 0.5 - p.y * imageAspectRatio);

    if (any(lessThan(uv, vec2(0.0))) || any(greaterThan(uv, vec2(1.0)))) {
        discard;
    }

    gl_FragColor = vec4(texture2D(image, uv).rgb, opacity);
}
