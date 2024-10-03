uniform vec3 color;
uniform float wallHeight;
uniform float solidBottomHeight;
uniform int orientation;
varying vec3 vPosition;
void main() {
    float heightFactor;
    if (orientation == 0) {
        heightFactor = vPosition.y / wallHeight + 0.5;
    }
    else {
        heightFactor = vPosition.z / wallHeight + 0.5;
    }

    float normalizedSolidHeight = solidBottomHeight / wallHeight;
    float opacity;

    if (heightFactor < normalizedSolidHeight) {
        opacity = 0.9;
    }
    else {
        opacity = smoothstep(1.0, normalizedSolidHeight, heightFactor) * 0.3;
    }

    gl_FragColor = vec4(color, opacity);
}
