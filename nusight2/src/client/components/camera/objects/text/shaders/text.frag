precision highp float;
precision lowp int;

uniform sampler2D texture;

uniform vec4 backgroundColor;
uniform vec4 textColor;

varying vec2 vUv;

void main() {

    // Lerp from background color to text color, with the mask alpha as input
    vec4 textureColor = texture2D(texture, vUv);
    vec4 color        = mix(backgroundColor, textColor, textureColor.a);

    gl_FragColor = color;
}
