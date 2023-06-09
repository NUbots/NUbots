precision highp float;

uniform vec4 color;
varying vec2 vUv;

void main() {
    // Anti-alias the line by smooth stepping the edges in the y direction
    float value = 1.0 - abs(vUv.y * 2.0 - 1.0);
    float alpha = smoothstep(0.0, 1.0, value * 3.0);

    gl_FragColor = vec4(color.rgb, color.a * alpha);
}
