precision highp float;

varying vec4 vColorA;
varying vec4 vColorB;
varying vec2 vUv;

void main() {
    gl_FragColor = mix(vColorA, vColorB, vUv.x);
}
