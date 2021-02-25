varying vec4 colour;

#define M_PI 3.1415926535897932384626433832795

void main() {
    // Calculate the distance the current fragment is from the center of the point
    float distFromCenter = abs(distance(vec2(0.5, 0.5), gl_PointCoord));

    if (distFromCenter > 0.5) {
        // Discard any fragments that are outside the circle
        discard;
    }

    // Creates a nice smooth circular shading on the point
    gl_FragColor.rgb =
        colour.rgb * (sin(M_PI * gl_PointCoord.y) * 0.5 + 0.5) * (sin(M_PI * gl_PointCoord.x) * 0.5 + 0.5);
    // Smooth the alpha to 0 near the border of the circle to provide some anti-aliasing
    gl_FragColor.a = 1.0;
}
