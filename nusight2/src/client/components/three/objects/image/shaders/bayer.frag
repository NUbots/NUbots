precision highp float;

// Source: Efficient, High-Quality Bayer Demosaic Filtering on GPUs
// Morgan McGuire, Williams College
// Uses the Malvar-He-Cutler technique

uniform sampler2D image;
uniform vec4 sourceSize;
uniform int mosaicSize;  // 1 (non-permutated bayer), 2 (perumutated bayer), or 4 (permutated polarised bayer)

varying vec4 center;
varying vec4 xCoord;
varying vec4 yCoord;

ivec2 mod(ivec2 a, int b) {
    return ivec2(a.x - (int(a.x / b) * b), a.y - (int(a.y / b) * b));
}

float fetch(vec2 src_point) {
    // Calculate the sub-image coordinates
    ivec2 src  = ivec2(src_point * sourceSize.xy);
    ivec2 dest = (src / ivec2(mosaicSize, mosaicSize))
                 + mod(src, mosaicSize) * ivec2(ivec2(sourceSize.xy) / ivec2(mosaicSize, mosaicSize));
    vec2 dest_point = (vec2(dest.x, dest.y) + 0.5) / sourceSize.xy;

    return texture2D(image, dest_point).r;
}

void main(void) {
    float C       = fetch(center.xy);  // ( 0, 0)
    const vec4 kC = vec4(4.0, 6.0, 5.0, 5.0) / 8.0;

    // Determine which of four types of pixels we are on.
    vec2 alternate = mod(floor(center.zw), 2.0);

    vec4 Dvec = vec4(fetch(vec2(xCoord.y, yCoord.y)),  // (-1,-1)
                     fetch(vec2(xCoord.y, yCoord.z)),  // (-1, 1)
                     fetch(vec2(xCoord.z, yCoord.y)),  // ( 1,-1)
                     fetch(vec2(xCoord.z, yCoord.z))   // ( 1, 1)
    );

    vec4 PATTERN = (kC.xyz * C).xyzz;

    // Can also be a dot product with (1,1,1,1) on hardware where that is
    // specially optimized.
    // Equivalent to: D = Dvec.x + Dvec.y + Dvec.z + Dvec.w;
    Dvec.xy += Dvec.zw;
    Dvec.x += Dvec.y;

    vec4 value = vec4(fetch(vec2(center.x, yCoord.x)),  // ( 0,-2)
                      fetch(vec2(center.x, yCoord.y)),  // ( 0,-1)
                      fetch(vec2(xCoord.x, center.y)),  // ( 1, 0)
                      fetch(vec2(xCoord.y, center.y))   // ( 2, 0)
    );

    vec4 temp = vec4(fetch(vec2(center.x, yCoord.w)),  // ( 0, 2)
                     fetch(vec2(center.x, yCoord.z)),  // ( 0, 1)
                     fetch(vec2(xCoord.w, center.y)),  // ( 2, 0)
                     fetch(vec2(xCoord.z, center.y))   // ( 1, 0)
    );

    // Even the simplest compilers should be able to constant-fold these to avoid the division.
    // Note that on scalar processors these constants force computation of some identical products twice.
    const vec4 kA = vec4(-1.0, -1.5, 0.5, -1.0) / 8.0;
    const vec4 kB = vec4(2.0, 0.0, 0.0, 4.0) / 8.0;
    const vec4 kD = vec4(0.0, 2.0, -1.0, -1.0) / 8.0;

// Conserve constant registers and take advantage of free swizzle on load
#define kE (kA.xywz)
#define kF (kB.xywz)

    value += temp;

    // There are five filter patterns (identity, cross, checker,
    // theta, phi). Precompute the terms from all of them and then
    // use swizzles to assign to color channels.
    //
    // Channel Matches
    // x cross (e.g., EE G)
    // y checker (e.g., EE B)
    // z theta (e.g., EO R)
    // w phi (e.g., EO R)

#define A (value.x)
#define B (value.y)
#define D (Dvec.x)
#define E (value.z)
#define F (value.w)

    // Avoid zero elements. On a scalar processor this saves two MADDs and it has no
    // effect on a vector processor.
    PATTERN.yzw += (kD.yz * D).xyy;

    PATTERN += (kA.xyz * A).xyzx + (kE.xyw * E).xyxz;
    PATTERN.xw += kB.xw * B;
    PATTERN.xz += kF.xz * F;

    gl_FragColor.rgb = (alternate.y == 0.0)
                           ? ((alternate.x == 0.0) ? vec3(C, PATTERN.xy) : vec3(PATTERN.z, C, PATTERN.w))
                           : ((alternate.x == 0.0) ? vec3(PATTERN.w, C, PATTERN.z) : vec3(PATTERN.yx, C));

    // Set the alpha to 1.0, as chromium browsers are not rendering the image when the alpha is not set
    gl_FragColor.a = 1.0;
}
