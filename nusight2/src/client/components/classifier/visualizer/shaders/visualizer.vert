const float T_UNCLASSIFIED = 117.0;
const float T_WHITE        = 119.0;  // line
const float T_GREEN        = 103.0;  // field
const float T_YELLOW       = 121.0;  // goal
const float T_ORANGE       = 111.0;  // ball
const float T_CYAN         = 99.0;
const float T_MAGENTA      = 109.0;

const int COLOUR_SPACE_YCBCR = 1;
const int COLOUR_SPACE_RGB   = 2;

float round(float value) {
    return floor(value + 0.5);
}

/**
 * Get the lookup table index given an RGBA colour
 * @param {vec4} The RGBA colour
 * @return The lookup table index;
 */
float getLutIndex(vec3 colour, float bitsX, float bitsY, float bitsZ) {
    float bitsRemovedX = 8.0 - bitsX;
    float bitsRemovedY = 8.0 - bitsY;
    float bitsRemovedZ = 8.0 - bitsZ;

    float index = 0.0;
    // bitwise operators not available in GLSL
    // shift x left by N is equivalent to x = x * 2^N
    // shift x right by N is equivalent to x = floor(x / 2^N)
    // also normalizes to from 0 - 1 to 0 - 255 range
    index = index + floor(colour.x / exp2(bitsRemovedX));
    index = index * exp2(bitsY);
    index = index + floor(colour.y / exp2(bitsRemovedY));
    index = index * exp2(bitsZ);
    index = index + floor(colour.z / exp2(bitsRemovedZ));

    return round(index);
}

float getLutIndex(vec4 colour, float bitsR, float bitsG, float bitsB) {
    return getLutIndex(colour.xyz * 255.0, bitsR, bitsG, bitsG);
}
/**
 * Convert a classification into a RGBA colour
 *
 * @param {float} classification The classification to convert, ranging from 0-255
 * @return {vec4} The RGBA colour
 */
vec4 getColour(float classification) {
    vec4 colour = vec4(0, 0, 0, 1);
    if (classification == T_UNCLASSIFIED) {
        colour = vec4(0, 0, 0, 1);
    }
    else if (classification == T_WHITE) {
        colour = vec4(1, 1, 1, 1);
    }
    else if (classification == T_GREEN) {
        colour = vec4(0, 1, 0, 1);
    }
    else if (classification == T_YELLOW) {
        colour = vec4(1, 1, 0, 1);
    }
    else if (classification == T_ORANGE) {
        colour = vec4(1, 0.565, 0, 1);
    }
    else if (classification == T_CYAN) {
        colour = vec4(0, 1, 1, 1);
    }
    else if (classification == T_MAGENTA) {
        colour = vec4(1, 0, 1, 1);
    }
    return colour;
}

vec2 getCoordinate(float index, float size) {
    // Calculates the x and y coordinates of the 2D texture given the 1D index.
    // Adds 0.5 as we want the coordinates to go through the center of the pixel.
    // e.g. Go go through the center of pixel (0, 0) you need to sample at (0.5, 0.5).
    float x = mod(index, size) + 0.5;
    float y = floor(index / size) + 0.5;
    return vec2(x, y);
}

float classify(sampler2D lut, vec2 coordinate) {
    // Flip the y lookup using (1 - x) as the LUT has been flipped with UNPACK_FLIP_Y_WEBGL.
    // Texture has only one channel, so only one component (texel.r) is needed.
    // Normalize to 0 - 255 range.
    // Round result to remove any precision errors.
    coordinate.y = 1.0 - coordinate.y;
    return round(texture2D(lut, coordinate).r * 255.0);
}

/**
 * Classify a given colour with a given lookup table.
 *
 * @param {vec4} colour The RGB colour to classify.
 * @param {sampler2D} lut The square lookup table texture to be used for classification.
 * @param {float} size The size of the square lookup table texture.
 * @return {float} The classification of the given colour, ranging between 0-255.
 */
float classify(vec3 colour, sampler2D lut, float size, float bitsX, float bitsY, float bitsZ) {
    // Find the appropriate 1D lookup index given a colour
    float index = getLutIndex(colour, bitsX, bitsY, bitsZ);
    // Get the texture coordinate given the 1D lut index
    vec2 coordinate = getCoordinate(index, size) / size;
    // Get classification colour with the coordinate
    return classify(lut, coordinate);
}

float classify(vec4 colour, sampler2D lut, float size, float bitsX, float bitsY, float bitsZ) {
    return classify(colour.xyz * 255.0, lut, size, bitsX, bitsY, bitsZ);
}

uniform sampler2D lut;
uniform float lutSize;
uniform float bitsX;
uniform float bitsY;
uniform float bitsZ;

uniform float scale;
uniform float size;

varying vec4 colour;

void main() {
    // Colours are given via their position attribute
    // Scale colour to the range: [0,1]
    vec4 rawColour =
        vec4(position.x / (exp2(bitsX) - 1.0), position.y / (exp2(bitsY) - 1.0), position.z / (exp2(bitsZ) - 1.0), 1.0);

    // Classify the colour
    float classification = classify(rawColour, lut, lutSize, bitsX, bitsY, bitsZ);

    if (classification == T_UNCLASSIFIED) {
        // The colour is unclassified, no need to display it
        // Put the point behind the camera to discard it
        gl_Position = vec4(0, 0, 2, 1);
    }
    else {
        colour = getColour(classification);
        // Scale the position to the range [-1,1] as that is the scale of the plot
        vec3 positionScaled = 2.0 * rawColour.zxy - 1.0;
        // Transform to eye-space
        vec4 mvPosition = modelViewMatrix * vec4(positionScaled, 1.0);
        // Scale the point size based on distance from the camera (aka. size attenuation)
        gl_PointSize = size * (scale / length(mvPosition.xyz));
        // Transform to clip space and set as position
        gl_Position = projectionMatrix * mvPosition;
    }
}
