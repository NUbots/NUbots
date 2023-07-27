

vec2 getSegmentVertex(vec2 start, vec2 end, vec2 position, float width) {

    // Current xy zoom level extracted from the scale part of the mv matrix
    vec2 zoom = vec2(length(modelViewMatrix[0].xyz), length(modelViewMatrix[1].xyz));

    // The GL direction and length of the current line segment.
    vec2 xBasis = end - start;

    // The orthogonal direction of the line segment to move in to create the width of the line.
    vec2 yBasis = normalize(vec2(-xBasis.y / viewSize.x, xBasis.x / viewSize.y)) / viewSize;

    // Position of the line segment vertex
    return start + (xBasis * position.x) + (yBasis * 2.0 * width * position.y * (1.0 / zoom));
}
