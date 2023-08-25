
vec2 getJoinVertex(vec2 joinCentre, vec2 position, float radius) {

    // Current xy zoom level. Used to keep the join size contant while zooming in/out
    vec2 zoom = vec2(length(modelViewMatrix[0].xyz), length(modelViewMatrix[1].xyz));

    // Get the vertex position .
    return joinCentre + 2.0 * radius * (position.xy / viewSize) * (1.0 / zoom);
}
