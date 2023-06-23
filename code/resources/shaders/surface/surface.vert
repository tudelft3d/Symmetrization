#version 150

in  vec3 vtx_position;
in  vec3 vtx_normal;
in  vec3 vtx_color;
in  vec2 vtx_texcoord;

uniform vec4 default_color = vec4(0.4f, 0.8f, 0.8f, 1.0f);
uniform bool per_vertex_color = false;

uniform mat4 MVP;
uniform mat4 MANIP = mat4(1.0);
uniform mat3 NORMAL = mat3(1.0);

uniform bool planeClippingDiscard = false;
uniform bool clippingPlaneEnabled = false;
uniform bool crossSectionEnabled = false;
uniform vec4 clippingPlane0;
uniform vec4 clippingPlane1;


out Data{
    vec2 texcoord;
    vec4 color;
    vec3 position;
    vec3 normal;
    float clipped;
} DataOut;

void main() {
    vec4 new_position = MANIP * vec4(vtx_position, 1.0);

    DataOut.clipped = 0.0;
    if (clippingPlaneEnabled) {
        gl_ClipDistance[0] = dot(new_position, clippingPlane0);
        if (planeClippingDiscard && gl_ClipDistance[0] < 0)
            DataOut.clipped = 1.0;
        if (crossSectionEnabled) {
            gl_ClipDistance[1] = dot(new_position, clippingPlane1);
            if (planeClippingDiscard && gl_ClipDistance[1] < 0)
                DataOut.clipped = 1.0;
        }
    }

    if (per_vertex_color)
        DataOut.color = vec4(vtx_color, 1.0);
    else
        DataOut.color = default_color;

    DataOut.position = new_position.xyz;
    DataOut.normal = NORMAL * vtx_normal;
    DataOut.texcoord = vtx_texcoord;

    gl_Position = MVP * new_position;
}
