#version 460 core

layout (location = 0) in vec3 pos;

uniform mat4 view;
uniform mat4 projection;
uniform mat4 model;
uniform float opacity;

out VS_OUT {
    vec4 world_pos;
    vec4 color;
} vs_out;

void main()
{
    vs_out.world_pos = model * vec4(pos, 1.0);
    vs_out.color = vec4(1.0f, 1.0f, 1.0f, opacity);

    vec4 view_space = view * vs_out.world_pos;

    gl_Position = projection * view_space;
}
