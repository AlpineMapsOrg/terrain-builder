#version 460 core

layout (location = 0) in vec3 pos;
layout (location = 1) in vec2 uv;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

out VS_OUT {
    vec4 world_pos;
    vec4 view_space_pos;
    vec4 view_space_light_dir;
    vec2 uvs;
} vs_out;

void main()
{
    vs_out.uvs = uv;
    
    vs_out.world_pos = model * vec4(pos, 1.0);

    vs_out.view_space_light_dir = view * vec4(0, 0, 1, 1);

    vs_out.view_space_pos = view * vs_out.world_pos;

    gl_Position = projection * vs_out.view_space_pos;
}
