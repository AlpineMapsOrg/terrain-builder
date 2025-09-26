#version 460 core

layout (location = 0) in vec3 pos;
layout (location = 1) in mat4 model;                //loc 1: column 0
                                                    //loc 2: column 1
                                                    //loc 3: column 2
                                                    //loc 4: column 3

uniform mat4 view;
uniform mat4 projection;

out VS_OUT {
    vec4 world_pos;
    vec4 color;
} vs_out;

void main()
{
    vs_out.world_pos = model * vec4(pos, 1.0);
    vs_out.color = vec4(1.0f, 1.0f, 1.0f, 1.0f);

    vec4 view_space = view * vs_out.world_pos;

    gl_Position = projection * view_space;
}
