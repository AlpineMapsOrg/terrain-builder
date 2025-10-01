#version 460 core

uniform float render_mode;
uniform sampler2D uTexture;
uniform bool has_texture;

in VS_OUT {
    vec4 world_pos;
    vec4 view_space_pos;
    vec4 view_space_light_dir;
    vec2 uvs;
} fs_in;

out vec4 FragColor;

vec4 phong(vec3 L, vec3 H, vec3 N, vec4 diff_color, vec4 spec_color, float ambient, float shininess) {

    float diff = clamp(dot(N, L), 0.0f, 1.0f);
    float spec = pow(clamp(dot(H, N), 0.0f, 1.0f), shininess);

    return diff * diff_color + spec * spec_color + ambient;
}

void main() {
    vec4 c = vec4(0);

    vec3 dpdx = dFdx(fs_in.view_space_pos.xyz);
    vec3 dpdy = dFdy(fs_in.view_space_pos.xyz);
    vec3 N = normalize(cross(dpdx, dpdy));

    vec3 dpdx_world = dFdx(fs_in.world_pos.xyz);
    vec3 dpdy_world = dFdy(fs_in.world_pos.xyz);
    vec3 N_world = normalize(cross(dpdx_world, dpdy_world));

    if (render_mode == 0) { 
        // WIREFRAME
        c = vec4(1.0f, 0.0f, 0.0f, 1.0f);
    } else if (render_mode == 1 || render_mode == 2) {
        // TEXTURED
        float shininess = 2.0f;

        vec3 L = normalize(fs_in.view_space_light_dir.xyz);
        vec3 H = normalize(L + normalize(-fs_in.view_space_pos.xyz));

        uvec2 checkerboard = uvec2(lessThan(mod(fs_in.uvs * 10.0f, 2.0f), vec2(1.0f)));
        bool checkerboard_reduced = (checkerboard.x ^ checkerboard.y) == 1u;

        vec4 error_texture = vec4(0.0f, 0.0f, 0.0f, 1.0f);

        if (checkerboard_reduced) {
            error_texture = vec4(1.0f, 0.0f, 1.0f, 1.0f);
        }

        vec4 diff_color = mix(error_texture, texture(uTexture, fs_in.uvs), has_texture);

        if (render_mode == 1) {
            // Shaded
            c = phong(L, H, N, diff_color, vec4(0.1f), 0.05f, 2.0f);
        } else {
            // Unshaded
            c = diff_color;
        }
        //TODO: Handle alpha better, now its just set manually to 1.0f
        c = vec4(c.rgb, 1.0f);
    } else if (render_mode == 3) {
        // CLAY

        float shininess = 2.0f;

        vec3 L = normalize(fs_in.view_space_light_dir.xyz);
        vec3 H = normalize(L + normalize(-fs_in.view_space_pos.xyz));

        c = phong(L, H, N, vec4(0.4f), vec4(0.4f), 0.1f, 2.0f);
        
        //TODO: Handle alpha better, now its just set manually to 1.0f
        c = vec4(c.rgb, 1.0f);
    } else if (render_mode == 4) {
        // FLAT NORMALS
        c = vec4(N_world, 1.0f);
    } else if (render_mode == 5) {
        // FLAT NORMALS
        c = vec4(fs_in.uvs, 0.0f, 1.0f);
    }

    FragColor = c;
}
