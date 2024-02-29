#include <cgltf_write.h>
#include <tl/expected.hpp>

#include "raw_gltf.h"

tl::expected<io::RawGltfMesh, cgltf_result> io::RawGltfMesh::load_from_path(const std::filesystem::path &path) {
    cgltf_options options = {};
    cgltf_data *data = NULL;
    const std::string path_str = path.string();
    const char *path_ptr = path_str.c_str();
    cgltf_result result = cgltf_parse_file(&options, path_ptr, &data);
    if (result != cgltf_result::cgltf_result_success) {
        return tl::unexpected(result);
    }
    result = cgltf_load_buffers(&options, data, path_ptr);
    if (result != cgltf_result::cgltf_result_success) {
        return tl::unexpected(result);
    }
    return io::RawGltfMesh(data);
}
