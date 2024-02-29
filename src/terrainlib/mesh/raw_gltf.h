#ifndef RAW_GLTF_H
#define RAW_GLTF_H

#include <filesystem>

#include <cgltf_write.h>
#include <tl/expected.hpp>

#include "non_copyable.h"

namespace io {
    
class RawGltfMesh : public NonCopyable {
public:
    cgltf_data *data;

    ~RawGltfMesh() {
        if (this->data != nullptr) {
            cgltf_free(this->data);
        }
    }

    // Move constructor
    RawGltfMesh(RawGltfMesh &&other) {
        this->data = other.data;
        other.data = nullptr;
    }

    // Move assignment operator
    RawGltfMesh &operator=(RawGltfMesh &&other) {
        if (this != &other) {
            this->data = other.data;
            other.data = nullptr;
        }
        return *this;
    }

    static tl::expected<RawGltfMesh, cgltf_result> load_from_path(const std::filesystem::path &path);

private:
    RawGltfMesh(cgltf_data *data)
        : data(data) {}
};

}

#endif
