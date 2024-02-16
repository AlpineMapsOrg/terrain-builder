#ifndef IO_H
#define IO_H

#include <filesystem>
#include <optional>
#include <vector>
#include <span>

#include <cgltf_write.h>
#include <fmt/core.h>
#include <glm/glm.hpp>
#include <tl/expected.hpp>

#include "terrain_mesh.h"
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

    static tl::expected<RawGltfMesh, cgltf_result> load_from_path(const std::filesystem::path &path) {
        cgltf_options options = {};
        cgltf_data *data = NULL;
        cgltf_result result = cgltf_parse_file(&options, path.string().c_str(), &data);
        if (result != cgltf_result::cgltf_result_success) {
            return tl::unexpected(result);
        }
        return RawGltfMesh(data);
    }

private:
    RawGltfMesh(cgltf_data *data)
        : data(data) {}
};

TerrainMesh load_mesh_from_raw(const RawGltfMesh &raw);

enum class LoadMeshErrorKind {
    FileNotFound,
    InvalidFormat,
    OutOfMemory
};

class LoadMeshError {
public:
    LoadMeshError() = default;
    constexpr LoadMeshError(LoadMeshErrorKind kind)
        : kind(kind) {}

    operator LoadMeshErrorKind() const {
        return this->kind;
    }
    constexpr bool operator==(LoadMeshError other) const {
        return this->kind == other.kind;
    }
    constexpr bool operator!=(LoadMeshError other) const {
        return this->kind != other.kind;
    }

    std::string description() const {
        switch (kind) {
        case LoadMeshErrorKind::FileNotFound:
            return "file not found";
        case LoadMeshErrorKind::InvalidFormat:
            return "invalid file format";
        case LoadMeshErrorKind::OutOfMemory:
            return "out of memory";
        default:
            return "undefined error";
        }
    }

private:
    LoadMeshErrorKind kind;
};

tl::expected<TerrainMesh, LoadMeshError> load_mesh_from_path(const std::filesystem::path &path);

enum class SaveMeshErrorKind { };

class SaveMeshError {
public:
    SaveMeshError() = default;
    constexpr SaveMeshError(SaveMeshErrorKind kind)
        : kind(kind) {}

    operator SaveMeshErrorKind() const {
        return this->kind;
    }
    constexpr bool operator==(SaveMeshError other) const {
        return this->kind == other.kind;
    }
    constexpr bool operator!=(SaveMeshError other) const {
        return this->kind != other.kind;
    }

    std::string description() const {
        switch (kind) {
        default:
            return "undefined error";
        }
    }

private:
    SaveMeshErrorKind kind;
};

// TODO: return errors
tl::expected<void, SaveMeshError> save_mesh_to_path(const std::filesystem::path &path, TerrainMesh &mesh);

}

#endif
