#pragma once

#include <filesystem>
#include <optional>
#include <string_view>
#include <vector>

#include "Codec.h"
#include "error/Result.h"
#include "io/serialize.h"
#include "io/Error.h"

namespace octree {

template <typename T>
struct ZppBitsCodec {
    using value_type = T;
    using LoadError = io::Error;
    using SaveError = io::Error;

    [[nodiscard]]
    static alp::Result<value_type, LoadError> load_from_path(const std::filesystem::path& path) {
        return io::read_from_path<value_type>(path);
    }

    [[nodiscard]]
    static alp::Result<void, SaveError> save_to_path(const value_type& value, const std::filesystem::path& path) {
        return io::write_to_path(value, path);
    }

    static std::vector<std::string_view> supported_extensions() {
        return {".bin"};
    }
};

} // namespace octree
