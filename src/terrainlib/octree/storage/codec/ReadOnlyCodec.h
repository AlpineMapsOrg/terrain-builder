#pragma once

#include <filesystem>

#include <libassert/assert.hpp>

#include "error/Result.h"

namespace octree {

// Wraps a codec so saving always fails loudly instead of writing, for storages that must stay
// read-only (e.g. a view that aliases another storage's files).
template <typename Codec>
struct ReadOnlyCodec : Codec {
    [[nodiscard]]
    static alp::Result<void, typename Codec::SaveError> save_to_path(
        const typename Codec::value_type &, const std::filesystem::path &) noexcept {
        PANIC("Attempted to write through a read-only codec");
    }
};

} // namespace octree
