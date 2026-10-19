#pragma once

#include <filesystem>
#include <string>
#include <system_error>

#include <fmt/format.h>

#include "error/Error.h"
#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "io/FileError.h"
#include "io/filesystem.h"
#include "octree/Id.h"
#include "octree/disk/Layout.h"
#include "octree/storage/codec/Codec.h"
#include "octree/storage/defaults.h"

namespace octree {

template <typename T = DefaultT, CodecFor<T> Codec = DefaultCodecFor<T>>
class RawStorage_ {
public:
    using value_type = T;
    using CodecType = Codec;
    using LoadError = typename Codec::LoadError;
    using SaveError = typename Codec::SaveError;
    using CopyError = alp::Error<io::FileError, LoadError, SaveError>;

    explicit RawStorage_(disk::Layout layout) noexcept : _layout(std::move(layout)) {}

    ~RawStorage_() = default;
    RawStorage_ &operator=(const RawStorage_ &) = delete;
    RawStorage_(const RawStorage_ &) = delete;
    RawStorage_(RawStorage_ &&) = default;
    RawStorage_ &operator=(RawStorage_ &&) = default;

    [[nodiscard]]
    alp::Result<T, LoadError> load(const Id &id) const {
        const auto path = this->path_for(id);
        ALP_TRY_CONTEXT(T node, Codec::load_from_path(path), fmt::format("loading node {}", id));
        return node;
    }

    [[nodiscard]]
    alp::Result<void, SaveError> save(const Id &id, const T &node) const {
        const auto path = this->path_for(id);
        ALP_TRY_CONTEXT(Codec::save_to_path(node, path), fmt::format("saving node {}", id));
        return {};
    }

    [[nodiscard]]
    alp::Result<void, CopyError> copy_to(const Id &id, RawStorage_ &target) const {
        return target.copy_from(id, *this);
    }

    [[nodiscard]]
    alp::Result<void, CopyError> copy_from(const Id &id, const RawStorage_ &source) {
        const auto context = alp::make_context("copying node {}", id);
        const auto source_path = source.path_for(id);
        const auto target_path = this->path_for(id);
        // Checked first so that a missing source leaves the target untouched.
        if (!source.has(id)) {
            return alp::make_unexpected(io::make_file_not_found(source_path), context);
        }

        if (source_path.extension() != target_path.extension()) {
            // TODO: should this error instead?
            ALP_TRY_CONTEXT(const T node, Codec::load_from_path(source_path), context);
            ALP_TRY_CONTEXT(Codec::save_to_path(node, target_path), context);
            return {};
        }

        ALP_TRY_CONTEXT(io::filesystem::remove(target_path), context);
        ALP_TRY_CONTEXT(io::filesystem::create_parent_directories(target_path), context);
        ALP_TRY_CONTEXT(io::filesystem::create_hard_link(source_path, target_path), context);

        return {};
    }

    bool remove(const Id &id) const noexcept {
        const auto path = this->path_for(id);
        std::error_code ec;
        const auto removed = std::filesystem::remove(path, ec);
        return !ec && removed;
    }

    bool has(const Id &id) const noexcept {
        const auto path = this->path_for(id);
        std::error_code ec;
        const auto exists = std::filesystem::exists(path, ec);
        return !ec && exists;
    }

    std::filesystem::path path_for(const Id &id) const noexcept {
        return this->_layout.get_node_path(id);
    }

    std::filesystem::path base_path() const noexcept {
        return this->_layout.base_path();
    }

    const disk::Layout &layout() const noexcept {
        return this->_layout;
    }

private:
    disk::Layout _layout;
};

} // namespace octree
