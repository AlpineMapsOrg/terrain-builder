/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Adam Celarek-Litofcenko
 * Copyright (C) 2026 Martin Braunsperger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *****************************************************************************/

#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>

#include "error/Error.h"
#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "io/FileError.h"
#include "io/envelope.h"
#include "io/filesystem.h"
#include "io/serde.h"
#include "octree/storage/index_format.h"
#include "store/IndexFormat.h"
#include "store/fingerprint.h"
#include "store/inputs.h"

namespace sf_merger::inputs {

// Everything that determines the merged meshes; a checkpoint must match it.
struct Merge {
    // Optional members defeat zpp_bits' member counting.
    using serialize = zpp::bits::members<4>;

    store::Fingerprint base;
    store::Fingerprint added;
    std::optional<std::string> mask;
    // Bump on changes that alter the merged meshes.
    std::uint32_t processing_version = 1;

    bool operator==(const Merge&) const = default;
};

using Schema = io::envelope::PayloadSchema<"sf_merger.Inputs", io::envelope::Version<2, Merge>>;

using MakeMergeError = alp::Error<store::FingerprintError, io::serde::SerializeError, store::ReadIndexError, io::FileError>;
// The inputs that merging the SF storages at `base` and `added` with `mask` depends on.
[[nodiscard]]
inline alp::Result<Merge, MakeMergeError> make_merge(const std::filesystem::path& base, const std::filesystem::path& added, const std::optional<std::filesystem::path>& mask) {
    Merge merge;
    for (auto [path, fingerprint] : { std::pair { &base, &merge.base }, std::pair { &added, &merge.added } }) {
        const auto context = alp::make_context("fingerprinting the input of the SF merge at {}", *path);
        ALP_TRY_CONTEXT(*fingerprint, store::fingerprint(*path, octree::storage::index_format()), context);
    }

    if (mask.has_value()) {
        ALP_TRY(const std::filesystem::path absolute_mask, io::filesystem::absolute(*mask));
        merge.mask = absolute_mask.lexically_normal().string();
    }

    return merge;
}

} // namespace sf_merger::inputs
