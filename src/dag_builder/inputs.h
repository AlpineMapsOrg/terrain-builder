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

#include "Range.h"
#include "build.h"
#include "error/Error.h"
#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "io/envelope.h"
#include "io/serde.h"
#include "octree/storage/index_format.h"
#include "store/IndexFormat.h"
#include "store/fingerprint.h"
#include "store/inputs.h"

namespace dag::inputs {

// Everything that determines the built DAG; a checkpoint must match it.
struct Build {
    // Optional members defeat zpp_bits' member counting.
    using serialize = zpp::bits::members<15>;

    store::Fingerprint input;
    std::uint32_t root_level = 0;
    std::uint64_t root_index = 0;
    std::optional<std::uint32_t> first_level;
    std::optional<std::uint32_t> last_level;
    std::uint32_t clusters_per_partition = 0;
    std::optional<float> target_ratio;
    std::optional<float> relative_target_error;
    bool allow_texture_reuse = true;
    std::uint32_t charting = 0;
    // Absent for texel budgets relative to the source textures.
    std::optional<double> texels_per_triangle;
    std::uint32_t max_node_texels = 0;
    std::uint32_t texture_gutter = 0;
    std::uint32_t include_mode = 0;
    // Bump on changes that alter the built DAG.
    std::uint32_t processing_version = 1;

    bool operator==(const Build&) const = default;
};

using Schema = io::envelope::PayloadSchema<"dag_builder.Inputs", io::envelope::Version<2, Build>>;

using MakeBuildError = alp::Error<store::FingerprintError, io::serde::SerializeError, store::ReadIndexError>;
// The inputs that building the DAG of the mesh storage at `input` with `options` over `levels` depends on.
[[nodiscard]]
inline alp::Result<Build, MakeBuildError> make_build(const std::filesystem::path& input, const BuildOptions& options, const AnyRange<uint32_t>& levels) {
    const auto context = alp::make_context("fingerprinting the input of the DAG build at {}", input);
    ALP_TRY_CONTEXT(store::Fingerprint fingerprint, store::fingerprint(input, octree::storage::index_format()), context);
    Build build;
    build.input = std::move(fingerprint);

    build.root_level = options.root_node.level();
    build.root_index = options.root_node.index_on_level();

    const auto first = levels.start_bound();
    if (first.kind != BoundKind::Unbounded) {
        build.first_level = first.kind == BoundKind::Included ? first.value.value() : first.value.value() + 1;
    }
    const auto last = levels.end_bound();
    if (last.kind != BoundKind::Unbounded) {
        build.last_level = last.kind == BoundKind::Included ? last.value.value() : last.value.value() - 1;
    }

    build.clusters_per_partition = options.clusters_per_partition;
    build.target_ratio = options.target_ratio;
    build.relative_target_error = options.relative_target_error;
    build.include_mode = static_cast<std::uint32_t>(options.include_mode);

    build.allow_texture_reuse = options.texture_options.allow_texture_reuse;
    build.charting = static_cast<std::uint32_t>(options.texture_options.charting);
    if (const auto* constant = std::get_if<ConstantQuality>(&options.texture_options.sizing.mode)) {
        build.texels_per_triangle = constant->target_texels_per_triangle;
    }
    build.max_node_texels = options.texture_options.sizing.max_node_texels;
    build.texture_gutter = options.texture_options.atlas.padding;

    return build;
}

} // namespace dag::inputs
