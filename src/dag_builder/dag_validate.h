/*****************************************************************************
 * AlpineMaps.org
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

#include <cmath>
#include <source_location>
#include <utility>

#include <fmt/format.h>

#include "dag_node.h"
#include "enumerate.h"
#include "error/MessageCode.h"
#include "error/Result.h"
#include "error/unwrap.h"
#include "error/utils.h"
#include "metadata.h"
#include "validate.h"

namespace dag {

struct InconsistentMetadata : alp::MessageCode<"the metadata is inconsistent"> {};
struct InconsistentBatch : alp::MessageCode<"the batch is inconsistent"> {};

[[nodiscard]]
inline alp::Result<void, InconsistentMetadata> validate(const NodeMetadata &metadata) {
    if (metadata.triangle_counts.size() != metadata.group_assignment.size()) {
        return alp::make_error(InconsistentMetadata{}, fmt::format("the metadata holds {} triangle counts for {} group assignments", metadata.triangle_counts.size(), metadata.group_assignment.size()));
    }
    for (const auto [cluster_index, group_index] : enumerate(metadata.group_assignment)) {
        if (group_index >= metadata.groups.size()) {
            return alp::make_error(InconsistentMetadata{}, fmt::format("cluster {} names group {}, but there are {} groups", cluster_index, group_index, metadata.groups.size()));
        }
    }
    for (const auto [group_index, group] : enumerate(metadata.groups)) {
        if (!std::isfinite(group.error)) {
            return alp::make_error(InconsistentMetadata{}, fmt::format("group {} has a non-finite error", group_index));
        }
    }
    return {};
}

[[nodiscard]]
inline alp::Result<void, InconsistentBatch> validate(const NodeMetadata &metadata, const Clustering &clustering) {
    auto metadata_result = validate(metadata);
    if (!metadata_result.has_value()) {
        return alp::make_error(InconsistentBatch{}, std::move(metadata_result).error());
    }
    auto clustering_result = ::validate(clustering);
    if (!clustering_result.has_value()) {
        return alp::make_error(InconsistentBatch{}, std::move(clustering_result).error());
    }
    if (metadata.group_assignment.size() != clustering.clusters.size()) {
        return alp::make_error(InconsistentBatch{}, fmt::format("the metadata holds {} group assignments for {} clusters", metadata.group_assignment.size(), clustering.clusters.size()));
    }
    return {};
}

[[nodiscard]]
inline alp::Result<void, InconsistentBatch> validate(const ClusterBatch &batch) {
    return validate(batch.metadata, batch.clustering);
}

inline void assert_valid(const NodeMetadata &metadata, const std::source_location location = std::source_location::current()) {
    alp::unwrap(validate(metadata), location);
}

inline void assert_valid(const NodeMetadata &metadata, const Clustering &clustering, const std::source_location location = std::source_location::current()) {
    alp::unwrap(validate(metadata, clustering), location);
}

inline void assert_valid(const ClusterBatch &batch, const std::source_location location = std::source_location::current()) {
    alp::unwrap(validate(batch), location);
}

} // namespace dag
