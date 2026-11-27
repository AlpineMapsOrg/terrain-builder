#pragma once

#include <algorithm>
#include <cstddef>
#include <expected>
#include <optional>
#include <string>

#include <fmt/format.h>

#include "io/serde.h"

#include "storage/batch_limits.h"
#include "bvh/Storage.h"
#include "bvh/flat_traverse.h"
#include "bvh/Node.h"
#include "bvh/NodeHeader.h"
#include "bvh/Reference.h"
#include "frontier/BatchWriter.h"
#include "frontier/Storage.h"
#include "frontier/layout.h"
#include "morton/utils.h"
#include "bvh/earth.h"
#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"

namespace frontier {

// The entry a stored node enters the frontier as.
inline Entry make_entry(const bvh::Reference &ref, const bvh::Node &node, const bvh::Bounds &scene) {
    return Entry{
        .morton = morton::encode(scene, node.header()),
        .header = node.header(),
        .ref = ref,
    };
}

struct BuildOptions {
    size_t max_batch_bytes = DEFAULT_MAX_BATCH_BYTES;
    bvh::Bounds scene_bounds = bvh::earth_bounds();
};

// Writes every stored node as a set of frontier run batches.
[[nodiscard]]
inline alp::Result<void, io::serde::Error> build_from_leaves(const bvh::Storage &nodes, Storage &output, const BuildOptions &options = {}) {
    return alp::run_with_context(
        alp::make_context("building a frontier from {} to {}", nodes.directory(), output.directory()),
        [&] -> alp::Result<void, io::serde::Error> {
            BatchWriter writer(output, options.max_batch_bytes);

            // The walk stops on the first failed write, leaving it to report.
            ALP_TRY(bvh::flat_traverse(nodes, [&](const bvh::Reference &ref, const bvh::Node &node) {
                const auto entry = make_entry(ref, node, options.scene_bounds);
                return writer.write(entry);
            }));
            return writer.flush();
        });
}

}
