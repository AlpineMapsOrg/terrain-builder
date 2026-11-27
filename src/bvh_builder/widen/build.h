#pragma once

#include <algorithm>
#include <cstdint>
#include <expected>
#include <filesystem>

#include <fmt/format.h>
#include <libassert/assert.hpp>

#include "io/serde.h"
#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"

#include "bvh/NodeCache.h"
#include "bvh/Storage.h"
#include "bvh/Reference.h"
#include "widen/options.h"
#include "widen/tabulate_splits.h"
#include "widen/write_wide_nodes.h"

namespace widen {

// Writes the tree at `root` into `output` with up to `arity` children per node, and returns its new root.
[[nodiscard]]
inline alp::Result<bvh::Reference, io::serde::Error> build(
    const bvh::Storage &input, const bvh::Reference &root, bvh::Storage &output,
    const std::filesystem::path &scratch, const Options &options) {
    return alp::run_with_context(
        alp::make_context("widening a tree from {} to {}", input.directory(), output.directory()),
        [&] -> alp::Result<bvh::Reference, io::serde::Error> {
            bvh::NodeCache tree(input, options.max_resident_bytes);

            ALP_TRY(SplitStorage splits, SplitStorage::create(scratch / "splits"));
            ALP_TRY(tabulate_splits(tree, root, splits, options));
            ALP_TRY(splits.save_index());

            ALP_TRY(const bvh::Reference wide_root, write_wide_nodes(splits, tree, root, output, options));
            ALP_TRY(output.save_index());
            return wide_root;
        });
}

}