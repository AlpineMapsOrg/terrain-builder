#include <utility>

#include <libassert/assert.hpp>

#include "frontier/build.h"
#include "cli.h"
#include "commands.h"
#include "lbvh/build.h"
#include "pseudo_lbvh/build.h"
#include "gated_ploc/build.h"
#include "hploc/build.h"
#include "bvh/flatten_dag.h"
#include "error/Result.h"
#include "error/try.h"
#include "io/Error.h"
#include "io/ScratchDirectory.h"
#include "io/filesystem.h"
#include "log.h"
#include "octree/OddLevelShifted.h"
#include "octree/storage/open.h"
#include "../dag_builder/storage.h"
#include "bvh/Storage.h"
#include "bvh/earth.h"
#include "frontier/Storage.h"
#include "sort/options.h"
#include "sort/sort_by_morton.h"
#include "widen/build.h"

namespace {

// Builds the tree over the sorted frontier with the algorithm the mode names.
[[nodiscard]]
alp::Result<bvh::Reference, io::Error, io::DirectoryNotEmpty> build_tree(
    const cli::BuildArgs &args, frontier::Storage sorted, bvh::Storage &nodes) {
    switch (args.mode) {
    case cli::Mode::PseudoLbvh:
        return pseudo_lbvh::build(sorted, nodes, pseudo_lbvh::Options{
            .max_batch_bytes = args.max_batch_bytes,
        });
    case cli::Mode::Lbvh:
        return lbvh::build(sorted, nodes, lbvh::Options{
            .max_batch_bytes = args.max_batch_bytes,
        });
    case cli::Mode::GatedPloc:
        return gated_ploc::build(std::move(sorted), nodes, args.scratch_path / "rounds", gated_ploc::Options{
            .max_batch_bytes = args.max_batch_bytes,
            .max_resident_bytes = args.max_resident_bytes,
        });
    case cli::Mode::Hploc:
        return hploc::build(sorted, nodes, hploc::Options{
            .max_batch_bytes = args.max_batch_bytes,
        });
    }
    UNREACHABLE("unknown build mode");
}

}

alp::ErasedResult<void> build_bvh(const cli::BuildArgs &args) {
    ALP_TRY(bvh::Storage wide, bvh::Storage::create(args.output_path));
    ALP_TRY(const io::ScratchDirectory scratch, io::create_scratch_directory(args.scratch_path));

    ALP_TRY(const octree::IndexedDagMetaStorage dag, octree::open_folder_indexed<dag::NodeMetadata>(args.input_path));

    // The binary tree is what widening consumes, so only its widened form is the run being asked for.
    ALP_TRY(bvh::Storage nodes, bvh::Storage::create(scratch.path() / "binary"));

    const bvh::FlattenOptions flatten_options{
        .max_batch_bytes = args.max_batch_bytes,
        .max_resident_bytes = args.max_resident_bytes,
    };
    ALP_TRY(bvh::flatten_dag(dag, nodes, flatten_options));
    ALP_TRY(nodes.save_index());
    LOG_INFO("Flattened {} into {} node batches", args.input_path, nodes.size());

    ALP_TRY(frontier::Storage run, frontier::Storage::create(scratch.path() / "leaves"));

    const frontier::BuildOptions frontier_options{
        .max_batch_bytes = args.max_batch_bytes,
        .scene_bounds = bvh::earth_bounds(),
    };
    ALP_TRY(frontier::build_from_leaves(nodes, run, frontier_options));
    ALP_TRY(run.save_index());
    LOG_INFO("Built a frontier of {} batches", run.size());

    ALP_TRY(frontier::Storage sorted, frontier::Storage::create(scratch.path() / "sorted"));

    const sort::Options sort_options{
        .max_batch_bytes = args.max_batch_bytes,
        .max_resident_bytes = args.max_resident_bytes,
    };
    ALP_TRY(io::ScratchDirectory sort_runs, io::create_scratch_directory(scratch.path() / "sort-runs"));
    ALP_TRY(sort::sort_by_morton(run, sorted, sort_runs.path(), sort_options));
    ALP_TRY(std::move(sort_runs).remove());
    LOG_INFO("Sorted the frontier into {} batches", sorted.size());

    ALP_TRY(const bvh::Reference root, build_tree(args, std::move(sorted), nodes));
    LOG_INFO("Built the tree, with its root in batch {} at node {}", root.batch, root.node);

    const widen::Options widen_options{
        .max_batch_bytes = args.max_batch_bytes,
        .max_resident_bytes = args.max_resident_bytes,
    };
    ALP_TRY(const bvh::Reference wide_root, widen::build(nodes, root, wide, scratch.path(), widen_options));
    LOG_INFO("Widened the tree into {} batches, with its root in batch {} at node {}", wide.size(), wide_root.batch, wide_root.node);
    return {};
}
