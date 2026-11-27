#pragma once

#include <array>
#include <cstddef>
#include <expected>
#include <ranges>
#include <span>
#include <utility>
#include <vector>

#include "io/serde.h"
#include "range_utils.h"
#include "error/Result.h"
#include "error/try.h"

#include "bvh/BatchWriter.h"
#include "bvh/Batch.h"
#include "bvh/NodeHeader.h"
#include "bvh/NodeInput.h"
#include "bvh/Reference.h"
#include "bvh/utils.h"
#include "frontier/layout.h"
#include "morton/utils.h"

namespace frontier {

namespace detail {

// Writes the parent of both entries and returns it, at the code `compute_morton` gives it.
template <typename F>
[[nodiscard]]
inline alp::Result<Entry, io::serde::Error> merge_entries(bvh::BatchWriter &sink, F &&compute_morton, const Entry &left, const Entry &right) {
    const std::array children{left, right};
    auto child_headers = std::views::transform(children, &Entry::header);
    auto child_refs = std::views::transform(children, &Entry::ref);

    bvh::NodeHeader header = bvh::merge_headers(child_headers);
    ALP_TRY(const bvh::Reference ref, sink.write_node(bvh::InnerInput{header, child_refs}));
    return Entry{
        .morton = compute_morton(header),
        .header = std::move(header),
        .ref = ref,
    };
}

} // namespace detail

// Writes the parent of both entries and returns it, at `morton`.
[[nodiscard]]
inline alp::Result<Entry, io::serde::Error> merge_entries(bvh::BatchWriter &sink, const morton::Code morton, const Entry &left, const Entry &right) {
    return detail::merge_entries(sink, [morton](const bvh::NodeHeader &) { return morton; }, left, right);
}

// Writes the parent of both entries and returns it, at the code of its own bounds.
[[nodiscard]]
inline alp::Result<Entry, io::serde::Error> merge_entries(bvh::BatchWriter &sink, const bvh::Bounds &scene, const Entry &left, const Entry &right) {
    return detail::merge_entries(sink, [&scene](const bvh::NodeHeader &h) { return morton::encode(scene, h); }, left, right);
}

// Merge two frontier entries into an inner node in a batch.
inline bvh::NodeIndex make_inner_node(bvh::Batch &batch, const Entry &a, const Entry &b) {
    const bvh::NodeHeader header = bvh::merge_headers(std::array{a.header, b.header});
    const bvh::NodeIndex node_index = batch.add_node(bvh::InnerInput{header, std::array{a.ref, b.ref}});
    return node_index;
}

// Merge a group of frontier entries into one inner node in a batch.
template <std::ranges::forward_range R>
inline bvh::NodeIndex make_inner_node(bvh::Batch &batch, R &&entries) {
    const auto child_headers = entries | std::views::transform(&Entry::header);
    const bvh::NodeHeader header = bvh::merge_headers(child_headers);
    const auto child_references = entries | std::views::transform(&Entry::ref);
    const bvh::NodeIndex node_index = batch.add_node(bvh::InnerInput{header, child_references});
    return node_index;
}

// Merge two frontier entries into an inner node in a batch.
inline Entry merge_entries(bvh::Batch &batch, const bvh::BatchIndex batch_index, const morton::Code morton, const Entry &a, const Entry &b) {
    const bvh::NodeHeader header = bvh::merge_headers(std::array{a.header, b.header});
    const bvh::NodeIndex node_index = batch.add_node(bvh::InnerInput{header, std::array{a.ref, b.ref}});

    return Entry{
        .morton = morton,
        .header = header,
        .ref = bvh::Reference{.batch = batch_index, .node = node_index},
    };
}

// Merge a group of frontier entries into one inner node in a batch.
template <std::ranges::forward_range R>
inline Entry merge_entries(bvh::Batch &batch, const bvh::BatchIndex batch_index, const morton::Code morton, R &&entries) {
    const auto child_headers = entries | std::views::transform(&Entry::header);
    const bvh::NodeHeader header = bvh::merge_headers(child_headers);
    const auto references = entries | std::views::transform(&Entry::ref);
    const bvh::NodeIndex node_index = batch.add_node(bvh::InnerInput{header, references});

    return Entry{
        .morton = morton,
        .header = header,
        .ref = bvh::Reference{.batch = batch_index, .node = node_index},
    };
}

}
