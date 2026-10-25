#pragma once

#include <concepts>
#include <ranges>

#include <fmt/format.h>
#include <fmt/std.h>

#include "type_utils.h"

#include "bvh/NodeHeader.h"
#include "bvh/NodeView.h"
#include "dag/SourceId.h"

namespace bvh {

// An inner node's header and children, before a batch stores them.
template <std::ranges::input_range R>
struct InnerInput {
    NodeHeader header;
    R children;

    bool operator==(const InnerInput &) const = default;
};

template <typename R>
InnerInput(NodeHeader, R &&) -> InnerInput<std::views::all_t<R>>;

// A leaf's header and source, before a batch stores them.
struct LeafInput {
    NodeHeader header;
    dag::SourceId source;

    bool operator==(const LeafInput &) const = default;
};

// A node a batch can store.
template <typename N>
concept NodeInput = is_specialization_of_v<N, InnerInput> || std::same_as<N, LeafInput> || std::same_as<N, NodeView>;

}

template <>
struct fmt::formatter<bvh::LeafInput> {
    constexpr auto parse(fmt::format_parse_context &ctx) {
        return ctx.begin();
    }
    auto format(const bvh::LeafInput &leaf, fmt::format_context &ctx) const {
        return fmt::format_to(ctx.out(), "LeafInput(header={}, source={})", leaf.header, leaf.source);
    }
};
