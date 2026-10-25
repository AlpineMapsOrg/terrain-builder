#pragma once

#include <cstdint>

#include <zpp_bits.h>

#include "octree/Id.h"
#include "hash_utils.h"

namespace dag {

// The id of a dag batch.
using BatchId = octree::Id;

// A reference to a cluster within a dag cluster batch.
struct ClusterId {
    BatchId batch;
    uint32_t cluster_index;

    auto operator<=>(const ClusterId &) const = default;
};

// A reference to a group within a dag cluster batch.
struct GroupId {
    BatchId batch;
    uint32_t group_index;

    auto operator<=>(const GroupId &) const = default;
};

}

template <>
struct std::hash<dag::ClusterId> {
    size_t operator()(const dag::ClusterId &id) const  {
        return ::hash::combine(id.batch, id.cluster_index);
    }
};

template <>
struct fmt::formatter<dag::ClusterId> {
    constexpr auto parse(fmt::format_parse_context &ctx) { return ctx.begin(); }
    auto format(const dag::ClusterId &id, fmt::format_context &ctx) const {
        return fmt::format_to(ctx.out(), "Cluster(batch={}, index={})", id.batch, id.cluster_index);
    }
};

template <>
struct fmt::formatter<dag::GroupId> {
    constexpr auto parse(fmt::format_parse_context &ctx) { return ctx.begin(); }
    auto format(const dag::GroupId &id, fmt::format_context &ctx) const {
        return fmt::format_to(ctx.out(), "Group(batch={}, index={})", id.batch, id.group_index);
    }
};

namespace zpp::bits {

template <typename Archive>
auto serialize(Archive &archive, const dag::ClusterId &id) {
    return archive(id.batch, id.cluster_index);
}

template <typename Archive>
auto serialize(Archive &archive, dag::ClusterId &id) {
    return archive(id.batch, id.cluster_index);
}

template <typename Archive>
auto serialize(Archive &archive, const dag::GroupId &id) {
    return archive(id.batch, id.group_index);
}

template <typename Archive>
auto serialize(Archive &archive, dag::GroupId &id) {
    return archive(id.batch, id.group_index);
}

}
