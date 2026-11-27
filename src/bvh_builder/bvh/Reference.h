#pragma once

#include <compare>
#include <cstdint>

namespace bvh {

using BatchIndex = uint32_t;
using NodeIndex = uint32_t;

struct Reference {
    BatchIndex batch;
    NodeIndex node;

    auto operator<=>(const Reference &) const = default;
};

}
