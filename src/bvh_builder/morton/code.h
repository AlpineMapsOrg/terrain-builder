#pragma once

#include <cstdint>
#include <limits>

#include "numeric/number_types.h"
#include "octree/Id.h"

namespace morton {

using CodeId = octree::Id_<3, 128 / 3>;
using Code = CodeId::Index;

constexpr uint32_t CODE_BITS = std::numeric_limits<Code>::digits;

}