#pragma once

#include <cstddef>
#include <optional>
#include <tuple>

#include <glaze/json.hpp>

#include "octree/Id.h"

// Declares octree ids as JSON arrays of their level and their index on that level.
namespace glz {

template <size_t Dimensions, size_t MaxLevel, typename LevelT, typename IndexT, typename CoordT>
struct from<JSON, octree::Id_<Dimensions, MaxLevel, LevelT, IndexT, CoordT>> {
    using Id = octree::Id_<Dimensions, MaxLevel, LevelT, IndexT, CoordT>;

    template <auto Opts>
    static void op(Id &value, is_context auto &&context, auto &&it, auto &&end) {
        // A tuple must be complete. An array may be cut short.
        std::tuple<typename Id::Level, typename Id::Index> parts;
        parse<JSON>::op<Opts>(parts, context, it, end);

        const auto [level, index] = parts;
        const std::optional<Id> id_opt = Id::try_make(level, index);
        if (!id_opt.has_value()) {
            context.error = error_code::constraint_violated;
            return;
        }
        value = *id_opt;
    }
};

template <size_t Dimensions, size_t MaxLevel, typename LevelT, typename IndexT, typename CoordT>
struct to<JSON, octree::Id_<Dimensions, MaxLevel, LevelT, IndexT, CoordT>> {
    using Id = octree::Id_<Dimensions, MaxLevel, LevelT, IndexT, CoordT>;

    template <auto Opts>
    static void op(const Id &value, is_context auto &&context, auto &&buffer, auto &&index) {
        const std::tuple parts{value.level(), value.index_on_level()};
        serialize<JSON>::op<Opts>(parts, context, buffer, index);
    }
};

}
