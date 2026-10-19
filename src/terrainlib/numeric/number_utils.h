#pragma once

#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

#include <libassert/assert.hpp>

template <typename T>
T next_higher(T x) {
    static_assert(std::is_arithmetic_v<T>, "T must be a primitive numeric type");

    if constexpr (std::is_integral_v<T>) {
        ASSERT(x != std::numeric_limits<T>::max(), "No higher value exists for this type");
        return x + 1;
    } else {
        return std::nextafter(x, std::numeric_limits<T>::infinity());
    }
}

template <typename T>
T next_lower(T x) {
    static_assert(std::is_arithmetic_v<T>, "T must be a primitive numeric type");

    if constexpr (std::is_integral_v<T>) {
        ASSERT(x != std::numeric_limits<T>::lowest(), "No lower value exists for this type");
        return x - 1;
    } else {
        return std::nextafter(x, -std::numeric_limits<T>::infinity());
    }
}
