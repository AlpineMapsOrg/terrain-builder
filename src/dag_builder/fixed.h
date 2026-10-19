#pragma once

#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace fixed {

template <int FractionBits, class Rep = std::int64_t>
struct Q {
    static_assert(std::is_signed_v<Rep>);
    static_assert(FractionBits > 0);

    // Keep headroom for intermediate signed arithmetic.
    static_assert(FractionBits < std::numeric_limits<Rep>::digits - 2);

    using Repr = Rep;

    static constexpr int fraction_bits = FractionBits;
    static constexpr Repr scale = Repr{1} << fraction_bits;

    // Raw fixed-point storage. One integer unit equals 1 / scale.
    Repr value = 0;

    // Converts an integral coordinate into fixed-point storage.
    static constexpr Q from_integer(Repr x) noexcept {
        return Q{x * scale};
    }

    // Converts a floating-point coordinate to the nearest fixed-point value.
    static Q from_real(double x) {
        return Q{static_cast<Repr>(std::llround(x * double(scale)))};
    }

    // Converts fixed-point storage back to floating-point form.
    constexpr double real() const noexcept {
        return double(value) / double(scale);
    }

    // Signed integer division rounded to the nearest integer.
    static constexpr Repr round_div(Repr numerator, Repr denominator) noexcept {
        assert(denominator != 0);

        if (denominator < 0) {
            numerator = -numerator;
            denominator = -denominator;
        }

        const bool negative = numerator < 0;
        const Repr n = negative ? -numerator : numerator;

        Repr quotient = n / denominator;
        const Repr remainder = n % denominator;

        if (remainder * 2 >= denominator) {
            ++quotient;
        }

        return negative ? -quotient : quotient;
    }

    // Multiplies two fixed-point values and rescales the result.
    static constexpr Repr mul(Repr a, Repr b) noexcept {
        return round_div(a * b, scale);
    }

    // Divides two fixed-point values while preserving fixed-point scale.
    static constexpr Repr div(Repr a, Repr b) noexcept {
        assert(b != 0);
        return round_div(a * scale, b);
    }
};

} // namespace fixed
