/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Martin Braunsperger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *****************************************************************************/

#pragma once

#include <any>
#include <concepts>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include <fmt/format.h>

#include "error/ErrorCode.h"
#include "error/TypedCode.h"
#include "std_compat.h"

namespace alp {

namespace detail {

// The code of an error created from text.
struct Message {
    std::string text;
};

inline std::string_view format_as(const Message &message) {
    return message.text;
}

} // namespace detail

// The code of an erased error: a code of any copyable type.
class ErasedCode {
public:
    // Whether this code can be a code of type E.
    template <typename E>
    static constexpr bool can_hold = ErrorCode<E> && std::copy_constructible<E>;

    // Whether this code can be compared with a T.
    // It knows only whether it holds a T, so it compares only a held code of type T.
    template <typename T>
    static constexpr bool compares_with = can_hold<T> && std::equality_comparable<T>;

    template <typename E> requires(can_hold<E>)
    explicit ErasedCode(E code) : _code(std::in_place_type<E>, std::move(code)), _format(&format<E>) {}

    // Erases the code `code` holds.
    template <ErrorCode... Es> requires((can_hold<Es> && ...))
    explicit ErasedCode(TypedCode<Es...> code)
        : ErasedCode(std::move(code).visit([]<typename E>(E &&held) { return ErasedCode(std::forward<E>(held)); })) {}

    // The held code if it is an E, otherwise null.
    template <typename E> requires(can_hold<E>)
    [[nodiscard]]
    const E *get_if() const {
        return std::any_cast<E>(&this->_code);
    }

    // Whether the held code is a T equal to `value`.
    template <typename T> requires(compares_with<T>)
    [[nodiscard]]
    bool equals(const T &value) const {
        const T *held = this->get_if<T>();
        return held != nullptr && *held == value;
    }

    [[nodiscard]]
    std::string message() const {
        return this->_format(this->_code);
    }

    // Calls `visitor` with the held code if it is in Fs, or returns nothing otherwise.
    template <ErrorCode... Fs, typename Self, typename Visitor>
    [[nodiscard]]
    auto visit_if(this Self &&self, Visitor &&visitor) {
        using Result = std::common_type_t<decltype(std::invoke(visitor, std_compat::forward_like<Self>(std::declval<Fs &>())))...>;
        std::optional<Result> result;
        const auto visit_if_held = [&]<typename F>() {
            if (auto *code = std::any_cast<F>(&self._code); code != nullptr) {
                result.emplace(std::invoke(visitor, std_compat::forward_like<Self>(*code)));
            }
        };
        (visit_if_held.template operator()<Fs>(), ...);
        return result;
    }

    // The held code as a code of the types Fs if it is one of them, otherwise nothing.
    template <ErrorCode... Fs, typename Self>
    [[nodiscard]]
    std::optional<TypedCode<Fs...>> narrow(this Self &&self) {
        return std::forward<Self>(self).template visit_if<Fs...>([]<typename F>(F &&code) { return TypedCode<Fs...>(std::forward<F>(code)); });
    }

private:
    template <ErrorCode E>
    static std::string format(const std::any &code) {
        return fmt::to_string(std::any_cast<const E &>(code));
    }

    std::any _code;
    std::string (*_format)(const std::any &);
};

} // namespace alp
