#pragma once

#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>

#include <fmt/format.h>

#include "error/ErrorCode.h"
#include "type_utils.h"

namespace alp {

// The code of a typed error: a code of one of the types Es.
template <ErrorCode... Es>
class TypedCode {
public:
    // Whether this code can be a code of type E.
    template <typename E>
    static constexpr bool can_hold = OneOf<E, Es...>;

    template <typename E> requires(can_hold<E>)
    explicit TypedCode(E code) : _code(std::in_place_type<E>, std::move(code)) {}

    // Takes the code `other` holds, which must be one of Es.
    template <ErrorCode... Fs> requires((can_hold<Fs> && ...))
    explicit TypedCode(TypedCode<Fs...> other)
        : _code(std::move(other).visit([]<typename F>(F &&code) {
              return std::variant<Es...>(std::in_place_type<std::remove_cvref_t<F>>, std::forward<F>(code));
          })) {}

    // The code, for a single code type.
    [[nodiscard]]
    const auto &get() const requires(sizeof...(Es) == 1) {
        return std::get<0>(this->_code);
    }

    // The held code if it is an E, otherwise null.
    template <typename E> requires(can_hold<E>)
    [[nodiscard]]
    const E *get_if() const {
        return std::get_if<E>(&this->_code);
    }

    [[nodiscard]]
    std::string message() const {
        return this->visit([](const auto &code) { return fmt::to_string(code); });
    }

    // Calls `f` with the held code.
    template <typename F, typename Self>
    decltype(auto) visit(this Self &&self, F &&f) {
        return std::visit(std::forward<F>(f), std::forward<Self>(self)._code);
    }

    // This code with only the types Fs if it holds one of them, otherwise nothing.
    template <ErrorCode... Fs, typename Self>
    [[nodiscard]]
    std::optional<TypedCode<Fs...>> narrow(this Self &&self) {
        return std::forward<Self>(self).visit([]<typename E>(E &&code) -> std::optional<TypedCode<Fs...>> {
            if constexpr (OneOf<std::remove_cvref_t<E>, Fs...>) {
                return TypedCode<Fs...>(std::forward<E>(code));
            } else {
                return std::nullopt;
            }
        });
    }

private:
    std::variant<Es...> _code;
};

} // namespace alp
