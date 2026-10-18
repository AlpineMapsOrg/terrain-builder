#pragma once

#include <any>
#include <concepts>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include <fmt/format.h>

#include "error/ErrorCode.h"
#include "error/TypedCode.h"

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

    [[nodiscard]]
    std::string message() const {
        return this->_format(this->_code);
    }

    // The held code as a code of the types Fs if it is one of them, otherwise nothing.
    template <ErrorCode... Fs, typename Self>
    [[nodiscard]]
    std::optional<TypedCode<Fs...>> narrow(this Self &&self) {
        return narrow_to<TypedCode<Fs...>, Self, Fs...>(std::forward<Self>(self));
    }

private:
    template <ErrorCode E>
    static std::string format(const std::any &code) {
        return fmt::to_string(std::any_cast<const E &>(code));
    }

    // The held code as a `Narrowed` if it is one of G, Gs, otherwise nothing.
    template <typename Narrowed, typename Self, ErrorCode G, ErrorCode... Gs>
    static std::optional<Narrowed> narrow_to(Self &&self) {
        if (auto *code = std::any_cast<G>(&self._code); code != nullptr) {
            return Narrowed(std::forward_like<Self>(*code));
        }
        if constexpr (sizeof...(Gs) == 0) {
            return std::nullopt;
        } else {
            return narrow_to<Narrowed, Self, Gs...>(std::forward<Self>(self));
        }
    }

    std::any _code;
    std::string (*_format)(const std::any &);
};

} // namespace alp
