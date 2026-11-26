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

#include <concepts>
#include <expected>
#include <functional>
#include <optional>
#include <source_location>
#include <span>
#include <string>
#include <type_traits>
#include <utility>

#include <fmt/format.h>
#include <libassert/assert.hpp>

#include "containers/indirect.h"
#include "error/CodeStorage.h"
#include "error/Context.h"
#include "error/ErrorCode.h"
#include "error/Trace.h"
#include "type_utils.h"

namespace alp {

namespace detail {

// The error type for the codes Es.
template <typename... Es>
struct ErrorFor {
    // Checked here instead of through a constraint on Error: with a precompiled header, GCC 15.2 can evaluate an alias's constraint twice with different results when a garbage collection falls in between, and then rejects valid code with "satisfaction value of atomic constraint changed".
    static_assert(((ErrorCodes<Es> || std::is_same_v<Es, BasicError<>>) && ...), "an error holds error codes and sets of codes");
    // An erased set can already hold every code, so it takes in the others.
    using type = std::conditional_t<(std::is_same_v<Es, BasicError<>> || ...), BasicError<>, unique_t<flatten_t<BasicError<Es...>>>>;
};

} // namespace detail

// An error with the codes Es, where a set among Es stands for its codes; each code is kept once, in the order it first appears.
// An erased set among Es makes the error erased.
template <typename... Es>
using Error = typename detail::ErrorFor<Es...>::type;

// An error with a code of any copyable type.
using ErasedError = BasicError<>;

// An error with a code of type Es, or an erased error when Es is empty, together with the frames it collected.
template <typename... Es>
class BasicError {
    static_assert((ErrorCode<Es> && ...), "an error holds only error codes; nest a set through alp::Error");

public:
    // Whether the error is erased, holding a code of any copyable type.
    static constexpr bool is_erased = sizeof...(Es) == 0;

private:
    using CodeStorage = alp::CodeStorage<Es...>;

public:
    // Whether the error has exactly one code type.
    static constexpr bool is_single = sizeof...(Es) == 1;

    // Whether this error can hold a code of type E.
    template <typename E>
    static constexpr bool can_hold = CodeStorage::template can_hold<E>;

    // Whether this error can compare its code with a T.
    template <typename T>
    static constexpr bool compares_with = CodeStorage::template compares_with<T>;

    // Whether this error can narrow to the codes Fs.
    template <ErrorCodes... Fs>
    static constexpr bool narrows_to = !Error<Fs...>::is_erased && std::is_convertible_v<Error<Fs...>, BasicError>;

    // Creates an error with `code` that originates at `location`.
    template <OneOf<Es...> E>
    explicit BasicError(E code, const std::source_location location = std::source_location::current())
        : _code(std::move(code)),
          _trace(location) {}

    // Creates an error with `code` that originates at `location`, with `context` as what was being done there.
    template <OneOf<Es...> E>
    BasicError(E code, Context auto &&context, const std::source_location location = std::source_location::current())
        : _code(std::move(code)),
          _trace(location, detail::resolve_message(std::forward<decltype(context)>(context))) {}

    // Creates an erased error from `message`.
    explicit BasicError(std::string message, const std::source_location location = std::source_location::current())
        requires(is_erased)
        : _code(detail::Message{.text = std::move(message)}),
          _trace(location) {}

    // Takes over an error whose codes are all among Es, or any error with copyable codes when this error is erased, keeping its frames and the error it was created from.
    template <typename... Fs> requires std::constructible_from<CodeStorage, alp::CodeStorage<Fs...>>
    BasicError(BasicError<Fs...> other)
        : _code(std::move(other._code)),
          _trace(std::move(other._trace)),
          _source(std::move(other._source)) {}

    // Converts into a failed result holding this error, for a result whose error this error converts to.
    template <typename T, typename... Fs> requires std::convertible_to<const BasicError &, BasicError<Fs...>>
    operator std::expected<T, BasicError<Fs...>>() const & {
        return std::unexpected(BasicError<Fs...>(*this));
    }

    // Converts into a failed result holding this error, moved there, for a result whose error this error converts to.
    template <typename T, typename... Fs> requires std::convertible_to<BasicError, BasicError<Fs...>>
    operator std::expected<T, BasicError<Fs...>>() && {
        return std::unexpected(BasicError<Fs...>(std::move(*this)));
    }

    // Adds a frame holding `context`.
    [[nodiscard]]
    BasicError context(Context auto &&context, const std::source_location location = std::source_location::current()) && {
        this->_trace.push(Frame{.message = detail::resolve_message(std::forward<decltype(context)>(context)), .location = location});
        return std::move(*this);
    }

    // Records `inner` as the error this one was created from.
    // The error must not have one yet.
    template <ErrorCode... Fs> requires std::constructible_from<ErasedError, BasicError<Fs...>>
    [[nodiscard]]
    BasicError with_source(BasicError<Fs...> inner) && {
        ASSERT(!this->_source.has_value(), "the error already has a source");
        this->_source.emplace(ErasedError(std::move(inner)));
        return std::move(*this);
    }

    // Adds a frame without a message.
    [[nodiscard]]
    BasicError propagate(const std::source_location location = std::source_location::current()) && {
        this->_trace.push(Frame{.location = location});
        return std::move(*this);
    }

    [[nodiscard]]
    const auto &code() const requires(is_single) {
        return this->_code.get();
    }

    // The held code if it is an E, otherwise null.
    template <typename E> requires(can_hold<E>)
    [[nodiscard]]
    const E *get_if() const {
        return this->_code.template get_if<E>();
    }

    // Calls `f` with the held code.
    template <typename F, typename Self> requires(!is_erased)
    decltype(auto) visit(this Self &&self, F &&f) {
        return std::forward<Self>(self)._code.visit(std::forward<F>(f));
    }

    // Calls `visitor` with the held code if it is in Fs, or returns nothing otherwise.
    template <ErrorCodes... Fs, typename Self, typename Visitor> requires(narrows_to<Fs...>)
    [[nodiscard]]
    auto visit_if(this Self &&self, Visitor &&visitor) {
        return Error<Fs...>::visit_if_from(std::forward<Self>(self), std::forward<Visitor>(visitor));
    }

    // Whether this error holds a code in Fs.
    template <ErrorCodes... Fs> requires(narrows_to<Fs...>)
    [[nodiscard]]
    bool holds() const {
        return this->holds_and<Fs...>([](const auto &) { return true; });
    }

    // Whether this error holds a code equal to `value`.
    // An erased error counts only a held code of type T.
    template <typename T> requires(compares_with<T>)
    [[nodiscard]]
    bool holds_code(const T &value) const {
        return this->_code.equals(value);
    }

    // Whether this error holds a code in Fs for which `predicate` is true.
    template <ErrorCodes... Fs, typename P> requires(narrows_to<Fs...>)
    [[nodiscard]]
    bool holds_and(P &&predicate) const {
        return this->visit_if<Fs...>(std::forward<P>(predicate)).value_or(false);
    }

    // Where the error originated.
    [[nodiscard]]
    std::source_location origin() const {
        return this->_trace.origin();
    }

    // The points the error was at, innermost first, starting at the origin.
    [[nodiscard]]
    std::span<const Frame> frames() const {
        return this->_trace.frames();
    }

    // The error this one was created from, or null if it was created from none.
    [[nodiscard]]
    const ErasedError *source() const {
        if (!this->_source.has_value()) {
            return nullptr;
        }
        return &**this->_source;
    }

    // This error with only the codes Fs if it holds one of them, otherwise this error unchanged; a set among Fs stands for its codes.
    template <ErrorCodes... Fs, typename Self> requires(narrows_to<Fs...>)
    [[nodiscard]]
    std::expected<Error<Fs...>, BasicError> narrow(this Self &&self) {
        using Narrowed = Error<Fs...>;
        using NarrowedCode = typename Narrowed::CodeStorage;

        std::optional<NarrowedCode> code_result = Narrowed::visit_if_from(std::forward<Self>(self), []<typename F>(F &&code) { return NarrowedCode(std::forward<F>(code)); });
        if (!code_result.has_value()) {
            return std::unexpected(std::forward<Self>(self));
        }
        return {Narrowed(std::move(*code_result), std::forward<Self>(self)._trace, std::forward<Self>(self)._source)};
    }

    // This error with only the codes Fs if it holds one of them, otherwise with only its other codes, which for an erased error is this error unchanged; a set among Fs stands for its codes.
    template <ErrorCodes... Fs, typename Self> requires(narrows_to<Fs...> && (is_erased || (!Error<Fs...>::template can_hold<Es> || ...)))
    [[nodiscard]]
    std::expected<Error<Fs...>, difference_t<BasicError, Error<Fs...>>> split(this Self &&self) {
        if constexpr (is_erased) {
            return std::forward<Self>(self).template narrow<Fs...>();
        } else {
            using Others = difference_t<BasicError, Error<Fs...>>;

            std::expected<Error<Fs...>, BasicError> named_result = std::forward<Self>(self).template narrow<Fs...>();
            if (named_result.has_value()) {
                return {*std::move(named_result)};
            }

            std::expected<Others, BasicError> others_result = std::move(named_result).error().template narrow<Others>();
            ASSERT(others_result.has_value(), "the held code belongs to neither side of the split");
            return std::unexpected(*std::move(others_result));
        }
    }

    // Maps this error with only the codes Fs by applying `f`, if it holds one of them.
    template <ErrorCodes... Fs, std::invocable<Error<Fs...>> F, typename Self> requires(narrows_to<Fs...>)
    [[nodiscard]]
    auto map(this Self &&self, F &&f) {
        using Result = std::remove_cvref_t<std::invoke_result_t<F, Error<Fs...>>>;
        if constexpr (is_erased) {
            std::expected<Error<Fs...>, BasicError> result = std::forward<Self>(self).template narrow<Fs...>();
            if (result.has_value()) {
                BasicError error = std::invoke(std::forward<F>(f), *std::move(result));
                return error;
            }
            return std::move(result).error();
        } else if constexpr (is_subset_v<BasicError, Error<Fs...>>) {
            Error<Result> error = std::invoke(std::forward<F>(f), Error<Fs...>(std::forward<Self>(self)));
            return error;
        } else {
            auto split_result = std::forward<Self>(self).template split<Fs...>();
            if (split_result.has_value()) {
                Error<Result, difference_t<BasicError, Error<Fs...>>> error = std::invoke(std::forward<F>(f), *std::move(split_result));
                return error;
            }
            Error<Result, difference_t<BasicError, Error<Fs...>>> error = std::move(split_result).error();
            return error;
        }
    }

    // Maps this error with only the codes Fs to an error with `code` created from it, if it holds one of them.
    template <ErrorCodes... Fs, ErrorCode E, typename Self> requires(narrows_to<Fs...>)
    [[nodiscard]]
    auto map(this Self &&self, E code, const std::source_location location = std::source_location::current()) {
        return std::forward<Self>(self).template map<Fs...>([&](Error<Fs...> error) {
            return Error<E>(std::move(code), location).with_source(std::move(error));
        });
    }

    // Maps this error by applying `f`, if it holds a code equal to `value`.
    template <typename T, std::invocable<BasicError> F, typename Self> requires(compares_with<T>)
    [[nodiscard]]
    auto map_code(this Self &&self, const T &value, F &&f) {
        using Result = std::remove_cvref_t<std::invoke_result_t<F, BasicError>>;
        if constexpr (is_erased) {
            if (self.holds_code(value)) {
                BasicError error = std::invoke(std::forward<F>(f), BasicError(std::forward<Self>(self)));
                return error;
            }
            return BasicError(std::forward<Self>(self));
        } else {
            if (self.holds_code(value)) {
                Error<Result, BasicError> error = std::invoke(std::forward<F>(f), BasicError(std::forward<Self>(self)));
                return error;
            }
            Error<Result, BasicError> error = std::forward<Self>(self);
            return error;
        }
    }

    // Maps this error to an error with `code` created from it, if it holds a code equal to `value`.
    template <typename T, ErrorCode E, typename Self> requires(compares_with<T>)
    [[nodiscard]]
    auto map_code(this Self &&self, const T &value, E code, const std::source_location location = std::source_location::current()) {
        return std::forward<Self>(self).map_code(value, [&](BasicError error) {
            return Error<E>(std::move(code), location).with_source(std::move(error));
        });
    }

private:
    template <typename... Fs>
    friend class BasicError;

    BasicError(CodeStorage code, Trace trace, std::optional<indirect<ErasedError>> source)
        : _code(std::move(code)),
          _trace(std::move(trace)),
          _source(std::move(source)) {}

    // Calls `visitor` with the code `error` holds if it is in Es, or returns nothing otherwise.
    template <typename Source, typename Visitor>
    static auto visit_if_from(Source &&error, Visitor &&visitor) {
        return std::forward<Source>(error)._code.template visit_if<Es...>(std::forward<Visitor>(visitor));
    }

    friend struct fmt::formatter<BasicError>;

    CodeStorage _code;
    Trace _trace;

    // The error this one was created from, if any.
    std::optional<indirect<ErasedError>> _source;
};

} // namespace alp
