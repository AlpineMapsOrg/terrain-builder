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
#include <indirect.h>
#include <libassert/assert.hpp>

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
    static_assert((ErrorCodes<Es> && ...), "an error holds error codes and sets of codes that are not erased");
    using type = unique_t<flatten_t<BasicError<Es...>>>;
};

} // namespace detail

// An error with the codes Es, where a set among Es stands for its codes; each code is kept once, in the order it first appears.
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

    // Creates an error with `code` that originates at `location`, created from `inner`.
    template <OneOf<Es...> E, ErrorCode... Fs> requires std::constructible_from<ErasedError, BasicError<Fs...>>
    BasicError(E code, BasicError<Fs...> inner, const std::source_location location = std::source_location::current())
        : _code(std::move(code)),
          _trace(location),
          _source(std::in_place, ErasedError(std::move(inner))) {}

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

    // Whether this error holds `code`.
    template <typename E> requires(can_hold<E> && std::equality_comparable<E>)
    [[nodiscard]]
    bool holds_code(const E &code) const {
        const E *held = this->get_if<E>();
        return held != nullptr && *held == code;
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

    // This error with only the codes Fs if it holds one of them, otherwise with only its other codes; a set among Fs stands for its codes.
    template <ErrorCodes... Fs, typename Self> requires(!is_erased && narrows_to<Fs...> && (!Error<Fs...>::template can_hold<Es> || ...))
    [[nodiscard]]
    std::expected<Error<Fs...>, difference_t<BasicError, Error<Fs...>>> split(this Self &&self) {
        using Others = difference_t<BasicError, Error<Fs...>>;

        std::expected<Error<Fs...>, BasicError> named_result = std::forward<Self>(self).template narrow<Fs...>();
        if (named_result.has_value()) {
            return {*std::move(named_result)};
        }

        std::expected<Others, BasicError> others_result = std::move(named_result).error().template narrow<Others>();
        ASSERT(others_result.has_value(), "the held code belongs to neither side of the split");
        return std::unexpected(*std::move(others_result));
    }

private:
    template <typename... Fs>
    friend class BasicError;

    BasicError(CodeStorage code, Trace trace, std::optional<xyz::indirect<ErasedError>> source)
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
    std::optional<xyz::indirect<ErasedError>> _source;
};

} // namespace alp
