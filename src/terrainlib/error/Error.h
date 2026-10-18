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

    // Creates an erased error from `message`.
    explicit BasicError(std::string message, const std::source_location location = std::source_location::current())
        requires(is_erased)
        : _code(detail::Message{.text = std::move(message)}),
          _trace(location) {}

    // Takes over an error whose codes are all among Es, or any error with copyable codes when this error is erased, keeping its frames.
    template <typename... Fs> requires std::constructible_from<CodeStorage, alp::CodeStorage<Fs...>>
    BasicError(BasicError<Fs...> other)
        : _code(std::move(other._code)),
          _trace(std::move(other._trace)) {}

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
    template <typename F> requires(!is_erased)
    decltype(auto) visit(F &&f) const {
        return this->_code.visit(std::forward<F>(f));
    }

    // Whether this error holds a code of type E.
    template <typename E> requires(can_hold<E>)
    [[nodiscard]]
    bool holds() const {
        return this->get_if<E>() != nullptr;
    }

    // Whether this error holds `code`.
    template <typename E> requires(can_hold<E> && std::equality_comparable<E>)
    [[nodiscard]]
    bool holds(const E &code) const {
        const E *held = this->get_if<E>();
        return held != nullptr && *held == code;
    }

    // Whether this error holds a code of type E for which `predicate` is true.
    template <typename E, std::predicate<const E &> P> requires(can_hold<E>)
    [[nodiscard]]
    bool holds_and(P &&predicate) const {
        const E *held = this->get_if<E>();
        return held != nullptr && std::invoke(std::forward<P>(predicate), *held);
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

    // This error with only the codes Fs if it holds one of them, otherwise this error unchanged; a set among Fs stands for its codes.
    template <ErrorCodes... Fs, typename Self> requires(narrows_to<Fs...>)
    [[nodiscard]]
    std::expected<Error<Fs...>, BasicError> narrow(this Self &&self) {
        return Error<Fs...>::narrow_from(std::forward<Self>(self));
    }

private:
    template <typename... Fs>
    friend class BasicError;

    BasicError(CodeStorage code, Trace trace) : _code(std::move(code)), _trace(std::move(trace)) {}

    // Narrows `error` to the codes Es if it holds one of them, otherwise hands it back unchanged.
    template <typename Source>
    static std::expected<BasicError, std::remove_cvref_t<Source>> narrow_from(Source &&error) {
        std::optional<CodeStorage> code_result = std::forward<Source>(error)._code.template narrow<Es...>();
        if (!code_result.has_value()) {
            return std::unexpected(std::forward<Source>(error));
        }
        return BasicError(std::move(*code_result), std::forward<Source>(error)._trace);
    }

    friend struct fmt::formatter<BasicError>;

    CodeStorage _code;
    Trace _trace;
};

} // namespace alp
