#include "PrunedLocation.h"

#include <source_location>
#include <string>
#include <string_view>

#include <cpptrace/utils.hpp>
#include <fmt/format.h>

fmt::format_context::iterator fmt::formatter<PrunedLocation>::format(const PrunedLocation location, fmt::format_context &context) const {
    const std::source_location &source = location.location;
    const std::string text = fmt::format("{} at {}:{}:{}", cpptrace::prune_symbol(source.function_name()), source.file_name(), source.line(), source.column());
    return fmt::formatter<std::string_view>::format(text, context);
}
