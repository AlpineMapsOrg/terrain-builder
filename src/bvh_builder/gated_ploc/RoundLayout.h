#pragma once

#include <cstdint>
#include <filesystem>
#include <utility>

#include <fmt/format.h>

#include "error/Result.h"
#include "io/FileError.h"

#include "frontier/Storage.h"

namespace gated_ploc {

namespace detail {

class RoundLayout {
public:
    explicit RoundLayout(std::filesystem::path root) : _root(std::move(root)) {}

    // Creates the storage a round writes the next run to, in morton order.
    [[nodiscard]]
    alp::Result<frontier::Storage, io::FileError> create_merge_scratch(const uint32_t round) const {
        return frontier::Storage::create(this->get_round_path(round) / "merged");
    }

private:
    [[nodiscard]]
    std::filesystem::path get_round_path(const uint32_t round) const {
        return this->_root / fmt::format("round-{:04}", round);
    }

    std::filesystem::path _root;
};

} // namespace detail

} // namespace gated_ploc
