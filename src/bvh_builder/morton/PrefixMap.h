#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>
#include <vector>

#include "code.h"

namespace morton {

// A dense table indexed by the leading bits of a morton code.
template <typename T>
class PrefixMap {
public:
    explicit PrefixMap(const uint32_t prefix_bits, T initial = {})
        : _cells(size_t{1} << prefix_bits, std::move(initial)), _prefix_bits(prefix_bits) {}

    const T &at(const Code &morton) const {
        return this->_cells[this->cell_of(morton)];
    }
    T &at(const Code &morton) {
        return this->_cells[this->cell_of(morton)];
    }

    std::span<const T> cells() const {
        return this->_cells;
    }
    std::span<T> cells() {
        return this->_cells;
    }

    uint32_t prefix_bits() const {
        return this->_prefix_bits;
    }

private:
    size_t cell_of(const Code &morton) const {
        return static_cast<size_t>(morton >> (CODE_BITS - this->_prefix_bits));
    }

    std::vector<T> _cells;
    uint32_t _prefix_bits;
};

}