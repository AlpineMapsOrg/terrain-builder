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

#include <atomic>
#include <cstdint>
#include <expected>
#include <type_traits>
#include <utility>

#include "containers/UniqueResource.h"

// A value that can be set once, by whichever thread gets there first.
template <typename T>
class OnceCell {
    static_assert(std::is_default_constructible_v<T>);
    static_assert(std::is_move_assignable_v<T>);

public:
    // Stores the value if the cell is still empty, and reports whether it did.
    bool set(T value) {
        Claim claim = this->try_claim();
        if (claim.is_empty()) {
            return false;
        }

        this->_value = std::move(value);
        this->publish(std::move(claim));
        return true;
    }

    // The stored value, or nullptr while the cell is empty. Never blocks, so it also
    // reports nothing while another thread is filling the cell.
    [[nodiscard]]
    const T *get() const {
        if (this->_slot.load(std::memory_order_acquire) != Slot::Ready) {
            return nullptr;
        }
        return &this->_value;
    }

    [[nodiscard]]
    bool is_set() const {
        return this->get() != nullptr;
    }

    // The stored value, filling the cell with make first if it is still empty. Blocks
    // while another thread is filling it, so make runs at most once for the cell.
    template <typename Make>
    const T &get_or_init(Make &&make) {
        Claim claim = this->try_claim();
        while (claim.is_empty()) {
            if (const T *held = this->get(); held != nullptr) {
                return *held;
            }

            this->_slot.wait(Slot::Writing, std::memory_order_relaxed);
            claim = this->try_claim();
        }

        this->_value = std::forward<Make>(make)();
        this->publish(std::move(claim));
        return this->_value;
    }

    // The stored value, filling the cell with make first if it is still empty. A make that
    // fails or throws leaves the cell empty, so the next caller tries again.
    template <typename Make>
    auto get_or_try_init(Make &&make) {
        // A reference cannot be the value of a std::expected, which stores it in a union.
        using Result = std::expected<const T *, typename std::invoke_result_t<Make>::error_type>;

        Claim claim = this->try_claim();
        while (claim.is_empty()) {
            if (const T *held = this->get(); held != nullptr) {
                return Result(held);
            }

            this->_slot.wait(Slot::Writing, std::memory_order_relaxed);
            claim = this->try_claim();
        }

        auto made = std::forward<Make>(make)();
        if (!made.has_value()) {
            return Result(std::unexpect, std::move(made.error()));
        }

        this->_value = std::move(made.value());
        this->publish(std::move(claim));
        return Result(&this->_value);
    }

    // Blocks until some thread has set the cell, and returns what it holds.
    const T &wait() const {
        Slot observed = this->_slot.load(std::memory_order_acquire);
        while (observed != Slot::Ready) {
            this->_slot.wait(observed, std::memory_order_relaxed);
            observed = this->_slot.load(std::memory_order_acquire);
        }
        return this->_value;
    }

private:
    // Writing is a state of its own so that a reader cannot see a value that is still
    // being assigned by the thread that won the cell.
    enum class Slot : uint8_t { Empty, Writing, Ready };

    // Gives the cell back if a claim is dropped without being filled.
    struct Abandon {
        void operator()(OnceCell *&cell) const noexcept {
            cell->_slot.store(Slot::Empty, std::memory_order_release);
            cell->_slot.notify_all();
        }
    };

    using Claim = UniquePointer<OnceCell, Abandon>;

    // Takes the cell for writing. The claim is empty if someone else got there first.
    Claim try_claim() {
        Slot empty = Slot::Empty;
        const bool won = this->_slot.compare_exchange_strong(
            empty, Slot::Writing, std::memory_order_relaxed, std::memory_order_relaxed);
        if (!won) {
            return Claim();
        }
        return Claim(this);
    }

    // Hands out the value written under the claim, spending it in the process.
    void publish(Claim claim) {
        claim.take();
        this->_slot.store(Slot::Ready, std::memory_order_release);
        this->_slot.notify_all();
    }

    std::atomic<Slot> _slot = Slot::Empty;
    T _value{};
};
