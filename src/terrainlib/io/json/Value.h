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

#include <algorithm>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <optional>
#include <ranges>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <libassert/assert.hpp>

#include "containers/indirect.h"
#include "numeric/checked_cast.h"
#include "range_utils.h"
#include "variant_utils.h"

namespace io::json {

class Array;
class Object;

// A character type, which converts to neither a JSON number nor a JSON string.
template <typename T>
concept Character = std::same_as<T, char> || std::same_as<T, wchar_t> || std::same_as<T, char8_t> || std::same_as<T, char16_t> || std::same_as<T, char32_t>;

// A type that converts to a JSON boolean.
template <typename T>
concept Boolean = std::same_as<std::remove_cvref_t<T>, bool>;

// A type that converts to an integral JSON number.
template <typename T>
concept Integer = std::integral<std::remove_cvref_t<T>> && !Boolean<T> && !Character<std::remove_cvref_t<T>>;

// A type that converts to a JSON number.
template <typename T>
concept FloatingPoint = std::floating_point<std::remove_cvref_t<T>>;

// A type that converts to a JSON string.
template <typename T>
concept String = std::convertible_to<const T &, std::string_view> && !std::is_pointer_v<std::remove_cvref_t<T>>;

// The kind of a JSON value.
enum class Kind {
    Null,
    Boolean,
    Number,
    String,
    Array,
    Object,
};

// A JSON value: null, a boolean, a number, a string, an array or an object.
class Value {
public:
    // The JSON null.
    Value() = default;
    Value(std::nullptr_t);

    template <Boolean T>
    Value(T value);

    // Equal integers compare equal regardless of their C++ type.
    template <Integer T>
    Value(T value);

    // A non-finite `value` is held as is; to_string rejects it.
    template <FloatingPoint T>
    Value(T value);

    template <String T>
    Value(T &&value);

    Value(Array value);
    Value(Object value);

    Kind kind() const;

    std::optional<bool> as_bool() const;
    // The number, if it is an integer that fits in int64_t.
    std::optional<int64_t> as_int64() const;
    // The number, if it is an integer that fits in uint64_t.
    std::optional<uint64_t> as_uint64() const;
    // The number, converted to double if it is an integer.
    std::optional<double> as_double() const;
    std::optional<std::string_view> as_string() const;
    const Array *as_array() const;
    const Object *as_object() const;

    bool operator==(const Value &) const = default;

private:
    friend struct Serde;

    std::variant<std::nullptr_t, bool, int64_t, uint64_t, double, std::string, alp::indirect<Array>, alp::indirect<Object>> _value;
};

// A sequence of JSON values.
class Array {
public:
    Array() = default;
    Array(std::initializer_list<Value> values);

    void push_back(Value value);

    size_t size() const;
    bool empty() const;
    // `index` must be less than size().
    const Value &operator[](size_t index) const;
    std::vector<Value>::const_iterator begin() const;
    std::vector<Value>::const_iterator end() const;

    bool operator==(const Array &) const = default;

private:
    std::vector<Value> _values;
};

// A JSON object whose entries keep their insertion order.
class Object {
public:
    using key_type = std::string;
    using mapped_type = Value;
    using const_iterator = std::vector<std::pair<key_type, mapped_type>>::const_iterator;

    Object() = default;
    // A repeated key replaces the earlier value in place.
    Object(std::initializer_list<std::pair<std::string, Value>> entries);

    // Inserts `value` under `key`, or replaces the existing value in place. Returns whether `key` was new.
    bool insert_or_assign(std::string key, Value value);

    size_t size() const;
    bool empty() const;
    bool contains(std::string_view key) const;
    // The entry for `key`, or end().
    const_iterator find(std::string_view key) const;
    const_iterator begin() const;
    const_iterator end() const;

    bool operator==(const Object &) const = default;

private:
    std::vector<std::pair<std::string, Value>> _entries;
};

// The member bodies follow all three classes, since constructing, moving or destroying a Value needs Array and Object complete.

template <Boolean T>
Value::Value(T value)
    : _value(std::in_place_type<bool>, value) {
}

template <Integer T>
Value::Value(T value) {
    static_assert(sizeof(T) <= sizeof(uint64_t), "JSON integers have at most 64 bits");
    // A non-negative integer is always held as uint64_t, so that equal integers of different types compare equal.
    if constexpr (std::signed_integral<T>) {
        if (value < 0) {
            this->_value.emplace<int64_t>(value);
        } else {
            this->_value.emplace<uint64_t>(value);
        }
    } else {
        this->_value.emplace<uint64_t>(value);
    }
}

template <FloatingPoint T>
Value::Value(T value)
    : _value(std::in_place_type<double>, value) {
}

template <String T>
Value::Value(T &&value) {
    if constexpr (std::same_as<std::remove_cvref_t<T>, std::string>) {
        this->_value.emplace<std::string>(std::forward<T>(value));
    } else {
        const std::string_view view = value;
        this->_value.emplace<std::string>(view);
    }
}

inline Value::Value(std::nullptr_t) {
}

inline Value::Value(Array value)
    : _value(std::in_place_type<alp::indirect<Array>>, std::move(value)) {
}

inline Value::Value(Object value)
    : _value(std::in_place_type<alp::indirect<Object>>, std::move(value)) {
}

inline Kind Value::kind() const {
    return match(
        this->_value,
        [](std::nullptr_t) { return Kind::Null; },
        [](bool) { return Kind::Boolean; },
        [](int64_t) { return Kind::Number; },
        [](uint64_t) { return Kind::Number; },
        [](double) { return Kind::Number; },
        [](const std::string &) { return Kind::String; },
        [](const alp::indirect<Array> &) { return Kind::Array; },
        [](const alp::indirect<Object> &) { return Kind::Object; });
}

inline std::optional<bool> Value::as_bool() const {
    return match<std::optional<bool>>(
        this->_value,
        [](const bool value) { return value; },
        [](const auto &) { return std::nullopt; });
}

inline std::optional<int64_t> Value::as_int64() const {
    return match<std::optional<int64_t>>(
        this->_value,
        [](const int64_t value) { return value; },
        [](const uint64_t value) { return try_cast<int64_t>(value); },
        [](const auto &) { return std::nullopt; });
}

inline std::optional<uint64_t> Value::as_uint64() const {
    return match<std::optional<uint64_t>>(
        this->_value,
        [](const uint64_t value) { return value; },
        [](const int64_t value) { return try_cast<uint64_t>(value); },
        [](const auto &) { return std::nullopt; });
}

inline std::optional<double> Value::as_double() const {
    return match<std::optional<double>>(
        this->_value,
        [](const double value) { return value; },
        [](const int64_t value) { return static_cast<double>(value); },
        [](const uint64_t value) { return static_cast<double>(value); },
        [](const auto &) { return std::nullopt; });
}

inline std::optional<std::string_view> Value::as_string() const {
    return match<std::optional<std::string_view>>(
        this->_value,
        [](const std::string &value) { return std::string_view(value); },
        [](const auto &) { return std::nullopt; });
}

inline const Array *Value::as_array() const {
    return match<const Array *>(
        this->_value,
        [](const alp::indirect<Array> &value) { return &*value; },
        [](const auto &) { return nullptr; });
}

inline const Object *Value::as_object() const {
    return match<const Object *>(
        this->_value,
        [](const alp::indirect<Object> &value) { return &*value; },
        [](const auto &) { return nullptr; });
}

inline Array::Array(std::initializer_list<Value> values)
    : _values(values) {
}

inline void Array::push_back(Value value) {
    this->_values.push_back(std::move(value));
}

inline size_t Array::size() const {
    return this->_values.size();
}

inline bool Array::empty() const {
    return this->_values.empty();
}

inline const Value &Array::operator[](const size_t index) const {
    DEBUG_ASSERT(index < this->_values.size());
    return this->_values[index];
}

inline std::vector<Value>::const_iterator Array::begin() const {
    return this->_values.begin();
}

inline std::vector<Value>::const_iterator Array::end() const {
    return this->_values.end();
}

inline Object::Object(std::initializer_list<std::pair<std::string, Value>> entries) {
    for (const auto &[key, value] : entries) {
        this->insert_or_assign(key, value);
    }
}

inline bool Object::insert_or_assign(std::string key, Value value) {
    const auto entry = std::ranges::find(this->_entries, key, &std::pair<std::string, Value>::first);
    if (entry == this->_entries.end()) {
        this->_entries.emplace_back(std::move(key), std::move(value));
        return true;
    } else {
        entry->second = std::move(value);
        return false;
    }
}

inline size_t Object::size() const {
    return this->_entries.size();
}

inline bool Object::empty() const {
    return this->_entries.empty();
}

inline bool Object::contains(const std::string_view key) const {
    return ::contains(this->_entries | std::views::keys, key);
}

inline Object::const_iterator Object::find(const std::string_view key) const {
    return std::ranges::find(this->_entries, key, &std::pair<std::string, Value>::first);
}

inline Object::const_iterator Object::begin() const {
    return this->_entries.begin();
}

inline Object::const_iterator Object::end() const {
    return this->_entries.end();
}

} // namespace io::json
