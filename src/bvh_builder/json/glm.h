#pragma once

#include <array>
#include <span>
#include <tuple>

#include <glaze/json.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

// Declares glm vectors as JSON arrays of their components.
namespace glz {

template <glm::length_t n_dims, typename T, glm::qualifier Q>
struct from<JSON, glm::vec<n_dims, T, Q>> {
    template <auto Opts>
    static void op(glm::vec<n_dims, T, Q> &value, is_context auto &&context, auto &&it, auto &&end) {
        // Only a tuple can be required to be complete, an array may be cut short.
        decltype(std::tuple_cat(std::array<T, n_dims>{})) components;
        parse<JSON>::op<Opts>(components, context, it, end);
        value = std::make_from_tuple<glm::vec<n_dims, T, Q>>(components);
    }
};

template <glm::length_t n_dims, typename T, glm::qualifier Q>
struct to<JSON, glm::vec<n_dims, T, Q>> {
    template <auto Opts>
    static void op(const glm::vec<n_dims, T, Q> &value, is_context auto &&context, auto &&buffer, auto &&index) {
        const std::span<const T, n_dims> components(glm::value_ptr(value), n_dims);
        serialize<JSON>::op<Opts>(components, context, buffer, index);
    }
};

}
