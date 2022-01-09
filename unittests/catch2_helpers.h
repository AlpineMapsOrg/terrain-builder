#ifndef CATCH2_HELPERS_H
#define CATCH2_HELPERS_H

#include <catch2/catch.hpp>

#include <glm/gtx/string_cast.hpp>

namespace Catch {

//template<>
template<glm::length_t s,  typename T>
struct StringMaker<glm::vec<s, T>> {
  static std::string convert(const glm::vec<s, T>& value) {
    return glm::to_string(value);
  }
};
}

#endif // CATCH2_HELPERS_H
