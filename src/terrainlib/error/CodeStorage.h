#pragma once

#include <type_traits>

#include "error/ErasedCode.h"
#include "error/TypedCode.h"

namespace alp {

// How an error with the codes Es stores its code: a TypedCode, or an ErasedCode when Es is empty.
template <typename... Es>
using CodeStorage = std::conditional_t<sizeof...(Es) == 0, ErasedCode, TypedCode<Es...>>;

} // namespace alp
