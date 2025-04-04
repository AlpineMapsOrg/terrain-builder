#pragma once

#include <algorithm>
#include <functional>
#include <glm/detail/qualifier.hpp>
#include <string>
#include <vector>

#include <glm/glm.hpp>

namespace tntn {

template <typename T, glm::qualifier P>
glm::tvec2<T, P> xy(const glm::tvec3<T, P>& v)
{
    return { v.x, v.y };
}

template <typename T>
struct SimpleRange {
    typedef T iterator_type;
    T begin;
    T end;

    size_t size() const { return std::distance(begin, end); }

    template <typename CallableT>
    void for_each(CallableT&& c) const
    {
        for (auto p = begin; p != end; ++p) {
            c(*p);
        }
    }
    template <typename CallableT>
    void for_each(CallableT&& c)
    {
        for (auto p = begin; p != end; ++p) {
            c(*p);
        }
    }

    template <typename ContainerT>
    void copy_into(ContainerT& c) const
    {
        c.reserve(size());
        std::copy(begin, end, std::back_inserter(c));
    }
};

template <typename T>
inline void hash_combine(size_t& seed, const T& t) noexcept
{
    seed ^= std::hash<T>()(t) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

void tokenize(const std::string& s,
    std::vector<std::string>& out_tokens,
    const char* delimiters = nullptr);
void tokenize(const char* s,
    const size_t s_size,
    std::vector<std::string>& out_tokens,
    const char* delimiters = nullptr);
void tokenize(const char* s,
    std::vector<std::string>& out_tokens,
    const char* delimiters = nullptr);

} // namespace tntn
