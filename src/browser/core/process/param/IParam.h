#pragma once

template <typename T>
class IParam
{
    virtual void write(T t) = 0;
    virtual constexpr const T &value() = 0;
    virtual constexpr bool has_value() = 0;
};