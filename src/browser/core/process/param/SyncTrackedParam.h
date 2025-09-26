#pragma once

#include "IParam.h"
#include "ITrackable.h"
#include <atomic>
#include <mutex>
#include <optional>

template <typename T>
class SyncTrackedParam : public IParam<T>, public ITrackable
{
public:
    SyncTrackedParam();

    void write(T t) override;
    constexpr const T &value() override;
    bool has_value() override;
    bool has_changed() override;
    void reset_changed() override;

private:
    std::optional<T> m_value;
    std::atomic_bool m_changed;

    std::mutex m_mutex;
};

template <typename T>
inline SyncTrackedParam<T>::SyncTrackedParam() : m_changed{false}
{
}

template <typename T>
inline void SyncTrackedParam<T>::write(T value)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_value == value)
    {
        return;
    }

    m_changed = true;
    m_value = value;
}

template <typename T>
inline constexpr const T &SyncTrackedParam<T>::value()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_value.value();
}

template <typename T>
inline bool SyncTrackedParam<T>::has_value()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_value.has_value();
}

template <typename T>
inline bool SyncTrackedParam<T>::has_changed()
{
    return m_changed;
}

template <typename T>
inline void SyncTrackedParam<T>::reset_changed()
{
    m_changed = false;
}
