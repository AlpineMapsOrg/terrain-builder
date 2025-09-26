#pragma once

#include <mutex>
#include <optional>
#include <queue>

template <typename T>
class SafeQueue
{
public:
    SafeQueue();

    void push(T element);
    T pop();

    bool empty();

private:
    std::mutex m_mutex;
    std::queue<T> m_queue;
};

template <typename T>
inline SafeQueue<T>::SafeQueue()
{
}

template <typename T>
void SafeQueue<T>::push(T element)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_queue.push(element);
}

template <typename T>
T SafeQueue<T>::pop()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    T element = m_queue.front();
    m_queue.pop();

    return element;
}

template <typename T>
bool SafeQueue<T>::empty()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    return m_queue.empty();
}
