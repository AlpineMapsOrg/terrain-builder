#pragma once

#include <functional>
#include <queue>
#include <vector>

// A priority queue whose top is its smallest element.
template <typename T>
using MinHeap = std::priority_queue<T, std::vector<T>, std::greater<>>;

// A priority queue whose top is its largest element.
template <typename T>
using MaxHeap = std::priority_queue<T, std::vector<T>, std::less<>>;
