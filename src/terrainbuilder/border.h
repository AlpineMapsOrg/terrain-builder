#ifndef BORDER_H
#define BORDER_H

#include <ostream>

#include <radix/geometry.h>

namespace terrainbuilder {

template <typename T>
class Border {
public:
    T top;
    T right;
    T bottom;
    T left;

    Border()
        : Border(0) {}
    Border(T uniform)
        : Border(uniform, uniform, uniform, uniform) {}
    Border(T horizontal, T vertical)
        : Border(vertical, horizontal, vertical, horizontal) {}
    Border(T top, T right, T bottom, T left)
        : top(top), right(right), bottom(bottom), left(left) {}

    bool is_empty() const {
        return this->top == 0 && this->right == 0 && this->bottom == 0 && this->left == 0;
    }

    bool operator==(const Border<T> &other) const {
        return top == other.top && right == other.right && bottom == other.bottom && left == other.left;
    }

    bool operator!=(const Border<T> &other) const {
        return !(*this == other);
    }

    friend std::ostream &operator<<(std::ostream &os, const Border<T> &border) {
        os << "Top: " << border.top << ", Right: " << border.right << ", Bottom: " << border.bottom << ", Left: " << border.left;
        return os;
    }
};

template <typename T1, typename T2>
inline void add_border_to_aabb(geometry::Aabb2<T1> &bounds, const Border<T2> &border) {
    bounds.min.x -= border.left;
    bounds.min.y -= border.top;
    bounds.max.x += border.right;
    bounds.max.y += border.bottom;
}
    
}

#endif
