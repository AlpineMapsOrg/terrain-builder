#ifndef IMAGE_H
#define IMAGE_H

#include <algorithm>
#include <bits/ranges_algo.h>
#include <cassert>
#include <glm/fwd.hpp>
#include <vector>

#include <glm/glm.hpp>

template <typename T>
class Image
{
public:
  Image() = default;
  Image(unsigned width, unsigned height) : m_width(width), m_height(height), m_data(size_t(m_width * m_height)) {}

  [[nodiscard]] unsigned width() const { return m_width; }
  [[nodiscard]] unsigned height() const { return m_height; }
  [[nodiscard]] T pixel(unsigned row, unsigned column) const {
    assert(column < m_width);
    assert(row < m_height);
    assert(m_data.size() == size_t(m_width * m_height));
    return m_data[row * m_width + column];
  }

  [[nodiscard]] float* data() { return m_data.data(); }

  [[nodiscard]] auto size() const { return m_data.size(); }
  [[nodiscard]] auto begin() { return m_data.begin(); }
  [[nodiscard]] auto end() { return m_data.end(); }
  [[nodiscard]] auto begin() const { return m_data.begin(); }
  [[nodiscard]] auto end() const { return m_data.end(); }


private:
  unsigned m_width = 0;
  unsigned m_height = 0;
  std::vector<T> m_data;
};

using HeightData = Image<float>;
using uchar = unsigned char;

namespace image {
void saveImageAsPng(const Image<glm::u8vec3>& image, const std::string& path);

template <typename T1, typename Fun>
[[nodiscard]] auto transformImage(const Image<T1>& i, Fun conversion_fun) -> Image<decltype(conversion_fun(*i.begin()))> {
  using T2 = decltype(conversion_fun(*i.begin()));
  Image<T2> i2(i.width(), i.height());
  std::transform(i.begin(), i.end(), i2.begin(), conversion_fun);
  return i2;
}

template <typename T>
void debugOut(const Image<T>& image, const std::string& path) {
  auto [min, max] = std::ranges::minmax(image);

  // [min=min, ..] is required for cpp correctness. min/max from the capture are not variables, we need to copy them:
  // https://stackoverflow.com/questions/50799719/reference-to-local-binding-declared-in-enclosing-function?noredirect=1&lq=1
  saveImageAsPng(transformImage(image, [min=min, max=max](auto v) {
                   const auto c = uchar(255.F * (float(v) - float(min)) / float(max-min));
                   return  glm::u8vec3(c, c, c);
                 }), path);
}
}

#endif // HEIGHTDATA_H
