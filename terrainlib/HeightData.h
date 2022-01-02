#ifndef HEIGHTDATA_H
#define HEIGHTDATA_H

#include <cassert>
#include <vector>

class HeightData
{
public:
  HeightData();
  HeightData(unsigned width, unsigned height) : m_width(width), m_height(height), m_data(width * height) {}

  unsigned width() const { return m_width; }
  unsigned height() const { return m_height; }
  float pixel(unsigned column, unsigned row) const {
    assert(column < m_width);
    assert(row < m_height);
    assert(m_data.size() == m_width * m_height);
    return m_data[row * m_width + column];
  }


private:
  unsigned m_width = 0;
  unsigned m_height = 0;
  std::vector<float> m_data;
};

#endif // HEIGHTDATA_H
