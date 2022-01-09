#ifndef LAYERJSONWRITER_H
#define LAYERJSONWRITER_H

#include <string>

#include "Tiler.h"

class MetaDataGenerator;
namespace ctb {
class Grid;
}

namespace layer_json_writer {
[[nodiscard]] std::string process(const MetaDataGenerator&);

namespace internal {
[[nodiscard]] std::string tilejson(const std::string& s);
[[nodiscard]] std::string name(const std::string& s);
[[nodiscard]] std::string description(const std::string& s);
[[nodiscard]] std::string version(const std::string& s);
[[nodiscard]] std::string format();

[[nodiscard]] std::string attribution(const std::string& s);
[[nodiscard]] std::string schema(const Tiler::Scheme& version);
[[nodiscard]] std::string tiles();
[[nodiscard]] std::string projection(int epsg_number);
[[nodiscard]] std::string bounds(const ctb::Grid& grid);

[[nodiscard]] std::string zoom_layer(const ctb::TileBounds& bounds);

}
}

#endif // LAYERJSONWRITER_H
