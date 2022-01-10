#include "layer_json_writer.h"

#include <fmt/core.h>
#include <string>

#include "Exception.h"
#include "MetaDataGenerator.h"

namespace {
std::string produce_string_attribute(const std::string& name, const std::string& s) {
  return fmt::format("\"{}\": \"{}\",", name, s);
}
}

std::string layer_json_writer::internal::tilejson(const std::string& version)
{
  return produce_string_attribute("tilejson", version);
}

std::string layer_json_writer::internal::name(const std::string& s)
{
  return produce_string_attribute("name", s);
}

std::string layer_json_writer::internal::description(const std::string& s)
{
  return produce_string_attribute("description", s);
}

std::string layer_json_writer::internal::version(const std::string& s)
{
  return produce_string_attribute("version", s);
}

std::string layer_json_writer::internal::format()
{
  return produce_string_attribute("format", "quantized-mesh-1.0");
}


std::string layer_json_writer::internal::attribution(const std::string& s)
{
  return produce_string_attribute("attribution", s);
}

std::string layer_json_writer::internal::schema(const Tiler::Scheme& version)
{
  switch (version) {
  case Tiler::Scheme::SlippyMap:
    return produce_string_attribute("schema", "slippyMap");
  case Tiler::Scheme::Tms:
    return produce_string_attribute("schema", "tms");
  }
  throw Exception("Not implemented!");
}

std::string layer_json_writer::internal::tiles()
{
  return std::string(R"("tiles": [ "{z}/{x}/{y}.terrain?v={version}" ],)");
}

std::string layer_json_writer::internal::projection(int epsg_number)
{
  return produce_string_attribute("projection", fmt::format("EPSG:{}", epsg_number));
}

std::string layer_json_writer::internal::bounds(const ctb::Grid& grid)
{
  const auto bounds = grid.getExtent();
  return fmt::format("\"bounds\": [{:.2f}, {:.2f}, {:.2f}, {:.2f}],", bounds.getMinX(), bounds.getMinY(), bounds.getMaxX(), bounds.getMaxY());
}

std::string layer_json_writer::internal::zoom_layer(const ctb::TileBounds& bounds)
{
  return fmt::format(R"([ {{ "startX": {}, "startY": {}, "endX": {}, "endY": {} }} ],)", bounds.getMinX(), bounds.getMinY(), bounds.getMaxX(), bounds.getMaxY());
}

std::string layer_json_writer::process(const MetaDataGenerator& m)
{
  std::string s = "{\n";
  s += internal::tilejson("2.1.0") + "\n";
  s += internal::name(m.dataset()->name()) + "\n";
  s += internal::description("") + "\n";
  s += internal::version("1.1.0") + "\n";
  s += internal::format() + "\n";
  s += internal::attribution("") + "\n";
  s += internal::schema(m.tiler().scheme()) + "\n";
  s += internal::tiles() + "\n";
  s += internal::projection(m.grid().getEpsgCode()) + "\n";
  s += internal::bounds(m.grid()) + "\n";
  s += R"("available": [)";
  s += "\n";
  for (const auto& z : m.availableTiles()) {
    s += internal::zoom_layer(z) + "\n";
  }
  s += R"(]})" "\n";
  return s;
}
