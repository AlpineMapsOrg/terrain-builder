#include <fstream>

#include <glm/glm.hpp>

#include "alpine_raster.h"
#include "cesium_tin_terra.h"
#include "ctb/Grid.hpp"
#include "Tiler.h"
#include "MetaDataGenerator.h"
#include "layer_json_writer.h"

int main() {
//  const std::string input_raster = "/home/madam/rajaton/raw/Oe_2020/OeRect_01m_gs_31287.img";
//  const std::string output_path = "/home/madam/rajaton/tiles/atb_terrain/";
////  const auto generator = alpine_raster::make_generator("./test_tiles/", "/home/madam/valtava/raw/Oe_2020/OeRect_01m_gs_31287.img", ctb::Grid::Srs::SphericalMercator, Tiler::Scheme::SlippyMap, Tiler::Border::No);
////  generator.process({16, 16});
//  const auto generator = cesium_tin_terra::make_generator(input_raster, output_path, ctb::Grid::Srs::WGS84, Tiler::Scheme::Tms, Tiler::Border::Yes);
//  generator.process({0, 5}, true, true);
//  generator.process({6, 16}, true, false);

//  const auto metadata = MetaDataGenerator::make(input_raster, ctb::Grid::Srs::WGS84, Tiler::Scheme::Tms);
//  const auto json = layer_json_writer::process(metadata);

  // todo: refactor, make more similar to terrain generator, whole world / part differentiation, tests + fix json
//  std::ofstream out(output_path + "layer.json");
//  out << json;
//  out.close();

    const std::string input_raster = "/home/madam/rajaton/raw/Oe_2020/OeRect_01m_gs_31287.img";
    const std::string output_path = "/home/madam/rajaton/tiles/alpine_png/";
    //  const auto generator = alpine_raster::make_generator("./test_tiles/", "/home/madam/valtava/raw/Oe_2020/OeRect_01m_gs_31287.img", ctb::Grid::Srs::SphericalMercator, Tiler::Scheme::SlippyMap, Tiler::Border::No);
    //  generator.process({16, 16});
    const auto generator = alpine_raster::make_generator(input_raster, output_path, ctb::Grid::Srs::SphericalMercator, Tiler::Scheme::Tms, Tiler::Border::Yes, 64);
//    generator.process({0, 5}, true, true);
    generator.process({17, 20}, true, false);

//     const auto metadata = MetaDataGenerator::make(input_raster, ctb::Grid::Srs::WGS84, Tiler::Scheme::Tms);
//     const auto json = layer_json_writer::process(metadata);

  return 0;
}
