#include <filesystem>
#include <iostream>
#include <iterator>
#include <optional>
#include <fstream>
#include <vector>

#include <fmt/core.h>
#include <gdal.h>
#include <gdal_priv.h>
#include <glm/glm.hpp>
#include <radix/geometry.h>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include "ParallelTiler.h"
#include "TileHeightsGenerator.h"
#include "alpine_raster.h"
#include "ctb/Grid.hpp"
#include "Dataset.h"
#include "DatasetReader.h"
#include "TopDownTiler.h"
#include "ctb/GlobalGeodetic.hpp"
#include "ctb/GlobalMercator.hpp"
#include "srs.h"

#include "terrain_mesh.h"
#include "mesh_io.h"

tile::SrsBounds encompassing_bounding_box_transfer(const OGRSpatialReference &source_srs, const OGRSpatialReference &target_srs, const tile::SrsBounds &source_bounds) {
    if (source_srs.IsSame(&target_srs)) {
        return source_bounds;
    }

    const std::unique_ptr<OGRCoordinateTransformation> transformation = srs::transformation(source_srs, target_srs);
    tile::SrsBounds target_bounds;
    const int result = transformation->TransformBounds(
        source_bounds.min.x, source_bounds.min.y, source_bounds.max.x, source_bounds.max.y,
        &target_bounds.min.x, &target_bounds.min.y, &target_bounds.max.x, &target_bounds.max.y,
        21);
    if (result != TRUE) {
        throw std::runtime_error("encompassing_bounding_box_transfer");
    }
    return target_bounds;
}

void print_bounds(const tile::SrsBounds &bounds) {
    fmt::print("{} {} {} {}\n", bounds.min.x, bounds.min.y, bounds.size().x, bounds.size().y);
}
void print_bounds(const geometry::Aabb2i &bounds) {
    fmt::print("{} {} {} {}\n", bounds.min.x, bounds.min.y, bounds.size().x, bounds.size().y);
}
void print_point(const glm::dvec2 &point) {
    fmt::print("{} {}\n", point.x, point.y);
}
void print_point(const glm::ivec2 &point) {
    fmt::print("{} {}\n", point.x, point.y);
}
glm::dvec2 apply_transform(std::array<double, 6> transform, const glm::dvec2 &v) {
    glm::dvec2 result;
    GDALApplyGeoTransform(transform.data(), v.x, v.y, &result.x, &result.y);
    return result;
}
glm::dvec2 apply_transform(std::array<double, 6> transform, const glm::ivec2 &v) {
    glm::dvec2 result;
    GDALApplyGeoTransform(transform.data(), static_cast<double>(v.x), static_cast<double>(v.y), &result.x, &result.y);
    return result;
}
glm::dvec2 apply_transform(OGRCoordinateTransformation *transform, const glm::dvec2 &v) {
    glm::dvec2 result(v);
    if (!transform->Transform(1, &result.x, &result.y))
        throw Exception("apply_transform() failed");
    return result;
}
glm::dvec3 apply_transform(OGRCoordinateTransformation *transform, const glm::dvec3 &v) {
    glm::dvec3 result(v);
    if (!transform->Transform(1, &result.x, &result.y, &result.z))
        throw Exception("apply_transform() failed");
    return result;
}
class RawDatasetReader {
public:
    RawDatasetReader(Dataset &target_dataset)
        : RawDatasetReader(target_dataset.gdalDataset()) {}
    RawDatasetReader(GDALDataset *target_dataset)
        : dataset(target_dataset) {
        if (!dataset) {
            throw std::invalid_argument("Invalid GDAL dataset provided");
        }

        if (dataset->GetGeoTransform(this->geo_transform.data()) != CE_None) {
            throw std::runtime_error("Failed to retrieve the GeoTransform");
        }

        if (GDALInvGeoTransform(this->geo_transform.data(), this->inv_geo_transform.data()) != TRUE) {
            throw std::runtime_error("Failed to invert the GeoTransform");
        }
    }

    GDALDataset *gdal_dataset() {
        return this->dataset;
    }

    HeightData read_data_in_pixel_bounds(const geometry::Aabb2i &bounds) {
        GDALRasterBand *heights_band = this->dataset->GetRasterBand(1); // non-owning pointer

        // Initialize the HeightData for reading
        // TODO: +1?
        HeightData height_data(bounds.width(), bounds.height());

        // Read data from the heights band into heights_data
        const int read_result = heights_band->RasterIO(
            GF_Read, bounds.min.x, bounds.min.y, bounds.width(), bounds.height(),
            static_cast<void *>(height_data.data()), bounds.width(), bounds.height(), GDT_Float32, 0, 0);

        if (read_result != CE_None) {
            throw std::runtime_error("Failed to read data");
        }

        return height_data;
    }

    HeightData read_data_in_srs_bounds(const tile::SrsBounds &bounds) {
        // Transform the SrsBounds to pixel space
        const geometry::Aabb2i pixel_bounds = this->transform_srs_bounds_to_pixel_bounds(bounds);

        // Use the transformed pixel bounds to read data
        return this->read_data_in_pixel_bounds(pixel_bounds);
    }

    geometry::Aabb2i transform_srs_bounds_to_pixel_bounds(const tile::SrsBounds &bounds) const {
        geometry::Aabb2d pixel_bounds_exact = this->transform_srs_bounds_to_pixel_bounds_exact(bounds);

        const geometry::Aabb2i pixel_bounds(
            glm::ivec2(static_cast<int>(std::floor(pixel_bounds_exact.min.x)), static_cast<int>(std::floor(pixel_bounds_exact.min.y))),
            glm::ivec2(static_cast<int>(std::ceil(pixel_bounds_exact.max.x)), static_cast<int>(std::ceil(pixel_bounds_exact.max.y))));
        
        return pixel_bounds;
    }

    tile::SrsBounds transform_srs_bounds_to_pixel_bounds_exact(const tile::SrsBounds &bounds) const {
        tile::SrsBounds transformed_bounds;
        transformed_bounds.min = this->transform_srs_point_to_pixel_exact(bounds.min);
        transformed_bounds.max = this->transform_srs_point_to_pixel_exact(bounds.max);
        transformed_bounds = tile::SrsBounds(glm::min(transformed_bounds.min, transformed_bounds.max),
                                             glm::max(transformed_bounds.min, transformed_bounds.max));
        return transformed_bounds;
    }

    tile::SrsBounds transform_pixel_bounds_to_srs_bounds(const tile::SrsBounds &bounds) const {
        tile::SrsBounds transformed_bounds;
        transformed_bounds.min = this->transform_pixel_to_srs_point(bounds.min);
        transformed_bounds.max = this->transform_pixel_to_srs_point(bounds.max);
        transformed_bounds = tile::SrsBounds(glm::min(transformed_bounds.min, transformed_bounds.max),
                                             glm::max(transformed_bounds.min, transformed_bounds.max));
        return transformed_bounds;
    }

    template <typename T>
    inline glm::ivec2 transform_srs_point_to_pixel(const glm::tvec2<T> &p) const {
        glm::dvec2 pixel_exact = this->transform_srs_point_to_pixel_exact(p);
        const glm::ivec2 pixel(std::rint(pixel_exact.x), std::rint(pixel_exact.y));
        return pixel;
    }

    template <typename T>
    inline glm::dvec2 transform_srs_point_to_pixel_exact(const glm::tvec2<T> &p) const {
        glm::dvec2 result;
        GDALApplyGeoTransform(this->inv_geo_transform.data(), p.x, p.y, &result.x, &result.y);
        return result;
    }

    template <typename T>
    inline glm::dvec2 transform_pixel_to_srs_point(const glm::tvec2<T> &p) const {
        glm::dvec2 result;
        GDALApplyGeoTransform(this->geo_transform.data(), p.x, p.y, &result.x, &result.y);
        return result;
    }

private:
    GDALDataset *dataset;
    mutable std::array<double, 6> geo_transform;     // transform from pixel space to source srs.
    mutable std::array<double, 6> inv_geo_transform; // transform from source srs to pixel space.
};

TerrainMesh build_reference_mesh_tile(
    Dataset & dataset,
    const OGRSpatialReference &mesh_srs,
    const OGRSpatialReference &tile_srs, const tile::SrsBounds &tile_bounds,
    const OGRSpatialReference &texture_srs, const tile::SrsBounds &texture_bounds) {
    const OGRSpatialReference &source_srs = dataset.srs();

    // Translate tile bounds from tile srs into the source srs, so we know what data to read.
    const tile::SrsBounds tile_bounds_in_source_srs = encompassing_bounding_box_transfer(tile_srs, source_srs, tile_bounds);

    // Read height data according to bounds directly from dataset (no interpolation).
    RawDatasetReader reader(dataset);
    const geometry::Aabb2i pixel_bounds = reader.transform_srs_bounds_to_pixel_bounds(tile_bounds_in_source_srs);
    const HeightData raw_tile_data = reader.read_data_in_pixel_bounds(pixel_bounds);

    // Allocate mesh data structure
    const unsigned int point_count = raw_tile_data.width() * raw_tile_data.height();
    const unsigned int triangle_count = (raw_tile_data.width() - 1) * (raw_tile_data.height() - 1) * 2;
    TerrainMesh mesh;
    mesh.positions.reserve(point_count);
    mesh.uvs.reserve(point_count);
    mesh.triangles.reserve(triangle_count);

    // Prepare transformations here to avoid io inside the loop.
    const std::unique_ptr<OGRCoordinateTransformation> transform_source_tile = srs::transformation(source_srs, tile_srs);
    const std::unique_ptr<OGRCoordinateTransformation> transform_source_mesh = srs::transformation(source_srs, mesh_srs);
    const std::unique_ptr<OGRCoordinateTransformation> transform_source_texture = srs::transformation(source_srs, texture_srs);

    for (unsigned int j = 0; j < raw_tile_data.height(); j++) {
        for (unsigned int i = 0; i < raw_tile_data.width(); i++) {
            const double height = raw_tile_data.pixel(i, j);
            // const glm::dvec2 coords_raster_relative(i + 0.5, j + 0.5); // TODO
            const glm::dvec2 coords_raster_relative(i, j); // TODO
            const glm::dvec2 coords_raster_absolute = coords_raster_relative + glm::dvec2(pixel_bounds.min);
            const glm::dvec3 coords_source(reader.transform_pixel_to_srs_point(coords_raster_absolute), height);

            const glm::dvec3 coords_mesh = apply_transform(transform_source_mesh.get(), coords_source);
            // const glm::dvec2 coords_tile = apply_transform(transform_source_tile.get(), glm::dvec2(coords_source));

            const glm::dvec2 coords_texture_absolute = apply_transform(transform_source_texture.get(), coords_source);
            const glm::dvec2 coords_texture_relative = (coords_texture_absolute - texture_bounds.min) / texture_bounds.size();

            const glm::ivec2 coords_center_raster_relative(i + 1, j + 1);
            const glm::ivec2 coords_center_raster_absolute = coords_center_raster_relative + pixel_bounds.min;
            const glm::dvec2 coords_center_source = reader.transform_pixel_to_srs_point(coords_center_raster_absolute);
            const glm::dvec2 coords_center_tile = apply_transform(transform_source_tile.get(), coords_center_source);

            // Current Vertex
            mesh.positions.push_back(coords_mesh);
            mesh.uvs.push_back(coords_texture_relative);

            if (i >= raw_tile_data.width() - 1 || j >= raw_tile_data.width() - 1) { 
                // Skip last row and column because they don't form new triangles
                continue;
            }

            if (!tile_bounds.contains(coords_center_tile)) {
                // Skip triangles out of bounds
                continue;
            }

            // Add triangles
            const unsigned int vertex_index = i * raw_tile_data.width() + j;
            mesh.triangles.emplace_back(vertex_index, vertex_index + raw_tile_data.width(), vertex_index + raw_tile_data.width() + 1);
            mesh.triangles.emplace_back(vertex_index, vertex_index + raw_tile_data.width() + 1, vertex_index + 1);
        }
    }

    assert(mesh.positions.size() == point_count);
    assert(mesh.uvs.size() == point_count);
    assert(mesh.triangles.size() <= triangle_count);

    // TODO: remove extra vertices

    return mesh;
}



    int main() {
        // Read MGI Dataset
        const DatasetPtr dataset = Dataset::make_shared("/mnt/c/Users/Admin/Downloads/innenstadt_gs_1m_mgi.tif");
        // const DatasetPtr dataset = Dataset::make_shared("/mnt/e/Code/TU/2023S/Project/terrain-builder/unittests/data/austria/vienna_20m_mgi.tif");
        // const DatasetPtr dataset = Dataset::make_shared("/mnt/e/Code/TU/2023S/Project/terrain-builder/unittests/data/austria/at_100m_mgi.tif");

        // Convert dataset bounds into google3857
        OGRSpatialReference webmercator;
        webmercator.importFromEPSG(3857);
        webmercator.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

        OGRSpatialReference wgs84;
        wgs84.importFromEPSG(4326);
        wgs84.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

        OGRSpatialReference ecef;
        ecef.importFromEPSG(4978);
        ecef.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

        // https://mapsneu.wien.gv.at/basemap/bmaporthofoto30cm/normal/google3857/18/90897/142994.jpeg
        const ctb::Grid grid = ctb::GlobalMercator();
        const tile::Id root_tile = {18, {142994, 90897}, tile::Scheme::SlippyMap};

        // const tile::SrsBounds original_bounds = raw_dataset->bounds();
        // const tile::SrsBounds target_bounds = encompassing_bounding_box_transfer(raw_dataset->srs(), webmercator, original_bounds);
        // TODO: automatically deduce tile from bounds.
        // const tile::Id root_tile = {16, {35748, 22724}, tile::Scheme::SlippyMap};
        // const tile::Id root_tile = {17, {71497, 45448}, tile::Scheme::SlippyMap};
        // const tile::Id root_tile = {19, {285989, 181795}, tile::Scheme::SlippyMap};

        // Translate tile bounds into MGI
        const tile::SrsBounds tile_bounds = grid.srsBounds(root_tile, false);

        /*
        int width, height, channels;
        unsigned char *image_data = stbi_load("/mnt/c/Users/Admin/Downloads/142994.jpeg", &width, &height, &channels, 0);
        RgbImage texture(width, height);
        if (image_data) {
            if (channels != 3) {
                throw std::runtime_error("Image not RGB");
            }

            memcpy(texture.data(), image_data, width * height * channels);
            stbi_image_free(image_data);
        } else {
            throw std::runtime_error(fmt::format("Error loading image: {}", stbi_failure_reason()));
        }
        */

        const TerrainMesh mesh = build_reference_mesh_tile(
            *dataset.get(),
            ecef,
            grid.getSRS(), tile_bounds,
            grid.getSRS(), tile_bounds);

        std::ifstream texture_input("/mnt/c/Users/Admin/Downloads/142994.jpeg", std::ios::binary);
        // std::ifstream texture_input("/mnt/c/Users/Admin/Downloads/testTexture.png", std::ios::binary);

        std::vector<unsigned char> texture_bytes(
            (std::istreambuf_iterator<char>(texture_input)),
            (std::istreambuf_iterator<char>()));

        texture_input.close();
        save_mesh_as_gltf(mesh, texture_bytes);

        return 0;
    }
