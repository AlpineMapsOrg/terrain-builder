#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <optional>
#include <vector>

#include <FreeImage.h>
#include <fmt/core.h>
#include <gdal.h>
#include <gdal_priv.h>
#include <glm/glm.hpp>
#include <radix/geometry.h>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include "Dataset.h"
#include "DatasetReader.h"
#include "ParallelTiler.h"
#include "TileHeightsGenerator.h"
#include "TopDownTiler.h"
#include "alpine_raster.h"
#include "ctb/GlobalGeodetic.hpp"
#include "ctb/GlobalMercator.hpp"
#include "ctb/Grid.hpp"
#include "srs.h"

#include "mesh_io.h"
#include "terrain_mesh.h"

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
template <typename T>
void print_point(const glm::tvec2<T> &point) {
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
    Dataset &dataset,
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

// Searches for the smallest webmercator tile that encompasses the given bounding box.
tile::Id bounds_to_tile_id(const tile::SrsBounds &bounds) {
    // We dont want to recurse indefinetely if the bounds are empty.
    if (bounds.width() == 0 || bounds.height() == 0) {
        throw std::invalid_argument("bounds cannot be empty");
    }

    const ctb::Grid grid = ctb::GlobalMercator();
    const std::array<glm::dvec2, 4> points = {
        bounds.min,
        bounds.max,
        glm::dvec2(bounds.min.x, bounds.max.y),
        glm::dvec2(bounds.max.x, bounds.min.y)};

    // We start at the root tile and repeatedly check every subtile until we find one that contains all
    // of the bounding box.
    tile::Id current_largest_encompassing_tile = {0, {0, 0}, tile::Scheme::Tms};
    while (true) {
        const std::array<tile::Id, 4> children = current_largest_encompassing_tile.children();

        bool all_points_inside_any_child = false;
        for (const auto &child : children) {
            // Get the bounds of the current child
            const tile::SrsBounds tile_bounds = grid.srsBounds(child, false);

            // Check if all of the target bounding box is inside this tile.
            bool all_points_inside = true;
            for (const auto &point : points) {
                if (!tile_bounds.contains(point)) {
                    all_points_inside = false;
                    break;
                }
            }

            // If all points are inside, we can recurse.
            if (!all_points_inside) {
                continue;
            }

            all_points_inside_any_child = true;
            current_largest_encompassing_tile = child;
            break;
        }

        // Check if all of the points are contained in any children, so we can stop the recursion.
        if (!all_points_inside_any_child) {
            break;
        }
    }

    return current_largest_encompassing_tile;
}

auto load_image_with_stb(const std::filesystem::path &path, size_t &width, size_t &height, size_t &channels) {
    int w;
    int h;
    int c;
    unsigned char *image_data = stbi_load(path.c_str(), &w, &h, &c, 0);
    if (image_data) {
        width = w;
        height = h;
        channels = c;
        auto stbi_deleter = [](unsigned char *data) {
            stbi_image_free(data);
        };
        return std::unique_ptr<unsigned char, decltype(stbi_deleter)>(image_data, stbi_deleter);
    } else {
        throw std::runtime_error(fmt::format("Error loading image: {}", stbi_failure_reason()));
    }
}

auto load_image_with_fi(const std::filesystem::path &filename) {
    // check the file signature and deduce its format
    FREE_IMAGE_FORMAT fif = FreeImage_GetFileType(filename.c_str(), 0);
    // if still unknown, try to guess the file format from the file extension
    if (fif == FIF_UNKNOWN) {
        fif = FreeImage_GetFIFFromFilename(filename.c_str());
    }
    // if still unkown, return failure
    if (fif == FIF_UNKNOWN) {
        throw std::runtime_error("unable to determine image file type");
    }

    // check that the plugin has reading capabilities and load the file
    if (!FreeImage_FIFSupportsReading(fif)) {
        throw std::runtime_error("no image reading capabilities");
    }

    FIBITMAP *image = FreeImage_Load(fif, filename.c_str());
    if (!image) {
        throw std::runtime_error("error during image loading");
    }

    auto fi_deleter = [](FIBITMAP *data) {
        FreeImage_Unload(data);
    };
    return std::unique_ptr<FIBITMAP, decltype(fi_deleter)>(image, fi_deleter);
}

std::vector<unsigned char> prepare_basemap_texture(
    const OGRSpatialReference &target_srs,
    const tile::SrsBounds &target_bounds,
    const std::function<std::filesystem::path(tile::Id)> tile_to_path_mapper) {
    if (target_bounds.width() == 0 || target_bounds.height() == 0) {
        return {};
    }

    OGRSpatialReference webmercator;
    webmercator.importFromEPSG(3857);
    webmercator.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

    const tile::SrsBounds webmercator_bounds = encompassing_bounding_box_transfer(target_srs, webmercator, target_bounds);
    const tile::Id largest_encompassing_tile = bounds_to_tile_id(webmercator_bounds).to(tile::Scheme::SlippyMap);

    std::vector<tile::Id> tiles_to_splatter;

    std::vector<tile::Id> tile_stack;
    tile_stack.emplace_back(largest_encompassing_tile);

    while (!tile_stack.empty()) {
        const tile::Id tile = tile_stack.back();
        tile_stack.pop_back();

        constexpr size_t subtile_count = 4;
        const std::array<tile::Id, subtile_count> subtiles = tile.children();
        std::array<bool, subtile_count> subtile_present;
        std::array<std::filesystem::path, subtile_count> subtile_path;

        for (size_t i = 0; i < subtile_count; i++) {
            subtile_path[i] = tile_to_path_mapper(subtiles[i]);
            subtile_present[i] = !subtile_path[i].empty() && std::filesystem::exists(subtile_path[i]);
        }

        bool all_present = std::all_of(subtile_present.begin(), subtile_present.end(), [](bool e) { return e; });
        if (!all_present) {
            const std::filesystem::path tile_path = tile_to_path_mapper(tile);
            tiles_to_splatter.push_back(tile);
            std::cout << tile << std::endl;
        }

        for (size_t i = 0; i < subtile_count; i++) {
            if (subtile_present[i]) {
                tile_stack.push_back(subtiles[i]);
            }
        }
    }
    fmt::print("begin splatter:\n");

    if (tiles_to_splatter.empty()) {
        return {};
    }
    if (tiles_to_splatter.size() == 1 && tiles_to_splatter[0] == largest_encompassing_tile) {
        const std::filesystem::path root_tile_path = tile_to_path_mapper(largest_encompassing_tile);
        if (root_tile_path.empty() || !std::filesystem::exists(root_tile_path)) {
            throw std::runtime_error("found no tile in bounds");
        }
    }

    unsigned int root_zoom_level = largest_encompassing_tile.zoom_level;
    unsigned int min_zoom_level = 0;
    unsigned int max_zoom_level = 0;
    for (const tile::Id &tile : tiles_to_splatter) {
        min_zoom_level = std::min(tile.zoom_level, min_zoom_level);
        max_zoom_level = std::max(tile.zoom_level, max_zoom_level);
    }
    const unsigned int zoom_level_range = max_zoom_level - root_zoom_level;

    const tile::Id &any_tile = tiles_to_splatter.front();
    const std::filesystem::path &any_tile_path = tile_to_path_mapper(any_tile);
    size_t tile_width;
    size_t tile_height;
    size_t tile_channels;
    load_image_with_stb(any_tile_path, tile_width, tile_height, tile_channels);
    const glm::uvec2 tile_size(tile_width, tile_height);

    const size_t full_image_size_factor = std::pow(2, zoom_level_range);

    auto src = load_image_with_fi(any_tile_path);
    print_point(tile_size);
    fmt::print("{}\n", full_image_size_factor);
    FIBITMAP *raw_image_ptr = FreeImage_Allocate(
        tile_width * full_image_size_factor, tile_height * full_image_size_factor,
        FreeImage_GetBPP(src.get()),
        FreeImage_GetRedMask(src.get()), FreeImage_GetGreenMask(src.get()), FreeImage_GetBlueMask(src.get()));
    if (!raw_image_ptr) {
        throw std::runtime_error{"failed to allocate image buffer"};
    }
    auto fi_deleter = [](FIBITMAP *data) {
        FreeImage_Unload(data);
    };
    auto image = std::unique_ptr<FIBITMAP, decltype(fi_deleter)>(raw_image_ptr, fi_deleter);

    for (const tile::Id &tile : tiles_to_splatter) {
        const std::filesystem::path &tile_path = tile_to_path_mapper(tile);
        size_t width;
        size_t height;
        size_t channels;
        const auto tile_data = load_image_with_stb(tile_path, width, height, channels);
        if (width != tile_width) {
            throw std::runtime_error{"tiles have inconsitent widths"};
        }
        if (height != tile_height) {
            throw std::runtime_error{"tiles have inconsitent heights"};
        }
        if (channels != tile_channels) {
            throw std::runtime_error{"tiles have inconsitent channel counts"};
        }

        const size_t relative_zoom_level = tile.zoom_level - root_zoom_level;
        const glm::uvec2 current_tile_size_factor = glm::uvec2(std::pow(2, max_zoom_level - tile.zoom_level));
        const glm::uvec2 current_tile_size = tile_size * current_tile_size_factor;
        const glm::uvec2 relative_tile_coords = tile.coords - largest_encompassing_tile.coords * glm::uvec2(std::pow(2, relative_zoom_level));
        const glm::uvec2 current_tile_position = relative_tile_coords * current_tile_size;

        auto tile_image = load_image_with_fi(tile_path);
        auto scaled_tile_image = FreeImage_Rescale(tile_image.get(), current_tile_size.x, current_tile_size.y, FILTER_BILINEAR);
        if (!scaled_tile_image) {
            throw std::runtime_error{"failed to rescale subtile"};
        }

        print_point(largest_encompassing_tile.coords);
        print_point(tile.coords);
        print_point(relative_tile_coords);
        print_point(current_tile_size_factor);
        print_point(current_tile_position);
        print_point(current_tile_size);
        if (!FreeImage_Paste(image.get(), scaled_tile_image, current_tile_position.x, current_tile_position.y, 256)) {
            throw std::runtime_error{"failed to paste subtile into texture buffer"};
        }
    }

    // FreeImage_Save(FIF_BMP, image.get(), "mybitmap.bmp", 0);

    std::vector<unsigned char> image_data;
    FIMEMORY *memStream = FreeImage_OpenMemory();
    if (FreeImage_SaveToMemory(FIF_JPEG, image.get(), memStream, JPEG_DEFAULT)) {
        // Get the size of the memory stream
        unsigned int size = FreeImage_TellMemory(memStream);
        fmt::print("{}\n", size);

        // FreeImage_SeekMemory(memStream, 0L, SEEK_SET);

        // Copy the image data from the memory stream to the vector
        unsigned char * data_ptr = nullptr;
        if (!FreeImage_AcquireMemory(memStream, &data_ptr, &size)) {
            throw std::runtime_error{"FreeImage_AcquireMemory failed"};
        }

        // Resize the vector to the size of the image data
        image_data.resize(size);
        std::copy(data_ptr, data_ptr + size, image_data.data());

        // Clean up
        FreeImage_CloseMemory(memStream);
    } else {
        throw std::runtime_error{"FreeImage_SaveToMemory failed"};
    }

    return image_data;
}

/**
FreeImage error handler
@param fif Format / Plugin responsible for the error
@param message Error message
*/
void FreeImageErrorHandler(FREE_IMAGE_FORMAT fif, const char *message) {
    printf("\n*** ");
    if (fif != FIF_UNKNOWN) {
        printf("%s Format\n", FreeImage_GetFormatFromFIF(fif));
    }
    printf(message);
    printf(" ***\n");
}

int main() {
    // In your main program …
    FreeImage_Initialise(true);
    std::cout << "FreeImage " << FreeImage_GetVersion() << "\n";
    std::cout << FreeImage_GetCopyrightMessage() << "\n\n";
    FreeImage_SetOutputMessage(FreeImageErrorHandler);

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

    /*
    std::ifstream texture_input("/mnt/c/Users/Admin/Downloads/142994.jpeg", std::ios::binary);
    // std::ifstream texture_input("/mnt/c/Users/Admin/Downloads/testTexture.png", std::ios::binary);

    std::vector<unsigned char> texture_bytes(
        (std::istreambuf_iterator<char>(texture_input)),
        (std::istreambuf_iterator<char>()));

    texture_input.close();
    */
    std::vector<unsigned char> texture_bytes = prepare_basemap_texture(
        grid.getSRS(), tile_bounds,
        [](tile::Id tile_id) { return fmt::format("/mnt/e/Code/TU/2023S/Project/tiles/{}/{}/{}.jpeg", tile_id.zoom_level, tile_id.coords.y, tile_id.coords.x); });

    save_mesh_as_gltf(mesh, texture_bytes);

    return 0;
}
