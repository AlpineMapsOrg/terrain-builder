#ifndef UV_MAP_H
#define UV_MAP_H

#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Surface_mesh_parameterization/Circular_border_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/Square_border_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/Barycentric_mapping_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/Discrete_authalic_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/Discrete_conformal_map_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/Mean_value_coordinates_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/parameterize.h>
#include <CGAL/Surface_mesh_parameterization/Error_code.h>
#include <CGAL/Unique_hash_map.h>

#include <opencv2/opencv.hpp>
#include <glm/glm.hpp>
#include <fmt/core.h>

#include "cgal.h"
#include "convert.h"
#include "terrain_mesh.h"
#include "merge.h"

typedef cv::Mat Texture;

namespace uv_map {

enum class Algorithm {
    TutteBarycentricMapping,
    DiscreteAuthalic,
    DiscreteConformalMap,
    FloaterMeanValueCoordinates
};

enum class Border {
    Circle,
    Square
};

class UvParameterizationError {
public:
    UvParameterizationError() = default;
    constexpr UvParameterizationError(int code)
        : code(code) {}

    operator int() const {
        return this->code;
    }

    std::string description() const {
        return CGAL::Surface_mesh_parameterization::get_error_message(this->code);
    }

private:
    int code;
};

typedef CGAL::Unique_hash_map<VertexDescriptor, Point2> UvMap;
typedef boost::associative_property_map<UvMap> UvPropertyMap;


std::vector<glm::dvec2> decode_uv_map(const UvMap& uv_map, size_t number_of_vertices) {
    std::vector<glm::dvec2> uvs;
    uvs.reserve(number_of_vertices);
    for (size_t i = 0; i < number_of_vertices; i++) {
        const Point2 &uv = uv_map[CGAL::SM_Vertex_index(i)];
        uvs.push_back(convert::cgal2glm(uv));
    }
    return uvs;
}

std::vector<glm::dvec2> decode_uv_map(const UvPropertyMap& uv_map, size_t number_of_vertices) {
    std::vector<glm::dvec2> uvs;
    uvs.reserve(number_of_vertices);
    for (size_t i = 0; i < number_of_vertices; i++) {
        const Point2 &uv = uv_map[CGAL::SM_Vertex_index(i)];
        uvs.push_back(convert::cgal2glm(uv));
    }
    return uvs;
}

tl::expected<UvMap, UvParameterizationError> parameterize_mesh(SurfaceMesh &mesh, Algorithm algorithm, Border border) {
    typedef CGAL::Surface_mesh_parameterization::Circular_border_uniform_parameterizer_3<SurfaceMesh> CircleBorderParameterizer;
    typedef CGAL::Surface_mesh_parameterization::Square_border_uniform_parameterizer_3<SurfaceMesh> SquareBorderParameterizer;

    typedef CGAL::Surface_mesh_parameterization::Barycentric_mapping_parameterizer_3<SurfaceMesh, CircleBorderParameterizer> TutteBarycentricMappingParameterizerCircularBorder;
    typedef CGAL::Surface_mesh_parameterization::Barycentric_mapping_parameterizer_3<SurfaceMesh, SquareBorderParameterizer> TutteBarycentricMappingParameterizerSquareBorder;
    typedef CGAL::Surface_mesh_parameterization::Discrete_authalic_parameterizer_3<SurfaceMesh, CircleBorderParameterizer> DiscreteAuthalicParameterizerCircularBorder;
    typedef CGAL::Surface_mesh_parameterization::Discrete_authalic_parameterizer_3<SurfaceMesh, SquareBorderParameterizer> DiscreteAuthalicParameterizerSquareBorder;
    typedef CGAL::Surface_mesh_parameterization::Discrete_conformal_map_parameterizer_3<SurfaceMesh, CircleBorderParameterizer> DiscreteConformalMapParameterizerCircularBorder;
    typedef CGAL::Surface_mesh_parameterization::Discrete_conformal_map_parameterizer_3<SurfaceMesh, SquareBorderParameterizer> DiscreteConformalMapParameterizerSquareBorder;
    typedef CGAL::Surface_mesh_parameterization::Mean_value_coordinates_parameterizer_3<SurfaceMesh, CircleBorderParameterizer> FloaterMeanValueCoordinatesParameterizerCircularBorder;
    typedef CGAL::Surface_mesh_parameterization::Mean_value_coordinates_parameterizer_3<SurfaceMesh, SquareBorderParameterizer> FloaterMeanValueCoordinatesParameterizerSquareBorder;

    const HalfedgeDescriptor bhd = CGAL::Polygon_mesh_processing::longest_border(mesh).first;

    UvMap uv_uhm;
    UvPropertyMap uv_map(uv_uhm);

    CGAL::Surface_mesh_parameterization::Error_code result;
    if (border != Border::Circle && border != Border::Square) {
        throw std::invalid_argument("illegal border specifier");
    }

    if (algorithm == Algorithm::TutteBarycentricMapping) {
        if (border == Border::Circle) {
            result = CGAL::Surface_mesh_parameterization::parameterize(mesh, TutteBarycentricMappingParameterizerCircularBorder(), bhd, uv_map);
        } else if (border == Border::Square) {
            result = CGAL::Surface_mesh_parameterization::parameterize(mesh, TutteBarycentricMappingParameterizerSquareBorder(), bhd, uv_map);
        }
    } else if (algorithm == Algorithm::DiscreteAuthalic) {
        if (border == Border::Circle) {
            result = CGAL::Surface_mesh_parameterization::parameterize(mesh, DiscreteAuthalicParameterizerCircularBorder(), bhd, uv_map);
        } else if (border == Border::Square) {
            result = CGAL::Surface_mesh_parameterization::parameterize(mesh, DiscreteAuthalicParameterizerSquareBorder(), bhd, uv_map);
        }
    } else if (algorithm == Algorithm::DiscreteConformalMap) {
        if (border == Border::Circle) {
            result = CGAL::Surface_mesh_parameterization::parameterize(mesh, DiscreteConformalMapParameterizerCircularBorder(), bhd, uv_map);
        } else if (border == Border::Square) {
            result = CGAL::Surface_mesh_parameterization::parameterize(mesh, DiscreteConformalMapParameterizerSquareBorder(), bhd, uv_map);
        }
    } else if (algorithm == Algorithm::FloaterMeanValueCoordinates) {
        if (border == Border::Circle) {
            result = CGAL::Surface_mesh_parameterization::parameterize(mesh, FloaterMeanValueCoordinatesParameterizerCircularBorder(), bhd, uv_map);
        } else if (border == Border::Square) {
            result = CGAL::Surface_mesh_parameterization::parameterize(mesh, FloaterMeanValueCoordinatesParameterizerSquareBorder(), bhd, uv_map);
        }
    } else {
        throw std::invalid_argument("illegal algorithm specifier");
    }

    if (result != CGAL::Surface_mesh_parameterization::OK) {
        return tl::unexpected(UvParameterizationError(result));
    }

    return uv_uhm;
}

tl::expected<UvMap, UvParameterizationError> parameterize_mesh(const TerrainMesh &mesh, Algorithm algorithm, Border border) {
    SurfaceMesh cgal_mesh = convert::mesh2cgal(mesh);
    return parameterize_mesh(cgal_mesh, algorithm, border);
}

namespace {
template <typename T>
cv::Rect_<T> clamp_rect_to_mat_bounds(const cv::Rect_<T> &rect, const cv::Mat &mat) {
    const T x = std::max(rect.x, 0);
    const T y = std::max(rect.y, 0);
    const T width = std::max(std::min(rect.width, mat.cols - x), 0);
    const T height = std::max(std::min(rect.height, mat.rows - y), 0);

    return cv::Rect(x, y, width, height);
}
}

void warp_triangle(cv::Mat &source_image, cv::Mat &target_image, std::array<cv::Point2f, 3> source_triangle, std::array<cv::Point2f, 3> target_triangle) {
    // Find bounding rectangle for each triangle
    const cv::Rect source_rect = clamp_rect_to_mat_bounds(cv::boundingRect(source_triangle), source_image);
    const cv::Rect target_rect = clamp_rect_to_mat_bounds(cv::boundingRect(target_triangle), target_image);
    if (source_rect.width == 0 || source_rect.height == 0
     || target_rect.width == 0 || target_rect.height == 0) {
        return;
    }

    // Relativize triangles to bounds
    std::array<cv::Point2f, 3> source_triangle_cropped;
    std::array<cv::Point2f, 3> target_triangle_cropped;
    for (size_t i = 0; i < 3; i++) {
        source_triangle_cropped[i] = cv::Point2f(source_triangle[i].x - source_rect.x, source_triangle[i].y - source_rect.y);
        target_triangle_cropped[i] = cv::Point2f(target_triangle[i].x - target_rect.x, target_triangle[i].y - target_rect.y);
    }
    
    // Convert points to int triangles as fillConvexPoly needs a vector of Point and not Point2f
    std::array<cv::Point2i, 3> target_triangle_cropped_int;
    for (size_t i = 0; i < 3; i++) {
        target_triangle_cropped_int[i]= cv::Point2i((int)(target_triangle[i].x - target_rect.x), (int)(target_triangle[i].y - target_rect.y));
    }

    // Read source region from source image
    cv::Mat source_image_cropped;
    source_image(source_rect).copyTo(source_image_cropped);

    // Given a pair of triangles, find the affine transform.
    const cv::Mat warp_tranform = cv::getAffineTransform(source_triangle_cropped, target_triangle_cropped);

    // Apply the affine transform just found to the source image
    cv::Mat target_image_cropped = cv::Mat::zeros(target_rect.height, target_rect.width, CV_32FC3);
    cv::warpAffine(source_image_cropped, target_image_cropped, warp_tranform, target_image_cropped.size(), cv::INTER_LINEAR, cv::BORDER_REFLECT_101);

    // Get mask by filling triangle
    cv::Mat mask = cv::Mat::zeros(target_rect.height, target_rect.width, CV_32FC3);
    cv::fillConvexPoly(mask, target_triangle_cropped_int, cv::Scalar(1.0, 1.0, 1.0), 16, 0);

    // Copy triangular region of the rectangular patch to the output image
    cv::multiply(target_image_cropped, mask, target_image_cropped);
    cv::multiply(target_image(target_rect), cv::Scalar(1.0, 1.0, 1.0) - mask, target_image(target_rect));
    target_image(target_rect) = target_image(target_rect) + target_image_cropped;
}

Texture merge_textures(
    const std::span<const TerrainMesh> original_meshes,
    const TerrainMesh& merged_mesh,
    const merge::VertexMapping &mapping,
    const UvMap& uv_map,
    const glm::uvec2 merged_texture_size) {

    std::vector<cv::Mat> original_textures;
    original_textures.reserve(original_meshes.size());
    for (size_t i = 0; i < original_meshes.size(); i++) {
        if (original_meshes[i].texture.has_value()) {
            const FiImage& texture = original_meshes[i].texture.value();
            const cv::Mat converted = convert::fi2cv(texture);
            original_textures.push_back(converted);
        } else {
            original_textures.push_back(cv::Mat());
        }
    }

    cv::Mat merged_atlas = cv::Mat::zeros(merged_texture_size.y, merged_texture_size.x, CV_32FC3);

    for (const glm::uvec3 &mapped_triangle : merged_mesh.triangles) {
        std::array<cv::Point2f, 3> mapped_uv_triangle;
        for (size_t i = 0; i < mapped_triangle.length(); i++) {
            const auto uv = uv_map[CGAL::SM_Vertex_index(mapped_triangle[i])];
            mapped_uv_triangle[i] = cv::Point2f(uv.x() * merged_texture_size.x, uv.y() * merged_texture_size.y);
        }

        const merge::TriangleInMesh source_mesh_and_triangle = mapping.find_source_triangle(mapped_triangle);
        const size_t source_mesh_index = source_mesh_and_triangle.mesh_index;
        const TerrainMesh &source_mesh = original_meshes[source_mesh_index];
        const glm::uvec3 source_triangle = source_mesh_and_triangle.triangle;

        std::array<cv::Point2f, 3> source_uv_triangle;
        for (size_t i = 0; i < source_triangle.length(); i++) {
            const glm::dvec2 uv = source_mesh.uvs[source_triangle[i]];
            source_uv_triangle[i] = cv::Point2f(uv.x * source_mesh.texture->width(), uv.y * source_mesh.texture->height());
        }

        warp_triangle(original_textures[source_mesh_index], merged_atlas, source_uv_triangle, mapped_uv_triangle);
    }

    return merged_atlas;
}

}

#endif
