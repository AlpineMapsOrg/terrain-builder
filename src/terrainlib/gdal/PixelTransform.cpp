#include <gdal.h>
#include <gdal_priv.h>

#include "error/utils.h"
#include "gdal/PixelTransform.h"

namespace gdal {

alp::Result<PixelTransform, PixelTransformError> PixelTransform::from_forward(const std::array<double, 6> &forward) {
    std::array<double, 6> inverse;
    if (GDALInvGeoTransform(forward.data(), inverse.data()) != TRUE) {
        return alp::make_unexpected(PixelTransformError::NotInvertible);
    }
    return PixelTransform(forward, inverse);
}

alp::Result<PixelTransform, PixelTransformError> PixelTransform::from_dataset(GDALDataset &dataset) {
    std::array<double, 6> forward;
    if (dataset.GetGeoTransform(forward.data()) != CE_None) {
        return alp::make_unexpected(PixelTransformError::Missing);
    }
    return PixelTransform::from_forward(forward);
}

glm::dvec2 PixelTransform::apply(const glm::dvec2 pixel) const {
    glm::dvec2 point;
    GDALApplyGeoTransform(this->_forward.data(), pixel.x, pixel.y, &point.x, &point.y);
    return point;
}

glm::dvec2 PixelTransform::apply_inverse(const glm::dvec2 point) const {
    glm::dvec2 pixel;
    GDALApplyGeoTransform(this->_inverse.data(), point.x, point.y, &pixel.x, &pixel.y);
    return pixel;
}

bool PixelTransform::is_axis_aligned() const {
    return this->_forward[2] == 0.0 && this->_forward[4] == 0.0;
}

} // namespace gdal
