#ifndef FIIMAGE_H
#define FIIMAGE_H

#include <filesystem>
#include <mutex>
#include <vector>

#include <FreeImage.h>
#include <glm/glm.hpp>
#include <radix/geometry.h>

#include "non_copyable.h"

namespace {
    std::once_flag g_freeimage_initialized_once_flag;

    struct FreeImageDeinitialiserHandler {
        ~FreeImageDeinitialiserHandler()
        {
            FreeImage_DeInitialise();
        }
    };
} // namespace

inline void initialize_freeimage_once() {
    std::call_once(g_freeimage_initialized_once_flag, []() {
        FreeImage_Initialise();
    });
    static FreeImageDeinitialiserHandler deinitialiser;
}

class FiImage : private NonCopyable {
public:
    static FiImage load_from_path(const std::filesystem::path &filename) {
        initialize_freeimage_once();

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

        return FiImage(image);
    }

    static FiImage load_from_buffer(const std::vector<unsigned char>& buffer) {
        return FiImage::load_from_buffer(buffer.data(), buffer.size());
    }
    static FiImage load_from_buffer(const unsigned char* data, const size_t size) {
        initialize_freeimage_once();

        // attach the binary data to a memory stream
        FIMEMORY *fi_buffer = FreeImage_OpenMemory(const_cast<unsigned char*>(data), size);
        // get the file type
        FREE_IMAGE_FORMAT fif = FreeImage_GetFileTypeFromMemory(fi_buffer, 0);
        if (fif == FIF_UNKNOWN) {
            FreeImage_CloseMemory(fi_buffer); // TODO: cleanup
            throw std::runtime_error("unable to determine image file type");
        }
        // load an image from the memory stream
        FIBITMAP *image = FreeImage_LoadFromMemory(fif, fi_buffer, 0);
        if (!image) {
            FreeImage_CloseMemory(fi_buffer); // TODO: cleanup
            throw std::runtime_error("error during image loading");
        }
        // always close the memory stream
        FreeImage_CloseMemory(fi_buffer); // TODO: cleanup

        return FiImage(image);
    }

    static FiImage allocate(const FREE_IMAGE_TYPE type, const glm::uvec2& size, const unsigned int bpp, const unsigned int red_mask = 0U, const unsigned int green_mask = 0U, const unsigned int blue_mask = 0U) {
        initialize_freeimage_once();

        FIBITMAP *raw_image_ptr = FreeImage_AllocateT(type, size.x, size.y, bpp, red_mask, green_mask, blue_mask);
        if (raw_image_ptr == nullptr) {
            throw std::runtime_error("FreeImage_Allocate failed");
        }
        return FiImage(raw_image_ptr);
    }

    static FiImage allocate_like(const FiImage &other, const glm::uvec2& size) {
        const FREE_IMAGE_TYPE type = other.type();
        const unsigned int bpp = other.bits_per_pixel();
        const unsigned int red_mask = other.red_mask();
        const unsigned int green_mask = other.green_mask();
        const unsigned int blue_mask = other.blue_mask();
        return FiImage::allocate(type, size, bpp, red_mask, green_mask, blue_mask);
    }

    FiImage(FIBITMAP *image)
        : raw_image(image) {}

    ~FiImage() {
        if (this->raw_image) {
            FreeImage_Unload(this->raw_image);
        }
    }

    // Move constructor
    FiImage(FiImage &&other) {
        this->raw_image = other.raw_image;
        other.raw_image = nullptr;
    }

    // Move assignment operator
    FiImage &operator=(FiImage &&other) {
        if (this != &other) {
            this->raw_image = other.raw_image;
            other.raw_image = nullptr;
        }
        return *this;
    }

    FIBITMAP * raw(){
        return this->raw_image;
    }
    const FIBITMAP * raw() const {
        return this->raw_image;
    }

    FREE_IMAGE_TYPE type() const {
        return FreeImage_GetImageType(this->raw_image);
    }
    unsigned int width() const {
        return FreeImage_GetWidth(this->raw_image);
    }
    unsigned int height() const {
        return FreeImage_GetHeight(this->raw_image);
    }
    glm::uvec2 size() const {
        return glm::uvec2(this->width(), this->height());
    }
    unsigned int bits_per_pixel() const {
        return FreeImage_GetBPP(this->raw_image);
    }
    unsigned int red_mask() const {
        return FreeImage_GetRedMask(this->raw_image);
    }
    unsigned int green_mask() const {
        return FreeImage_GetGreenMask(this->raw_image);
    }
    unsigned int blue_mask() const {
        return FreeImage_GetBlueMask(this->raw_image);
    }

    FiImage rescale(const glm::uvec2 new_size, const FREE_IMAGE_FILTER filter = FILTER_BILINEAR) const {
        FIBITMAP * scaled = FreeImage_Rescale(this->raw_image, new_size.x, new_size.y, filter);
        if (!scaled) {
            throw std::runtime_error{"FreeImage_Rescale failed"};
        }
        return FiImage(scaled);
    }

    FiImage copy(const geometry::Aabb<2, unsigned int> &box) const {
        assert(box.min.x >= 0);
        assert(box.max.x <= this->width());
        assert(box.min.y >= 0);
        assert(box.max.y <= this->height());
        
        FIBITMAP *copy = FreeImage_Copy(this->raw_image, box.min.x, box.min.y, box.max.x, box.max.y);
        if (!copy) {
            throw std::runtime_error{"FreeImage_Copy failed"};
        }
        return FiImage(copy);
    }

    void paste(const FiImage& src, const glm::ivec2& target_position, const bool trim_excess = false) {
        if (trim_excess) {
            const glm::ivec2 image_size = this->size();
            const geometry::Aabb2i target_bounds(
                target_position,
                target_position + glm::ivec2(src.size()));
            const geometry::Aabb2ui clamped_target_bounds(
                glm::max(glm::min(target_bounds.min, image_size - glm::ivec2(1)), glm::ivec2(0)),
                glm::max(glm::min(target_bounds.max, image_size - glm::ivec2(1)), glm::ivec2(0)));

            if (clamped_target_bounds.width() == 0 || clamped_target_bounds.height() == 0) {
                // src is fully outside of the bounds of this image and thus we have nothing to do.
                return;
            }

            if (glm::uvec2(target_bounds.size()) != clamped_target_bounds.size()) {
                const geometry::Aabb2ui image_bounds_no_excess(
                    glm::ivec2(clamped_target_bounds.min) - target_bounds.min,
                    glm::ivec2(clamped_target_bounds.max) - target_bounds.min);
                const FiImage src_no_excess = src.copy(image_bounds_no_excess);
                this->paste(src_no_excess, clamped_target_bounds.min, false);
                return;
            }
        }

        assert(target_position.x >= 0);
        assert(target_position.x < this->width());
        assert(target_position.y >= 0);
        assert(target_position.y < this->height());

        if (!FreeImage_Paste(this->raw_image, src.raw_image, target_position.x, target_position.y, 256 /* no alpha blending */)) {
            throw std::runtime_error{"FreeImage_Paste failed"};
        }
    }

    void flip_horizontal() {
        if (!FreeImage_FlipHorizontal(this->raw_image)) {
            throw std::runtime_error{"FreeImage_FlipHorizontal failed"};
        }
    }

    void flip_vertical() {
        if (!FreeImage_FlipVertical(this->raw_image)) {
            throw std::runtime_error{"FreeImage_FlipVertical failed"};
        }
    }

    void save(const std::filesystem::path filename, const int flags = 0 /* default settings */) const {
        const FREE_IMAGE_FORMAT fif = FreeImage_GetFIFFromFilename(filename.c_str());
        if (fif == FIF_UNKNOWN) {
            throw std::runtime_error("unable to determine image file type");
        }

        // check that the plugin has writing capabilities
        if (!FreeImage_FIFSupportsWriting(fif)) {
            throw std::runtime_error("no image writing capabilities");
        }

        if (!FreeImage_Save(fif, this->raw_image, filename.c_str(), flags)) {
            throw std::runtime_error("FreeImage_Save failed");
        }
    }

    std::vector<unsigned char> save_to_vector(const FREE_IMAGE_FORMAT format, const int flags = 0 /* default settings */) const {
        std::vector<unsigned char> buffer;

        FIMEMORY *fi_buffer = FreeImage_OpenMemory();
        if (!FreeImage_SaveToMemory(format, this->raw_image, fi_buffer, flags)) {
            throw std::runtime_error{"FreeImage_SaveToMemory failed"};
        }

        // Get the size of the memory stream
        unsigned int size = FreeImage_TellMemory(fi_buffer);

        // Encode the image into the FreeImage buffer.
        unsigned char *data_ptr = nullptr;
        if (!FreeImage_AcquireMemory(fi_buffer, &data_ptr, &size)) {
            throw std::runtime_error{"FreeImage_AcquireMemory failed"};
        }

        // Resize the vector to the size of the image data
        buffer.resize(size);

        // Copy the FreeImage buffer to our buffer.
        std::copy(data_ptr, data_ptr + size, buffer.data());

        // Clean up
        FreeImage_CloseMemory(fi_buffer);

        return buffer;
    }

private:
    FIBITMAP *raw_image;
};

#endif
