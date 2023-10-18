#include <filesystem>

#include <FreeImage.h>

#include "tntn/gdal_init.h"

#include "non_copyable.h"

class FiImage : private NonCopyable {
public:
    static FiImage load_from_path(const std::filesystem::path &filename) {
        tntn::initialize_freeimage_once();

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

    static FiImage allocate(const FREE_IMAGE_TYPE type, const glm::uvec2& size, const unsigned int bpp, const unsigned int red_mask = 0U, const unsigned int green_mask = 0U, const unsigned int blue_mask = 0U) {
        tntn::initialize_freeimage_once();

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
        // FreeImage_Unload(this->raw_image);
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

    FiImage copy(const geometry::Aabb<2, unsigned int> &box) {
        assert(box.min.x >= 0);
        assert(box.max.x <= this->width());
        assert(box.min.y >= 0);
        assert(box.max.y <= this->height());

        /*
        const geometry::Aabb2ui fi_box(
            {box.min.x, this->height() - box.max.y},
            {box.max.y, this->height() - box.min.y}
        );
        assert(fi_box.min.x >= 0);
        assert(fi_box.max.x <= this->width());
        assert(fi_box.min.y >= 0);
        assert(fi_box.max.y <= this->height());
        */

        // FIBITMAP *copy = FreeImage_Copy(this->raw_image, fi_box.min.x, fi_box.min.y, fi_box.max.x - 1, fi_box.max.y - 1);
        FIBITMAP *copy = FreeImage_Copy(this->raw_image, box.min.x, box.min.y, box.max.x, box.max.y);
        if (!copy) {
            throw std::runtime_error{"FreeImage_Copy failed"};
        }
        return FiImage(copy);
    }

    void paste(const FiImage& src, const glm::uvec2 target_position) {
        assert(target_position.x >= 0);
        assert(target_position.x < this->width());
        assert(target_position.y >= 0);
        assert(target_position.y < this->height());

        const glm::uvec2 fi_target_position(
            target_position.x, 
            this->height() - target_position.y - src.height()
        );
        assert(fi_target_position.x >= 0);
        assert(fi_target_position.x < this->width());
        assert(fi_target_position.y >= 0);
        assert(fi_target_position.y < this->height());

        // if (!FreeImage_Paste(this->raw_image, src.raw_image, target_position.x, target_position.y, 256 /* no alpha blending */)) {
        if (!FreeImage_Paste(this->raw_image, src.raw_image, fi_target_position.x, fi_target_position.y, 256 /* no alpha blending */)) {
            throw std::runtime_error{"FreeImage_Paste failed"};
        }
        // if (!FreeImage_Paste(this->raw_image, src.raw_image, target_position.x, target_position.y, 256 /* no alpha blending */)) {
        //     throw std::runtime_error{"FreeImage_Paste failed"};
        // }
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

    void save(const std::filesystem::path filename, const int flags = 0 /* default settings */) {
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
