#include <fstream>

#include "io/bytes.h"
#include "io/utils.h"

namespace io
{

    tl::expected<void, Error> write_bytes_to_path(const std::span<const uint8_t> bytes, const std::filesystem::path &path, bool make_dirs)
    {
        if (make_dirs)
        {
            utils::create_parent_directories(path);
        }

        std::ofstream file(path, std::ios::binary);
        if (!file.is_open())
        {
            return tl::unexpected(Error::OpenFile);
        }

        file.write(reinterpret_cast<const char *>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
        if (!file.good())
        {
            return tl::unexpected(Error::WriteBytes);
        }

        return {};
    }

    tl::expected<std::vector<uint8_t>, Error> read_bytes_from_path(const std::filesystem::path &path)
    {
        LOG_TRACE("io::read_bytes_from_path({})", path);
        std::ifstream file(path, std::ios::binary | std::ios::ate);
        LOG_TRACE("io::read_bytes_from_path() opened file");
        if (!file.is_open())
        {
            return tl::unexpected(Error::OpenFile);
        }
        LOG_TRACE("io::read_bytes_from_path(): file is open");

        const std::streamsize size = file.tellg();
        if (size < 0)
        {
            return tl::unexpected(Error::DetermineSize);
        }

        std::vector<uint8_t> buffer(static_cast<size_t>(size));
        file.seekg(0);
        file.read(reinterpret_cast<char *>(buffer.data()), size);

        if (!file.good())
        {
            return tl::unexpected(Error::ReadBytes);
        }

        return buffer;
    }

}
