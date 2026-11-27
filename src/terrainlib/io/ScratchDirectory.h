#pragma once

#include <filesystem>
#include <utility>

#include "containers/UniqueResource.h"
#include "error/Result.h"
#include "error/log.h"
#include "error/try.h"
#include "io/FileError.h"
#include "io/filesystem.h"

namespace io {

// A directory that is removed, with everything below it, when this goes away.
class ScratchDirectory {
    struct Remove {
        void operator()(std::filesystem::path &path) const noexcept {
            LOG_IF_ERROR(filesystem::remove_all(path));
        }
    };

public:
    const std::filesystem::path &path() const {
        return this->_path.get();
    }

    // Removes the directory now, reporting what failed.
    [[nodiscard]]
    alp::Result<void, FileError> remove() && {
        const std::filesystem::path path = this->_path.take();
        ALP_TRY(filesystem::remove_all(path));
        return {};
    }

private:
    friend alp::Result<ScratchDirectory, FileError> create_scratch_directory(const std::filesystem::path &path);

    explicit ScratchDirectory(std::filesystem::path path) : _path(std::move(path)) {}

    UniqueResource<std::filesystem::path, Remove> _path;
};

// Creates an empty scratch directory at `path`, along with its missing parents.
// Fails when a directory at `path` already holds something.
[[nodiscard]]
inline alp::Result<ScratchDirectory, FileError> create_scratch_directory(const std::filesystem::path &path) {
    ALP_TRY(filesystem::create_empty_directory(path));
    return ScratchDirectory(path);
}

} // namespace io
