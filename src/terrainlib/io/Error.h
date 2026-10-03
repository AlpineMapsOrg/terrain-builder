#pragma once

#include "error/Error.h"
#include "error/Result.h"
#include "error/format.h"
#include "io/FileError.h"
#include "io/SerdeError.h"

namespace io {

// An error from a file operation or from serialization.
using Error = alp::Error<FileError, SerdeError>;

// A value, or an error from a file operation or from serialization.
template <typename T>
using Result = alp::Result<T, Error>;

} // namespace io

