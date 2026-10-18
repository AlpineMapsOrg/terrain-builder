#pragma once

#include "error/Result.h"
#include "io/Error.h"

#include "frontier/BatchWriter.h"
#include "frontier/layout.h"

namespace frontier {

// A sink that writes each entry through `writer`, and writes out the last batch when its stream ends.
class BatchWriterSink {
public:
    explicit BatchWriterSink(BatchWriter &writer) : _writer(writer) {}

    [[nodiscard]]
    alp::Result<void, io::Error> push(const Entry &entry) {
        return this->_writer.write(entry);
    }

    [[nodiscard]]
    alp::Result<void, io::Error> finish() && {
        return this->_writer.flush();
    }

private:
    BatchWriter &_writer;
};

} // namespace frontier
