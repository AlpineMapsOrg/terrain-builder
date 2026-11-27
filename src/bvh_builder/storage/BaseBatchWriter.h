#pragma once

#include <cstddef>
#include <expected>

#include "io/Error.h"

#include "containers/UniqueResource.h"
#include "log.h"
#include "storage/batch_limits.h"
#include "error/Result.h"
#include "error/format.h"
#include "error/log.h"
#include "error/try.h"

namespace detail {

// A batch being filled, together with the storage it belongs to.
template <typename Storage>
struct PendingBatch {
    using Batch = typename Storage::Batch;

    // Writes a batch that its writer left open.
    struct Flush {
        void operator()(PendingBatch &pending) const noexcept {
            LOG_WARN("Batch was not written upon writer destruction, use flush().");

            LOG_UNEXPECTED_OF(pending.output->write(pending.batch), "closing a batch writer for {}", pending.output->directory());
        }
    };

    Storage *output;
    Batch batch;
};

} // namespace detail

// Owns the batch being filled, writing it out once it grows past a maximum size.
template <typename Storage>
class BaseBatchWriter {
public:
    using Batch = typename Storage::Batch;

    BaseBatchWriter(Storage &output, const size_t max_batch_bytes = DEFAULT_MAX_BATCH_BYTES) : _output(output), _max_batch_bytes(max_batch_bytes) {}

    // How much the batch being filled holds.
    size_t compute_buffered_bytes() const {
        if (this->_current.is_empty()) {
            return 0;
        }
        return this->compute_buffered_bytes(this->_current->batch);
    }

    // Writes the current batch to disk.
    [[nodiscard]]
    alp::Result<void, io::Error> flush() {
        if (this->_current.is_empty()) {
            return {};
        }

        ALP_TRY(this->_output.write(this->_current->batch));
        this->_current.take();
        return {};
    }

protected:
    using Pending = detail::PendingBatch<Storage>;

    // A batch for the writer to start filling.
    virtual Batch create_batch(Storage &output) = 0;

    // How much `batch` holds.
    virtual size_t compute_buffered_bytes(const Batch &batch) const = 0;

    // The batch being filled, created if there is none.
    Batch &open_batch() {
        if (this->_current.is_empty()) {
            this->_current.reset(Pending {
                .output = &this->_output,
                .batch = this->create_batch(this->_output),
            });
        }
        return this->_current->batch;
    }

    // Writes the batch being filled if it has grown past the maximum.
    [[nodiscard]]
    alp::Result<void, io::Error> flush_if_full() {
        if (this->compute_buffered_bytes() < this->_max_batch_bytes) {
            return {};
        }
        return this->flush();
    }

private:
    Storage &_output;
    size_t _max_batch_bytes;
    UniqueResource<Pending, typename Pending::Flush> _current;
};