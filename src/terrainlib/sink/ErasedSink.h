#pragma once

#include <memory>
#include <utility>

#include "error/Result.h"
#include "sink/Sink.h"
#include "sink/SinkBase.h"

// A sink that holds any other sink, whatever its type.
template <typename T, typename... Es>
class ErasedSink {
public:
    // Holds `sink`, by reference if it is an lvalue.
    template <typename S> requires Sink<S, T, Es...>
    explicit ErasedSink(S &&sink) : _sink(std::make_unique<Holder<S>>(std::forward<S>(sink))) {}

    alp::Result<void, Es...> push(T value) {
        return this->_sink->push(std::move(value));
    }

    alp::Result<void, Es...> finish() && {
        return std::move(*this->_sink).finish();
    }

private:
    template <typename S>
    class Holder final : public SinkBase<T, Es...> {
    public:
        explicit Holder(S &&sink) : _sink(std::forward<S>(sink)) {}

        alp::Result<void, Es...> push(T value) override {
            return this->_sink.push(std::move(value));
        }

        alp::Result<void, Es...> finish() && override {
            return std::move(this->_sink).finish();
        }

    private:
        S _sink;
    };

    std::unique_ptr<SinkBase<T, Es...>> _sink;
};
