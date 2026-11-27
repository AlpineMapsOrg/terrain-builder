/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Martin Braunsperger
 * Copyright (C) 2026 Adam Celarek-Litofcenko
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *****************************************************************************/

#pragma once

#include <array>
#include <cstdint>
#include <expected>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

#include <curl/curl.h>
#include <fmt/format.h>

#include "CurlError.h"
#include "error/Context.h"
#include "error/MessageCode.h"
#include "error/Result.h"
#include "error/try.h"
#include "error/unwrap.h"
#include "error/utils.h"
#include "io/alloc.h"
#include "io/bytes.h"
#include "stop.h"
#include "type_utils.h"

namespace tile_downloader {

// cURL could not create a handle.
struct CurlInitError : alp::MessageCode<"cURL could not create a handle"> {};

// The server answered with an HTTP error status.
struct StatusError {
    long status_code;

    bool operator==(const StatusError &) const = default;
};

inline std::string format_as(const StatusError &error) {
    return fmt::format("the server answered with HTTP status {}", error.status_code);
}

// The transfer took longer than allowed.
struct TimedOut : alp::MessageCode<"the transfer took longer than allowed"> {};

// The response could not be received.
struct TransportError : alp::MessageCode<"the response could not be received"> {};

// What downloading into memory can fail with.
using GetError = alp::Error<StatusError, TimedOut, TransportError, io::AllocError>;

// What a finished transfer reported, apart from its body.
struct TransferInfo {
    std::string content_type;
    long status_code = 0;

    bool operator==(const TransferInfo &) const = default;
};

// A transfer whose body was kept in memory.
struct HttpResponse {
    TransferInfo info;
    io::bytes::vector body;
};

// Takes each chunk of a response body. Failing aborts the transfer.
template <typename F>
concept HttpSink = std::is_nothrow_invocable_v<F &, io::bytes::span>
    && is_specialization_of_v<std::invoke_result_t<F &, io::bytes::span>, std::expected>
    && std::is_void_v<typename std::invoke_result_t<F &, io::bytes::span>::value_type>;

// Learns the fraction downloaded, if the total is known. Returning false aborts the transfer.
template <typename F>
concept HttpProgress = std::is_nothrow_invocable_r_v<bool, F &, std::optional<double>>;

// The error a sink fails with.
template <HttpSink Sink>
using SinkError = typename std::invoke_result_t<Sink &, io::bytes::span>::error_type;

// What streaming a download into `Sink` can fail with.
template <HttpSink Sink>
using StreamError = alp::Error<StatusError, TimedOut, TransportError, Stopped, SinkError<Sink>>;

class HttpClient {
public:
    [[nodiscard]]
    static alp::Result<HttpClient, CurlInitError> make() {
        CURL *const curl = curl_easy_init();
        if (curl == nullptr) {
            return alp::make_error(CurlInitError{});
        }
        return HttpClient(Handle(curl, &curl_easy_cleanup));
    }

    // Downloads `url` into memory.
    [[nodiscard]]
    alp::Result<HttpResponse, GetError> get(const std::string &url) {
        HttpResponse response;
        const auto append = [&](const io::bytes::span chunk) noexcept {
            return io::try_append_range(response.body, chunk);
        };
        const auto keep_going = [](std::optional<double>) noexcept { return true; };
        ALP_TRY(response.info, alp::unwrap_partial<Stopped>(this->stream(url, append, keep_going)));
        return response;
    }

    // Downloads `url`, passing each chunk of the body to `sink` and the progress to `progress`.
    // Fails with Stopped once `progress` returns false.
    template <HttpSink Sink, HttpProgress Progress>
    [[nodiscard]]
    alp::Result<TransferInfo, StreamError<Sink>> stream(const std::string &url, Sink &&sink, Progress &&progress) {
        alp::Result<void, SinkError<Sink>> sink_result;
        const auto write = [&](const io::bytes::span chunk) noexcept {
            sink_result = sink(chunk);
            return sink_result.has_value();
        };
        alp::Result<void, Stopped> progress_result;
        const auto report = [&](const std::optional<double> fraction) noexcept {
            if (!progress(fraction)) {
                progress_result = alp::make_error(Stopped{});
            }
            return progress_result.has_value();
        };
        auto transfer_result = this->perform(url, write, report);
        // An aborted transfer fails with TransportError, so the cause it was aborted for comes first.
        const auto context = alp::make_context("downloading {}", url);
        ALP_TRY_CONTEXT(std::move(sink_result), context);
        ALP_TRY_CONTEXT(std::move(progress_result), context);
        ALP_TRY_CONTEXT(TransferInfo info, std::move(transfer_result), context);
        return info;
    }

private:
    using Handle = std::unique_ptr<CURL, decltype(&curl_easy_cleanup)>;

    explicit HttpClient(Handle curl) : _curl(std::move(curl)) {}

    Handle _curl;

    // Transfers `url`, passing each chunk of the body to `write` and the progress to `progress`.
    // A transfer that `write` or `progress` aborts fails with TransportError.
    template <typename Write, typename Progress>
    alp::Result<TransferInfo, StatusError, TimedOut, TransportError> perform(const std::string &url, Write &write, Progress &progress) {
        std::array<char, CURL_ERROR_SIZE> error_buffer{};
        CURL *const curl = this->_curl.get();
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, error_buffer.data());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &write_cb<Write>);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &write);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 5L);
        curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 5L);
        curl_easy_setopt(curl, CURLOPT_FAILONERROR, 1L);
        curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 0L);
        curl_easy_setopt(curl, CURLOPT_XFERINFOFUNCTION, &progress_cb<Progress>);
        curl_easy_setopt(curl, CURLOPT_XFERINFODATA, &progress);
        const CURLcode curl_code = curl_easy_perform(curl);
        // The buffer goes out of scope, so the handle must not keep pointing at it.
        curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, nullptr);

        TransferInfo info;
        char *content_type = nullptr;
        if (curl_easy_getinfo(curl, CURLINFO_CONTENT_TYPE, &content_type) == CURLE_OK && content_type != nullptr) {
            info.content_type = content_type;
        }
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &info.status_code);
        if (curl_code == CURLE_OK) {
            return info;
        }

        const std::string_view buffered_message(error_buffer.data());
        CurlError curl_error(curl_code, std::string(buffered_message.empty() ? curl_easy_strerror(curl_code) : buffered_message));
        if (curl_error == CURLE_OPERATION_TIMEDOUT) {
            return alp::make_error(TimedOut{}, std::move(curl_error));
        }
        if (curl_error == CURLE_HTTP_RETURNED_ERROR) {
            return alp::make_error(StatusError{info.status_code}, std::move(curl_error));
        }
        return alp::make_error(TransportError{}, std::move(curl_error));
    }

    template <typename Write>
    static size_t write_cb(char *data, size_t size, size_t count, void *user_data) noexcept {
        // Failure is reported through the return value: an exception thrown through cURL's C code would end the process.
        Write &write = *static_cast<Write *>(user_data);
        const size_t total = size * count;
        const io::bytes::span chunk(reinterpret_cast<const uint8_t *>(data), total);
        return write(chunk) ? total : 0;
    }

    template <typename Progress>
    static int progress_cb(void *user_data, curl_off_t download_total, curl_off_t download_now, curl_off_t /*upload_total*/, curl_off_t /*upload_now*/) noexcept {
        Progress &progress = *static_cast<Progress *>(user_data);
        std::optional<double> fraction;
        if (download_total > 0) {
            const double total = download_total;
            const double now = download_now;
            fraction = now / total;
        }
        return progress(fraction) ? 0 : 1;
    }
};

} // namespace tile_downloader
