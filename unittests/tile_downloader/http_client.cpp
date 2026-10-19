#include "HttpClient.h"

#include <filesystem>
#include <stdexcept>
#include <string>

#include <catch2/catch_test_macros.hpp>

#include "../temp_directory.h"
#include "io/filesystem.h"

namespace {

// Returns the file URL of `path`.
std::string make_file_url(const std::filesystem::path &path) {
    return "file://" + io::filesystem::canonicalize(path).string();
}

class CallbackError : public std::runtime_error {
public:
    CallbackError()
        : std::runtime_error("callback failed") {}
};

}

TEST_CASE("http client propagates response writer exceptions") {
    const TempFile source("atb-http-client.txt", "response body");
    HttpClient client([](std::vector<char> &, const char *, size_t) {
        throw CallbackError();
    });

    CHECK_THROWS_AS(client.get(make_file_url(source.path())), CallbackError);
}

TEST_CASE("http client propagates progress callback exceptions") {
    const TempFile source("atb-http-client.txt", "response body");
    HttpClient client;

    CHECK_THROWS_AS(
        client.get(make_file_url(source.path()), [](double) {
            throw CallbackError();
        }),
        CallbackError);
}
