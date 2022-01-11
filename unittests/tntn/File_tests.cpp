#include <catch2/catch.hpp>

#include <fstream>
#include <filesystem>
#include <random>
#include <chrono>
//#include <boost/scope_exit.hpp>

#include <fmt/format.h>

#include "tntn/File.h"

namespace tntn {
namespace unittests {

namespace fs = std::filesystem;

namespace {
const auto milliseconds_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
static auto gen64 = std::mt19937_64(uint_fast64_t(milliseconds_since_epoch));
std::string unique_path() {
  return fmt::format("tntn_{}_{}", gen64(), gen64());
}

class FileKiller {
  fs::path m_file_path;
public:
  FileKiller(const fs::path& p) : m_file_path(p) {}
  ~FileKiller() {
    fs::remove(m_file_path);
    REQUIRE(!fs::exists(m_file_path));
  }
};
}


TEST_CASE("File open existing and read", "[tntn]")
{
    auto tempfilename = fs::temp_directory_path() / unique_path();
    const auto fk = FileKiller{tempfilename};

    File fout;
    REQUIRE(fout.open(tempfilename.c_str(), File::OM_RWC));
    CHECK(fout.write(0, "fooo", 4));
    CHECK(fout.write(3, "bar", 3));
    CHECK(fout.size() == 6);
    CHECK(fout.close());

    REQUIRE(fs::exists(tempfilename));

    File fin;
    REQUIRE(fin.open(tempfilename.c_str(), File::OM_R));
    CHECK(fin.size() == 6);
    std::string foobar;
    fin.read(0, foobar, 6);
    CHECK(fin.is_good());
    CHECK(foobar == "foobar");
}

TEST_CASE("File write past end", "[tntn]")
{
    auto tempfilename = fs::temp_directory_path() / unique_path();
    const auto fk = FileKiller{tempfilename};

    File fout;
    REQUIRE(fout.open(tempfilename.c_str(), File::OM_RWC));

    CHECK(fout.write(42, "foo", 3));

    std::vector<char> foo;
    fout.read(0, foo, 42 + 3);
    CHECK(foo.size() == 42 + 3);
}

TEST_CASE("getline on MemoryFile", "[tntn]")
{

    MemoryFile f;
    std::string foo_line = "foo\n";
    std::string bar_line = "bar\n";
    std::string baz_line = "baz";

    f.write(0, foo_line);
    f.write(f.size(), bar_line);
    f.write(f.size(), baz_line);

    REQUIRE(f.size() == foo_line.size() + bar_line.size() + baz_line.size());

    std::string line;
    FileLike::position_type from_offset = 0;
    from_offset = getline(from_offset, f, line);
    CHECK(from_offset < f.size());
    CHECK(line == "foo");

    from_offset = getline(from_offset, f, line);
    CHECK(from_offset < f.size());
    CHECK(line == "bar");

    from_offset = getline(from_offset, f, line);
    CHECK(from_offset == f.size());
    CHECK(line == "baz");
}

} //namespace unittests
} //namespace tntn
