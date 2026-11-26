/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Martin Braunsperger
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

#include "../catch2_helpers.h"

#include <vector>

#include "containers/UniqueResource.h"

namespace {

struct Recorder {
    std::vector<int> released;
};

struct Release {
    Recorder *recorder = nullptr;

    void operator()(int &resource) const noexcept {
        this->recorder->released.push_back(resource);
    }
};

using Owner = UniqueResource<int, Release>;
using SentinelOwner = UniqueResource<int, Release, SentinelStorage<int, -1>>;

struct Node {
    int value;
};

struct ReleaseNode {
    Recorder *recorder = nullptr;

    void operator()(Node *&node) const noexcept {
        this->recorder->released.push_back(node->value);
    }
};

struct IgnoreNode {
    void operator()(Node *&) const noexcept {}
};

struct IgnoreVoid {
    void operator()(void *&) const noexcept {}
};

template <typename T>
concept Dereferenceable = requires(const T &value) { *value; };

} // namespace

TEST_CASE("UniqueResource default construction holds nothing") {
    const Owner owner;
    CHECK(owner.is_empty());
    CHECK(!owner.is_held());
    CHECK(!owner);
}

TEST_CASE("UniqueResource releases what it holds when destroyed") {
    Recorder recorder;
    {
        const Owner owner(7, Release{&recorder});
        CHECK(owner.is_held());
        CHECK(!owner.is_empty());
        CHECK(owner.get() == 7);
        CHECK(recorder.released.empty());
    }
    CHECK(recorder.released == std::vector<int>{7});
}

TEST_CASE("UniqueResource take gives up the resource without releasing it") {
    Recorder recorder;
    {
        Owner owner(7, Release{&recorder});
        CHECK(owner.take() == 7);
        CHECK(owner.is_empty());
    }
    CHECK(recorder.released.empty());
}

TEST_CASE("UniqueResource move construction empties the source") {
    Recorder recorder;
    {
        Owner source(7, Release{&recorder});
        const Owner target(std::move(source));
        CHECK(source.is_empty());
        CHECK(target.get() == 7);
        CHECK(recorder.released.empty());
    }
    CHECK(recorder.released == std::vector<int>{7});
}

TEST_CASE("UniqueResource move assignment releases what the target held") {
    Recorder recorder;
    {
        Owner target(1, Release{&recorder});
        Owner source(2, Release{&recorder});
        target = std::move(source);
        CHECK(recorder.released == std::vector<int>{1});
        CHECK(target.get() == 2);
        CHECK(source.is_empty());
    }
    CHECK(recorder.released == std::vector<int>{1, 2});
}

TEST_CASE("UniqueResource reset releases what it holds") {
    Recorder recorder;
    Owner owner(7, Release{&recorder});

    owner.reset();
    CHECK(owner.is_empty());
    CHECK(recorder.released == std::vector<int>{7});

    owner.reset();
    CHECK(recorder.released == std::vector<int>{7});
}

TEST_CASE("UniqueResource reset takes the new resource after releasing the old") {
    Recorder recorder;
    Owner owner(7, Release{&recorder});

    owner.reset(9);
    CHECK(owner.get() == 9);
    CHECK(recorder.released == std::vector<int>{7});
}

TEST_CASE("UniqueResource over a sentinel storage releases the same way") {
    Recorder recorder;
    {
        SentinelOwner owner(7, Release{&recorder});
        CHECK(owner.is_held());
        CHECK(owner.get() == 7);

        SentinelOwner target(std::move(owner));
        CHECK(owner.is_empty());
        CHECK(target.get() == 7);
    }
    CHECK(recorder.released == std::vector<int>{7});
}

TEST_CASE("UniquePointer reaches the resource through the pointer") {
    Node node{5};
    Recorder recorder;
    {
        const UniquePointer<Node, ReleaseNode> owner(&node, ReleaseNode{&recorder});
        CHECK(owner.get() == &node);
        CHECK(owner->value == 5);
        CHECK((*owner).value == 5);
    }
    CHECK(recorder.released == std::vector<int>{5});
}

TEST_CASE("UniquePointer costs no more than the pointer it holds") {
    STATIC_REQUIRE(sizeof(UniquePointer<Node, IgnoreNode>) == sizeof(Node *));
}

TEST_CASE("UniquePointer to void cannot be dereferenced") {
    STATIC_REQUIRE(!Dereferenceable<UniquePointer<void, IgnoreVoid>>);
    STATIC_REQUIRE(Dereferenceable<UniquePointer<Node, IgnoreNode>>);
}

TEST_CASE("every storage models the storage concept") {
    STATIC_REQUIRE(ResourceStorage<OptionalStorage<int>, int>);
    STATIC_REQUIRE(ResourceStorage<SentinelStorage<int, -1>, int>);
    STATIC_REQUIRE(ResourceStorage<PointerStorage<Node>, Node *>);
}
