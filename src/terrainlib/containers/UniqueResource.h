#pragma once

#include <concepts>
#include <optional>
#include <type_traits>
#include <utility>

#include <libassert/assert.hpp>

// Holds the resource of a UniqueResource, either engaged or empty.
template <typename S, typename Resource>
concept ResourceStorage = requires(S storage, const S const_storage, Resource resource) {
    { const_storage.has_value() } -> std::same_as<bool>;
    { *storage } -> std::same_as<Resource &>;
    { *const_storage } -> std::same_as<const Resource &>;
    storage.reset();
    storage.emplace(std::move(resource));
};

// Resource storage that marks the empty state with a flag.
template <typename Resource>
using OptionalStorage = std::optional<Resource>;

// Resource storage that marks the empty state with a sentinel value instead of a flag.
template <typename Resource, Resource sentinel>
class SentinelStorage {
public:
    SentinelStorage() noexcept = default;

    [[nodiscard]]
    bool has_value() const noexcept {
        return this->_resource != sentinel;
    }

    void reset() noexcept {
        this->_resource = sentinel;
    }

    void emplace(Resource resource) noexcept {
        this->_resource = resource;
    }

    Resource &operator*() noexcept {
        return this->_resource;
    }

    const Resource &operator*() const noexcept {
        return this->_resource;
    }

private:
    Resource _resource = sentinel;
};

// Resource storage for a pointer, which needs no flag because null marks the empty state.
template <typename T>
using PointerStorage = SentinelStorage<T *, nullptr>;

// A unique_ptr whose resource is an arbitrary value rather than a pointer.
template <typename Resource, typename Release, ResourceStorage<Resource> Storage = OptionalStorage<Resource>>
class UniqueResource {
    static_assert(std::is_nothrow_invocable_v<Release &, Resource &>, "the release of a resource must not throw");

public:
    UniqueResource() noexcept = default;

    UniqueResource(Resource resource, Release release) noexcept : _release(std::move(release)) {
        this->_storage.emplace(std::move(resource));
    }

    explicit UniqueResource(Resource resource) noexcept
        requires std::is_default_constructible_v<Release>
    {
        this->_storage.emplace(std::move(resource));
    }

    UniqueResource(UniqueResource &&other) noexcept
        : _release(std::move(other._release)),
          _storage(std::move(other._storage)) {
        other._storage.reset();
    }

    UniqueResource &operator=(UniqueResource &&other) noexcept {
        if (this == &other) {
            return *this;
        }

        this->reset();
        this->_release = std::move(other._release);
        this->_storage = std::move(other._storage);
        other._storage.reset();
        return *this;
    }

    // Before GCC 17, destroying a std::optional that was reset by hand gives a false -Wmaybe-uninitialized: https://gcc.gnu.org/bugzilla/show_bug.cgi?id=127005
    // Valgrind memcheck reports no uninitialized reads.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif
    ~UniqueResource() {
        this->reset();
    }
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif

    UniqueResource(const UniqueResource &) = delete;
    UniqueResource &operator=(const UniqueResource &) = delete;

    // Releases the resource, if one is still held.
    void reset() noexcept {
        if (this->_storage.has_value()) {
            this->_release(*this->_storage);
            this->_storage.reset();
        }
    }

    void reset(Resource resource) noexcept {
        this->reset();
        this->_storage.emplace(std::move(resource));
    }

    // Gives up the resource without releasing it.
    Resource take() noexcept {
        DEBUG_ASSERT(this->is_held(), "nothing to take from an empty resource");

        Resource resource = std::move(*this->_storage);
        this->_storage.reset();
        return resource;
    }

    [[nodiscard]]
    bool is_held() const noexcept {
        return this->_storage.has_value();
    }

    [[nodiscard]]
    bool is_empty() const noexcept {
        return !this->_storage.has_value();
    }

    explicit operator bool() const noexcept {
        return this->is_held();
    }

    [[nodiscard]]
    auto &get(this auto &&self) noexcept {
        DEBUG_ASSERT(self.is_held(), "nothing held by an empty resource");
        return *self._storage;
    }

    auto operator->(this auto &&self) noexcept {
        if constexpr (std::is_pointer_v<Resource>) {
            return self.get();
        } else {
            return &self.get();
        }
    }

    auto &operator*(this auto &&self) noexcept
        requires std::is_pointer_v<Resource>
    {
        return *self.get();
    }

private:
    [[no_unique_address]] Release _release{};
    Storage _storage{};
};

template <typename Resource, typename Release>
UniqueResource(Resource, Release) -> UniqueResource<Resource, Release>;

// A unique_ptr with a release of its own, without the flag of the general storage.
template <typename T, typename Release>
using UniquePointer = UniqueResource<T *, Release, PointerStorage<T>>;
