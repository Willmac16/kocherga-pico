#include <stdlib.h>
#include <stdint.h>
#include <cstdint>
#include <array>

struct AppDescriptor
{
    std::uint64_t               magic = 0x5E44'1514'6FC0'C4C7ULL;
    std::array<std::uint8_t, 8> signature{{'A', 'P', 'D', 'e', 's', 'c', '0', '0'}};

    std::uint64_t                               image_crc  = 0;     // Populated after build
    std::uint32_t                               image_size = 0;     // Populated after build
    [[maybe_unused]] std::array<std::byte, 4>   _reserved_a{};
    std::uint8_t                                version_major = SOFTWARE_VERSION_MAJOR;
    std::uint8_t                                version_minor = SOFTWARE_VERSION_MINOR;
    std::uint8_t                                flags =
#if RELEASE_BUILD
        Flags::ReleaseBuild
#else
        0
#endif
    ;
    [[maybe_unused]] std::array<std::byte, 1>   _reserved_b{};
    std::uint32_t                               build_timestamp_utc = TIMESTAMP_UTC;
    std::uint64_t                               vcs_revision_id     = GIT_HASH;
    [[maybe_unused]] std::array<std::byte, 16>  _reserved_c{};

    struct Flags
    {
        static constexpr std::uint8_t ReleaseBuild = 1U;
        static constexpr std::uint8_t DirtyBuild   = 2U;
    };

    [[maybe_unused]] std::array<std::byte, 192>  _reserved_d{}; // 192 Bytes reserved to maintain the 256 byte alignment the Vector Table usually gets
};
