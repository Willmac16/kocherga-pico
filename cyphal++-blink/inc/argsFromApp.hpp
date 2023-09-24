#include <stdlib.h>
#include <stdint.h>
#include <cstdint>
#include <array>

constexpr uint32_t ARGS_ADDR = 0x2004'0000U - 0x0800;

/// The application may pass this structure when rebooting into the bootloader.
/// Feel free to modify the contents to suit your system.
/// It is a good idea to include an explicit version field here for future-proofing.
struct ArgumentsFromApplication
{
    std::uint8_t  args_from_app_version_major;
    std::uint16_t cyphal_serial_node_id;                    ///< Invalid if unknown.

    std::uint8_t                            cyphal_can_node_id;         ///< Invalid if unknown.
    std::uint16_t                 file_server_node_id;      ///< Invalid if unknown.
    std::array<std::uint8_t, 256> remote_file_path;         ///< Null-terminated string.
};

static_assert(std::is_trivial_v<ArgumentsFromApplication>);

class CRC64
{
public:
    static constexpr std::size_t Size = 8U;

    void update(const std::uint8_t* const data, const std::size_t len)
    {
        const auto* bytes = data;
        for (auto remaining = len; remaining > 0; remaining--)
        {
            crc_ ^= static_cast<std::uint64_t>(*bytes) << InputShift;
            ++bytes;
            // Unrolled for performance reasons. This path directly affects the boot-up time, so it is very
            // important to keep it optimized for speed. Rolling this into a loop causes a significant performance
            // degradation at least with GCC since the compiler refuses to unroll the loop when size optimization
            // is selected (which is normally used for bootloaders).
            crc_ = ((crc_ & Mask) != 0) ? ((crc_ << 1U) ^ Poly) : (crc_ << 1U);
            crc_ = ((crc_ & Mask) != 0) ? ((crc_ << 1U) ^ Poly) : (crc_ << 1U);
            crc_ = ((crc_ & Mask) != 0) ? ((crc_ << 1U) ^ Poly) : (crc_ << 1U);
            crc_ = ((crc_ & Mask) != 0) ? ((crc_ << 1U) ^ Poly) : (crc_ << 1U);
            crc_ = ((crc_ & Mask) != 0) ? ((crc_ << 1U) ^ Poly) : (crc_ << 1U);
            crc_ = ((crc_ & Mask) != 0) ? ((crc_ << 1U) ^ Poly) : (crc_ << 1U);
            crc_ = ((crc_ & Mask) != 0) ? ((crc_ << 1U) ^ Poly) : (crc_ << 1U);
            crc_ = ((crc_ & Mask) != 0) ? ((crc_ << 1U) ^ Poly) : (crc_ << 1U);
        }
    }

    /// The current CRC value.
    [[nodiscard]] auto get() const { return crc_ ^ Xor; }

    /// The current CRC value represented as a big-endian sequence of bytes.
    /// This method is designed for inserting the computed CRC value after the data.
    [[nodiscard]] auto getBytes() const -> std::array<std::uint8_t, Size>
    {
        auto                           x = get();
        std::array<std::uint8_t, Size> out{};
        const auto                     rend = std::rend(out);
        for (auto it = std::rbegin(out); it != rend; ++it)
        {
            *it = static_cast<std::uint8_t>(x);
            x >>= 8U;
        }
        return out;
    }

    /// True if the current CRC value is a correct residue (i.e., CRC verification successful).
    [[nodiscard]] auto isResidueCorrect() const { return crc_ == Residue; }

private:
    static constexpr auto Poly    = static_cast<std::uint64_t>(0x42F0'E1EB'A9EA'3693ULL);
    static constexpr auto Mask    = static_cast<std::uint64_t>(1) << 63U;
    static constexpr auto Xor     = static_cast<std::uint64_t>(0xFFFF'FFFF'FFFF'FFFFULL);
    static constexpr auto Residue = static_cast<std::uint64_t>(0xFCAC'BEBD'5931'A992ULL);

    static constexpr auto InputShift = 56U;

    std::uint64_t crc_ = Xor;
};

template <typename Container>
class VolatileStorage
{
public:
    /// The amount of memory required to store the data. This is the size of the container plus 8 bytes for the CRC.
    static constexpr auto StorageSize = sizeof(Container) + CRC64::Size;  // NOLINT

    explicit VolatileStorage(std::uint8_t* const location) : ptr_(location) {}

    /// Checks if the data is available and reads it, then erases the storage to prevent deja-vu.
    /// Returns an empty option if no data is available (in that case the storage is not erased).
    [[nodiscard]] auto take() -> std::optional<Container>
    {
        CRC64 crc;
        crc.update(ptr_, StorageSize);
        if (crc.isResidueCorrect())
        {
            Container out{};
            (void) std::memmove(&out, ptr_, sizeof(Container));
            (void) std::memset(ptr_, EraseFillValue, StorageSize);
            return out;
        }
        return {};
    }

    /// Writes the data into the storage with CRC64 protection.
    void store(const Container& data)
    {
        (void) std::memmove(ptr_, &data, sizeof(Container));
        CRC64 crc;
        crc.update(ptr_, sizeof(Container));
        const auto crc_ptr = ptr_ + sizeof(Container);  // NOLINT NOSONAR pointer arithmetic
        (void) std::memmove(crc_ptr, crc.getBytes().data(), CRC64::Size);
    }

protected:
    static constexpr std::uint8_t EraseFillValue = 0xCA;

    std::uint8_t* const ptr_;
};
