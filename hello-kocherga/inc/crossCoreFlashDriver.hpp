#include "kocherga.hpp"

#define FLASH_MAILBOX_SIZE 8U

class PicoFlashBackend final : public kocherga::IROMBackend
{
public:
  void beginWrite() override;

  auto write(const std::size_t offset, const std::byte *const data, const std::size_t size) -> std::optional<std::size_t> override;

  auto
  read(const std::size_t offset, std::byte *const out_data, const std::size_t size) const -> std::size_t override;

  void endWrite() override;

  static void coreOneWorker();

private:
  uint32_t last_erased_sector;
};
