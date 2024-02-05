#include "crossCoreFlashDriver.hpp"
#include "hello-kocherga.hpp"

#include "hardware/flash.h"
#include "hardware/sync.h"

#include "pico/multicore.h"

enum FlashFlags
{
  LOADING = 0x00,
  QUEUED = 0x01,
  WRITTEN = 0x02
};

struct FlashSectorMailbox
/**
 * @brief Structure representing the flash driver.
 *
 * This structure contains the following members:
 * - flags: FlashFlags object representing the current status of this flash mailbox
 * - data: Array of uint8_t representing the data buffer for flash operations.
 * - sector_addr: uint32_t representing the address of the flash sector.
 */
{
  FlashFlags flags;
  uint8_t data[FLASH_SECTOR_SIZE];
  uint32_t sector_addr;
};

volatile static FlashSectorMailbox mailbox[FLASH_MAILBOX_SIZE];

void PicoFlashBackend::beginWrite()
{
  // Clear out all the mailboxes
  for (uint8_t mailbox_index = 0; mailbox_index < FLASH_MAILBOX_SIZE; mailbox_index++)
  {
    mailbox[mailbox_index].flags = WRITTEN;
  }
}

auto PicoFlashBackend::write(const std::size_t offset, const std::byte *const data, const std::size_t size) -> std::optional<std::size_t>
{
  // Assert that the write is contained within a single sector
  const uint32_t startOfWriteSector = (APP_OFFSET + offset) & ~(FLASH_SECTOR_SIZE - 1);
  const uint32_t endOfWriteSector = (APP_OFFSET + offset + size - 1) & ~(FLASH_SECTOR_SIZE - 1);
  assert(startOfWriteSector == endOfWriteSector);

  // Since the write is in a single sector, we will only use one mailbox
  static uint8_t mailbox_index = 0;

  bool satisfied = false;

  while (!satisfied)
  {
    if (mailbox[mailbox_index].flags == WRITTEN)
    {
      // Take over this mailbox
      mailbox[mailbox_index].flags = QUEUED;
      mailbox[mailbox_index].sector_addr = startOfWriteSector;

      satisfied = true;
    }
    else
    {
      // Check if the mailbox matches the sector we want to write to
      if (mailbox[mailbox_index].sector_addr == startOfWriteSector)
      {
        satisfied = true;
      }
      else
      {
        // Set the mailbox we are leaving as QUEUED
        mailbox[mailbox_index].flags = QUEUED;
        __sev();

        // Increment the mailbox index
        mailbox_index++;
        mailbox_index %= FLASH_MAILBOX_SIZE;
      }
    }
  }

  // Copy the data into the mailbox
  uint8_t *const payload_ptr = (uint8_t *const)&mailbox[mailbox_index].data;

  uint8_t *const off_payload_ptr = payload_ptr + (offset & (FLASH_SECTOR_SIZE - 1));
  memcpy(off_payload_ptr, data, size);

  return size;
}

auto PicoFlashBackend::read(const std::size_t offset, std::byte *const out_data, const std::size_t size) const -> std::size_t
{
  // Bypass XIP Caching Checking to ensure the app image isn't invalid at app launch
  // This forces a cache refresh of this access
  void *const src_addr = (void *const)(XIP_NOCACHE_BASE + APP_OFFSET + offset);

  memcpy(out_data, src_addr, size);
  return size;
}

void PicoFlashBackend::endWrite()
{
  // This stalls until all mailboxes have been written to flash
  for (uint8_t mailbox_index = 0; mailbox_index < FLASH_MAILBOX_SIZE; mailbox_index++)
  {
    while (mailbox[mailbox_index].flags == QUEUED)
    {
      tight_loop_contents();
    }
  }

  return;
}

void writeMailboxToFlash(FlashSectorMailbox *const box)
{
  if (box->flags != QUEUED)
  {
    return;
  }

  uint32_t ints = save_and_disable_interrupts();

  // Erase the sector
  flash_range_erase(box->sector_addr, FLASH_SECTOR_SIZE);

  // Write the data
  flash_range_program(box->sector_addr, box->data, FLASH_SECTOR_SIZE);

  restore_interrupts(ints);

  // Mark the sector as written
  box->flags = WRITTEN;
}

void PicoFlashBackend::coreOneWorker()
{

  // Infinite loop over all the mailboxes
  for (uint8_t mailbox_index = 0;; mailbox_index++)
  {
    // Modulo to ensure our index wraps around
    mailbox_index %= FLASH_MAILBOX_SIZE;

    // The function checks if the mailbox has already been written so we don't need to
    writeMailboxToFlash((FlashSectorMailbox *const)&mailbox[mailbox_index]);

    // Empty MultiCore FIFO if not
    if (multicore_fifo_rvalid())
    {
      multicore_fifo_drain();
    }
  }
}
