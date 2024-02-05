#include "hello-kocherga.hpp"
#include "can2040Driver.hpp"
#include "crossCoreFlashDriver.hpp"

#include "RP2040.h"

#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
#include "hardware/structs/xip_ctrl.h"
#include "hardware/resets.h"

#include "pico/platform.h"
#include "pico/rand.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/unique_id.h"

#include <kocherga_serial.hpp>
#include <o1heap.h>

extern "C"
{
#include "lkb.h"
}

static O1HeapInstance *oh_one_heap;

static uint8_t raw_heap[2U * 1024U * sizeof(void *) * 4U] __attribute__((used, section(".o1heap")));

void o1heapSetup()
{
  assert(raw_heap != NULL);
  assert((((size_t)raw_heap) % O1HEAP_ALIGNMENT) == 0U);

  oh_one_heap = o1heapInit(raw_heap, sizeof(raw_heap));

  assert(oh_one_heap != NULL);
}

void *o1malloc(const size_t amount)
{
  return o1heapAllocate(oh_one_heap, amount);
}

void o1free(void *const pointer)
{
  return o1heapFree(oh_one_heap, pointer);
}

auto kocherga::getRandomByte() -> std::uint8_t
{
  static uint32_t loaf = get_rand_32();
  static uint8_t piece = 0U;

  if (piece == 4U)
  {
    loaf = get_rand_32();
    piece = 0U;
  }

  return (uint8_t)loaf >> (8U * piece++) && 0xFFU;
}

struct kocherga::SystemInfo picoBoardSysInfo()
{
  pico_unique_board_id_t flash_unique_id;
  pico_get_unique_board_id(&flash_unique_id);

  std::array<uint8_t, 16> unique_id;

  for (uint8_t i = 0; i < 8; i++)
  {
    unique_id[2 * i] = flash_unique_id.id[2];
  }

  struct kocherga::SystemInfo sysInf = {
      .hardware_version = {0, 1},
      .unique_id = unique_id,
      .node_name = "org.cwrubaja.pico.testboard"};

  return sysInf;
}

struct ArgumentsFromApplication
{
  std::uint8_t args_from_app_version_major;
  std::uint16_t cyphal_serial_node_id; ///< Invalid if unknown.

  std::uint8_t cyphal_can_node_id;                ///< Invalid if unknown.
  std::uint16_t file_server_node_id;              ///< Invalid if unknown.
  std::array<std::uint8_t, 256> remote_file_path; ///< Null-terminated string.

  ArgumentsFromApplication() = default;
};

static_assert(std::is_trivial_v<ArgumentsFromApplication>);

// Wipe out anything the loader may accidentally be leaving behind for the app
void cleanUpKocherga()
{
  // Disconnect and suspend the CAN Transciever
  gpio_put(SPEED_CTRL, 1);
  gpio_deinit(CAN2040_RX_PIN);
  gpio_deinit(CAN2040_TX_PIN);

  // Reset Core 1
  multicore_reset_core1();

  // Re-Enable the XIP cache
  xip_ctrl_hw->ctrl = 0x01;

  return;
}

void picoRestart()
{
  cleanUpKocherga();

  // Reset the watchdog timer
  watchdog_reboot(0U, 0U, 0x7fffff);
}

void app_start(void)
{
  cleanUpKocherga();

  launch_kocherga_bin();
}

void handleFin(kocherga::Bootloader &boot, uint_fast64_t uptime)
{
  if (const auto fin = boot.poll(std::chrono::microseconds(uptime)))
  {
    if (*fin == kocherga::Final::BootApp)
    {
      app_start();
    }
    else if (*fin == kocherga::Final::Restart)
    {
      picoRestart();
    }
    // Restart or boot returned. This is bad
    assert(false);
  }
}

// Function called by cross core can after it has been initialized
// This must not return
void coreOneWork()
{
  PicoFlashBackend::coreOneWorker();
}

int main()
{
  watchdog_enable(1000, 1);

  // Disable & Flush XIP Cache
  xip_ctrl_hw->ctrl = 0x00;
  xip_ctrl_hw->flush = 1;

  o1heapSetup();

  // Check if the application has passed any arguments to the bootloader via shared RAM.
  // The address where the arguments are stored obviously has to be shared with the application.
  // If the application uses heap, then it might be a good idea to alias this area with the heap.
  std::optional<ArgumentsFromApplication> args =
      kocherga::VolatileStorage<ArgumentsFromApplication>(reinterpret_cast<std::uint8_t *>(0x2004'0000U - 0x0800)).take();

  // Set to linger if we have args
  bool linger = false;

  if (args)
  {
    linger = true;
  }

  // Initialize the bootloader core.
  PicoFlashBackend rom_backend;
  kocherga::SystemInfo system_info = picoBoardSysInfo();
  kocherga::Bootloader::Params params{
      .max_app_size = 0x001c'0000U, .linger = linger, /* .boot_delay = std::chrono::seconds(1) */}; // Read the docs on the available params.
  kocherga::Bootloader boot(rom_backend, system_info, params);
  // It's a good idea to check if the app is valid and safe to boot before adding the nodes.
  // This way you can skip the potentially slow or disturbing interface initialization on the happy path.
  // You can do it by calling poll() here once.

  if (!args)
  {
    const uint_fast64_t uptime = time_us_64();
    handleFin(boot, uptime);
  }

  // // Add a Cyphal/serial node to the bootloader instance.
  // MySerialPort serial_port;
  // kocherga::serial::SerialNode serial_node(serial_port, system_info.unique_id);
  // if (args && (args->cyphal_serial_node_id <= kocherga::serial::MaxNodeID))
  // {
  //     serial_node.setLocalNodeID(args->cyphal_serial_node_id);
  // }
  // boot.addNode(&serial_node);

  // Add a Cyphal/CAN node to the bootloader instance.
  std::optional<kocherga::can::ICANDriver::Bitrate> can_bitrate;
  std::optional<std::uint8_t> cyphal_can_not_dronecan;
  std::optional<kocherga::NodeID> cyphal_can_node_id;
  can_bitrate = kocherga::can::ICANDriver::Bitrate{CAN_BITRATE, CAN_BITRATE};
  cyphal_can_not_dronecan = 1;

  if (args)
  {
    cyphal_can_node_id = args->cyphal_can_node_id;
  }
  Can2040Driver can_driver;
  kocherga::can::CANNode can_node(can_driver,
                                  system_info.unique_id,
                                  can_bitrate,
                                  cyphal_can_not_dronecan,
                                  cyphal_can_node_id);

  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

  if (!boot.addNode(&can_node))
  {
    picoRestart();
  }

  // Trigger the update process internally if the required arguments are provided by the application.
  // The trigger method cannot be called before the first poll().
  if (args)
  {
    handleFin(boot, time_us_64());

    (void)boot.trigger(&can_node,                                                // Use serial or CAN?
                       args->file_server_node_id,                                // Which node to download the file from?
                       std::strlen((const char *)args->remote_file_path.data()), // Remote file path length.
                       args->remote_file_path.data());
    args.reset();
  }

  while (true)
  {
    const uint_fast64_t uptime = time_us_64();
    handleFin(boot, uptime);

    gpio_put(PICO_DEFAULT_LED_PIN, uptime & (1U << 20U));

    watchdog_update();

    absolute_time_t deadline = make_timeout_time_us(100);
    best_effort_wfe_or_timeout(deadline);
  }
}
