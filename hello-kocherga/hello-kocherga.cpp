#include "RP2040.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
// #include <string.h>

#include "hardware/flash.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"

#include "pico/platform.h"
#include "pico/rand.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include <kocherga_serial.hpp>
#include <kocherga_can.hpp>
#include <o1heap.h>

extern "C" {
    #include <can2040.h>
}

#define APP_OFFSET 0x0004'0000U
#define APP_BASE (XIP_BASE + APP_OFFSET)

#define FLAG_VALUE 0xDEAD'BEEFU


static O1HeapInstance* oh_one_heap;

static uint8_t* raw_heap[2U * 1024U * sizeof(void*) * 4U] __attribute__((aligned(32U)));

void o1heapSetup() {
    assert(raw_heap != NULL);
    assert((((size_t) raw_heap) % O1HEAP_ALIGNMENT) == 0U);

    oh_one_heap = o1heapInit(raw_heap, sizeof(raw_heap));

    assert(oh_one_heap != NULL);
}

void* o1malloc(const size_t amount) {
    return o1heapAllocate(oh_one_heap, amount);
}

void o1free(void* const pointer) {
    return o1heapFree(oh_one_heap, pointer);
}

auto kocherga::getRandomByte() -> std::uint8_t
{
    return (uint8_t) get_rand_32() & 0xFF;
}

class PicoFlashBackend final : public kocherga::IROMBackend
{
    // void beginWrite() override
    // {

    // }

    // So this is a stupid way to do it, but its single erase, multi write flash backend
    auto write(const std::size_t offset, const std::byte* const data, const std::size_t size) -> std::optional<std::size_t> override
    {
        uint32_t intStatus = save_and_disable_interrupts();

        const uint32_t finalWriteSector = (APP_OFFSET + offset + size - 1) & ~(FLASH_SECTOR_SIZE - 1);

        if (finalWriteSector > last_erased_sector) {
            flash_range_erase(last_erased_sector + FLASH_SECTOR_SIZE, last_erased_sector - finalWriteSector);
            last_erased_sector = finalWriteSector;
        }

        for (uint32_t off = offset; off < offset + size; off += FLASH_PAGE_SIZE)
        {
            const uint32_t startOfWriteAddr = (APP_OFFSET + off);
            const uint32_t startOfWritePage = startOfWriteAddr & ~(FLASH_PAGE_SIZE - 1);
            uint32_t endOfWriteAddr = startOfWriteAddr + size - 1;

            if (endOfWriteAddr > startOfWritePage + FLASH_PAGE_SIZE - 1)
            {
                endOfWriteAddr = startOfWritePage + FLASH_PAGE_SIZE - 1;
            }

            uint8_t pageBuffer[FLASH_PAGE_SIZE];

            memset(pageBuffer, 0xFF, sizeof(pageBuffer));
            memcpy(pageBuffer + (startOfWriteAddr - startOfWritePage), data, endOfWriteAddr - startOfWriteAddr + 1);

            flash_range_program(startOfWriteAddr, (uint8_t *) pageBuffer, FLASH_PAGE_SIZE);
        }

        // TODO: check if write was successful--could just be checksum in sw, ideally like a DMA to interp checksum
        if (true)
        {
            restore_interrupts(intStatus);
            return size;
        }

        restore_interrupts(intStatus);
        return {};
    }

    auto read(const std::size_t offset, std::byte* const out_data, const std::size_t size) const -> std::size_t override
    {
        memcpy(out_data, (const void*) (APP_BASE + offset), size);
        return size;
    }

    uint32_t last_erased_sector = APP_OFFSET - FLASH_SECTOR_SIZE;
};


static struct can2040 cbus;

struct can2040mailbox {
    uint8_t flags; // 0x01 - written, 0x02 - read
    struct can2040_msg msg;
};

volatile struct can2040mailbox mailbox[32];

static volatile kocherga::can::ICANDriver::Bitrate bitrateArg;


static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
    if (notify == CAN2040_NOTIFY_RX) {
        // Try to find a mailbox to put the msg in
        for (uint8_t i = 0; i < sizeof(mailbox) / sizeof(struct can2040mailbox); i++) {
            // If mailbox is empty or marked read then use it (isn't unread)
            if ((mailbox[i].flags & 0x03) != 0x01) {
                // Yes I know I am casting away volatile
                // The agreement that both the mailperson and mailbox owner have is such that
                // the mailperson will only write to the mailbox if it is marked read or empty
                // and the mailbox owner will only read from the mailbox if it is marked written.
                // This flip flop flag system keeps only one of the two writing to the mailbox at a time.
                can2040_msg *const m = (can2040_msg *const) &mailbox[i].msg;
                memcpy(m, msg, sizeof(struct can2040_msg));
                mailbox[i].flags = 0x01;
                return;
            }
        }

        // No available Mailbox, msg lost
        return;
    }
}

static void
PIOx_IRQHandler(void)
{
    can2040_pio_irq_handler(&cbus);
}

void
can2040Init(void)
{
    uint32_t pio_num = 1;
    uint32_t sys_clock = 125000000, bitrate = (uint32_t) bitrateArg.arbitration;
    uint32_t gpio_rx = 4, gpio_tx = 5;

    can2040_setup(&cbus, pio_num);
    can2040_callback_config(&cbus, can2040_cb);

    // Enable irqs
    const IRQn_Type pio_irq = pio_num ? PIO1_IRQ_0_IRQn : PIO0_IRQ_0_IRQn;
    irq_set_exclusive_handler(pio_irq, PIOx_IRQHandler);
    NVIC_SetPriority(pio_irq, 1);
    NVIC_EnableIRQ(pio_irq);

    // Start canbus
    can2040_start(&cbus, sys_clock, bitrate, gpio_rx, gpio_tx);
}

void multicore_can2040Init(void) {
    can2040Init();

    spin_locks_reset();

    // Confirm startup through fifo flag exchange
    // multicore_fifo_push_blocking(FLAG_VALUE);
    // const uint32_t g = multicore_fifo_pop_blocking();
    // assert(g == ~FLAG_VALUE);

    // Keep this core "busy" so it doesnt try to execute random memory
    while (1) {
        tight_loop_contents();
    }
}

class Can2040Driver final : public kocherga::can::ICANDriver
{
    auto configure(const Bitrate&                                  bitrate,
                   const bool                                      silent,
                   const kocherga::can::CANAcceptanceFilterConfig& filter) -> std::optional<Mode> override
    {
        bitrateArg.arbitration = bitrate.arbitration;

        tx_queue_.clear();
        multicore_reset_core1();
        multicore_launch_core1(multicore_can2040Init);

        // Confirm can core startup through flag exchange
        // const uint32_t g = multicore_fifo_pop_blocking();
        // assert(g == FLAG_VALUE);
        // multicore_fifo_push_blocking(~FLAG_VALUE);

        return Mode::Classic;
    }

    // Kocherga -> CAN2040 add to SW then "HW" FIFO
    auto push(const bool force_classic_can,
              const std::uint32_t extended_can_id,
              const std::uint8_t  payload_size,
              const void* const   payload) -> bool override
    {
        const std::chrono::microseconds now(time_us_64());

        // Add new message to Software FIFO and empty as much of the FIFO as possible
        const bool ok = tx_queue_.push(now, force_classic_can, extended_can_id, payload_size, payload);
        pollTxQueue(now);
        return ok;
    }

    // CAN2040 -> Kocherga pop from "HW" FIFO
    auto pop(PayloadBuffer& payload_buffer) -> std::optional<std::pair<std::uint32_t, std::uint8_t>> override
    {
        const std::chrono::microseconds now(time_us_64());

        pollTxQueue(now);
        return unloadCanMailbox(payload_buffer.data());
    }

    int can2040Enqueue(const std::uint32_t extended_can_id,
                       const std::uint8_t  payload_size,
                       const void* const   payload)
    {
        const uint32_t *const data = (const uint32_t *) payload;

        struct can2040_msg msg = {
            .id = extended_can_id | CAN2040_ID_EFF,
            .dlc = LengthToDLC[payload_size],
            .data32 = { data[0], data[1] }
        };

        return can2040_transmit(&cbus, &msg);
    }

    auto unloadCanMailbox(void* const payload) -> std::optional<std::pair<std::uint32_t, std::uint8_t>>
    {
        for (uint8_t i = 0; i < sizeof(mailbox) / sizeof(can2040mailbox); i++) {
            // If mailbox is full and unread read it and mark as read
            if ((mailbox[i].flags ^ 0x02) == 0x03) {
                const can2040_msg *const m = (can2040_msg *const) &mailbox[i].msg;
                uint8_t len = DLCToLength[m->dlc];
                memcpy(payload, m->data, len);
                auto result = std::make_pair(m->id, len);

                mailbox[i].flags |= 0x02;

                return result;
            }
        }

        // No unread messages
        return {};
    }

    void pollTxQueue(const std::chrono::microseconds now)
    {
        if (const auto* const item = tx_queue_.peek())
        {
            const bool expired = now > (item->timestamp + kocherga::can::SendTimeout);
            if (expired || !can2040Enqueue(item->extended_can_id,
                                          item->payload_size,
                                          item->payload))
            {
                tx_queue_.pop();
            }
        }
    }

    kocherga::can::TxQueue<void*(*)(std::size_t), void(*)(void*)> tx_queue_ = {o1malloc, o1free};
};

struct kocherga::SystemInfo picoBoardSysInfo()
{
    struct kocherga::SystemInfo sysInf = {
        .hardware_version = {0, 1},
        .unique_id = {1, 2, 3, 4, 5, 6, 7, 8},
        .node_name = "PicoTestBoard"
    };

    return sysInf;
}

/// The application may pass this structure when rebooting into the bootloader.
/// Feel free to modify the contents to suit your system.
/// It is a good idea to include an explicit version field here for future-proofing.
// struct ArgumentsFromApplication
// {
//     std::uint32_t                           cyphal_can_bitrate_primary  = 1000000;
//     std::uint32_t                           cyphal_can_bitrate_flexible = 8000000;
//     std::uint8_t                            cyphal_can_not_dronecan = 1;    ///< 0xFF-unknown; 0-DroneCAN; 1-Cyphal/CAN.
//     std::uint8_t                            cyphal_can_node_id = 123;         ///< Invalid if unknown.

//     std::uint8_t                  trigger_node_index = 1;       ///< 0 - serial, 1 - CAN, >1 - none.
//     std::uint16_t                 file_server_node_id = 321;      ///< Invalid if unknown
//     std::array<std::uint8_t, 256> remote_file_path = {"/sbin/leet/hacks.bin"};

// };

struct ArgumentsFromApplication
{
    std::uint32_t                           cyphal_can_bitrate_primary;
    std::uint32_t                           cyphal_can_bitrate_flexible;
    std::uint8_t                            cyphal_can_not_dronecan;    ///< 0xFF-unknown; 0-DroneCAN; 1-Cyphal/CAN.
    std::uint8_t                            cyphal_can_node_id;         ///< Invalid if unknown.

    std::uint8_t                  trigger_node_index;       ///< 0 - serial, 1 - CAN, >1 - none.
    std::uint16_t                 file_server_node_id;      ///< Invalid if unknown
    std::array<std::uint8_t, 256> remote_file_path;

    ArgumentsFromApplication() = default;
};

// static assert trivially copyable for each field of ArgumentsFromApplication
static_assert(std::is_trivially_copyable<std::uint8_t>::value);
static_assert(std::is_trivially_copyable<std::uint16_t>::value);
static_assert(std::is_trivially_copyable<std::uint32_t>::value);
static_assert(std::is_trivially_copyable<std::array<std::uint8_t, 256>>::value);



static_assert(std::is_trivially_copyable<ArgumentsFromApplication>::value);
static_assert(std::is_trivially_default_constructible<ArgumentsFromApplication>::value);
static_assert(std::is_trivial_v<ArgumentsFromApplication>);


void picoRestart()
{
    // Reset the watchdog timer
    watchdog_enable(1, 1);
    while(1) {
        tight_loop_contents(); // Literally a no-op, but the one the SDK uses
    }
}

int main()
{
    o1heapSetup();

    // Check if the application has passed any arguments to the bootloader via shared RAM.
    // The address where the arguments are stored obviously has to be shared with the application.
    // If the application uses heap, then it might be a good idea to alias this area with the heap.
    std::optional<ArgumentsFromApplication> args =
        kocherga::VolatileStorage<ArgumentsFromApplication>(reinterpret_cast<std::uint8_t*>(0x2004'1800U)).take();

    // Initialize the bootloader core.
    PicoFlashBackend rom_backend;
    kocherga::SystemInfo system_info = picoBoardSysInfo();
    kocherga::Bootloader::Params params{.max_app_size = 0x0001'0000U, .linger = args.has_value()};  // Read the docs on the available params.
    kocherga::Bootloader boot(rom_backend, system_info, params);
    // It's a good idea to check if the app is valid and safe to boot before adding the nodes.
    // This way you can skip the potentially slow or disturbing interface initialization on the happy path.
    // You can do it by calling poll() here once.

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
    std::optional<std::uint8_t>                       cyphal_can_not_dronecan;
    std::optional<kocherga::NodeID>                   cyphal_can_node_id;

    can_bitrate = kocherga::can::ICANDriver::Bitrate{1'000'000U, 8'000'000U};
    cyphal_can_not_dronecan = 1;
    // cyphal_can_node_id = 111;

    if (args)
    {
        cyphal_can_node_id = args->cyphal_can_node_id;          // Will be ignored if invalid.
    }
    Can2040Driver can_driver;
    kocherga::can::CANNode can_node(can_driver,
                                    system_info.unique_id,
                                    can_bitrate,
                                    cyphal_can_not_dronecan,
                                    cyphal_can_node_id);

    spin_locks_reset();

    if (boot.addNode(&can_node)) {

        while (true)
        {
            const uint_fast64_t uptime = time_us_64();
            if (const auto fin = boot.poll(std::chrono::microseconds(uptime)))
            {
                if (*fin == kocherga::Final::BootApp)
                {
                    void (*app_start)(void) = (void (*)(void)) (APP_BASE + 64U);
                    app_start();
                }
                if (*fin == kocherga::Final::Restart)
                {
                    picoRestart();
                }
                assert(false);
            }
            // Trigger the update process internally if the required arguments are provided by the application.
            // The trigger method cannot be called before the first poll().
            if (args && (args->trigger_node_index < 2))
            {
                (void) boot.trigger(args->trigger_node_index,                   // Use serial or CAN?
                                    args->file_server_node_id,                  // Which node to download the file from?
                                    std::strlen((const char*) args->remote_file_path.data()), // Remote file path length.
                                    args->remote_file_path.data());
                args.reset();
            }
            // Sleep until the next hardware event (like reception of CAN frame or UART byte) but no longer than
            // 1 second. A fixed sleep is also acceptable but the resulting polling interval should be adequate
            // to avoid data loss (about 100 microseconds is usually ok).
            busy_wait_us_32(10U);
        }
    }

    picoRestart();
}
