#include "hello-kocherga.hpp"

#include "RP2040.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "hardware/flash.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
#include "hardware/structs/xip_ctrl.h"
#include "hardware/resets.h"

#include "pico/platform.h"
#include "pico/rand.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include <kocherga_serial.hpp>
#include <kocherga_can.hpp>
#include <o1heap.h>

extern "C" {
    #include "lkb.h"
    #include <can2040.h>
}

static const uint32_t gpio_rx = 4, gpio_tx = 5;

static O1HeapInstance* oh_one_heap;

static uint8_t raw_heap[2U * 1024U * sizeof(void*) * 4U] __attribute__((used, section(".o1heap")));

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
    // So this is a stupid way to do it, but its single erase, multi write flash backend

    void beginWrite() override {
        // Tell the write function that it needs to erase anything it tries to write to
        last_erased_sector = APP_OFFSET - FLASH_SECTOR_SIZE;
    }

    auto write(const std::size_t offset, const std::byte* const data, const std::size_t size) -> std::optional<std::size_t> override
    {
        uint32_t intStatus = save_and_disable_interrupts();

        // Starting address of the last sector this write will touch
        const uint32_t finalWriteSector = (APP_OFFSET + offset + size - 1) & ~(FLASH_SECTOR_SIZE - 1);

        // Erase all yet to be erased sectors that this write needs
        if (finalWriteSector > last_erased_sector) {
            flash_range_erase(last_erased_sector + FLASH_SECTOR_SIZE, finalWriteSector - last_erased_sector);


            uint8_t erased = 0xFF;

            for (uint32_t i = last_erased_sector + FLASH_SECTOR_SIZE; i < finalWriteSector; i++)
            {
                const uint8_t * const xip_pointer = (uint8_t *) (XIP_NOCACHE_NOALLOC_BASE + i);

                // Check that we erased the sector
                erased &= * xip_pointer;
            }

            assert(erased == 0xFF);

            last_erased_sector = finalWriteSector;
        }


        // Write the data one page at a time
        for (uint32_t off = offset; off < offset + size; off += FLASH_PAGE_SIZE)
        {
            const uint32_t startOfWriteOffset = (APP_OFFSET + off);
            const uint32_t startOfWritePage = startOfWriteOffset & ~(FLASH_PAGE_SIZE - 1);

            assert(startOfWriteOffset >= startOfWritePage);

            // Clamp the memory we are writing to page boundaries
            uint32_t endOfWriteOffset = startOfWriteOffset + size; // (exclusive of last byte)
            if (endOfWriteOffset > startOfWritePage + FLASH_PAGE_SIZE)
            {
                endOfWriteOffset = startOfWritePage + FLASH_PAGE_SIZE;
            }

            uint8_t pageBuffer[FLASH_PAGE_SIZE];

            // "Erase" the contents of the page buffer so we don't touch the rest of the flash
            memset(pageBuffer, 0xFF, FLASH_PAGE_SIZE);

            assert(startOfWriteOffset - startOfWritePage < FLASH_PAGE_SIZE);

            assert(endOfWriteOffset - startOfWritePage <= FLASH_PAGE_SIZE);

            // Copy the page from data buffer into the page buffer
            memcpy(pageBuffer + (startOfWriteOffset - startOfWritePage), data + off - offset, endOfWriteOffset - startOfWriteOffset);

            bool matches = true;

            // Check that we copied what we wanted to
            for (uint32_t o = startOfWriteOffset; o < endOfWriteOffset && matches; o++)
            {
                const uint8_t * const buffer_pointer = (uint8_t *) pageBuffer + (o - startOfWritePage);
                const uint8_t * const data_pointer = (uint8_t *) data + o - startOfWriteOffset + off - offset;

                // Matches only stays true if the data we wrote matches the data we wanted to write
                matches &= (* buffer_pointer == * data_pointer);
            }

            if (!matches) {
                restore_interrupts(intStatus);
                return {};
            }

            // Write the page buffer to flash
            flash_range_program(startOfWritePage, (uint8_t *) pageBuffer, FLASH_PAGE_SIZE);

            // Check that we wrote what we wanted to
            // This also seeds the cache with the new contents of the flash
            for (uint32_t i = startOfWriteOffset; i < endOfWriteOffset && matches; i++)
            {
                const uint8_t * const xip_pointer = (uint8_t *) (XIP_NOCACHE_NOALLOC_BASE + i);
                const uint8_t * const data_pointer = (uint8_t *) (data + i - startOfWriteOffset + off - offset);

                // Matches only stays true if the data we wrote matches the data we wanted to write
                matches &= (* xip_pointer == * data_pointer);
            }

            if (!matches) {
                restore_interrupts(intStatus);
                return {};
            }
        }


        restore_interrupts(intStatus);
        return size;
    }

    auto read(const std::size_t offset, std::byte* const out_data, const std::size_t size) const -> std::size_t override
    {
        // Bypass XIP Caching Checking to ensure the app image isn't invalid at app launch
        // This forces a cache refresh of this access
        void * const src_addr = (void * const) (XIP_NOCACHE_BASE + APP_OFFSET + offset);

        memcpy(out_data, src_addr, size);
        return size;
    }

    uint32_t last_erased_sector;
};


static struct can2040 cbus;

struct can2040mailbox {
    uint8_t flags; // 0x01 - written, 0x02 - read
    struct can2040_msg msg;
};

volatile struct can2040mailbox mailbox[CAN_MAILBOX_SIZE];

static volatile kocherga::can::ICANDriver::Bitrate bitrateArg;


static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
    if (notify == CAN2040_NOTIFY_RX) {

        const uint8_t num_mailboxes = sizeof(mailbox) / sizeof(struct can2040mailbox);

        // Try to find a mailbox to put the msg in
        for (uint8_t mailbox_index = 0; mailbox_index < num_mailboxes; mailbox_index++) {
            // If mailbox is empty or marked read then use it (isn't unread)
            if ((mailbox[mailbox_index].flags & 0x03) != 0x01) {
                // Yes I know I am casting away volatile
                // The agreement that both the mailperson and mailbox owner have is such that
                // the mailperson will only write to the mailbox if it is marked read or empty
                // and the mailbox owner will only read from the mailbox if it is marked written.
                // This flip flop flag system keeps only one of the two writing to the mailbox at a time.
                can2040_msg * const m = (can2040_msg * const) &mailbox[mailbox_index].msg;
                memcpy(m, msg, sizeof(struct can2040_msg));
                mailbox[mailbox_index].flags = 0x01;

                __sev();

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

void multicore_can2040Stop(void) {
    // Pullup and take back from PIO both rx and tx pins
    gpio_init(gpio_rx);
    gpio_init(gpio_tx);

    can2040_stop(&cbus);

    while(1) {
        tight_loop_contents(); // Literally a no-op, but the one the SDK uses
    }
}

void multicore_can2040Init(void) {
    can2040Init();

    // Keep this core "busy" so it doesnt try to execute random memory
    while (1) {
        __wfe();
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

        extern uint32_t __StackOneBottom;
        uint32_t *stack_bottom = (uint32_t *) &__StackOneBottom;
        multicore_launch_core1_with_stack(multicore_can2040Init, stack_bottom, PICO_CORE1_STACK_SIZE);

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
        static uint8_t mailbox_index = 0;
        const uint8_t num_mailboxes = sizeof(mailbox) / sizeof(struct can2040mailbox);

        for (mailbox_index = mailbox_index % num_mailboxes; mailbox_index < num_mailboxes; mailbox_index++) {
            // If mailbox is full and unread read it and mark as read
            if ((mailbox[mailbox_index].flags ^ 0x02) == 0x03) {
                const can2040_msg *const m = (can2040_msg *const) &mailbox[mailbox_index].msg;
                uint8_t len = DLCToLength[m->dlc];
                memcpy(payload, m->data, len);
                auto result = std::make_pair(m->id, len);

                mailbox[mailbox_index].flags |= 0x02;

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
        .unique_id = {2, 2, 2, 2, 2, 2, 2, 2},
        .node_name = "org.cwrubaja.pico.testboard"
    };

    return sysInf;
}

struct ArgumentsFromApplication
{
    std::uint8_t  args_from_app_version_major;
    std::uint16_t cyphal_serial_node_id;                    ///< Invalid if unknown.

    std::uint8_t                            cyphal_can_node_id;         ///< Invalid if unknown.
    std::uint16_t                 file_server_node_id;      ///< Invalid if unknown.
    std::array<std::uint8_t, 256> remote_file_path;         ///< Null-terminated string.

    ArgumentsFromApplication() = default;
};

static_assert(std::is_trivial_v<ArgumentsFromApplication>);


void cleanUpKocherga() {
    // Wipe out anything the loader may accidentally be leaving behind for the app
    multicore_reset_core1();

    extern uint32_t __StackOneBottom;
    uint32_t *stack_bottom = (uint32_t *) &__StackOneBottom;
    multicore_launch_core1_with_stack(multicore_can2040Stop, stack_bottom, PICO_CORE1_STACK_SIZE);

    spin_locks_reset();

    * ((uint32_t *) XIP_CTRL_BASE) = 0x0000'0001U;

    return;
}

void picoRestart()
{
    cleanUpKocherga();

    // Reset the watchdog timer
    watchdog_enable(1, 0);
    while(1) {
        tight_loop_contents(); // Literally a no-op, but the one the SDK uses
    }
}

void app_start(void) {
    cleanUpKocherga();

    launch_kocherga_bin();
}

bool repeating_timer_callback(struct repeating_timer *t) {
    watchdog_update();
    return true;
}

int main()
{
    * ((uint32_t *) XIP_CTRL_BASE) = 0x0000'0000U;
    o1heapSetup();

    // Check if the application has passed any arguments to the bootloader via shared RAM.
    // The address where the arguments are stored obviously has to be shared with the application.
    // If the application uses heap, then it might be a good idea to alias this area with the heap.
    std::optional<ArgumentsFromApplication> args =
        kocherga::VolatileStorage<ArgumentsFromApplication>(reinterpret_cast<std::uint8_t*>(0x2004'0000U - 0x0800)).take();

    // Initialize the bootloader core.
    PicoFlashBackend rom_backend;
    kocherga::SystemInfo system_info = picoBoardSysInfo();
    kocherga::Bootloader::Params params{.max_app_size = 0x001c'0000U, .linger = false};  // Read the docs on the available params.
    kocherga::Bootloader boot(rom_backend, system_info, params);
    // It's a good idea to check if the app is valid and safe to boot before adding the nodes.
    // This way you can skip the potentially slow or disturbing interface initialization on the happy path.
    // You can do it by calling poll() here once.

    const uint_fast64_t uptime = time_us_64();
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
    can_bitrate = kocherga::can::ICANDriver::Bitrate{1'000'000U, 1'000'000U};
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


    spin_locks_reset();

    if (!boot.addNode(&can_node)) {
        picoRestart();
    }

    // struct repeating_timer timer;
    // add_repeating_timer_ms(-250, repeating_timer_callback, NULL, &timer);
    // watchdog_enable(500, 0);

    while (true)
    {
        const uint_fast64_t uptime = time_us_64();
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

        // Trigger the update process internally if the required arguments are provided by the application.
        // The trigger method cannot be called before the first poll().
        if (args)
        {
            (void) boot.trigger(&can_node,                   // Use serial or CAN?
                                args->file_server_node_id,                  // Which node to download the file from?
                                std::strlen((const char*) args->remote_file_path.data()), // Remote file path length.
                                args->remote_file_path.data());
            args.reset();
        }

        gpio_put(PICO_DEFAULT_LED_PIN, uptime & (1U << 20U));
        __wfe();
    }
}
