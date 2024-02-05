#include "hello-kocherga.hpp"
#include "can2040Driver.hpp"

#include <kocherga_can.hpp>

#include "RP2040.h"

#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/resets.h"
#include "hardware/exception.h"
#include "hardware/watchdog.h"

#include "pico/platform.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"

extern "C"
{
#include <can2040.h>
}

static struct can2040 cbus;

struct can2040mailbox
{
  uint8_t flags; // 0x01 - written, 0x02 - read
  struct can2040_msg msg;
};

volatile static struct can2040mailbox mailbox[CAN_MAILBOX_SIZE];

static volatile kocherga::can::ICANDriver::Bitrate bitrateArg;

static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
  if (notify == CAN2040_NOTIFY_RX)
  {

    const uint8_t num_mailboxes = sizeof(mailbox) / sizeof(struct can2040mailbox);

    // Try to find a mailbox to put the msg in
    for (uint8_t mailbox_index = 0; mailbox_index < num_mailboxes; mailbox_index++)
    {
      // If mailbox is empty or marked read then use it (isn't unread)
      if ((mailbox[mailbox_index].flags & 0x03) != 0x01)
      {
        // Yes I know I am casting away volatile
        // The agreement that both the mailperson and mailbox owner have is such that
        // the mailperson will only write to the mailbox if it is marked read or empty
        // and the mailbox owner will only read from the mailbox if it is marked written.
        // This flip flop flag system keeps only one of the two writing to the mailbox at a time.
        can2040_msg *const m = (can2040_msg *const)&mailbox[mailbox_index].msg;
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

void can2040Init(void)
{
  uint32_t pio_num = 1;
  uint32_t sys_clock = 125000000, bitrate = (uint32_t)bitrateArg.arbitration;

  can2040_setup(&cbus, pio_num);
  can2040_callback_config(&cbus, can2040_cb);

  // Enable irqs
  const IRQn_Type pio_irq = pio_num ? PIO1_IRQ_0_IRQn : PIO0_IRQ_0_IRQn;
  irq_set_exclusive_handler(pio_irq, PIOx_IRQHandler);
  NVIC_SetPriority(pio_irq, 1);
  NVIC_EnableIRQ(pio_irq);

  // Start canbus
  can2040_start(&cbus, sys_clock, bitrate, CAN2040_RX_PIN, CAN2040_TX_PIN);
}

void multicore_can2040Init(void)
{
  can2040Init();

  coreOneWork();
}

void ex_handler_hard_fault(void)
{
  // Shut off the CAN transceiver with the standby/speed control pin
  gpio_put(SPEED_CTRL, 1);

  __breakpoint();

  watchdog_reboot(0U, 0U, 0x7fffff);
}

auto Can2040Driver::configure(const Bitrate &bitrate,
                              const bool silent,
                              const kocherga::can::CANAcceptanceFilterConfig &filter) -> std::optional<Mode>
{
  bitrateArg.arbitration = bitrate.arbitration;

  // Bring the CAN Speed Control pin to ground for high speed operation
  gpio_init(SPEED_CTRL);
  gpio_set_dir(SPEED_CTRL, GPIO_OUT);
  gpio_put(SPEED_CTRL, 0);

  // Register the CAN Shutdown handler
  exception_set_exclusive_handler(HARDFAULT_EXCEPTION, ex_handler_hard_fault);

  tx_queue_.clear();
  multicore_reset_core1();

  extern uint32_t __StackOneBottom;
  uint32_t *stack_bottom = (uint32_t *)&__StackOneBottom;
  multicore_launch_core1_with_stack(multicore_can2040Init, stack_bottom, PICO_CORE1_STACK_SIZE);

  return Mode::Classic;
}

// Kocherga -> CAN2040 add to SW then "HW" FIFO
auto Can2040Driver::push(const bool force_classic_can,
                         const std::uint32_t extended_can_id,
                         const std::uint8_t payload_size,
                         const void *const payload) -> bool
{
  const std::chrono::microseconds now(time_us_64());

  // Add new message to Software FIFO and empty as much of the FIFO as possible
  const bool ok = tx_queue_.push(now, force_classic_can, extended_can_id, payload_size, payload);
  pollTxQueue(now);
  return ok;
}

// CAN2040 -> Kocherga pop from "HW" FIFO
auto Can2040Driver::pop(PayloadBuffer &payload_buffer) -> std::optional<std::pair<std::uint32_t, std::uint8_t>>
{
  const std::chrono::microseconds now(time_us_64());

  pollTxQueue(now);
  return unloadCanMailbox(payload_buffer.data());
}

int Can2040Driver::can2040Enqueue(const std::uint32_t extended_can_id,
                                  const std::uint8_t payload_size,
                                  const void *const payload)
{
  const uint32_t *const data = (const uint32_t *)payload;

  struct can2040_msg msg = {
      .id = extended_can_id | CAN2040_ID_EFF,
      .dlc = LengthToDLC[payload_size],
      .data32 = {data[0], data[1]}};

  return can2040_transmit(&cbus, &msg);
}

auto Can2040Driver::unloadCanMailbox(void *const payload) -> std::optional<std::pair<std::uint32_t, std::uint8_t>>
{
  static uint8_t mailbox_index = 0;
  const uint8_t num_mailboxes = sizeof(mailbox) / sizeof(struct can2040mailbox);

  for (mailbox_index = mailbox_index % num_mailboxes; mailbox_index < num_mailboxes; mailbox_index++)
  {
    // If mailbox is full and unread read it and mark as read
    if ((mailbox[mailbox_index].flags ^ 0x02) == 0x03)
    {
      const can2040_msg *const m = (can2040_msg *const)&mailbox[mailbox_index].msg;
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

void Can2040Driver::pollTxQueue(const std::chrono::microseconds now)
{
  if (const auto *const item = tx_queue_.peek())
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
