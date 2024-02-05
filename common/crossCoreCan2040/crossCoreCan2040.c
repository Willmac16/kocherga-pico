#include "crossCoreCan2040.h"

#include "RP2040.h"

#include <stdio.h>
#include <stdlib.h>

#include <string.h>

#include "hardware/irq.h"
#include "hardware/sync.h"

#include "pico/platform.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/flash.h"

#include <can2040.h>


static struct can2040 cbus;

static uint32_t bitrate, gpio_rx, gpio_tx;

struct can2040mailbox {
    uint8_t flags; // 0x01 - written, 0x02 - read
    struct can2040_msg msg;
    uint64_t rx_stamp;
};

volatile struct can2040mailbox mailbox[CAN_MAILBOX_SIZE];

static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
    if (notify == CAN2040_NOTIFY_RX) {
        const uint8_t num_mailboxes = sizeof(mailbox) / sizeof(struct can2040mailbox);

        uint64_t rx_stamp = time_us_64();

        // Try to find a mailbox to put the msg in
        for (uint8_t mailbox_index = 0; mailbox_index < num_mailboxes; mailbox_index++) {
            // If mailbox is empty or marked read then use it (isn't unread)
            if ((mailbox[mailbox_index].flags & 0x03) != 0x01) {
                // Yes I know I am casting away volatile
                // The agreement that both the mailperson and mailbox owner have is such that
                // the mailperson will only write to the mailbox if it is marked read or empty
                // and the mailbox owner will only read from the mailbox if it is marked written.
                // This flip flop flag system keeps only one of the two writing to the mailbox at a time.
                struct can2040_msg * const m = (struct can2040_msg * const) &mailbox[mailbox_index].msg;
                memcpy(m, msg, sizeof(struct can2040_msg));
                mailbox[mailbox_index].rx_stamp = rx_stamp;
                mailbox[mailbox_index].flags = 0x01;

                __sev();

                return;
            }
        }

        // No available Mailbox, msg lost
        return;
    } else if (notify == CAN2040_NOTIFY_TX) {
        // Wake up core1 to load another message
        __sev();
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
    uint32_t sys_clock = 125000000;

    can2040_setup(&cbus, pio_num);
    can2040_callback_config(&cbus, can2040_cb);

    // Enable IRQs
    const IRQn_Type pio_irq = pio_num ? PIO1_IRQ_0_IRQn : PIO0_IRQ_0_IRQn;
    irq_set_exclusive_handler(pio_irq, PIOx_IRQHandler);
    NVIC_SetPriority(pio_irq, 1);
    NVIC_EnableIRQ(pio_irq);

    // Start CAN BUS
    can2040_start(&cbus, sys_clock, bitrate, gpio_rx, gpio_tx);

    return;
}

void multicore_can2040Init(void) {
    can2040Init();

    // flash_safe_execute_core_init();

    // Keep this core "busy" so it doesn't try to execute random memory
    while (1) {
        __wfe();
    }
}

void crossCoreCanInit(uint32_t br, uint32_t rx_pin, uint32_t tx_pin) {
    multicore_reset_core1();

    for (uint8_t i = 0; i < CAN_MAILBOX_SIZE; i++) {
        mailbox[i].flags = 0x00;
        mailbox[i].msg.id = 0x00000000;
        mailbox[i].msg.dlc = 0x00;
        mailbox[i].msg.data32[0] = 0x00000000;
        mailbox[i].msg.data32[1] = 0x00000000;
    }

    bitrate = br;
    gpio_rx = rx_pin;
    gpio_tx = tx_pin;

    extern uint32_t __StackOneBottom;
    uint32_t *stack_bottom = (uint32_t *) &__StackOneBottom;
    multicore_launch_core1_with_stack(multicore_can2040Init, stack_bottom, PICO_CORE1_STACK_SIZE);
}

bool crossCoreCanSend(const CanardFrame * const frame)
{
    const uint32_t *const data = (const uint32_t *) frame->payload;

    struct can2040_msg msg = {
        .id = frame->extended_can_id | CAN2040_ID_EFF,
        .dlc = CanardCANLengthToDLC[frame->payload_size],
        .data32 = { data[0], data[1] }
    };

    return !can2040_transmit(&cbus, &msg);
}

int crossCoreCanReceive(CanardFrame * const outFrame)
{
    static uint8_t mailbox_index = 0;
    const uint8_t num_mailboxes = sizeof(mailbox) / sizeof(struct can2040mailbox);

    for (mailbox_index = mailbox_index % num_mailboxes; mailbox_index < num_mailboxes; mailbox_index++) {
        // If mailbox is full and unread read it and mark as read
        if (((mailbox[mailbox_index].flags ^ 0x02) & 0x03) == 0x03) {
            const struct can2040_msg * const m = (struct can2040_msg *const) &mailbox[mailbox_index].msg;
            uint8_t len = CanardCANDLCToLength[m->dlc];
            memcpy((void * restrict) outFrame->payload, m->data, len);
            outFrame->extended_can_id = m->id & ~(CAN2040_ID_RTR | CAN2040_ID_EFF);
            outFrame->payload_size = len;

            // TODO: add stamp

            mailbox[mailbox_index].flags |= 0x02;

            return 1;
        }
    }

    // No unread messages
    // Set all non ID bits high to signify no message
    return 0;
}
