#pragma once

#include <stdint.h>
#include <canard.h>

void crossCoreCanInit(uint32_t br, uint32_t rx_pin, uint32_t tx_pin);

bool crossCoreCanSend(const CanardFrame * const frame);

int crossCoreCanReceive(CanardFrame * const outFrame);

#define PICO_USE_STACK_GUARDS 1
#define PICO_STACK_SIZE _u(0x10000)
#define PICO_CORE1_STACK_SIZE _u(0x1000)

#define CAN_MAILBOX_SIZE 8U
