/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hardware/structs/scb.h"

#include "pico/stdlib.h"

#include <stdlib.h>
#include <stdint.h>
#include <cstdint>

#define PROBLEM_IRQ 3U

extern "C" {
    #include "lkb.h"
}

int main() {
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    gpio_put(LED_PIN, 1);
    sleep_ms(250);
    gpio_put(LED_PIN, 0);
    sleep_ms(250);

    auto_launch();
}
