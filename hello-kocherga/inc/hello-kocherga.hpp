#include "hello-kocherga-defines.hpp"

static int const CAN_BITRATE = 1'000'000;

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

void *o1malloc(const size_t amount);

void o1free(void *const pointer);
