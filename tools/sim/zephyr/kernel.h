/*
 * Host-build shim for <zephyr/kernel.h>.
 *
 * display.c only needs the fixed-width integer and bool types that the real
 * Zephyr header drags in.  Anything the firmware genuinely calls into the
 * kernel for belongs in a stub in sim_stubs.c instead, not here.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
