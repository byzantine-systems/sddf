/*
 * Copyright 2024, UNSW
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#define BLK_STORAGE_INFO_REGION_SIZE 0x1000

/* Device serial number max string length */
#define BLK_MAX_SERIAL_NUMBER 63

typedef struct blk_storage_info {
    char serial_number[BLK_MAX_SERIAL_NUMBER + 1];
    /* device does not accept write requests */
    bool read_only;
    /* whether this configuration is populated yet */
    bool ready;
    /* size of a sector, in bytes */
    uint16_t sector_size;
    /* optimal block size, specified in BLK_TRANSFER_SIZE sized units */
    uint16_t block_size;
    uint16_t queue_depth;
    /* geometry to guide FS layout */
    uint16_t cylinders, heads, blocks;
    /* total capacity of the device, specified in BLK_TRANSFER_SIZE sized units. */
    uint64_t capacity;
    /*
     * Chrysopolis (driver restart): incremented by the driver every time it
     * (re-)initialises the device.
     *
     * The `ready` flag alone cannot express a restart. The driver clears it and
     * re-sets it within a single init(), and the driver runs at a higher
     * priority than the virtualiser, so the virtualiser is never scheduled in
     * the gap and only ever observes ready = true. The sDDF design document
     * makes exactly this point about hotplug ("even if the client and/or
     * virtualiser only sees a movement from Ready to Ready it must be treated as
     * Ready -> Not Ready -> Ready") and prescribes a generation number as the
     * fix. A monotonically increasing counter is unambiguous regardless of
     * scheduling.
     *
     * Appended at the END of the struct, and the struct lives alone in a 4 KiB
     * region (see the static assert below), so this does not disturb the offsets
     * of any existing field for components that only read the earlier ones.
     * Upstream candidate.
     */
    uint64_t generation;
} blk_storage_info_t;
_Static_assert(sizeof(blk_storage_info_t) <= BLK_STORAGE_INFO_REGION_SIZE,
               "struct blk_storage_info must be smaller than the region size");

/**
 * Load from shared memory whether the block storage device is ready.
 * This does an atomic acquire operation.
 *
 * @param storage_info the block storage device to check
 * @return true the block storage device is ready to use
 * @return false the block storage device is not ready (removed, or initialising)
 */
static inline bool blk_storage_is_ready(blk_storage_info_t *storage_info)
{
    return __atomic_load_n(&storage_info->ready, __ATOMIC_ACQUIRE);
}

/**
 * Set shared memory whether the block storage device is ready.
 * This does an atomic release operation.
 */
static inline void blk_storage_set_ready(blk_storage_info_t *storage_info, bool ready)
{
    __atomic_store_n(&storage_info->ready, ready, __ATOMIC_RELEASE);
}

/**
 * Chrysopolis (driver restart): read the device's init generation.
 * Acquire, pairing with the release in blk_storage_next_generation, so a reader
 * that observes a new generation also observes every storage_info field the
 * driver wrote before bumping it.
 *
 * @param storage_info the block storage device to read
 * @return the number of times the driver has initialised the device
 */
static inline uint64_t blk_storage_generation(blk_storage_info_t *storage_info)
{
    return __atomic_load_n(&storage_info->generation, __ATOMIC_ACQUIRE);
}

/**
 * Chrysopolis (driver restart): advance the device's init generation.
 * Called by the driver at the end of every (re-)initialisation, BEFORE it sets
 * ready, so that any client which waits for ready is guaranteed to then read the
 * final generation rather than a torn intermediate one.
 *
 * @param storage_info the block storage device to update
 */
static inline void blk_storage_next_generation(blk_storage_info_t *storage_info)
{
    uint64_t next = __atomic_load_n(&storage_info->generation, __ATOMIC_RELAXED) + 1;
    __atomic_store_n(&storage_info->generation, next, __ATOMIC_RELEASE);
}
