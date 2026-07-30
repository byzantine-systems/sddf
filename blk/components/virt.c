/*
 * Copyright 2024, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <os/sddf.h>
#include <stdint.h>
#include <stdbool.h>
#include <sddf/util/cache.h>
#include <sddf/util/printf.h>
#include <string.h>
#include <sddf/util/util.h>

#include "virt.h"

#define DRIVER_MAX_NUM_BUFFERS 1024

__attribute__((__section__(".blk_virt_config"))) blk_virt_config_t config;

/* Driver queue handle */
blk_queue_handle_t drv_h;
/* Client queue handles */
blk_queue_handle_t client_queues[SDDF_BLK_MAX_CLIENTS];

/* Request info to be bookkept from client */
typedef struct reqbk {
    uint32_t cli_id;
    uint32_t cli_req_id;
    uintptr_t vaddr;
    uint16_t count;
    blk_req_code_t code;
} reqbk_t;
static reqbk_t reqsbk[DRIVER_MAX_NUM_BUFFERS];

/* Index allocator for driver request id */
ialloc_t ialloc;
static uint32_t ialloc_idxlist[DRIVER_MAX_NUM_BUFFERS];

bool initialised = false;

/*
 * Chrysopolis (driver restart): the driver's init generation as of the last time
 * we looked. A change means the driver re-initialised (crashed and was restarted
 * by the Root fault handler), which invalidates every request we have
 * outstanding. See blk_driver_restarted() below.
 */
static uint64_t last_seen_generation;

/*
 * Chrysopolis (driver restart): which ialloc ids correspond to a real CLIENT
 * request, i.e. which entries of reqsbk[] are meaningful.
 *
 * ialloc is shared: blk/components/partitioning.c allocates from it for the
 * virtualiser's OWN internal MBR/GPT scan requests, and those never get a
 * reqsbk[] entry (only handle_client() writes those). So "allocated" alone does
 * not imply "a client is waiting on it", and failing an internal request back
 * as though it were a client's sends a response carrying an uninitialised
 * cli_id/cli_req_id. Downstream that is not a harmless no-op: the fat server
 * indexes request_pool[id] with no validation and signals that worker's
 * semaphore, so a bogus id faults fatfs outright.
 */
static bool reqsbk_valid[DRIVER_MAX_NUM_BUFFERS];

/*
 * Chrysopolis (driver restart): the channel the Root fault handler signals when
 * it has PERMANENTLY stopped the driver, having spent the whole restart budget
 * without it coming back. Pinned in tools/sdf/system.zig, the other half of this
 * ABI: it is not an sDDF connection, so no config blob carries it.
 *
 * This closes the gap the sDDF design document lists as an open limitation of
 * its own hotplug design, "Initialisation failures are not communicated to the
 * clients" (sDDF Design, Release 0.6, S6.4). A driver that never re-initialises
 * is exactly that case. Without a signal we would sit waiting for a generation
 * bump that can no longer happen, leaving every outstanding request unanswered
 * and every client blocked on it forever.
 */
#define BLK_VIRT_DRIVER_GONE_CH 61

/*
 * Chrysopolis (driver restart): set once root has stopped the driver for good.
 *
 * One-way by design. Root only stops a child after spending its entire restart
 * budget, so there is no path back to a working driver within this boot, and a
 * flag that could clear would just reopen the window it exists to close.
 */
static bool driver_gone;

void init(void)
{
    assert(blk_config_check_magic(&config));

    blk_storage_info_t *driver_storage_info = config.driver.conn.storage_info.vaddr;
    while (!blk_storage_is_ready(driver_storage_info));

    /*
     * Chrysopolis (driver restart): sample the generation only AFTER the ready
     * spin above. The driver bumps the generation before publishing ready, so
     * waiting for ready guarantees we read the final value and do not
     * immediately mistake the driver's own cold-boot init for a restart.
     */
    last_seen_generation = blk_storage_generation(driver_storage_info);

    /* Initialise client queues */
    for (int i = 0; i < config.num_clients; i++) {
        blk_virt_config_client_t *client = &config.clients[i];
        blk_req_queue_t *curr_req = client->conn.req_queue.vaddr;
        blk_resp_queue_t *curr_resp = client->conn.resp_queue.vaddr;
        uint32_t queue_capacity = client->conn.num_buffers;
        blk_queue_init(&client_queues[i], curr_req, curr_resp, queue_capacity);
    }

    /* Initialise driver queue */
    uint16_t driver_num_buffers = config.driver.conn.num_buffers;
    assert(driver_num_buffers <= DRIVER_MAX_NUM_BUFFERS);
    blk_queue_init(&drv_h, config.driver.conn.req_queue.vaddr, config.driver.conn.resp_queue.vaddr, driver_num_buffers);

    /* Initialise index allocator */
    ialloc_init(&ialloc, ialloc_idxlist, DRIVER_MAX_NUM_BUFFERS);

    virt_partition_init();
}

/*
 * Chrysopolis (driver restart): reconcile after the driver has re-initialised.
 *
 * The driver discards the request ring during its init(), so every request we
 * still have outstanding is now guaranteed dead: it will never be completed for
 * real. Fail them all back to their owning clients.
 *
 * This is what stops a driver crash from wedging the system. A lost completion
 * parks the fatfs worker that issued it forever, and beam_server's
 * fs_blocking_wait() polls WITHOUT yielding, so one orphaned request takes down
 * the whole image rather than failing a single fs read. An error completion, by
 * contrast, unparks the worker with a status fat maps to FS_STATUS_ERROR, which
 * the client can retry. "Pending jobs are discarded, clients will reissue"
 * (Shen & Elkaduwe, seL4 Summit 2025).
 *
 * ORDER MATTERS. handle_driver() must run first, before we fail anything:
 * responses the dying instance managed to enqueue are still sitting in the
 * shared response ring with perfectly valid ids. Draining them first completes
 * those requests properly AND frees their ialloc slots, so the sweep below sees
 * only genuinely orphaned ids. Failing first would leave those stale responses
 * to be processed afterwards against ids we had already freed and possibly
 * reallocated -- a double ialloc_free and a response delivered to the wrong
 * client request.
 */
static void handle_driver();

static void blk_reconcile_outstanding(const char *reason)
{
    LOG_BLK_VIRT("%s, reconciling outstanding requests\n", reason);

    /* 1. Drain real responses the previous instance left behind (see above). */
    handle_driver();

    /* 2. Fail whatever is still outstanding back to its owning client. */
    bool client_notify[SDDF_BLK_MAX_CLIENTS];
    memset(client_notify, 0, SDDF_BLK_MAX_CLIENTS);

    unsigned int failed = 0;
    unsigned int reclaimed = 0;
    for (uint32_t id = 0; id < DRIVER_MAX_NUM_BUFFERS; id++) {
        if (!ialloc_in_use(&ialloc, id)) {
            continue;
        }

        /*
         * A full sweep like this one needs ialloc_in_use() to be exact, and as
         * of sDDF 0.7.0 it is. Before 0.7.0 the predicate was "idxlist[id] ==
         * -1" and the free list's tail also held -1, so the tail read as in use
         * and this loop had to skip it by hand: freeing it created a self-loop
         * in the free list, after which ialloc_alloc handed the same id to two
         * requests, one client received another's response, and the second
         * completion tripped the assert in handle_driver(), faulting this PD,
         * which is top-level and has no fault handler to restart it.
         *
         * 0.7.0 changed the sentinel: ialloc_init leaves the last slot at
         * `capacity` and ialloc_free writes "idxlist[id] = ia->capacity", so
         * only allocated ids hold -1 and the predicate is now unambiguous. The
         * hand-written guard that used to sit here has been dropped along with
         * the ialloc_t fields it read (`offset` is gone, `size` now counts
         * allocated rather than total).
         */

        /* Only ids that name a real CLIENT request get a response. An allocated
         * id with no reqsbk entry belongs to the virtualiser's own partition
         * scan (partitioning.c allocates from this same ialloc), and reqsbk[id]
         * would be uninitialised garbage -- responding with it corrupts a
         * client's request stream. Its slot is still reclaimed below: the reply
         * it was waiting for died with the driver, so leaving it allocated would
         * leak an ialloc slot on every restart. */
        if (reqsbk_valid[id]) {
            reqbk_t reqbk = reqsbk[id];
            /* The response queue can never be full here: the number of in-flight
             * requests is bounded by the ialloc size, which is bounded by the
             * response queue capacity. Same invariant handle_driver() relies
             * on. */
            int rerr = blk_enqueue_resp(&client_queues[reqbk.cli_id], BLK_RESP_ERR_UNSPEC, 0, reqbk.cli_req_id);
            assert(!rerr);
            client_notify[reqbk.cli_id] = true;
            reqsbk_valid[id] = false;
            failed++;
        } else {
            reclaimed++;
        }

        int err = ialloc_free(&ialloc, id);
        assert(!err);
    }

    for (int i = 0; i < config.num_clients; i++) {
        if (client_notify[i]) {
            sddf_notify(config.clients[i].conn.id);
        }
    }

    LOG_BLK_VIRT("%s: failed %u client request(s), reclaimed %u internal\n", reason, failed, reclaimed);

    /*
     * NOTE: the partition policy is deliberately NOT reset here.
     *
     * The sDDF design document pairs its restart handling with a policy reset
     * (Release 0.6 calls it policy_reset(); the unmerged sdhc-hotplugging branch
     * implements it as virt_partition_reset()), but that exists for HOTPLUG,
     * where the device that comes back may be a different disk with a different
     * layout. A driver restart is the same disk, unchanged, so the partition
     * mapping and the clients' ready flags remain valid and re-deriving them
     * buys nothing.
     *
     * It would also be actively wrong as our pinned sDDF stands:
     * virt_partition_init() short-circuits on mbr_state.sent_request, which is
     * still true from the first boot, so clearing `initialised` would NOT
     * re-issue the MBR read. It would fall straight into mbr_handle_response()
     * and parse whatever is left in the scan buffer. A real reset needs
     * something that clears mbr_state/gpt_state, which the pinned rev has no
     * equivalent of. Adopt virt_partition_reset() from sdhc-hotplugging if that
     * branch merges and this system ever supports swapping the disk.
     */
}

/* Chrysopolis (driver restart): the driver re-initialised, so everything we had
 * outstanding died with the previous instance. */
static void blk_driver_restarted(void)
{
    blk_reconcile_outstanding("driver restarted");
}

/*
 * Chrysopolis (driver restart): root spent the driver's whole restart budget and
 * stopped it for good, so unlike a restart nothing is coming back.
 *
 * Reconciling is necessary but not sufficient here. Failing the outstanding
 * requests unblocks the clients that were waiting on them, but leaves those
 * clients free to immediately issue more against a device that no longer exists,
 * and every one of those would sit unanswered exactly as before. So we also drop
 * each client's ready flag, which is the state the sDDF design document already
 * gives this meaning: "the virtualiser is responsible for broadcasting the
 * driver's ready flag to each client's ready flag" (sDDF Design, Release 0.6,
 * S6.3.1). A client that honours the flag stops issuing; one that ignores it
 * still gets a prompt error rather than silence.
 *
 * Every client is notified, not just those with a request outstanding, because
 * the flag changed for all of them and a client with nothing in flight would
 * otherwise never look again.
 *
 * The flag broadcast deliberately mirrors notify_clients_ready() on the unmerged
 * sdhc-hotplugging branch, so it is the half to drop on a future rebase. The
 * reconcile above is NOT redundant with that branch: it stops on the driver
 * becoming not-ready without failing anything already in flight, which leaves
 * those requests pending for a response that, once root has stopped the driver,
 * nothing can ever send.
 */
static void blk_driver_gone(void)
{
    driver_gone = true;

    blk_reconcile_outstanding("driver stopped");

    for (int i = 0; i < config.num_clients; i++) {
        blk_storage_info_t *client_storage_info = config.clients[i].conn.storage_info.vaddr;
        blk_storage_set_ready(client_storage_info, false);
        sddf_notify(config.clients[i].conn.id);
    }

    LOG_BLK_VIRT("driver stopped: %d client(s) marked not ready\n", config.num_clients);
}

static void handle_driver()
{
    bool client_notify[SDDF_BLK_MAX_CLIENTS];
    memset(client_notify, 0, SDDF_BLK_MAX_CLIENTS);

    blk_resp_status_t drv_status = 0;
    uint16_t drv_success_count = 0;
    uint32_t drv_resp_id = 0;

    int err = 0;
    while (!blk_queue_empty_resp(&drv_h)) {
        err = blk_dequeue_resp(&drv_h, &drv_status, &drv_success_count, &drv_resp_id);
        assert(!err);

        reqbk_t reqbk = reqsbk[drv_resp_id];
        err = ialloc_free(&ialloc, drv_resp_id);
        /* Chrysopolis (driver restart): this id no longer names a live client
         * request, so a later restart sweep must not fail it again. */
        reqsbk_valid[drv_resp_id] = false;
        assert(!err);

        switch (reqbk.code) {
        case BLK_REQ_READ:
            if (drv_status == BLK_RESP_OK) {
                /* Invalidate the cache following the DMA write to memory.
                 * This relies on the cache already being clean for correctness,
                 * which we ensure by performing a cache_clean() in handle_client().
                 */
                cache_clean_and_invalidate(reqbk.vaddr, reqbk.vaddr + (BLK_TRANSFER_SIZE * reqbk.count));
            }
            break;
        case BLK_REQ_WRITE:
        case BLK_REQ_FLUSH:
        case BLK_REQ_BARRIER:
            break;
        default:
            /* This should never happen as we will have sanitized request codes before they are bookkept */
            LOG_BLK_VIRT_ERR("bookkept client %d request code %d is invalid, this should never happen\n", reqbk.cli_id,
                             reqbk.code);
            assert(false);
        }

        blk_queue_handle_t *h = &client_queues[reqbk.cli_id];

        /* Response queue should never be full since number of inflight requests (ialloc size)
         * should always be less than or equal to resp queue capacity.
         */
        err = blk_enqueue_resp(h, drv_status, drv_success_count, reqbk.cli_req_id);
        assert(!err);
        client_notify[reqbk.cli_id] = true;
    }

    /* Notify corresponding client if a response was enqueued */
    for (int i = 0; i < config.num_clients; i++) {
        if (client_notify[i]) {
            sddf_notify(config.clients[i].conn.id);
        }
    }
}

static bool handle_client(int cli_id)
{
    int err = 0;
    blk_queue_handle_t h = client_queues[cli_id];
    uintptr_t cli_data_base_paddr = config.clients[cli_id].data.io_addr;
    uintptr_t cli_data_base_vaddr = (uintptr_t)config.clients[cli_id].data.region.vaddr;
    uint64_t cli_data_region_size = config.clients[cli_id].data.region.size;

    blk_req_code_t cli_code = 0;
    uintptr_t cli_offset = 0;
    uint64_t cli_block_number = 0;
    uint16_t cli_count = 0;
    uint32_t cli_req_id = 0;

    bool driver_notify = false;
    bool client_notify = false;

    /*
     * Chrysopolis (driver restart): the driver is gone, so fail everything this
     * client asks for rather than forwarding it.
     *
     * Reconciling what was already in flight is not sufficient on its own. A
     * client whose request we just failed is free to retry immediately, and
     * forwarding that retry into a stopped driver strands it exactly as the
     * first one was stranded. Failing here is what makes the give-up path
     * actually terminate instead of moving the hang one request along.
     *
     * The response queue is checked for space rather than assumed to have it:
     * the usual "in-flight is bounded by the ialloc size" invariant that lets
     * the rest of this file assert on enqueue does not hold here, because these
     * requests are being failed without ever occupying an ialloc slot.
     */
    if (driver_gone) {
        while (!blk_queue_empty_req(&h) && !blk_queue_full_resp(&h)) {
            err = blk_dequeue_req(&h, &cli_code, &cli_offset, &cli_block_number, &cli_count, &cli_req_id);
            assert(!err);
            err = blk_enqueue_resp(&h, BLK_RESP_ERR_UNSPEC, 0, cli_req_id);
            assert(!err);
            client_notify = true;
        }

        if (client_notify) {
            sddf_notify(config.clients[cli_id].conn.id);
        }

        return false;
    }

    /*
     * In addition to checking the client actually has a request, we check that the
     * we can enqueue the request into the driver as well as that our index state tracking
     * is not full. We check the index allocator as there can be more in-flight requests
     * than currently in the driver queue.
     */
    while (!blk_queue_empty_req(&h) && !blk_queue_full_req(&drv_h) && !ialloc_full(&ialloc)) {

        err = blk_dequeue_req(&h, &cli_code, &cli_offset, &cli_block_number, &cli_count, &cli_req_id);
        assert(!err);

        uint64_t drv_block_number = 0;

        blk_resp_status_t resp_status = BLK_RESP_ERR_UNSPEC;

        switch (cli_code) {
        case BLK_REQ_READ:
        case BLK_REQ_WRITE: {
            resp_status = get_drv_block_number(cli_block_number, cli_count, cli_id, &drv_block_number);
            if (resp_status != BLK_RESP_OK) {
                goto req_fail;
            }

            if ((cli_offset + BLK_TRANSFER_SIZE * cli_count) > cli_data_region_size) {
                /* Requested offset is out of bounds from client data region */
                LOG_BLK_VIRT_ERR("client %d request offset 0x%lx is invalid\n", cli_id, cli_offset);
                resp_status = BLK_RESP_ERR_INVALID_PARAM;
                goto req_fail;
            }

            if (cli_count == 0) {
                LOG_BLK_VIRT_ERR("client %d requested zero blocks\n", cli_id);
                resp_status = BLK_RESP_ERR_INVALID_PARAM;
                goto req_fail;
            }

            if ((cli_data_base_vaddr + cli_offset) % BLK_TRANSFER_SIZE != 0) {
                LOG_BLK_VIRT_ERR(
                    "client %d requested dma address is not aligned to page size (same as blk transfer size)\n",
                    cli_id);
                resp_status = BLK_RESP_ERR_INVALID_PARAM;
                goto req_fail;
            }

            /* Perform a cache_clean operation.
             * - For BLK_REQ_READ (i.e. DMA writes happen), we need to ensure that
             *   the cache is clean prior to DMA, as we perform cache_clean_and_invalidate()
             *   in handle_driver(); if the cache was dirty that post-DMA clean
             *   could overwrite DMA'd data.
             * - For BLK_REQ_WRITE (i.e. DMA reads happen), we need to ensure that
             *   the data in the cache is written out to memory, so clean
             *   forces that flush.
             */
            cache_clean(cli_data_base_vaddr + cli_offset,
                        cli_data_base_vaddr + cli_offset + (BLK_TRANSFER_SIZE * cli_count));

            break;
        }
        case BLK_REQ_FLUSH:
        case BLK_REQ_BARRIER:
            break;
        default:
            /* Invalid request code given */
            LOG_BLK_VIRT_ERR("client %d gave an invalid request code %d\n", cli_id, cli_code);
            resp_status = BLK_RESP_ERR_INVALID_PARAM;
            goto req_fail;
        }

        /* Bookkeep client request and generate driver req id */
        uint32_t drv_req_id = 0;
        err = ialloc_alloc(&ialloc, &drv_req_id);
        assert(!err);
        reqsbk[drv_req_id] = (reqbk_t) { cli_id, cli_req_id, cli_data_base_vaddr + cli_offset, cli_count, cli_code };
        /* Chrysopolis (driver restart): mark this id as a real client request,
         * distinguishing it from the virtualiser's own partition-scan requests
         * which share this allocator. See reqsbk_valid[]. */
        reqsbk_valid[drv_req_id] = true;

        err = blk_enqueue_req(&drv_h, cli_code, cli_data_base_paddr + cli_offset, drv_block_number, cli_count,
                              drv_req_id);
        assert(!err);
        driver_notify = true;
        continue;

    req_fail:
        /* Response queue should never be full since number of inflight requests (ialloc size)
         * should always be less than or equal to resp queue capacity.
         */
        err = blk_enqueue_resp(&h, resp_status, 0, cli_req_id);
        assert(!err);
        client_notify = true;
    }

    if (client_notify) {
        sddf_notify(config.clients[cli_id].conn.id);
    }

    return driver_notify;
}

static void handle_clients()
{
    bool driver_notify = false;
    for (int i = 0; i < config.num_clients; i++) {
        if (handle_client(i)) {
            driver_notify = true;
        }
    }

    if (driver_notify) {
        sddf_notify(config.driver.conn.id);
    }
}

void notified(sddf_channel ch)
{
    /*
     * Chrysopolis (driver restart): root has permanently stopped the driver.
     *
     * Handled before anything else and returned from immediately: there is no
     * driver left to service, so falling through to handle_driver() would only
     * scan a response queue that can never gain another entry.
     *
     * Before partition init there is nothing to reconcile (the only outstanding
     * id belongs to the scan itself, and the clients' ready flags are still
     * false), so this degenerates to a log line. That leaves this PD stuck
     * uninitialised, which is the same boot-only window the restart path
     * documents below and is equally not recoverable without a way to rewind the
     * scan state machine.
     */
    if (ch == BLK_VIRT_DRIVER_GONE_CH) {
        if (initialised) {
            blk_driver_gone();
        } else {
            LOG_BLK_VIRT("driver stopped before partition init, nothing to reconcile\n");
        }
        return;
    }

    /*
     * Chrysopolis (driver restart): detect a driver restart before servicing the
     * queues. The driver signals us at the end of its init(), and a changed
     * generation is the unambiguous marker (the ready flag is not: the driver
     * clears and re-sets it within one init() and outranks us in priority, so we
     * would only ever observe ready = true).
     *
     * Only the driver channel can report this, and only once ready is back up.
     * The driver publishes ready last, after its virtqueue is configured, so
     * observing ready means the device is genuinely able to serve requests and we
     * are not racing its setup.
     */
    if (ch == config.driver.conn.id) {
        blk_storage_info_t *driver_storage_info = config.driver.conn.storage_info.vaddr;
        if (blk_storage_is_ready(driver_storage_info)) {
            uint64_t generation = blk_storage_generation(driver_storage_info);
            if (generation != last_seen_generation) {
                last_seen_generation = generation;
                /*
                 * Only reconcile once partition initialisation has finished.
                 *
                 * The reconciliation calls handle_driver() and then sweeps the
                 * ialloc slots, but during partition init the single outstanding
                 * id belongs to the MBR/GPT scan, not to a client, and it is
                 * mbr_handle_response() that must consume that response. Running
                 * the sweep here instead would dequeue the scan response,
                 * interpret its request-backup entry as a client request, and
                 * fault this PD on the unreachable-default assert; freeing the
                 * scan's id would also strand the init state machine, which still
                 * believes its request is in flight.
                 *
                 * Known limitation: a driver restart DURING the partition scan is
                 * therefore not recovered. The scan request was discarded by the
                 * restarting driver, so nothing will complete it and this PD stays
                 * uninitialised. That window is boot-only, before any client is
                 * being served, and closing it needs a way to rewind the scan
                 * state machine, which the policy interface does not currently
                 * expose. Not reconciling is the safe half of the tradeoff: it
                 * cannot fault, and it cannot corrupt a client's id space.
                 */
                if (initialised) {
                    blk_driver_restarted();
                }
            }
        }
    }

    if (!initialised) {
        /* Continue processing partitions until initialisation has finished. */
        LOG_BLK_VIRT("initialising partitions\n");
        initialised = virt_partition_init();
    }

    /* Do not handle driver/clients until we have finished initialisation. */
    if (initialised) {
        if (ch == config.driver.conn.id) {
            handle_driver();
            handle_clients();
        } else {
            handle_clients();
        }
    }
}
