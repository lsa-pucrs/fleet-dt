#ifndef FLEET_DT_DB_H
#define FLEET_DT_DB_H

#include <fleet_dt/model.h>

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

/**
 * @file db.h
 * @brief The database the states and actuation of the fleet are stored in.
 *
 * Figure 4: "Newly generated states and actuation are sent to the MCS and
 * stored within a database." Section IV: a state occupies 48 bytes, so "for
 * a 125 ms period, the queue would grow by 23 KB per minute elapsed, per
 * vessel". The in-memory queue of queue.h stays bounded; this file is where
 * the history grows instead, and it grows by exactly that payload.
 *
 * Format, little-endian, append-only:
 *
 *     file header, 16 bytes:  "FDTDB" 0 0 1 | u32 version (1) | u32 0
 *     record:                 u32 tick | u16 vessel | u8 kind | u8 len
 *                             | payload (len bytes) | u32 CRC-32
 *
 * The payload is the wire encoding of codec.h: 48 bytes for B^t and g^t,
 * 8 for A^t. The CRC covers the record header and the payload, so a reader
 * tells a torn tail (a crash mid-write) from a complete record and stops
 * there, keeping everything before it.
 */

/** Bytes of the file header. */
#define FDT_DB_HEADER_BYTES 16

/** Bytes a record adds around its payload: header and CRC. */
#define FDT_DB_RECORD_OVERHEAD 12

/** What a record holds. */
typedef enum {
    FDT_DB_STATE = 1, /**< B^t, 48 bytes. */
    FDT_DB_ACT   = 2, /**< A^t, 8 bytes. */
    FDT_DB_GOAL  = 3  /**< g^t, 48 bytes. */
} fdt_db_kind_t;

/** An open database being written. */
typedef struct {
    FILE    *f;
    uint64_t records;       /**< Records appended since open. */
    uint64_t payload_bytes; /**< Payload bytes appended since open. */
    uint64_t file_bytes;    /**< Bytes appended since open, overhead too. */
} fdt_db_t;

/** One record read back. */
typedef struct {
    uint32_t        tick;
    uint16_t        vessel;
    fdt_db_kind_t   kind;
    fdt_state_t     state;  /**< Valid for FDT_DB_STATE and FDT_DB_GOAL. */
    fdt_actuation_t act;    /**< Valid for FDT_DB_ACT. */
} fdt_db_record_t;

/** A database being read. */
typedef struct {
    FILE    *f;
    uint64_t records;
    int      torn;  /**< 1 when reading stopped at an incomplete record. */
} fdt_db_reader_t;

/**
 * @brief Opens @p path for appending, creating it with a header if absent.
 * @return 0, or -1 when the file cannot be opened or holds a foreign header.
 */
int fdt_db_open(fdt_db_t *db, const char *path);

/** @brief Appends one B^t. @return 0, or -1 on a bad argument or I/O error. */
int fdt_db_put_state(fdt_db_t *db, uint32_t tick, uint16_t vessel,
                     const fdt_state_t *b);

/** @brief Appends one A^t. @return 0, or -1 on a bad argument or I/O error. */
int fdt_db_put_act(fdt_db_t *db, uint32_t tick, uint16_t vessel,
                   const fdt_actuation_t *a);

/** @brief Appends one g^t. @return 0, or -1 on a bad argument or I/O error. */
int fdt_db_put_goal(fdt_db_t *db, uint32_t tick, uint16_t vessel,
                    const fdt_goal_t *g);

/**
 * @brief Appends what one fleet frame produced: B^t and A^t of every vessel.
 * @return 0, or -1 on the first failed append.
 */
int fdt_db_put_frame(fdt_db_t *db, uint32_t tick, const fdt_state_t *bs,
                     const fdt_actuation_t *as, size_t n);

/** @brief Pushes buffered records to the operating system. */
int fdt_db_flush(fdt_db_t *db);

/** @brief Flushes and closes. */
int fdt_db_close(fdt_db_t *db);

/** @brief Opens @p path for reading. @return 0, or -1 on a foreign file. */
int fdt_db_read_open(fdt_db_reader_t *r, const char *path);

/**
 * @brief Reads the next record.
 * @return 1 with @p rec filled; 0 at the end of the file, or at a torn tail
 *         (r->torn set); -1 on a record whose CRC fails before the tail.
 */
int fdt_db_next(fdt_db_reader_t *r, fdt_db_record_t *rec);

/** @brief Closes the reader. */
void fdt_db_read_close(fdt_db_reader_t *r);

/** @brief CRC-32 (IEEE 802.3, reflected), as the records carry it. */
uint32_t fdt_db_crc32(const uint8_t *p, size_t n);

#endif /* FLEET_DT_DB_H */
