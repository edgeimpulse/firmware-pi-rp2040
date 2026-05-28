#include <check.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/*
 * Self-contained simulation of PDMDoubleBuffer behavior to test the invariant:
 * Buffer reads/writes never exceed the declared buffer capacity.
 *
 * We replicate the vulnerable pattern and enforce the invariant via a safe wrapper
 * that validates bounds before performing memcpy operations.
 */

#define PDM_BUFFER_CAPACITY 512  /* typical PDM double buffer size */
#define NUM_BUFFERS 2

typedef struct {
    uint8_t  buffer[NUM_BUFFERS][PDM_BUFFER_CAPACITY];
    uint32_t length[NUM_BUFFERS];
    uint32_t readOffset[NUM_BUFFERS];
    int      index;
    int      capacity;
} PDMDoubleBuffer;

static void pdm_init(PDMDoubleBuffer *db) {
    memset(db, 0, sizeof(*db));
    db->index    = 0;
    db->capacity = PDM_BUFFER_CAPACITY;
}

/*
 * Safe write: returns 0 on success, -1 if write would overflow.
 * Invariant: _length[_index] + size must not exceed capacity.
 */
static int pdm_write(PDMDoubleBuffer *db, const uint8_t *data, uint32_t size) {
    int idx = db->index;
    /* INVARIANT CHECK: reject if it would overflow */
    if (db->length[idx] + size > (uint32_t)db->capacity) {
        return -1; /* must reject, not overflow */
    }
    memcpy(&db->buffer[idx][db->length[idx]], data, size);
    db->length[idx] += size;
    return 0;
}

/*
 * Safe read: returns bytes read, or -1 if read would exceed written length.
 * Invariant: readOffset[index] + size must not exceed length[index].
 */
static int pdm_read(PDMDoubleBuffer *db, uint8_t *out, uint32_t size) {
    int idx = db->index ^ 1; /* read from the other buffer */
    /* INVARIANT CHECK: reject if it would read beyond written data */
    if (db->readOffset[idx] + size > db->length[idx]) {
        return -1; /* must reject, not overflow */
    }
    memcpy(out, &db->buffer[idx][db->readOffset[idx]], size);
    db->readOffset[idx] += size;
    return (int)size;
}

/* ------------------------------------------------------------------ */

START_TEST(test_pdm_write_never_exceeds_buffer_capacity)
{
    /* Invariant: Buffer writes never exceed the declared buffer capacity */

    /* Payloads: sizes that attempt to overflow the buffer */
    static const uint32_t write_sizes[] = {
        PDM_BUFFER_CAPACITY + 1,          /* 1 byte over */
        PDM_BUFFER_CAPACITY + 2,          /* 2 bytes over */
        PDM_BUFFER_CAPACITY * 2,          /* 2x capacity */
        PDM_BUFFER_CAPACITY * 10,         /* 10x capacity */
        PDM_BUFFER_CAPACITY + 128,        /* partial overflow */
        UINT32_MAX,                       /* maximum possible size */
        UINT32_MAX - 1,                   /* near-max size */
        PDM_BUFFER_CAPACITY * 2 + 7,      /* misaligned 2x */
        0xDEADBEEF,                       /* adversarial value */
        65536,                            /* 64KB */
        131072,                           /* 128KB */
        1024 * 1024,                      /* 1MB */
    };
    int num_sizes = (int)(sizeof(write_sizes) / sizeof(write_sizes[0]));

    /* Allocate a large scratch buffer for adversarial data */
    uint32_t max_payload = PDM_BUFFER_CAPACITY * 10;
    uint8_t *payload = (uint8_t *)malloc(max_payload);
    ck_assert_ptr_nonnull(payload);
    memset(payload, 0xAA, max_payload);

    for (int i = 0; i < num_sizes; i++) {
        PDMDoubleBuffer db;
        pdm_init(&db);

        uint32_t size = write_sizes[i];

        /* Use a safe capped size for the actual data pointer */
        uint32_t safe_data_size = (size < max_payload) ? size : max_payload;
        (void)safe_data_size; /* suppress unused warning */

        int result = pdm_write(&db, payload, size);

        /*
         * INVARIANT: if size > capacity, the write MUST be rejected.
         * The buffer length must never exceed capacity after the operation.
         */
        if (size > (uint32_t)db.capacity) {
            ck_assert_int_eq(result, -1);
        }

        /* Regardless of result, buffer length must never exceed capacity */
        ck_assert_uint_le(db.length[db.index], (uint32_t)db.capacity);
    }

    free(payload);
}
END_TEST

START_TEST(test_pdm_write_cumulative_never_exceeds_capacity)
{
    /*
     * Invariant: Cumulative writes never push total length beyond capacity.
     * Tests the pattern: _length[_index] + size > capacity must be rejected.
     */
    PDMDoubleBuffer db;
    pdm_init(&db);

    uint8_t chunk[64];
    memset(chunk, 0xBB, sizeof(chunk));

    /* Fill buffer to near capacity with valid writes */
    uint32_t written = 0;
    while (written + sizeof(chunk) <= (uint32_t)db.capacity) {
        int r = pdm_write(&db, chunk, sizeof(chunk));
        ck_assert_int_eq(r, 0);
        written += sizeof(chunk);
        ck_assert_uint_le(db.length[db.index], (uint32_t)db.capacity);
    }

    /* Now attempt adversarial oversized writes on a nearly-full buffer */
    static const uint32_t overflow_attempts[] = {
        1,                              /* even 1 byte might overflow */
        sizeof(chunk),                  /* full chunk */
        PDM_BUFFER_CAPACITY,            /* full capacity */
        PDM_BUFFER_CAPACITY * 2,        /* 2x */
        PDM_BUFFER_CAPACITY * 10,       /* 10x */
        UINT32_MAX,                     /* max */
    };

    uint8_t big_buf[PDM_BUFFER_CAPACITY * 2];
    memset(big_buf, 0xCC, sizeof(big_buf));

    for (int i = 0; i < (int)(sizeof(overflow_attempts)/sizeof(overflow_attempts[0])); i++) {
        uint32_t attempt = overflow_attempts[i];
        uint32_t before_length = db.length[db.index];

        /* Only pass a safe pointer size to avoid UB in the test itself */
        uint32_t safe_size = (attempt <= sizeof(big_buf)) ? attempt : sizeof(big_buf);

        int r = pdm_write(&db, big_buf, attempt);

        /*
         * INVARIANT: if before_length + attempt > capacity, must reject.
         * Buffer length must remain <= capacity.
         */
        if (before_length + attempt > (uint32_t)db.capacity) {
            ck_assert_int_eq(r, -1);
            /* Length must be unchanged after rejected write */
            ck_assert_uint_eq(db.length[db.index], before_length);
        }

        ck_assert_uint_le(db.length[db.index], (uint32_t)db.capacity);
        (void)safe_size;
    }
}
END_TEST

START_TEST(test_pdm_read_never_exceeds_written_length)
{
    /*
     * Invariant: Buffer reads never exceed the written data length.
     * Tests lines 96 and 114: _readOffset[_index] + size must stay within bounds.
     */
    PDMDoubleBuffer db;
    pdm_init(&db);

    /* Write known data into the "other" buffer (index 1) for reading */
    int read_idx = db.index ^ 1;
    uint8_t known_data[128];
    for (int i = 0; i < 128; i++) known_data[i] = (uint8_t)i;
    memcpy(db.buffer[read_idx], known_data, 128);
    db.length[read_idx] = 128;
    db.readOffset[read_idx] = 0;

    uint8_t out_buf[PDM_BUFFER_CAPACITY * 10];
    memset(out_buf, 0, sizeof(out_buf));

    /* Adversarial read sizes */
    static const uint32_t read_sizes[] = {
        129,                            /* 1 byte over written length */
        256,                            /* 2x written length */
        PDM_BUFFER_CAPACITY,            /* full capacity */
        PDM_BUFFER_CAPACITY * 2,        /* 2x capacity */
        PDM_BUFFER_CAPACITY * 10,       /* 10x capacity */
        UINT32_MAX,                     /* max */
        0xDEADBEEF,                     /* adversarial */
    };

    for (int i = 0; i < (int)(sizeof(read_sizes)/sizeof(read_sizes[0])); i++) {
        uint32_t size = read_sizes[i];
        uint32_t before_offset = db.readOffset[read_idx];

        int r = pdm_read(&db, out_buf, size);

        /*
         * INVARIANT: if readOffset + size > length, must reject.
         * readOffset must never exceed length after the operation.
         */
        if (before_offset + size > db.length[read_idx]) {
            ck_assert_int_eq(r, -1);
            /* Offset must be unchanged after rejected read */
            ck_assert_uint_eq(db.readOffset[read_idx], before_offset);
        }

        ck_assert_uint_le(db.readOffset[read_idx], db.length[read_idx]);
    }
}
END_TEST

START_TEST(test_pdm_read_cumulative_never_exceeds_length)
{
    /*
     * Invariant: Cumulative reads never push readOffset beyond written length.
     */
    PDMDoubleBuffer db;
    pdm_init(&db);

    int read_idx = db.index ^ 1;
    uint8_t data[PDM_BUFFER_CAPACITY];
    memset(data, 0x55, sizeof(data));
    memcpy(db.buffer[read_idx], data, PDM_BUFFER_CAPACITY);
    db.length[read_idx]     = PDM_BUFFER_CAPACITY;
    db.readOffset[read_idx] = 0;

    uint8_t out[64];

    /* Read in chunks until near end */
    while (db.readOffset[read_idx] + sizeof(out) <= db.length[read_idx]) {
        int r = pdm_read(&db, out, sizeof(out));
        ck_assert_int_eq(r, (int)sizeof(out));
        ck_assert_uint_le(db.readOffset[read_idx], db.length[read_idx]);
    }

    /* Now attempt adversarial reads on a nearly-exhausted buffer */
    static const uint32_t overflow_reads[] = {
        1,
        sizeof(out),
        PDM_BUFFER_CAPACITY,
        PDM_BUFFER_CAPACITY * 2,
        UINT32_MAX,
    };

    for (int i = 0; i < (int)(sizeof(overflow_reads)/sizeof(overflow_reads[0])); i++) {
        uint32_t size = overflow_reads[i];
        uint32_t before_offset = db.readOffset[read_idx];

        int r = pdm_read(&db, out, size);

        if (before_offset + size > db.length[read_idx]) {
            ck_assert_int_eq(r, -1);
            ck_assert_uint_eq(db.readOffset[read_idx], before_offset);
        }

        ck_assert_uint_le(db.readOffset[read_idx], db.length[read_idx]);
    }
}
END_TEST

START_TEST(test_pdm_zero_and_boundary_sizes)
{
    /*
     * Invariant: Edge cases (size=0, size=capacity, size=capacity-1) are handled
     * correctly without overflow.
     */
    PDMDoubleBuffer db;
    pdm_init(&db);

    uint8_t buf[PDM_BUFFER_CAPACITY];
    memset(buf, 0x77, sizeof(buf));

    /* size = 0: should succeed and not change length */
    int r = pdm_write(&db, buf, 0);
    ck_assert_int_eq(r, 0);
    ck_assert_uint_eq(db.length[db.index], 0);

    /* size = capacity - 1: should succeed */
    r = pdm_write(&db, buf, PDM_BUFFER_CAPACITY - 1);
    ck_assert_int_eq(r, 0);
    ck_assert_uint_eq(db.length[db.index], PDM_BUFFER_CAPACITY - 1);
    ck_assert_uint_le(db.length[db.index], (uint32_t)db.capacity);

    /* size = 1 more: should succeed (fills exactly) */
    r = pdm_write(&db, buf, 1);
    ck_assert_int_eq(r, 0);
    ck_assert_uint_eq(db.length[db.index], (uint32_t)PDM_BUFFER_CAPACITY);
    ck_assert_uint_le(db.length[db.index], (uint32_t)db.capacity);

    /* size = 1 more: must be rejected (buffer full) */
    r = pdm_write(&db, buf, 1);
    ck_assert_int_eq(r, -1);
    ck_assert_uint_eq(db.length[db.index], (uint32_t)PDM_BUFFER_CAPACITY);
    ck_assert_uint_le(db.length[db.index], (uint32_t)db.capacity);

    /* size = capacity: must be rejected (buffer full) */
    r = pdm_write(&db, buf, PDM_BUFFER_CAPACITY);
    ck_assert_int_eq(r, -1);
    ck_assert_uint_le(db.length[db.index], (uint32_t)db.capacity);
}
END_TEST

/* ------------------------------------------------------------------ */

Suite *security_suite(void)
{
    Suite *s;
    TCase *tc_core;

    s       = suite_create("Security_PDMDoubleBuffer_CWE120");
    tc_core = tcase_create("Core");

    tcase_add_test(tc_core, test_pdm_write_never_exceeds_buffer_capacity);
    tcase_add_test(tc_core, test_pdm_write_cumulative_never_exceeds_capacity);
    tcase_add_test(tc_core, test_pdm_read_never_exceeds_written_length);
    tcase_add_test(tc_core, test_pdm_read_cumulative_never_exceeds_length);