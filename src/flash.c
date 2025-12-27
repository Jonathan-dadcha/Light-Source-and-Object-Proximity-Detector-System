#include "../header/flash.h"
#include <string.h>
#include <msp430.h>

/* Directory image in RAM */
static fm_directory_t g_dir;

/* Flash controller flags */
#define FLASH_KEY  0xA500
#define ERASE_BIT  0x0002
#define WRT_BIT    0x0040
#define LOCK_BIT   0x0010
#define BUSY_BIT   0x0001

#ifndef SEG_SIZE
#define SEG_SIZE   512u          /* MSP430G2553 main flash segment = 512 B */
#endif
#ifndef SEG_MASK
#define SEG_MASK   (SEG_SIZE - 1u)
#endif
#ifndef FILE_BUF_CHUNK
#define FILE_BUF_CHUNK 96u       /* temp copy buffer used by compactor */
#endif
#define SEG_BASE(a)  ((uint16_t)((a) & ~SEG_MASK))
#define ALIGN_UP(x)  ((uint16_t)(((x) + SEG_MASK) & ~SEG_MASK))

typedef struct {
    char     name[FM_MAX_NAME+1];
    uint8_t  type;
    uint16_t expected;
    uint16_t written;
    uint16_t buf_used;
    uint8_t  buf[FILE_BUF_CHUNK];
    bool     active;
} fm_put_ctx_t;

typedef struct {
    uint16_t dst;           /* absolute address in main flash */
    uint16_t start_offset;  /* relative offset inside file pool */
    uint16_t expected;      /* total bytes expected */
    uint16_t written;       /* bytes written so far */
    uint8_t  checksum;      /* running checksum-256 */
    uint8_t  type;          /* file type */
    uint8_t  active;        /* 1 while a PUT session is open */
    char     name[FM_MAX_NAME];
} put_state_t;

static uint16_t fm_used_bytes(void);
static uint16_t fm_high_water_end(void);
static void flash_erase_segment_main(uint16_t any_addr_in_segment);

static bool range_is_ff(uint16_t abs_addr, uint16_t len) {
    const uint8_t* p = (const uint8_t*)abs_addr;
    while (len--) { if (*p++ != 0xFF) return false; }
    return true;
}

static void erase_trailing_free_segments_from(uint16_t abs_from) {
    uint16_t a = (uint16_t)((abs_from + SEG_SIZE - 1) & ~SEG_MASK); // next segment boundary
    for (; a < FILE_POOL_BASE + FILE_POOL_SIZE; a = (uint16_t)(a + SEG_SIZE)) {
        flash_erase_segment_main(a);
    }
}

void fm_query_space(uint16_t* total_free, uint16_t* tail_free) {
    if (total_free) *total_free = (uint16_t)(FILE_POOL_SIZE - fm_used_bytes());
    if (tail_free)  *tail_free  = (uint16_t)(FILE_POOL_SIZE - g_dir.next_free);
}

/* Internal helpers */
/* Convert absolute address <-> segment index inside the file pool */
static uint8_t seg_index_from_abs(uint16_t abs_addr)
{
    return (uint8_t)((abs_addr - FILE_POOL_BASE) / SEG_SIZE);
}

static uint16_t seg_abs_from_index(uint8_t idx)
{
    return (uint16_t)(FILE_POOL_BASE + (uint16_t)idx * SEG_SIZE);
}

/* Check if a segment is occupied by a file */
static bool seg_is_occupied(uint16_t seg_abs)
{
    const uint16_t seg_start = seg_abs;
    const uint16_t seg_end   = (uint16_t)(seg_abs + SEG_SIZE);
    uint8_t i;

    for (i = 0u; i < FM_MAX_FILES; ++i) {
        const fm_dir_entry_t* e = &g_dir.entries[i];
        if (e->valid == 1u && e->size_bytes > 0u) {
            const uint16_t f_start = (uint16_t)(FILE_POOL_BASE + e->offset);
            const uint16_t f_end   = (uint16_t)(f_start + e->size_bytes);
            if (f_start < seg_end && f_end > seg_start) {
                return true; /* overlap */
            }
        }
    }
    return false;
}

static inline void flash_wait_ready(void) { while (FCTL3 & BUSY_BIT) { __no_operation(); } }

static void flash_write_main(uint16_t addr, const uint8_t* data, uint16_t n)
{
    uint8_t* p = (uint8_t*)addr;
    __disable_interrupt();
    FCTL3 = FLASH_KEY;
    FCTL1 = FLASH_KEY | WRT_BIT;
    uint16_t i;
    for (i = 0; i < n; ++i) {
        *p++ = data[i];
        flash_wait_ready();
    }
    FCTL1 = FLASH_KEY;
    FCTL3 = FLASH_KEY | LOCK_BIT;
    __enable_interrupt();
}

static void flash_erase_segment_main(uint16_t any_addr_in_segment)
{
    uint8_t* p = (uint8_t*)(any_addr_in_segment & 0xFE00u); /* 512B boundary */
    __disable_interrupt();
    FCTL3 = FLASH_KEY;
    FCTL1 = FLASH_KEY | ERASE_BIT;
    *p = 0;
    flash_wait_ready();
    FCTL1 = FLASH_KEY;
    FCTL3 = FLASH_KEY | LOCK_BIT;
    __enable_interrupt();
}

static void info_write(uint16_t addr, const uint8_t* data, uint16_t n)
{
    uint8_t* p = (uint8_t*)addr;
    __disable_interrupt();
    FCTL3 = FLASH_KEY;
    FCTL1 = FLASH_KEY | WRT_BIT;
    uint16_t i;
    for (i = 0; i < n; ++i) {
        *p++ = data[i];
        flash_wait_ready();
    }
    FCTL1 = FLASH_KEY;
    FCTL3 = FLASH_KEY | LOCK_BIT;
    __enable_interrupt();
}

static void info_erase_segment(uint16_t any_addr_in_segment)
{
    uint8_t* p = (uint8_t*)(any_addr_in_segment & 0xFFC0u); /* 64B boundary */
    __disable_interrupt();
    FCTL3 = FLASH_KEY;
    FCTL1 = FLASH_KEY | ERASE_BIT;
    *p = 0;
    flash_wait_ready();
    FCTL1 = FLASH_KEY;
    FCTL3 = FLASH_KEY | LOCK_BIT;
    __enable_interrupt();
}

/* Utilities */
uint8_t fm_checksum8(const uint8_t* data, uint16_t n)
{
    uint16_t s = 0;
    uint16_t i;
    for (i = 0; i < n; ++i) s += data[i];
    return (uint8_t)(s & 0xFF);
}

void fm_format(void){
    uint16_t a;
    for (a = FILE_POOL_BASE; a < FILE_POOL_BASE + FILE_POOL_SIZE; a += SEG_SIZE)
        flash_erase_segment_main(a);
    memset(&g_dir, 0, sizeof g_dir);
    fm_save_directory();
}

/* -------- helpers that match our packed directory image -------- */

static bool fm_dir_has_free_slot(void) {
    uint8_t i;
    for (i = 0; i < FM_MAX_FILES; ++i) {
        if (g_dir.entries[i].valid == 0) return true;
    }
    return false;
}

static bool fm_dir_has_tombstones(void) {
    /* We rewrite the whole directory image; there are no tombstones-on-flash. */
    return false;
}

// Compact in-RAM file directory by removing invalid entries (preserves order & next_free); return FM_ERR_BAD_PARAM if no change.
static fm_status_t fm_compact_dir(void) {
    fm_directory_t packed = {0};
    uint8_t w = 0;
    uint8_t i;
    for (i = 0; i < FM_MAX_FILES; ++i) {
        if (g_dir.entries[i].valid == 1) {
            packed.entries[w++] = g_dir.entries[i];
        }
    }
    packed.next_free = g_dir.next_free;                  // keep current tail
    if (memcmp(&packed, &g_dir, sizeof(g_dir)) == 0) {
        return FM_ERR_BAD_PARAM;                         // nothing to do
    }
    g_dir = packed;
    fm_save_directory();
    return FM_OK;
}

static fm_status_t fm_compact_data(void)
{
    /* If nothing is used, just bulk erase the pool and reset. */
    if (fm_used_bytes() == 0u) {
        uint16_t a;
        g_dir.next_free = 0u;
        for (a = FILE_POOL_BASE; a < FILE_POOL_BASE + FILE_POOL_SIZE; a = (uint16_t)(a + SEG_SIZE)) {
            flash_erase_segment_main(a);
        }
        fm_save_directory();
        return FM_OK;
    }

    /* 1) Erase any segment that has NO live bytes at all (safe). */
    {
        const uint8_t num_segs = (uint8_t)(FILE_POOL_SIZE / SEG_SIZE);
        uint8_t si;
        for (si = 0u; si < num_segs; ++si) {
            uint16_t base = (uint16_t)(FILE_POOL_BASE + (uint16_t)si * SEG_SIZE);
            if (!seg_is_occupied(base)) {
                if (!range_is_ff(base, SEG_SIZE)) {
                    flash_erase_segment_main(base);
                }
            }
        }
    }

    /* 2) Set tail to end of the highest live byte (no alignment). */
    uint16_t tail = fm_high_water_end();
    if (tail > FILE_POOL_SIZE) tail = FILE_POOL_SIZE;
    g_dir.next_free = tail;

    /* 3) Erase all segments strictly after the new tail. */
    erase_trailing_free_segments_from((uint16_t)(FILE_POOL_BASE + g_dir.next_free));

    fm_save_directory();
    return FM_OK;
}


/* ---------- Streaming PUT state (keeps RAM low, writes in batches) ---------- */
static put_state_t s_put;

fm_status_t fm_put_begin(const char* name, uint8_t type, uint16_t expected)
{
    uint16_t total_free = (uint16_t)(FILE_POOL_SIZE - fm_used_bytes());
    uint16_t tail_free  = (uint16_t)(FILE_POOL_SIZE - g_dir.next_free);

    if (expected > total_free) return FM_ERR_NO_SPACE;
    // Distinguish “not enough contiguous tail” from “no total space”.
    if (expected > tail_free)  return FM_ERR_NEEDS_REBUILD;   // new code


    if (!name || !name[0] || expected == 0 || expected > FILE_POOL_SIZE)
        return FM_ERR_BAD_PARAM;

    if (s_put.active) return FM_ERR_BUSY;

    // ---------- 1) Ensure a directory slot exists ----------
    if (!fm_dir_has_free_slot()) {
        if (fm_dir_has_tombstones()) {
            (void)fm_compact_dir();  // try to reclaim a slot
        }
        if (!fm_dir_has_free_slot()) {
            // Still no slot; fall back to full compact (dir + data),
            // in case data compaction changes where the dir wants to point.
            (void)fm_compact();
        }
        if (!fm_dir_has_free_slot()) {
            return FM_ERR_DIR_FULL;
        }
    }

    // ---------- 2) Ensure data area has space (total & contiguous) ----------
    total_free = (uint16_t)(FILE_POOL_SIZE - fm_used_bytes());
    if (expected > total_free) return FM_ERR_NO_SPACE;

    tail_free  = (uint16_t)(FILE_POOL_SIZE - g_dir.next_free);

    if (expected > tail_free) {

        // Try a data-only compact first (cheaper).
        (void)fm_compact_data();

        tail_free = (uint16_t)(FILE_POOL_SIZE - g_dir.next_free);
        if (expected > tail_free) {
            // As a last resort, do a full compact (dir + data).
            (void)fm_compact();
            tail_free = (uint16_t)(FILE_POOL_SIZE - g_dir.next_free);
            if (expected > tail_free) return FM_ERR_NO_SPACE;
        }
    }

    // If pool is totally empty, bulk-erase main segments once.
    if (g_dir.next_free == 0) {
        uint16_t a;
        for (a = FILE_POOL_BASE; a < FILE_POOL_BASE + FILE_POOL_SIZE; a += SEG_SIZE) {
            flash_erase_segment_main(a);
        }
    }

    // ---------- 3) Initialize streaming state ----------
    {
        uint16_t dst_abs = (uint16_t)(FILE_POOL_BASE + g_dir.next_free);
        uint16_t end_abs = (uint16_t)(dst_abs + expected);



        uint16_t first_seg_base = (uint16_t)(dst_abs & ~SEG_MASK);
        uint16_t first_seg_end  = (uint16_t)(first_seg_base + SEG_SIZE);
        uint16_t first_chunk    = (end_abs <= first_seg_end)
                                  ? (uint16_t)(end_abs - dst_abs)
                                  : (uint16_t)(first_seg_end - dst_abs);

        if (first_chunk != 0u) {
            if (!range_is_ff(dst_abs, first_chunk)) {
                dst_abs = first_seg_end;
                g_dir.next_free = (uint16_t)(dst_abs - FILE_POOL_BASE);

                if ((uint32_t)g_dir.next_free + expected > FILE_POOL_SIZE) {
                    return FM_ERR_NO_SPACE;
                }

                end_abs = (uint16_t)(dst_abs + expected);
                first_chunk = 0u;
            }
        }

        {
            uint16_t seg = (uint16_t)((dst_abs + first_chunk + SEG_MASK) & ~SEG_MASK);
            uint16_t last_full_end = (uint16_t)(end_abs & ~SEG_MASK);
            while (seg < last_full_end) {
                if (!range_is_ff(seg, SEG_SIZE)) {
                    flash_erase_segment_main(seg);
                }
                seg = (uint16_t)(seg + SEG_SIZE);
            }
        }

        /* Initialize streaming state */
        memset(s_put.name, 0, sizeof(s_put.name));
        strncpy(s_put.name, name, FM_MAX_NAME - 1);

        s_put.active       = 1u;
        s_put.type         = type;
        s_put.expected     = expected;
        s_put.written      = 0u;
        s_put.checksum     = 0u;
        s_put.start_offset = g_dir.next_free;
        s_put.dst          = dst_abs;

        return FM_OK;
    }

}

// Abort a PUT session without committing anything to the directory
void fm_put_abort(void)
{
    s_put.active   = 0;
    s_put.written  = 0;
    s_put.expected = 0;
}

fm_status_t fm_put_chunk(const uint8_t* data, uint16_t len)
{
    if (!s_put.active)                     return FM_ERR_BAD_PARAM; /* NO_SESSION */
    if (!data || len == 0)                 return FM_ERR_BAD_PARAM;
    if ((uint32_t)s_put.written + len > s_put.expected)
        return FM_ERR_BAD_PARAM; /* OVERFLOW */

    /* Write this batch at absolute destination + current offset */
    flash_write_main((uint16_t)(s_put.dst + s_put.written), data, len);

    /* Update running state */
    s_put.written = (uint16_t)(s_put.written + len);

    /* Update checksum-256 (sum mod 256) */
    uint16_t i;
    for (i = 0; i < len; ++i)
        s_put.checksum = (uint8_t)(s_put.checksum + data[i]);

    return FM_OK;
}

fm_status_t fm_put_end(void)
{
    if (!s_put.active) return FM_ERR_BAD_PARAM;
    if (s_put.written != s_put.expected) return FM_ERR_BAD_PARAM; /* SIZE_MISMATCH */

    /* Allocate a directory entry and finalize metadata */
    fm_dir_entry_t* e = 0;
    {
        uint8_t i;
        for (i = 0; i < FM_MAX_FILES; ++i) {
            if (g_dir.entries[i].valid == 0) { e = &g_dir.entries[i]; break; }
        }
    }
    if (!e) return FM_ERR_NO_SPACE;

    memset(e, 0, sizeof(*e));
    strncpy(e->name, s_put.name, FM_MAX_NAME-1);
    e->type       = s_put.type;
    e->size_bytes = s_put.written;
    e->offset     = s_put.start_offset;
    e->checksum   = s_put.checksum;
    e->valid      = 1;

    g_dir.next_free = (uint16_t)(g_dir.next_free + s_put.written);
    fm_save_directory();  /* persists entries + next_free in Info Flash */

    s_put.active = 0;
    return FM_OK;
}

const fm_directory_t* fm_get_directory(void)
{
    return &g_dir;
}

static fm_dir_entry_t* find_entry(const char* name)
{
    uint8_t i;
    for (i = 0; i < FM_MAX_FILES; ++i) {
        if (g_dir.entries[i].valid == 1 && strncmp(g_dir.entries[i].name, name, FM_MAX_NAME) == 0) {
            return &g_dir.entries[i];
        }
    }
    return 0;
}

static fm_dir_entry_t* first_free_entry(void)
{
    uint8_t i;
    for (i = 0; i < FM_MAX_FILES; ++i) {
        if (g_dir.entries[i].valid == 0) {
            return &g_dir.entries[i];
        }
    }
    return 0;
}

bool fm_has_space(uint16_t nbytes)
{
    return (g_dir.next_free + nbytes) <= FILE_POOL_SIZE;
}

/* End offset of the last valid file */
static uint16_t fm_high_water_end(void) {
    uint16_t high = 0;
    uint8_t i;
    for (i = 0; i < FM_MAX_FILES; ++i) {
        const fm_dir_entry_t* e = &g_dir.entries[i];
        if (e->valid == 1 && e->size_bytes > 0) {
            uint16_t end = (uint16_t)(e->offset + e->size_bytes);
            if (end > high) high = end;
        }
    }
    if (high > FILE_POOL_SIZE) high = FILE_POOL_SIZE;
    return high;
}

/* Init and persist */
void fm_init(void)
{
    memcpy(&g_dir, (const void*)DIR_INFO_BASE, sizeof(fm_directory_t));

    if (g_dir.next_free > FILE_POOL_SIZE) {
        memset(&g_dir, 0, sizeof g_dir);
        fm_save_directory();
    }

    uint16_t high = fm_high_water_end();
    if (g_dir.next_free < high || g_dir.next_free > FILE_POOL_SIZE) {
        g_dir.next_free = high;
        fm_save_directory();
    }

    // Ensure we start writing into erased space after boot
    (void)fm_compact_data();
}

void fm_save_directory(void)
{
    /* Erase exactly Info D (0x1000), C (0x1040), B (0x1080). */
    uint16_t addr = DIR_INFO_BASE;
    info_erase_segment(addr);        /* D */
    addr += 0x40;
    info_erase_segment(addr);        /* C */
    addr += 0x40;
    info_erase_segment(addr);        /* B */

    /* Write the full packed directory image back starting at D. */
    info_write(DIR_INFO_BASE, (const uint8_t*)&g_dir, sizeof(fm_directory_t));
}

/* File operations */
fm_status_t fm_write_file(const char* name, fm_file_type_t type,
                          const uint8_t* data, uint16_t nbytes)
{
    if (!name || !data || nbytes == 0) return FM_ERR_BAD_PARAM;

    // Capacity check (total)
    if ((uint32_t)fm_used_bytes() + nbytes > FILE_POOL_SIZE) return FM_ERR_NO_SPACE;

    // Directory slot
    fm_dir_entry_t* e = first_free_entry();
    if (!e) return FM_ERR_DIR_FULL;

    uint16_t dst_abs = (uint16_t)(FILE_POOL_BASE + g_dir.next_free);
    uint16_t end_abs = (uint16_t)(dst_abs + nbytes);

    // If pool is empty, bulk erase all file-pool segments
    if (g_dir.next_free == 0u) {
        uint16_t a;
        for (a = FILE_POOL_BASE; a < FILE_POOL_BASE + FILE_POOL_SIZE; a = (uint16_t)(a + SEG_SIZE))
            flash_erase_segment_main(a);
    }

    // Try using the erased tail of the current segment
    uint16_t first_seg_base = (uint16_t)(dst_abs & ~SEG_MASK);
    uint16_t first_seg_end  = (uint16_t)(first_seg_base + SEG_SIZE);
    uint16_t first_chunk    = (end_abs <= first_seg_end) ? (uint16_t)(end_abs - dst_abs)
                                                         : (uint16_t)(first_seg_end - dst_abs);

    if (first_chunk && !range_is_ff(dst_abs, first_chunk)) {
        // Bump to next segment; don't erase this mixed segment
        dst_abs = first_seg_end;
        g_dir.next_free = (uint16_t)(dst_abs - FILE_POOL_BASE);
        if ((uint32_t)g_dir.next_free + nbytes > FILE_POOL_SIZE) return FM_ERR_NO_SPACE;
        end_abs = (uint16_t)(dst_abs + nbytes);
        first_chunk = 0u;
    }

    // Erase any FULL segments after the first partial region
    uint16_t seg = (uint16_t)((dst_abs + first_chunk + SEG_MASK) & ~SEG_MASK);
    uint16_t last_full_end = (uint16_t)(end_abs & ~SEG_MASK);
    while (seg < last_full_end) {
        if (!range_is_ff(seg, SEG_SIZE)) flash_erase_segment_main(seg);
        seg = (uint16_t)(seg + SEG_SIZE);
    }

    // Program
    flash_write_main(dst_abs, data, nbytes);

    // Directory
    memset(e, 0, sizeof(*e));
    strncpy(e->name, name, FM_MAX_NAME - 1);
    e->type       = (uint8_t)type;
    e->size_bytes = nbytes;
    e->offset     = g_dir.next_free;
    e->checksum   = fm_checksum8(data, nbytes);
    e->valid      = 1;

    g_dir.next_free = (uint16_t)(g_dir.next_free + nbytes);
    fm_save_directory();
    return FM_OK;
}

fm_status_t fm_read_file(const char* name,
                         uint8_t* out_buf,
                         uint16_t max_len,
                         uint16_t* out_nbytes)
{
    fm_dir_entry_t* e = find_entry(name);
    if (!e) return FM_ERR_NOT_FOUND;
    if (e->size_bytes > max_len) return FM_ERR_BAD_PARAM;

    const uint8_t* src = (const uint8_t*)(FILE_POOL_BASE + e->offset);
    memcpy(out_buf, src, e->size_bytes);
    if (out_nbytes) *out_nbytes = e->size_bytes;

    if (fm_checksum8(out_buf, e->size_bytes) != e->checksum) return FM_ERR_CHECKSUM;
    return FM_OK;
}

fm_status_t fm_delete_file(const char* name)
{
    fm_dir_entry_t* e = find_entry(name);
    if (!e) return FM_ERR_NOT_FOUND;

    e->valid = 0;

    // Recompute next_free safely and erase trailing free segments
    (void)fm_compact_data();

    fm_save_directory();
    return FM_OK;
}

uint8_t fm_list_files(char* out_buf, uint16_t max_len)
{
    uint16_t w = 0;
    uint8_t count = 0;
    if (max_len == 0) return 0;
    uint8_t i;
    for (i = 0; i < FM_MAX_FILES; ++i) {
        if (g_dir.entries[i].valid == 1) {
            const char* nm = g_dir.entries[i].name;
            uint16_t len = (uint16_t)strlen(nm);
            if (w + len + 1 >= max_len) break;
            memcpy(out_buf + w, nm, len);
            w += len;
            out_buf[w++] = ',';
            count++;
        }
    }
    if (w == 0) out_buf[0] = '\0';
    else out_buf[w - 1] = '\0';
    return count;
}

fm_status_t fm_compact(void)
{

    fm_status_t s = fm_compact_dir();
    if (s != FM_OK && s != FM_ERR_BAD_PARAM) return s; // BAD_PARAM => nothing to do

    s = fm_compact_data();
    if (s != FM_OK && s != FM_ERR_BAD_PARAM) return s;

    return FM_OK;
}


static uint16_t fm_used_bytes(void) {
    uint32_t used = 0;
    uint8_t i;
    for (i = 0; i < FM_MAX_FILES; ++i) {
        const fm_dir_entry_t* e = &g_dir.entries[i];
        if (e->valid == 1 && e->size_bytes > 0) used += e->size_bytes;
    }
    if (used > FILE_POOL_SIZE) used = FILE_POOL_SIZE;
    return (uint16_t)used;
}

