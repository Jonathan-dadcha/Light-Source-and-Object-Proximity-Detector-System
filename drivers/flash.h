#ifndef FLASH_H
#define FLASH_H

#include <stdint.h>
#include <stdbool.h>

/* File pool location in main flash. Verify against linker map. */
#ifndef FILE_POOL_BASE
#define FILE_POOL_BASE   0xF000u
#endif

#ifndef FILE_POOL_SIZE
#define FILE_POOL_SIZE   2048u    /* 2 KB */
#endif

/* Directory in Information Memory. Entire directory fits in 256B. */
#ifndef DIR_INFO_BASE
#define DIR_INFO_BASE    0x1000u  /* start of Info Flash segment D */
#endif

#define FM_MAX_FILES   10u
#define FM_MAX_NAME    12u        /* null terminated */

/* Status codes */
typedef enum {
    FM_OK = 0,
    FM_ERR_BAD_PARAM = 1,
    FM_ERR_NO_SPACE  = 2,   // data area can't fit file even after compact
    FM_ERR_DIR_FULL  = 3,   // no directory slot even after compact
    FM_ERR_BUSY      = 4,
    FM_ERR_NOT_FOUND = 5,
    FM_ERR_CHECKSUM  = 6,
    FM_ERR_NEEDS_REBUILD = 7,  // total space ok, but not enough contiguous tail
} fm_status_t;

typedef enum {
    FM_TYPE_TEXT   = 0,
    FM_TYPE_SCRIPT = 1
} fm_file_type_t;

/* ---- Directory layout in Information Flash (D..B only, 3×64B = 192B) ---- */
/* Pack to 1-byte alignment so the whole directory fits in 192 bytes. */
#pragma pack(push,1)

/* One directory entry (19 bytes when packed) */
typedef struct {
    char     name[FM_MAX_NAME]; /* 12 */
    uint16_t size_bytes;        /* 2  */
    uint16_t offset;            /* 2  */
    uint8_t  type;              /* 1  */
    uint8_t  checksum;          /* 1  */
    uint8_t  valid;             /* 1  */
} fm_dir_entry_t;

/* Directory image persisted in Info Flash (190 + 2 = 192 bytes) */
typedef struct {
    uint16_t       next_free;                /* 2  */
    fm_dir_entry_t entries[FM_MAX_FILES];    /* 10×19 = 190 */
} fm_directory_t;

#pragma pack(pop)

/* Build-time guarantee: never exceed Info D..B (192 bytes total). */
#define INFO_DIR_MAX_BYTES (3u * 64u)
typedef char _dir_must_fit_info[(sizeof(fm_directory_t) <= INFO_DIR_MAX_BYTES) ? 1 : -1];

/* Init and persist */
void     fm_init(void);
void     fm_save_directory(void);
void     fm_format(void);

/* File operations */
fm_status_t fm_write_file(const char* name,
                          fm_file_type_t type,
                          const uint8_t* data,
                          uint16_t nbytes);

fm_status_t fm_read_file(const char* name,
                         uint8_t* out_buf,
                         uint16_t max_len,
                         uint16_t* out_nbytes);

fm_status_t fm_delete_file(const char* name);
/* Streaming PUT API (for UART ASCII-hex CHUNK lines) */
fm_status_t fm_put_begin(const char* name, uint8_t type, uint16_t expected);
fm_status_t fm_put_chunk(const uint8_t* data, uint16_t len);
fm_status_t fm_put_end(void);


uint8_t     fm_list_files(char* out_buf, uint16_t max_len);

/* Utilities */
uint8_t     fm_checksum8(const uint8_t* data, uint16_t n);
bool        fm_has_space(uint16_t nbytes);
fm_status_t fm_compact(void);

/* Debug */
const fm_directory_t* fm_get_directory(void);

void fm_query_space(uint16_t* total_free, uint16_t* tail_free);

void fm_put_abort(void);


#endif /* FLASH_H */
