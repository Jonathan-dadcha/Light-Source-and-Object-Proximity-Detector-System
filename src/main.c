#include <msp430g2553.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>

#include "../header/app.h"
#include "../header/flash.h"
#include "../header/hal.h"
#include "../header/api.h"
#include "../header/bsp.h"
// -------- FILE MODE: helpers --------

// Small command line buffer
#define CMD_MAX 64
static char cmd[CMD_MAX];
static unsigned int rx_len = 0;

// Suppress heartbeat / delays while a FILE,PUT session is active
static uint8_t put_session = 0;

#define PUT_BEGIN_PREFIX  "FILE,PUT,BEGIN,"
#define PUT_BEGIN_LEN     (sizeof(PUT_BEGIN_PREFIX) - 1)  // 15

#define PUT_CHUNK_PREFIX  "FILE,PUT,CHUNK,"
#define PUT_CHUNK_LEN     (sizeof(PUT_CHUNK_PREFIX) - 1)  // 15

// CHUNK streaming state
static uint8_t  chunk_mode = 0;      // 1 while decoding FILE,PUT,CHUNK
static uint8_t  nibble_pending = 0;  // 0/1: first half of a byte seen
static uint8_t  nibble_hi = 0;       // first half value
// Rolling matcher for "FILE,PUT,CHUNK," prefix
static uint8_t chunk_pre_i = 0;    // how many prefix chars matched so far

// Accumulator for one ASCII-hex line
#define LINE_MAX 96u
static uint8_t  line_buf[LINE_MAX];
static uint16_t line_len = 0;

// Generic TX helpers to keep lines consistent with the rest of the firmware
static inline void fm_tx(const char* s){ hal_uart_puts(s); }
static inline void fm_txu(unsigned v){ hal_uart_putu(v); }
static inline void fm_txln(const char* s){ hal_uart_puts(s); hal_uart_puts("\r\n"); }

// ASCII-hex helpers
static int hex_nib(char c){
    if (c>='0' && c<='9') return c-'0';
    if (c>='A' && c<='F') return c-'A'+10;
    if (c>='a' && c<='f') return c-'a'+10;
    return -1;
}

static uint16_t hex_decode_bytes(const char* hex, uint8_t* out, uint16_t max_out){
    // decode pairs until comma/CR/LF or end; returns #bytes, or 0xFFFF on error
    uint16_t n = 0;
    while (*hex && *hex!=',' && *hex!='\r' && *hex!='\n'){
        int hi = hex_nib(*hex++); if (hi<0) return 0xFFFF;
        int lo = hex_nib(*hex++); if (lo<0) return 0xFFFF;
        if (n>=max_out) return 0xFFFF;
        out[n++] = (uint8_t)((hi<<4)|lo);
    }
    return n;
}

static void hex_send_bytes(const uint8_t* p, uint16_t n){
    uint16_t i;
    for (i = 0; i < n; ++i) {
        uint8_t v = p[i];
        uint8_t hi = (uint8_t)((v >> 4) & 0x0F);
        uint8_t lo = (uint8_t)(v & 0x0F);
        char c = (char)((hi < 10) ? ('0' + hi) : ('A' + (hi - 10)));
        hal_uart_putc(c);
        c = (char)((lo < 10) ? ('0' + lo) : ('A' + (lo - 10)));
        hal_uart_putc(c);
    }
}

static const fm_dir_entry_t* fm_find_by_name(const char* name){
    const fm_directory_t* D = fm_get_directory();
    if (!D) return NULL;
    uint8_t i;
    for (i=0;i<FM_MAX_FILES;i++){
        const fm_dir_entry_t* e = &D->entries[i];
        if (e->valid && strncmp(e->name, name, FM_MAX_NAME)==0) return e;
    }
    return NULL;
}

static inline void fm_txhex(uint8_t v){
    uint8_t hi = (v >> 4) & 0x0F, lo = v & 0x0F;
    char c = (char)((hi < 10) ? ('0' + hi) : ('A' + (hi - 10)));
    fm_tx((char[]){c, '\0'});
    c = (char)((lo < 10) ? ('0' + lo) : ('A' + (lo - 10)));
    fm_tx((char[]){c, '\0'});
}


#define BOARD_VCC_MV 3300u

static void tx_putc(char c){
    while (!(IFG2 & UCA0TXIFG)) {}
    UCA0TXBUF = (unsigned char)c;
}
static void tx_write(const char* s){ while (*s) tx_putc(*s++); }
static void tx_line(const char* s){ tx_write(s); tx_putc('\r'); tx_putc('\n'); }
static void delay_ms(unsigned int ms){ while (ms--) __delay_cycles(1000); }

void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;

    clock_init_1mhz();
    uart_init_9600();

    sysConfig();

    IE2 &= ~UCA0RXIE;            // <— disable UART RX IRQ explicitly

    fm_init();

    // --- peripherals for sensors/actuators ---
    api_servo_set_angle(90);
    hal_ultra_init_polling();
    hal_ldr_begin();
    api_servo_begin();


    tx_line("READY");

    unsigned int ticks = 0;

    for (;;) {
        // --- poll RX directly ---
        if (IFG2 & UCA0RXIFG) {
            char c = (char)UCA0RXBUF;

            // --- CHUNK streaming mode: decode ASCII-hex directly into s_put.buf ---
            if (chunk_mode) {
                if (c == '\r') { /* ignore */ }
                else if (c == '\n') {
                    // End of this CHUNK line — flush and return to command mode
                    chunk_mode = 0;
                    rx_len = 0;
                    chunk_pre_i = 0;
                    if (nibble_pending) {
                        // Odd number of hex nibbles — report the condition, then reset
                        nibble_pending = 0;
                        line_len = 0;
                        fm_put_abort();
                        put_session = 0;
                        fm_txln("FILE,PUT,ERR,BAD_HEX");
                    } else {
                        fm_status_t st = fm_put_chunk(line_buf, line_len);
                        line_len = 0;
                        if (st == FM_OK) {
                            fm_txln("FILE,PUT,CHUNK,OK");
                        } else {
                            fm_tx("FILE,PUT,ERR,"); fm_txu((unsigned)st); fm_txln("");
                        }
                    }
                }
                else {
                    // ignore spaces
                    if (c == ' ') continue;

                    int v = hex_nib(c);
                    if (v < 0) {
                        // Print the exact offending character code
                        fm_tx("DEBUG,BADCH,0x"); fm_txhex((uint8_t)c); fm_txln("");
                        nibble_pending = 0;
                        chunk_mode = 0;
                        line_len = 0;
                        chunk_pre_i = 0;
                        fm_put_abort();
                        put_session = 0;
                        fm_txln("FILE,PUT,ERR,BAD_HEX");
                    } else if (!nibble_pending) {
                        nibble_pending = 1;
                        nibble_hi = (uint8_t)v;
                    } else {
                        // have a full byte now
                        uint8_t byte = (uint8_t)((nibble_hi << 4) | (uint8_t)v);
                        nibble_pending = 0;

                        if (line_len < LINE_MAX) {
                            line_buf[line_len++] = byte;
                        } else {
                            // too many bytes in one CHUNK line → abort this line cleanly
                            nibble_pending = 0;
                            chunk_mode = 0;
                            line_len = 0;
                            chunk_pre_i = 0;
                            fm_put_abort();
                            put_session = 0;
                            fm_txln("FILE,PUT,ERR,TOO_LONG");
                        }
                    }
                }
                continue;  // handled this char
            }

            // --- Normal line mode  ---
            if (c == '\r') continue;

            if (c == '\n') {
                cmd[rx_len] = '\0';
                memset(cmd + rx_len, 0, CMD_MAX - rx_len);

                // ===== handle commands using 'cmd' =====
                if ((rx_len==4) &&
                    (cmd[0]=='P'||cmd[0]=='p') &&
                    (cmd[1]=='I'||cmd[1]=='i') &&
                    (cmd[2]=='N'||cmd[2]=='n') &&
                    (cmd[3]=='G'||cmd[3]=='g')) {
                    tx_line("PONG");
                }
                else if (rx_len>=7 && strncmp(cmd,"STATE,1",7)==0) {
                    servo_ultra_sweep();
                    tx_line("FIN");
                }
                else if (rx_len>=7 && strncmp(cmd,"STATE,2",7)==0) {
                    unsigned int deg = 90;
                    const char* p = cmd+7;
                    if (*p==',') {
                        ++p; deg=0;
                        while (*p>='0' && *p<='9') { deg = deg*10 + (*p - '0'); ++p; }
                        if (deg>180) deg=180;
                    }
                    api_servo_set_angle((uint8_t)deg);
                    delay_ms(150u);
                    uint16_t cm;
                    if (api_ultra_get_cm(&cm)) {
                        tx_write("DIST,"); hal_uart_putu(deg); tx_putc(','); hal_uart_putu(cm); tx_line("");
                    } else {
                        tx_write("DIST,"); hal_uart_putu(deg); tx_line(",-1");
                    }
                }
                else if (rx_len>=7 && strncmp(cmd,"STATE,3",7)==0) {
                    api_servo_set_angle(90);
                    api_ldr_step_print(BOARD_VCC_MV);
                }
                else if (rx_len>=7 && strncmp(cmd,"STATE,4",7)==0) {
                    servo_ldr_sweep(BOARD_VCC_MV);
                    tx_line("FIN");
                }
                else if (rx_len>=7 && strncmp(cmd,"STATE,5",7)==0) {
                    servo_combo_sweep(BOARD_VCC_MV);
                    tx_line("FIN");
                }
                else if (rx_len>=7 && strncmp(cmd,"STATE,6",7)==0) {
                    api_servo_set_angle(90);
                    api_ldr_calibrate_run(BOARD_VCC_MV);
                    tx_line("FIN");
                }
                else if (rx_len >= 8 && strncmp(cmd, "CAL,DUMP", 8) == 0) {
                    api_ldr_calib_dump_uart();
                }
                /* ==================== FILE COMMANDS ==================== */
                else if (rx_len>=7 && strncmp(cmd,"FILE,LS",7)==0) {
                    // Stream names directly, no stack buffer.
                    const fm_directory_t* dir = fm_get_directory();
                    uint8_t cnt = 0, i, first = 1;

                    // count files
                    for (i = 0; i < FM_MAX_FILES; i++) {
                        if (dir->entries[i].valid == 1) cnt++;
                    }

                    // header
                    fm_tx("FILE,LS,"); fm_txu(cnt); fm_tx(",");

                    // names + sizes => name:size;name2:size2
                    for (i = 0; i < FM_MAX_FILES; i++) {
                        const fm_dir_entry_t* e = &dir->entries[i];
                        if (e->valid == 1) {
                            if (!first) tx_putc(';');
                            first = 0;
                            // print name (up to FM_MAX_NAME or early NUL)
                            uint8_t k;
                            for (k = 0; k < FM_MAX_NAME; k++) {
                                char ch = e->name[k];
                                if (!ch) break;
                                tx_putc(ch);
                            }
                            tx_putc(':');
                            hal_uart_putu(e->size_bytes);
                        }
                    }
                    fm_txln("");
                }
                else if (rx_len>=9 && strncmp(cmd,"FILE,DEL,",9)==0) {
                    const char* name = cmd+9;
                    fm_status_t st = fm_delete_file(name);
                    if (st==FM_OK) fm_txln("FILE,DEL,OK");
                    else { fm_tx("FILE,DEL,ERR,"); fm_txu((unsigned)st); fm_txln(""); }
                }
                else if (rx_len>=9 && strncmp(cmd,"FILE,GET,",9)==0) {
                    const char* name = cmd+9;
                    const fm_dir_entry_t* e = fm_find_by_name(name);
                    if (!e){ fm_txln("FILE,GET,ERR,NOT_FOUND"); }
                    else {
                        fm_tx("FILE,GET,BEGIN,"); fm_tx(name); fm_tx(","); fm_txu(e->size_bytes); fm_txln("");
                        uint32_t pos = 0;
                        while (pos < e->size_bytes){
                            uint16_t chunk = (uint16_t)((e->size_bytes - pos) > 64 ? 64 : (e->size_bytes - pos));
                            const uint8_t* src = (const uint8_t*)(uintptr_t)(FILE_POOL_BASE + e->offset + pos);
                            fm_tx("FILE,GET,DATA,"); hex_send_bytes(src, chunk); fm_txln("");
                            pos += chunk;
                        }
                        fm_txln("FILE,GET,END,OK");
                    }
                }
                else if (rx_len>=9 && strncmp(cmd,"FILE,SHOW",9)==0) {
                    /* Usage:
                       - "FILE,SHOW"            -> open LCD browser (PB0/PB1)
                       - "FILE,SHOW,<name>"     -> show that file immediately on LCD
                    */
                    if (rx_len == 9 || cmd[9] == '\r' || cmd[9] == '\n') {
                        api_file_browse_lcd();
                        fm_txln("FILE,SHOW,OK");
                    } else if (cmd[9] == ',') {
                        const char* name = cmd + 10;
                        api_file_show_lcd(name);
                        fm_txln("FILE,SHOW,OK");
                    } else {
                        fm_txln("FILE,SHOW,ERR");
                    }
                }
                else if (rx_len >= 14 && strncmp(cmd, "FILE,SHOW,EXIT", 14) == 0) {
                    /* If the LCD viewer is running, this flag makes it bail out immediately.
                       If it hasn't started yet, the flag will still be seen when it does. */
                    api_file_show_request_exit();
                    fm_txln("FILE,SHOW,OK");
                }

                else if (rx_len >= 12 && strncmp(cmd, "FILE,FORMAT", 11) == 0) {
                    fm_format();
                    fm_txln("FILE,FORMAT,OK");
                }
                else if (rx_len >= 11 && strncmp(cmd, "FILE,SPACE", 10) == 0) {
                    uint16_t tf, tail;
                    fm_query_space(&tf, &tail);
                    fm_tx("FILE,SPACE,"); fm_txu(tf); fm_tx(","); fm_txu(tail); fm_txln("");
                }
                else if (strncmp(cmd, PUT_BEGIN_PREFIX, PUT_BEGIN_LEN) == 0) {
                    const char* p = cmd + PUT_BEGIN_LEN;
                    const char* q = strchr(p, ',');
                    if (!q) { fm_txln("FILE,PUT,ERR,BAD_ARGS"); }
                    else {
                        uint16_t name_len = (uint16_t)(q - p);
                        if (name_len == 0 || name_len > FM_MAX_NAME) {
                            fm_txln("FILE,PUT,ERR,BAD_NAME");
                        } else {
                            char fname[FM_MAX_NAME+1];
                            memcpy(fname, p, name_len);
                            fname[name_len] = '\0';

                            /* reject duplicates */
                            if (fm_find_by_name(fname)) {
                                fm_txln("FILE,PUT,ERR,EXISTS");
                                goto put_begin_done;
                            }

                            p = q + 1; q = strchr(p, ',');
                            if (!q) { fm_txln("FILE,PUT,ERR,BAD_ARGS"); }
                            else {
                                unsigned t = 0;
                                while (p < q) {
                                    if (*p < '0' || *p > '9') { fm_txln("FILE,PUT,ERR,BAD_TYPE"); goto put_begin_done; }
                                    t = t*10 + (*p - '0'); p++;
                                }
                                uint8_t type = (uint8_t)t;

                                p = q + 1; unsigned sz = 0;
                                while (*p && *p != '\r' && *p != '\n') {
                                    if (*p < '0' || *p > '9') { fm_txln("FILE,PUT,ERR,BAD_SIZE"); goto put_begin_done; }
                                    sz = sz*10 + (*p - '0'); p++;
                                }

                                fm_status_t st = fm_put_begin(fname, type, (uint16_t)sz);
                                if (st == FM_ERR_BUSY) {
                                    // Self-heal a stale session once
                                    fm_put_abort();
                                    put_session = 0;
                                    st = fm_put_begin(fname, type, (uint16_t)sz);
                                }
                                if (st == FM_OK) {
                                    put_session = 1;
                                    fm_txln("FILE,PUT,BEGIN,OK");
                                } else if (st == FM_ERR_NEEDS_REBUILD) {
                                    fm_txln("FILE,PUT,ERR,NEEDS_REBUILD");
                                } else {
                                    fm_tx("FILE,PUT,ERR,"); fm_txu((unsigned)st); fm_txln("");
                                }

                            }
                        }
                    }
                put_begin_done: ;
                }
                else if (rx_len >= PUT_CHUNK_LEN && strncmp(cmd, PUT_CHUNK_PREFIX, PUT_CHUNK_LEN) == 0) {
                    const char* hex = cmd + PUT_CHUNK_LEN;
                    while (*hex == ' ') hex++;     // tolerate spaces

                    if (*hex) {
                        // Inline payload present on the same line — decode & write now
                        uint8_t tmp[LINE_MAX];
                        uint16_t n = 0;
                        while (*hex && *hex != '\r' && *hex != '\n') {
                            int hi = hex_nib(*hex++); if (hi < 0) { fm_txln("FILE,PUT,ERR,BAD_HEX"); goto chunk_header_done; }
                            int lo = hex_nib(*hex++); if (lo < 0) { fm_txln("FILE,PUT,ERR,BAD_HEX"); goto chunk_header_done; }
                            if (n >= LINE_MAX)        { fm_txln("FILE,PUT,ERR,TOO_LONG"); goto chunk_header_done; }
                            tmp[n++] = (uint8_t)((hi << 4) | lo);
                        }
                        fm_status_t st = fm_put_chunk(tmp, n);
                        if (st == FM_OK) fm_txln("FILE,PUT,CHUNK,OK");
                        else { fm_tx("FILE,PUT,ERR,"); fm_txu((unsigned)st); fm_txln(""); }
                    } else {
                        // No inline data — switch to streaming for the next line(s)
                        chunk_mode = 1;
                        nibble_pending = 0;
                        line_len = 0;
                        fm_txln("FILE,PUT,CHUNK,READY");
                    }
                chunk_header_done:;
                }
                else if (rx_len >= 12 && strncmp(cmd, "FILE,PUT,END", 12) == 0) {
                    fm_status_t st = fm_put_end();
                    put_session = 0;   // end quiet period regardless of the result
                    if (st == FM_OK) {
                        fm_txln("FILE,PUT,END,OK");
                    } else {
                        fm_tx("FILE,PUT,END,ERR,"); fm_txu((unsigned)st); fm_txln("");
                    }
                }
                else if (rx_len >= 14 && strncmp(cmd, "FILE,PUT,ABORT", 14) == 0) {
                    fm_put_abort();
                    put_session = 0;
                    fm_txln("FILE,PUT,ABORT,OK");
                }

                /* ================= end FILE COMMANDS ==================== */

                else if (rx_len>0) {
                    tx_write("ECHO,"); tx_line(cmd);
                }

                rx_len = 0;
            }
            else {
                // accumulate a short command line; rolling detect "FILE,PUT,CHUNK,"
                if (!chunk_mode) {
                    // Rolling prefix match
                    if (c == PUT_CHUNK_PREFIX[chunk_pre_i]) {
                        chunk_pre_i++;
                        if (chunk_pre_i == PUT_CHUNK_LEN) {
                            // matched the whole "FILE,PUT,CHUNK," — switch to stream mode immediately
                            chunk_mode = 1;
                            nibble_pending = 0;
                            line_len = 0;
                            rx_len = 0;        // drop any partial command
                            chunk_pre_i = 0;
                            continue;          // next char is the first hex nibble on this same line
                        }
                    } else {
                        chunk_pre_i = (c == PUT_CHUNK_PREFIX[0]) ? 1 : 0;
                    }

                    // Only store into cmd[] while not streaming
                    if (rx_len < CMD_MAX - 1) {
                        cmd[rx_len++] = c;
                    } else {
                        // overflow → drop this line
                        rx_len = 0;
                    }
                }
                // if chunk_mode was set above, the next loop iteration will handle hex decoding
            }
        }

        if (!put_session && !chunk_mode && rx_len == 0 && line_len == 0) {
            if (!(IFG2 & UCA0RXIFG)) {           // RX idle right now
                if (++ticks >= 25000) {          // ~25s at 1MHz/9600bps
                    // periodic heartbeat when idle
                    tx_line("HB");
                    ticks = 0;
                }
            } else {
                // RX activity right now — reset the idle timer
                ticks = 0;
            }
        } else {
            // while receiving/streaming, never stall or chat
            ticks = 0;
        }
    }
}
