#include "../header/api.h"
#include "../header/flash.h"
#include "../header/hal.h"
#include "../header/bsp.h"
#include "../header/app.h"

#include <msp430g2553.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ============================================================
   API layer: high-level features + text protocol for GUI
   - UART low-level is in HAL (hal_uart_* + RX ISR ring)
   - Ultrasonic uses polling-mode HAL (no interrupts)
   - LDR uses HAL single/dual-channel helpers
   ============================================================ */

/* -------------------- Local helpers -------------------- */

static unsigned int median3(unsigned int a, unsigned int b, unsigned int c)
{
    unsigned int hi = (a>b)?a:b, lo = (a<b)?a:b;
    if (c > hi) return hi;
    if (c < lo) return lo;
    return c;
}

static unsigned int majority5(unsigned int v1, unsigned int v2, unsigned int v3,
                              unsigned int v4, unsigned int v5)
{
    unsigned int vals[5] = {v1, v2, v3, v4, v5};

    // Sort the 5 values
    int i;
    for (i = 1; i < 5; i++) {
        unsigned int key = vals[i];
        int j = i - 1;
        while (j >= 0 && vals[j] > key) {
            vals[j + 1] = vals[j];
            j--;
        }
        vals[j + 1] = key;
    }

    // Find the tightest cluster of 3 consecutive values
    unsigned int best_start = 0;
    unsigned int best_range = vals[2] - vals[0]; // initial range for first window

    for (i = 1; i <= 2; i++) {
        unsigned int range = vals[i + 2] - vals[i];
        if (range < best_range) {
            best_range = range;
            best_start = i;
        }
    }

    // Average of the chosen cluster
    unsigned int sum = vals[best_start] + vals[best_start + 1] + vals[best_start + 2];
    return sum / 3;
}

/* ---- FILE,SHOW exit watcher (polled while LCD view is active) ---- */
static volatile uint8_t file_show_abort = 0;

/* Match literal "FILE,SHOW,EXIT" in the RX stream; ignore CR/LF; overlap OK */
static void _file_show_service_exit_uart(void)
{
    static const char pat[] = "FILE,SHOW,EXIT";
    static uint8_t m = 0;   /* match index into pat */

    while (IFG2 & UCA0RXIFG) {
        char c = (char)UCA0RXBUF;
        if (c == '\r' || c == '\n') { m = 0; continue; }
        if (c == pat[m]) {
            if (++m == (sizeof(pat) - 1)) {   /* reached NUL */
                file_show_abort = 1;
                m = 0;
            }
        } else {
            m = (c == pat[0]) ? 1 : 0;
        }
    }
}
static inline uint8_t _file_show_should_exit(void)
{
    _file_show_service_exit_uart();
    return file_show_abort;
}
static inline void _file_show_exit_clear(void) { file_show_abort = 0; }

/* Allow main.c to request an exit even if our UART sniffer hasn't seen it yet */
void api_file_show_request_exit(void)
{
    file_show_abort = 1;
}


/* --- Poll-aware delay: breaks long delays so RX never overruns --- */
static void _delay_with_exit_poll(uint16_t ms)
{
    while (ms--) {
        /* At 1 MHz/9600 bps, ~1 byte every ~1.04 ms.
           Service RX several times per ms to prevent overrun. */
        _file_show_service_exit_uart();
        __delay_cycles(200);
        _file_show_service_exit_uart();
        __delay_cycles(200);
        _file_show_service_exit_uart();
        __delay_cycles(200);
        _file_show_service_exit_uart();
        __delay_cycles(200);
        _file_show_service_exit_uart();
    }
}


/* --- HEX TX helper for FILE,GET --- */
static void _hex_send_bytes(const uint8_t* p, uint16_t n)
{
    static const char H[] = "0123456789ABCDEF";
    uint8_t i;
    for (i = 0; i < n; ++i) {
        char pair[2];
        pair[0] = H[p[i] >> 4];
        pair[1] = H[p[i] & 0x0F];
        hal_uart_putc(pair[0]);
        hal_uart_putc(pair[1]);
    }
}

/* --- LDR dual-read helper (A5/LDR1 + A7/LDR2) --- */
static bool s_ultra_inited;
static inline void ultra_lazy_init(void)
{
    if(!s_ultra_inited){
        hal_ultra_init_polling();    /* uses TA0; leaves TA1 for servo PWM */
        s_ultra_inited = 1;
    }
}

/* 0..180 safety clamp */
static inline uint8_t clamp_deg(int v)
{
    if (v < 0)   return 0;
    if (v > 180) return 180;
    return (uint8_t)v;
}

/* ---------------- Public-ish wrappers --------------- */
void api_servo_begin(void)
{
    /* PWM output on P2.1 */
    P2SEL2 &= ~BIT1;
    P2SEL  |=  BIT1;
    P2DIR  |=  BIT1;

    /* Timer1_A @ 50 Hz PWM on CCR1 */
    TA1CTL   = TASSEL_2 | MC_1 | TACLR;          /* SMCLK, up mode */
    TA1CCR0  = SERVO_PWM_PERIOD_TICKS - 1;       /* 20 ms @ 1 MHz */
    TA1CCTL1 = OUTMOD_7;                         /* P2.1 */
    TA1CCR1  = 1500;                             /* center */

    TA1CCTL2 = 0;
}

/* One-shot arg storage for main loop */
static volatile uint16_t s_arg_deg;
static volatile uint8_t  s_arg_valid;

void api_init(void)
{
    sysConfig();
    s_arg_deg = 90;
    hal_servo_set_angle(90);  /* center the servo on boot */
}

void api_servo_set_angle(uint8_t angle_deg)
{
    hal_servo_set_angle(clamp_deg(angle_deg));
}


bool api_ultra_get_cm(uint16_t* cm_out)
{
    ultra_lazy_init();
    return hal_ultra_measure_cm_polling(cm_out);
}

/* Prepare ADC10 for dual LDR read (A5 + A7) */
void api_ldr_step_print(uint16_t vcc_mv)
{
    uint16_t raw = hal_ldr_read_avg(8);
    uint32_t t   = (uint32_t)raw * vcc_mv + 511u;
    uint16_t mv  = (uint16_t)(t / 1023u);

    hal_uart_puts("LDR,");
    hal_uart_putu(raw);
    hal_uart_puts(",");
    hal_uart_putu(mv);
    hal_uart_puts("\r\n");
    hal_delay_ms(100u);
}

/* Read both legacy LDR channels (A5/A7) and print:
   LDR1,<raw>,<mV>\r\n
   LDR2,<raw>,<mV>\r\n  */
void api_ldr_separate_test(uint16_t vcc_mv)
{
    uint16_t v[2];
    ldrs_read_blocking(v);

    uint16_t mv = (uint32_t)v[0] * vcc_mv / 1023u;
    hal_uart_puts("LDR1,"); hal_uart_putu(v[0]); hal_uart_puts(","); hal_uart_putu(mv); hal_uart_puts("\r\n");

    mv = (uint32_t)v[1] * vcc_mv / 1023u;
    hal_uart_puts("LDR2,"); hal_uart_putu(v[1]); hal_uart_puts(","); hal_uart_putu(mv); hal_uart_puts("\r\n");

    hal_delay_ms(100u);
}

/* ====================== LDR CALIBRATION (PB0-driven) ======================= */
/* --- LDR calibration: file names used by calibrate+loader --- */
#ifndef CAL_FILE_LDR1
#define CAL_FILE_LDR1   "LDR1_CAL"
#define CAL_FILE_LDR2   "LDR2_CAL"
#endif

void api_ldr_calibrate_run(uint16_t vcc_mv)
{
    (void)vcc_mv;
    uint16_t ldr1[10] = {0};
    uint16_t ldr2[10] = {0};
    uint16_t v[2];
    unsigned i, j;

    ldrs_read_blocking(v); /* one dummy read to settle */

    hal_enable_interrupts();

    hal_uart_puts("CAL,BEGIN,10x2\r\n");
    hal_uart_puts("CAL,INSTR,Place light at each mark (5..50cm), press PB0 to sample\r\n");

    for (i = 0; i < 10; ++i) {
        unsigned dist_cm = (i + 1u) * 5u;


        hal_uart_puts("CAL,PLACE,");
        hal_uart_putu(dist_cm);
        hal_uart_puts("\r\n");

        /* clear any stale press */
        pb0_pressed = false;

        /* wait for PB0 press */
        while (!pb0_pressed) { __no_operation(); }
        /* basic debounce */
        hal_delay_ms(50u);
        pb0_pressed = false;

        /* take an averaged dual-sample for robustness */
        uint32_t acc1 = 0, acc2 = 0;
        for (j = 0; j < 8; ++j) {
            ldrs_adc_read_blocking(v);   /* v[0] = LDR1(A5), v[1] = LDR2(A7) */
            acc1 += v[0];
            acc2 += v[1];
            __delay_cycles(500);     /* ~0.5 ms between reads */
        }
        ldr1[i] = (uint16_t)(acc1 / 8u);
        ldr2[i] = (uint16_t)(acc2 / 8u);

        /* echo what we saved */
        hal_uart_puts("CAL,SAVED,");
        hal_uart_putu(i);            hal_uart_putc(',');
        hal_uart_putu(dist_cm);      hal_uart_putc(',');
        hal_uart_putu(ldr1[i]);      hal_uart_putc(',');
        hal_uart_putu(ldr2[i]);      hal_uart_puts("\r\n");
    }

    hal_disable_interrupts();

    /* Store into FLASH as two small binary files (10 * uint16_t each) */
    (void)fm_delete_file(CAL_FILE_LDR1);
    (void)fm_delete_file(CAL_FILE_LDR2);

    fm_status_t s1 = fm_write_file(CAL_FILE_LDR1, FM_TYPE_TEXT,  (const uint8_t*)ldr1, sizeof(ldr1));
    fm_status_t s2 = fm_write_file(CAL_FILE_LDR2, FM_TYPE_TEXT,  (const uint8_t*)ldr2, sizeof(ldr2));

    if (s1 == FM_OK && s2 == FM_OK) {
        hal_uart_puts("CAL,END,OK\r\n");
    } else {
        hal_uart_puts("CAL,END,ERR,");
        hal_uart_putu((unsigned)s1); hal_uart_putc(',');
        hal_uart_putu((unsigned)s2); hal_uart_puts("\r\n");
    }
}

/* Load the two 10-entry tables (raw ADC) from FLASH files. Returns true on success. */
bool api_ldr_calib_load(uint16_t out_ldr1[10], uint16_t out_ldr2[10])
{
    uint16_t n = 0;
    fm_status_t s;

    if (!out_ldr1 || !out_ldr2) return false;

    s = fm_read_file(CAL_FILE_LDR1, (uint8_t*)out_ldr1, sizeof(uint16_t)*10, &n);
    if (s != FM_OK || n != sizeof(uint16_t)*10) return false;

    s = fm_read_file(CAL_FILE_LDR2, (uint8_t*)out_ldr2, sizeof(uint16_t)*10, &n);
    if (s != FM_OK || n != sizeof(uint16_t)*10) return false;

    return true;
}



/* Load 10 samples per LDR from FLASH into caller buffers.
   Returns true on success (both files present & size OK). */
bool api_ldr_calib_get(uint16_t out_ldr1[10], uint16_t out_ldr2[10])
{
    if (!out_ldr1 || !out_ldr2) return false;

    uint16_t n = 0;
    fm_status_t st;

    st = fm_read_file(CAL_FILE_LDR1, (uint8_t*)out_ldr1, 10u * sizeof(uint16_t), &n);
    if (st != FM_OK || n != 10u * sizeof(uint16_t)) return false;

    st = fm_read_file(CAL_FILE_LDR2, (uint8_t*)out_ldr2, 10u * sizeof(uint16_t), &n);
    if (st != FM_OK || n != 10u * sizeof(uint16_t)) return false;

    return true;
}

void api_ldr_calib_dump_uart(void)
{
    uint16_t t1[10], t2[10];
    if (!api_ldr_calib_get(t1, t2)) {
        hal_uart_puts("CAL,DUMP,ERR\r\n");
        return;
    }

    unsigned i;
    for (i = 0; i < 10; ++i) {
        unsigned dist_cm = (i + 1u) * 5u;
        hal_uart_puts("CAL,TBL,");
        hal_uart_putu(dist_cm);  hal_uart_putc(',');
        hal_uart_putu(t1[i]);    hal_uart_putc(',');
        hal_uart_putu(t2[i]);    hal_uart_puts("\r\n");
    }
    hal_uart_puts("CAL,DUMP,OK\r\n");
}


/* Sweep 0→180→0 in 3° steps, measure distance 3× per angle,
   output CSV lines "angle,cm" where cm=-1 means timeout/no echo. */
void servo_ultra_sweep(void)
{
    unsigned int angle;

    ultra_lazy_init();
    api_servo_begin();

    /* forward sweep */
    for (angle = 0U; angle <= 180U; angle += 1U) {
        unsigned int cm1 = 0xFFFF, cm2 = 0xFFFF, cm3 = 0xFFFF, cm4 = 0xFFFF, cm5 = 0xFFFF, cm;

        api_servo_set_angle((uint8_t)angle);

        (void)api_ultra_get_cm(&cm1);
        //hal_delay_ms(15u);
        (void)api_ultra_get_cm(&cm2);
        //hal_delay_ms(15u);
        (void)api_ultra_get_cm(&cm3);
        //hal_delay_ms(15u);
        (void)api_ultra_get_cm(&cm4);
        //hal_delay_ms(15u);
        (void)api_ultra_get_cm(&cm5);

        if (cm1==0xFFFF && cm2==0xFFFF && cm3==0xFFFF && cm4==0xFFFF && cm5==0xFFFF){
            hal_uart_putu(angle); hal_uart_putc(','); hal_uart_puts("-1\r\n");
        } else {
            if (cm1==0xFFFF) cm1=65000U;
            if (cm2==0xFFFF) cm2=65000U;
            if (cm3==0xFFFF) cm3=65000U;
            if (cm4==0xFFFF) cm4=65000U;
            if (cm5==0xFFFF) cm5=65000U;
            cm = majority5(cm1,cm2,cm3,cm4,cm5);
            if (cm==65000U){ hal_uart_putu(angle); hal_uart_putc(','); hal_uart_puts("-1\r\n"); }
            else           { hal_uart_putu(angle); hal_uart_putc(','); hal_uart_putu(cm); hal_uart_puts("\r\n"); }
        }
    }
}

void servo_ldr_sweep(uint16_t vcc_mv)
{
    unsigned int angle;
    uint16_t v[2];

    api_servo_begin();

    for (angle = 0U; angle <= 180U; angle += 1U) {
        api_servo_set_angle((uint8_t)angle);
        hal_delay_ms(20u);  /* wait for servo to settle */

        /* v[0] = LDR1 on A5, v[1] = LDR2 on A7 */
        ldrs_read_blocking(v);

        /* Convert to mV with rounding: (raw * Vcc + 511) / 1023 */
        uint16_t mv1 = (uint16_t)(((uint32_t)v[0] * vcc_mv + 511u) / 1023u);
        uint16_t mv2 = (uint16_t)(((uint32_t)v[1] * vcc_mv + 511u) / 1023u);

        /* CSV line per angle */
        hal_uart_puts("LDRS,");
        hal_uart_putu(angle);  hal_uart_putc(',');
        hal_uart_putu(v[0]);   hal_uart_putc(',');
        hal_uart_putu(mv1);    hal_uart_putc(',');
        hal_uart_putu(v[1]);   hal_uart_putc(',');
        hal_uart_putu(mv2);    hal_uart_puts("\r\n");
    }
}

void servo_combo_sweep(uint16_t vcc_mv)
{
    unsigned int angle;
    ultra_lazy_init();
    api_servo_begin();

    for (angle = 0; angle <= 180; ++angle) {
        unsigned int cm1 = 0xFFFF, cm2 = 0xFFFF, cm3 = 0xFFFF, cm4 = 0xFFFF, cm5 = 0xFFFF, cm;

        api_servo_set_angle((uint8_t)angle);

        (void)api_ultra_get_cm(&cm1);
        //hal_delay_ms(15u);
        (void)api_ultra_get_cm(&cm2);
        //hal_delay_ms(15u);
        (void)api_ultra_get_cm(&cm3);
        //hal_delay_ms(15u);
        (void)api_ultra_get_cm(&cm4);
        //hal_delay_ms(15u);
        (void)api_ultra_get_cm(&cm5);

        // give a short gap so hardware is idle
        hal_delay_ms(5u);

        // --- now LDRs ---
        uint16_t v[2];
        ldrs_read_blocking(v);
        uint16_t mv1 = (uint32_t)v[0] * vcc_mv / 1023u;
        uint16_t mv2 = (uint32_t)v[1] * vcc_mv / 1023u;

        // --- combined UART line ---
        hal_uart_puts("COMBO,");
        hal_uart_putu(angle);  hal_uart_putc(',');
        if (cm1==0xFFFF && cm2==0xFFFF && cm3==0xFFFF && cm4==0xFFFF && cm5==0xFFFF){
            hal_uart_putu(angle); hal_uart_putc(','); hal_uart_puts("-1\r\n");
        } else {
            if (cm1==0xFFFF) cm1=65000U;
            if (cm2==0xFFFF) cm2=65000U;
            if (cm3==0xFFFF) cm3=65000U;
            if (cm4==0xFFFF) cm4=65000U;
            if (cm5==0xFFFF) cm5=65000U;
            cm = majority5(cm1,cm2,cm3,cm4,cm5);
            if (cm==65000U){ hal_uart_puts("-1\r"); }
            else           { hal_uart_putu(cm); hal_uart_puts("\r"); }
        }
        hal_uart_putc(',');
        hal_uart_putu(v[0]);   hal_uart_putc(',');
        hal_uart_putu(mv1);    hal_uart_putc(',');
        hal_uart_putu(v[1]);   hal_uart_putc(',');
        hal_uart_putu(mv2);    hal_uart_puts("\r\n");
    }
}


/* ======================= UART GUI protocol ===========================
   Commands (newline-terminated):
     STATE,<n>[,<arg>] : set FSM state (0..9). Optional arg depends on state.
     H                 : help
     V                 : version banner
     S                 : sweep 0→180→0, stream "angle,cm" per step, then "FIN"
     A,<deg>           : set servo to <deg>, measure once, reply "DIST,<deg>,<cm|-1>"
     L                 : single LDR reading
   ==================================================================== */

extern enum FSMstate state;           // from main/app

bool api_try_take_angle(uint16_t* out)
{
    if (!out) return false;
    if (s_arg_valid) { *out = s_arg_deg; s_arg_valid = 0; return true; }
    return false;
}

static void _cmd_help(void)
{
    hal_uart_puts("STATE,<n>[,<arg>]  H  V  S  A,<deg>  L\r\n");
}

static void _process_line(const char* s)
{
    hal_uart_puts("DBG,line=");
    hal_uart_puts(s);
    hal_uart_puts("\r\n");

    /* trim leading spaces */
    while (*s==' ' || *s=='\t') ++s;

    /* STATE,<n>[,<arg>] */
    if ((s[0]=='S'||s[0]=='s') && (s[1]=='T'||s[1]=='t') &&
        (s[2]=='A'||s[2]=='a') && (s[3]=='T'||s[3]=='t') &&
        (s[4]=='E'||s[4]=='e') && s[5]==',')
    {
        const char* p = s + 6;
        unsigned int n = 0U;

        while (*p>='0' && *p<='9') { n = n*10U + (unsigned)(*p - '0'); ++p; }
        if (n > 9U) n = 0U;  // clamp 0..9

        if (*p==',') {
            ++p;
            unsigned int v = 0U, have = 0U;
            while (*p>='0' && *p<='9') { v = v*10U + (unsigned)(*p - '0'); ++p; have = 1U; }
            if (have) { s_arg_deg = (v > 180U) ? 180U : (uint16_t)v; s_arg_valid = 1; }
        }

        state = (enum FSMstate)n;           // hand control to main.c FSM
        hal_uart_puts("ACK,STATE,");
        hal_uart_putu((unsigned int)n);
        hal_uart_puts("\r\n");
        return;
    }

    /* Legacy single-letter helpers */
    if (s[0]=='S' && s[1]==0) {
        servo_ultra_sweep();
        hal_uart_puts("FIN\r\n");
        return;
    }

    if (s[0]=='A') {
        unsigned int deg = 0U;
        const char* p = s+1;
        if (*p==',') ++p;
        while (*p>='0' && *p<='9') { deg = deg*10U + (unsigned)(*p - '0'); ++p; }
        if (deg > 180U) deg = 180U;

        api_servo_set_angle((uint8_t)deg);
        hal_delay_ms(150u);

        unsigned int cm;
        if (api_ultra_get_cm((uint16_t*)&cm)) {
            hal_uart_puts("DIST,"); hal_uart_putu(deg); hal_uart_putc(','); hal_uart_putu(cm); hal_uart_puts("\r\n");
        } else {
            hal_uart_puts("DIST,"); hal_uart_putu(deg); hal_uart_puts(",-1\r\n");
        }
        return;
    }

    if (s[0]=='L' && s[1]==0) {
        api_ldr_step_print(3300u);  /* default Vcc = 3.3V */
        return;
    }

    if (s[0]=='V' && s[1]==0) { hal_uart_puts("VER,FinalProject,UART1\r\n"); return; }
    if (s[0]=='H' && s[1]==0) { _cmd_help(); return; }

    hal_uart_puts("ERR,unknown\r\n");
}

void api_uart_begin(void)
{
    hal_uart_enable_rx_irq(true);
    hal_uart_puts("READY\r\n");
}

void api_uart_poll(void)
{
    static char line[32];
    static unsigned int len = 0U;
    char c;

    while (hal_uart_getc(&c)) {
        if (c=='\r') continue;
        if (c=='\n') {
            line[len] = '\0';
            if (len) _process_line(line);
            len = 0U;
        } else {
            if (len < sizeof(line)-1U) line[len++] = c;
            else len = 0U; /* overflow → drop line */
        }
    }
}

/* ========= LCD File Browser / Viewer ========= */
#include <stddef.h>
static void lcd_print_padded(const char* s, uint8_t max_chars)
{
    uint8_t i = 0;
    for (; i < max_chars && s[i] && s[i] != '\r' && s[i] != '\n'; ++i) {
        lcd_putc(s[i]);
    }
    for (; i < max_chars; ++i) lcd_putc(' ');
}

/* Print directory name field into exactly 16 cols */
static void lcd_print_dir_name(const char name[], uint8_t name_len)
{
    uint8_t i = 0;
    for (; i < 16 && i < name_len; ++i) {
        char c = name[i];
        if (c == '\0') break;
        lcd_putc(c);
    }
    for (; i < 16; ++i) lcd_putc(' ');
}

/* Build a list of valid entry indices; returns count (0..FM_MAX_FILES) */
static uint8_t build_valid_index_list(uint8_t out_idx[FM_MAX_FILES])
{
    const fm_directory_t* dir = fm_get_directory();
    uint8_t n = 0;
    uint8_t i;
    for (i = 0; i < FM_MAX_FILES; ++i) {
        if (dir->entries[i].valid == 1 && dir->entries[i].size_bytes > 0) {
            out_idx[n++] = i;
        }
    }
    return n;
}

/* Render up to 32 display characters starting at *pos */
static void view_draw_page(const fm_dir_entry_t* e, uint16_t* pos_inout)
{
    const uint8_t* base = (const uint8_t*)(uintptr_t)(FILE_POOL_BASE + e->offset);
    const uint16_t size = e->size_bytes;
    uint16_t pos = *pos_inout;
    uint8_t printed = 0;
    uint8_t row = 0, col = 0;

    lcd_clear();
    lcd_goto(0,0);

    while (printed < 32 && pos < size) {
        char c = (char)base[pos++];
        if (c == '\r') continue;
        if (c == '\n') {
            if (row == 0) { row = 1; col = 0; lcd_goto(1,0); }
            else { break; }
            continue;
        }
        if ((unsigned char)c < 32 || (unsigned char)c > 126) c = ' ';
        lcd_putc(c);
        printed++;
        if (++col >= 16 && row == 0) { row = 1; col = 0; lcd_goto(1,0); }
    }

    /* Pad remaining cells with spaces */
    while (row < 2) {
        while (col < 16) { lcd_putc(' '); col++; }
        if (row == 0) { row = 1; col = 0; lcd_goto(1,0); } else break;
    }

    if (pos > *pos_inout) *pos_inout = pos;
}

static void lcd_view_entry(const fm_dir_entry_t* e)
{
    uint16_t pos = 0;
    view_draw_page(e, &pos);

    _file_show_exit_clear();                 /* reset any stale abort */

    for (;;) {
        _file_show_service_exit_uart();      /* sniff UART while waiting */
        if (_file_show_should_exit()) {      /* PC requested exit */
            hal_lcd_clear();
            /* Drain any leftover bytes so main() doesn't later echo a tail like "IT" */
            while (IFG2 & UCA0RXIFG) { (void)UCA0RXBUF; }
            return;
        }

        /* PB0 = next 32 displayed chars (wrap to start); PB1 = back to browser */
        if (pb0_pressed) {
            pb0_pressed = false;
            if (pos >= e->size_bytes) { pos = 0; }
            view_draw_page(e, &pos);
            _delay_with_exit_poll(120);
        }
        if (pb1_pressed) {
            pb1_pressed = false;
            _delay_with_exit_poll(120);
            break;
        }
    }
}


void api_file_show_lcd(const char* name)
{
    const fm_directory_t* dir = fm_get_directory();
    uint8_t i;
    for (i=0;i<FM_MAX_FILES;i++){
        const fm_dir_entry_t* e = &dir->entries[i];
        if (e->valid != 1) continue;
        /* Compare up to FM_MAX_NAME */
        uint8_t ok = 1;
        uint8_t k;
        for (k=0;k<FM_MAX_NAME;k++){
            char a = name[k];
            char b = e->name[k];
            if (a=='\0' && b=='\0') break;
            if (a=='\0' || b=='\0' || a!=b){ ok=0; break; }
        }
        if (ok) { lcd_view_entry(e); return; }
    }
    lcd_clear();
    lcd_goto(0,0); lcd_print_padded("NOT FOUND", 16);
    lcd_goto(1,0); lcd_print_padded(name, 16);
    _delay_with_exit_poll(1200);
}

/* Main entry: PB0 scrolls file names (wrap), PB1 selects; then PB0 pages 32 chars, PB1 returns */
void api_file_browse_lcd(void)
{
    uint8_t idxs[FM_MAX_FILES];
    uint8_t n = build_valid_index_list(idxs);
    uint8_t cur = 0;
    _file_show_exit_clear();
    if (n == 0) {
        lcd_clear();
        lcd_goto(0,0); lcd_print_padded("No files", 16);
        lcd_goto(1,0); lcd_print_padded("PB1 to exit", 16);
        while (!pb1_pressed) {
            _file_show_service_exit_uart();
            if (_file_show_should_exit()) {
                hal_lcd_clear();
                return;
            }
        }
        pb1_pressed = false;
        _delay_with_exit_poll(120);

        return;
    }


    for (;;) {
        _file_show_service_exit_uart();
        if (_file_show_should_exit()) {
            hal_lcd_clear();
            return;
        }

        const fm_directory_t* dir = fm_get_directory();
        const fm_dir_entry_t* e = &dir->entries[idxs[cur]];

        lcd_clear();
        lcd_goto(0,0);
        /* Row0: "Select X/Y" (X and Y are 1..n) */
        const char* S = "Select ";
        uint8_t i;
        for (i=0; S[i]; ++i) lcd_putc(S[i]);
        uint8_t a = (uint8_t)(cur + 1);
        if (a >= 10) { lcd_putc('0' + (a/10)); lcd_putc('0' + (a%10)); }
        else { lcd_putc('0' + a); }
        lcd_putc('/');
        a = n;
        if (a >= 10) { lcd_putc('0' + (a/10)); lcd_putc('0' + (a%10)); }
        else { lcd_putc('0' + a); }
        /* Pad remainder of row0 */
        for (i=8 + (n>=10) + (cur+1>=10) + 1; i<16; ++i) lcd_putc(' ');

        /* Row1: filename (truncated/padded to 16) */
        lcd_goto(1,0);
        lcd_print_dir_name(e->name, FM_MAX_NAME);

        /* Input handling */
        if (pb0_pressed) {
            pb0_pressed = false;
            cur = (uint8_t)((cur + 1) % n);
            hal_delay_ms(120);
        } else if (pb1_pressed) {
            pb1_pressed = false;
            hal_delay_ms(120);
            /* Enter viewing mode for current file; returns on PB1 */
            lcd_view_entry(e);
            /* After returning, re-show selection screen with same 'cur' */
        }
    }
}

/*** BEGIN: disable legacy FILE,PUT path (moved to main.c) ***/
#if 0

/* File PUT buffer */
static uint8_t  fm_rx_buf[FILE_POOL_SIZE];
static uint16_t fm_rx_expected = 0;
static uint16_t fm_rx_written  = 0;
static char     fm_rx_name[FM_MAX_NAME];
static uint8_t  fm_rx_type = 0;

void api_file_put_begin(const char* name, uint8_t type, uint16_t size)
{
    memset(fm_rx_buf, 0, sizeof(fm_rx_buf));
    memset(fm_rx_name, 0, sizeof(fm_rx_name));
    strncpy(fm_rx_name, name, FM_MAX_NAME - 1);
    fm_rx_type = type;
    fm_rx_expected = size;
    fm_rx_written = 0;
    uart_printf("FILE,PUT,BEGIN,OK\r\n");
}

void api_file_put_chunk(const uint8_t* data, uint16_t len)
{
    if (fm_rx_written + len > sizeof(fm_rx_buf)) {
        uart_printf("FILE,PUT,CHUNK,ERR,OVERFLOW\r\n");
        return;
    }
    memcpy(&fm_rx_buf[fm_rx_written], data, len);
    fm_rx_written = (uint16_t)(fm_rx_written + len);
    uart_printf("FILE,PUT,CHUNK,OK,%u\r\n", fm_rx_written);
}

void api_file_put_end(void)
{
    if (fm_rx_written != fm_rx_expected) {
        uart_printf("FILE,PUT,END,ERR,LEN\r\n");
        return;
    }
    fm_status_t st = fm_write_file(fm_rx_name, (fm_file_type_t)fm_rx_type, fm_rx_buf, fm_rx_written);
    if (st == FM_OK) uart_printf("FILE,PUT,END,OK\r\n");
    else uart_printf("FILE,PUT,END,ERR,%u\r\n", (unsigned)st);
}

/* Local: robust hex sender (no lookup table) */
static void _hex_send_bytes(const uint8_t* p, uint16_t n)
{
    for (uint16_t i = 0; i < n; ++i) {
        uint8_t v = p[i];
        uint8_t hi = (uint8_t)((v >> 4) & 0x0F);
        uint8_t lo = (uint8_t)(v & 0x0F);
        char c = (char)((hi < 10) ? ('0' + hi) : ('A' + (hi - 10)));
        hal_uart_putc(c);
        c = (char)((lo < 10) ? ('0' + lo) : ('A' + (lo - 10)));
        hal_uart_putc(c);
    }
}

void api_file_get(const char* name)
{
    /* Stream from FLASH as hex DATA lines; keep in sync with main.c handler */
    const fm_directory_t* D = fm_get_directory();
    if (!D) { hal_uart_puts("FILE,GET,ERR,DIR\r\n"); return; }

    const fm_dir_entry_t* e = NULL;
    for (uint8_t i = 0; i < FM_MAX_FILES; ++i) {
        const fm_dir_entry_t* cand = &D->entries[i];
        if (cand->valid && strncmp(cand->name, name, FM_MAX_NAME) == 0) { e = cand; break; }
    }
    if (!e) { hal_uart_puts("FILE,GET,ERR,NOT_FOUND\r\n"); return; }

    /* BEGIN includes name + size (GUI accepts this form) */
    hal_uart_puts("FILE,GET,BEGIN,"); hal_uart_puts(name); hal_uart_puts(",");
    hal_uart_putu(e->size_bytes); hal_uart_puts("\r\n");

    /* Emit <=64-byte chunks as hex lines */
    uint32_t pos = 0;
    while (pos < e->size_bytes) {
        uint16_t chunk = (uint16_t)((e->size_bytes - pos) > 64 ? 64 : (e->size_bytes - pos));
        const uint8_t* src = (const uint8_t*)(uintptr_t)(FILE_POOL_BASE + e->offset + pos);

        hal_uart_puts("FILE,GET,DATA,");
        _hex_send_bytes(src, chunk);
        hal_uart_puts("\r\n");

        pos += chunk;
    }

    hal_uart_puts("FILE,GET,END,OK\r\n");
}

void api_file_del(const char* name)
{
    fm_status_t st = fm_delete_file(name);
    if (st == FM_OK) uart_printf("FILE,DEL,OK\r\n");
    else uart_printf("FILE,DEL,ERR,%u\r\n", (unsigned)st);
}

void api_file_ls(void)
{
    char line[128];
    uint8_t cnt = fm_list_files(line, sizeof(line));
    uart_printf("FILE,LS,%u,%s\r\n", cnt, line);
}

/* and any functions that start with api_file_put_* and any
   command parsing for "FILE,PUT" in _process_line() */
#endif

