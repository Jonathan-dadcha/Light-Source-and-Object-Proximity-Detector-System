#ifndef _api_H_
#define _api_H_

#include <stdint.h>
#include <stdbool.h>

/* System bring-up (implemented in HAL; exposed here so main includes only API+APP) */
void     sysConfig(void);

/* Optional legacy wrapper (safe even if unused) */
void     api_init(void);

/* -------- Servo (high-level) ---------- */
void     api_servo_begin(void);                  /* (re)map pins & start PWM safely */
void     api_servo_set_angle(uint8_t deg);       /* 0..180° */
void     api_servo_write_us(uint16_t pulse_us);  /* raw pulse (µs), e.g. 1000..2000 */
void     servo_slow_sweep(void);                 /* servo-only demo sweep */

/* -------- Ultrasonic (polling) -------- */
bool     api_ultra_get_us(uint16_t* us_out);     /* echo time (µs) */
bool     api_ultra_get_cm(uint16_t* cm_out);     /* distance (cm) */

/* -------- LDRs ------------------------ */
void     api_ldr_begin(void);                    /* prepares ADC10 (already in sysConfig) */
uint16_t api_ldr1_read_raw(void);                /* LDR1=A6 raw 0..1023 */
void     api_ldr_step_print(uint16_t vcc_mv);    /* CSV "LDR,<raw>,<mV>" demo */

// Convenience: print one reading (CSV "LDR,<raw>,<mV>")
void     api_ldr_step_print(uint16_t vcc_mv);
void     api_ldr_separate_test(uint16_t mv);

/* LDR calibration: collect 10 samples per sensor (5..50cm, step 5cm), store in FLASH */
void api_ldr_calibrate_run(uint16_t vcc_mv);

/* Optional helper to load the stored calibration tables later */
bool api_ldr_calib_load(uint16_t out_ldr1[10], uint16_t out_ldr2[10]);

/* --- LDR calibration: load from FLASH --- */
bool api_ldr_calib_get(uint16_t out_ldr1[10], uint16_t out_ldr2[10]);

/* (optional) Print the table over UART as CSV:
   CAL,TBL,<dist_cm>,<ldr1>,<ldr2> ... then CAL,DUMP,OK */
void api_ldr_calib_dump_uart(void);

/* -------- Combined demo --------
   Performs a full sweep (0→180→0). At each step:
     - move servo,
     - wait settle,
     - take ultrasonic measurement(s),
     - print CSV line: "angle,cm" (or "angle,-1" on no echo).
   Returns when one full sweep is done. */
void     servo_ultra_sweep(void);
/* Sweep 0→180 and stream both LDRs per angle. CSV:
   LDRS,<deg>,<ldr1_raw>,<ldr1_mV>,<ldr2_raw>,<ldr2_mV> */
void     servo_ldr_sweep(uint16_t vcc_mv);
/* Combined sweep: ultrasonic + both LDRs per angle.
   Format: COMBO,<deg>,<dist_cm>,<ldr1_raw>,<ldr1_mv>,<ldr2_raw>,<ldr2_mv> */
void     servo_combo_sweep(uint16_t vcc_mv);


/* ---- UART GUI protocol (call from main loop) ----
   Commands (newline-terminated):
     - "S"           : 0→180→0 sweep; stream "angle,cm" lines; end with "FIN"
     - "A,<deg>"     : set servo to <deg>, measure; reply "DIST,<deg>,<cm|-1>"
     - "L"           : one LDR reading; reply "LDR,<raw>,<mV>"
     - "V"           : version string
     - "H"           : help
*/
void api_uart_begin(void);   /* enables RX IRQ, prints "READY" banner */
void api_uart_poll(void);    /* parse & execute commands (non-blocking) */

// Pull a one-shot angle argument pushed by the GUI (if any).
// Returns true and writes *deg_out when an arg was present (then clears it).
bool api_try_take_angle(uint16_t* deg_out);

/* File mode API for CLI or GUI commands */
void api_file_put_begin(const char* name, uint8_t type, uint16_t size);
void api_file_put_chunk(const uint8_t* data, uint16_t len);
void api_file_put_end(void);

void api_file_get(const char* name);
void api_file_del(const char* name);
void api_file_ls(void);

/* -------- LCD file browser / viewer -------- */
/* File selection: PB0 = next file (wrap), PB1 = select.
   Viewing: PB0 = next 32 chars, PB1 = back to selection. */
void api_file_browse_lcd(void);

/* Optional: show a specific file immediately (starts in viewing mode).
   Name must be a C-string; directory names aren’t guaranteed NUL-terminated. */
void api_file_show_lcd(const char* name);

/* Request LCD FILE,SHOW viewer to exit (can be called from main.c parser) */
void api_file_show_request_exit(void);


#endif /* _api_H_ */

