#ifndef HAL_H
#define HAL_H

#include <stdint.h>
#include <stdbool.h>
#include "../header/bsp.h"
#include "../header/app.h"

// ---- System ------------------
void     sysConfig(void);
void     hal_enable_interrupts(void);
void     hal_disable_interrupts(void);
void     hal_delay_ms(uint16_t ms);

// ---- Buttons -----------------
extern volatile bool pb0_pressed;
extern volatile bool pb1_pressed;

// ---- LCD ---------------------
void     hal_lcd_init(void);
void     hal_lcd_clear(void);
void     hal_lcd_home(void);
void     hal_lcd_goto(uint8_t row, uint8_t col);
void     hal_lcd_putc(char c);
void     hal_lcd_puts(const char* s);

// ---- Servo -------------------
void     hal_servo_enable(bool enable);
void     hal_servo_write_us(uint16_t pulse_us);
void     hal_servo_set_angle(uint8_t angle_deg);
uint16_t hal_servo_angle_to_us(uint8_t angle_deg);

// ---- Ultrasonic --------------
void     hal_ultra_init(void);
bool     hal_ultra_measure_us(uint16_t* us_out);
bool     hal_ultra_measure_cm(uint16_t* cm_out);
void     hal_ultra_init_polling(void);
bool     hal_ultra_measure_cm_polling(uint16_t* cm_out);

// ---- LDR ---------------------
void     hal_ldr_begin(void);               // configure ADC10 dual; returns LDR1 via reads
uint16_t hal_ldr_read_raw(void);            // 0..1023 (LDR1=A6)
uint16_t hal_ldr_read_mv(uint16_t vcc_mv);
uint16_t hal_ldr_read_avg(uint8_t samples); // average raw
void ldr_adc_init(void);
uint16_t ldr_read_A5(void);                 // LDR1 on P1.5
uint16_t ldr_read_A7(void);                 // LDR2 on P1.7
void ldrs_read_blocking(uint16_t out2[2]);  // both LDRs

// ---- UART --------------------
void hal_uart_enable_rx_irq(bool enable);
void hal_uart_putc(char c);
void hal_uart_puts(const char* s);
void hal_uart_putu(unsigned int v);         // decimal
bool hal_uart_getc(char *out_c);            // non-blocking; true if a byte was read


#endif
