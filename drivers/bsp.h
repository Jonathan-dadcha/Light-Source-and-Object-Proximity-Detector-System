#ifndef BSP_H
#define BSP_H

#include <msp430g2553.h>
#include <stdint.h>
#include <stdbool.h>

// ---- System ------------------
#define F_CPU_HZ 1000000UL

void    bsp_init(void);
void    clock_init_1mhz(void);
void    stop_all_timers(void);

// ---- UART --------------------
void    uart_init_9600(void);

// ---- LDR ---------------------
void    adc_init_ldr_dual(void);

// ---- Timers ------------------
void    timerA0_start_upmode(uint16_t period_ticks);
void    timerA1_start_upmode(uint16_t period_ticks);

// ---- Servo -------------------
#define SERVO_PWM_PORT_SEL      P2SEL
#define SERVO_PWM_PORT_DIR      P2DIR
#define SERVO_PWM_PIN           BIT1    // P2.1 -> TA1.1
#define SERVO_PWM_PERIOD_TICKS  20000U  // 20ms @1MHz
#define SERVO_PWM_MIN_US        380U    // 0°
#define SERVO_PWM_MAX_US        1900U   // 180°

void    servo_pwm_init(void);
void    servo_pwm_enable(bool enable);
void    servo_pwm_write_us(uint16_t pulse_us);

// ---- Ultrasonic --------------
#define US_TRIG_PORT_OUT        P2OUT
#define US_TRIG_PORT_DIR        P2DIR
#define US_TRIG_PORT_SEL        P2SEL
#define US_TRIG_PIN             BIT3    // P2.3 as GPIO

#define US_ECHO_PORT_DIR        P2DIR
#define US_ECHO_PORT_SEL        P2SEL
#define US_ECHO_PIN             BIT2    // P2.2 -> TA1 CCR1 input CCI1B

#define US_CAP_CCTL             TA1CCTL1
#define US_CAP_CCR              TA1CCR1

void    ultrasonic_pins_init(void);
void    ultrasonic_capture_init(void);
void    ultrasonic_trigger_pulse_10us(void);

// ---- LDR ---------------------
#define LDR1_PIN          BIT5        // P1.5 -> A5  (LDR1)
#define LDR2_PIN          BIT7        // P1.7 -> A7  (LDR2)
#define LDR1_ADC_CHANNEL  INCH_5
#define LDR2_ADC_CHANNEL  INCH_7

void    ldrs_adc_init_dual(void);
void    ldrs_adc_read_blocking(uint16_t out2[2]); // out2[0]=A5 (LDR1), out2[1]=A7 (LDR2)

// ---- File mode flash layout --
#define FILE_POOL_BASE  0xF000u
#define FILE_POOL_SIZE  2048u
#define DIR_INFO_BASE   0x1000u

// ---- LCD ---------------------
// Wiring: D4..D7 -> P2.4..P2.7, RS -> P1.3, RW -> P1.4, EN -> P1.0
#define LCD_RS_DIR    P1DIR
#define LCD_RS_OUT    P1OUT
#define LCD_RS_BIT    BIT3

#define LCD_RW_DIR    P1DIR
#define LCD_RW_OUT    P1OUT
#define LCD_RW_BIT    BIT4

#define LCD_EN_DIR    P1DIR
#define LCD_EN_OUT    P1OUT
#define LCD_EN_BIT    BIT0

#define LCD_D_DIR     P2DIR
#define LCD_D_OUT     P2OUT
#define LCD_D_SEL     P2SEL
#define LCD_D_SEL2    P2SEL2
#define LCD_D4_BIT    BIT4
#define LCD_D5_BIT    BIT5
#define LCD_D6_BIT    BIT6
#define LCD_D7_BIT    BIT7
#define LCD_D_MASK   (LCD_D4_BIT|LCD_D5_BIT|LCD_D6_BIT|LCD_D7_BIT)

void lcd_init_4bit(void);
void lcd_write_cmd(uint8_t cmd);
void lcd_putc(char c);
void lcd_puts(const char* s);
void lcd_clear(void);
void lcd_home(void);
void lcd_goto(uint8_t row, uint8_t col);

// ---- Buttons ----------------
// PB0 on P2.0
#define PB0_PORT_DIR   P2DIR
#define PB0_PORT_OUT   P2OUT
#define PB0_PORT_REN   P2REN
#define PB0_PORT_SEL   P2SEL
#define PB0_PORT_IES   P2IES
#define PB0_PORT_IE    P2IE
#define PB0_PORT_IFG   P2IFG
#define PB0_PIN        BIT0

// PB1 on P1.6
#define PB1_PORT_DIR   P1DIR
#define PB1_PORT_OUT   P1OUT
#define PB1_PORT_REN   P1REN
#define PB1_PORT_SEL   P1SEL
#define PB1_PORT_IES   P1IES
#define PB1_PORT_IE    P1IE
#define PB1_PORT_IFG   P1IFG
#define PB1_PIN        BIT6

void pb0_init_pullup_irq(void);
void pb1_init_pullup_irq(void);


#endif /* BSP_H */
