#include "../header/hal.h"
#include "../header/bsp.h"
#include <stdint.h>

// ---------------- System ----------------
void sysConfig(void)
{
    bsp_init();                 // 1MHz, UART 9600, ADC, servo PWM default

    hal_lcd_init();
    hal_lcd_clear();
    hal_lcd_goto(0,0);
    hal_lcd_puts("READY");

    hal_servo_set_angle(90);    // center servo on boot
    // Do NOT enable GIE here
}

void hal_enable_interrupts(void)  { __bis_SR_register(GIE);  }
void hal_disable_interrupts(void) { __bic_SR_register(GIE);  }
void hal_delay_ms(uint16_t ms) { while (ms--) __delay_cycles(1000); }
void hal_delay_us(uint16_t us) { while (us--) __delay_cycles(1); }

// ---- Buttons ----------------
volatile bool pb0_pressed;
volatile bool pb1_pressed;

// ===== LCD (HAL wrappers over BSP) =====
void hal_lcd_init(void)                 { lcd_init_4bit(); }
void hal_lcd_clear(void)                { lcd_clear(); }
void hal_lcd_home(void)                 { lcd_home(); }
void hal_lcd_goto(uint8_t row, uint8_t col) { lcd_goto(row, col); }
void hal_lcd_putc(char c)               { lcd_putc(c); }
void hal_lcd_puts(const char* s)        { lcd_puts(s); }

// ---------------- Servo wrappers ----------------
uint16_t hal_servo_angle_to_us(uint8_t angle_deg)
{
    uint32_t span, us;
    const uint16_t min_us = SERVO_PWM_MIN_US;
    const uint16_t max_us = SERVO_PWM_MAX_US;
    if (angle_deg > 180) angle_deg = 180;
    span = (uint32_t)(max_us - min_us);
    us   = min_us + (span * angle_deg) / 180u;
    return (uint16_t)us;
}
void hal_servo_enable(bool enable)            { servo_pwm_enable(enable); }
void hal_servo_write_us(uint16_t pulse_us)    { servo_pwm_write_us(pulse_us); }
void hal_servo_set_angle(uint8_t angle_deg)   { hal_servo_write_us(hal_servo_angle_to_us(angle_deg)); }

// =====================================================================
//                  ULTRASONIC
// =====================================================================
static volatile uint16_t u_rise;
static volatile uint16_t u_fall;
static volatile uint16_t u_width;
static volatile uint8_t  u_wait_fall;
static volatile uint8_t  u_done;

void hal_ultra_init(void)
{
    hal_servo_enable(false);
    ultrasonic_pins_init();
    ultrasonic_capture_init();

    u_rise = u_fall = u_width = 0;
    u_wait_fall = 0;
    u_done = 0;
}

bool hal_ultra_measure_us(uint16_t* us_out)
{
    uint32_t i;
    if (!us_out) return false;

    u_done = 0;
    u_wait_fall = 0;

    ultrasonic_trigger_pulse_10us();

    // wait ~35 ms max (≈ 6 m round-trip)
    for (i = 0UL; i < 35000UL; i++) {
        if (u_done) { *us_out = u_width; return true; }
        __delay_cycles(1);
    }
    return false; // timeout
}

bool hal_ultra_measure_cm(uint16_t* cm_out)
{
    uint16_t us;
    if (!cm_out) return false;
    if (!hal_ultra_measure_us(&us)) return false;
    *cm_out = (uint16_t)((us + 29U) / 58U);  // rounded us/58
    return true;
}

// Capture ISR
#pragma vector = TIMER1_A1_VECTOR
__interrupt void Timer1_A1_ISR(void)
{
    switch (__even_in_range(TA1IV, TA1IV_TAIFG)) {
    case TA1IV_TACCR1: {
        uint16_t cap = TA1CCR1;
        if (!u_wait_fall) {
            u_rise = cap;         // rising edge
            u_wait_fall = 1;
        } else {
            u_fall  = cap;        // falling edge
            u_width = (uint16_t)(u_fall - u_rise);  // wrap-safe
            u_wait_fall = 0;
            u_done = 1;
        }
        break;
    }
    case TA1IV_TAIFG:
        break;
    default:
        break;
    }
}

// =====================================================================
//                  ULTRASONIC
// =====================================================================
// --- Ultrasonic Polling Mode ---
void hal_ultra_init_polling(void)
{
    hal_servo_enable(true);    // keep servo PWM enabled

    // Trigger pin (P2.3) as output
    US_TRIG_PORT_DIR |= US_TRIG_PIN;
    US_TRIG_PORT_OUT &= ~US_TRIG_PIN;

    // Echo pin (P2.2) as GPIO input
    US_ECHO_PORT_SEL &= ~US_ECHO_PIN;
    US_ECHO_PORT_DIR &= ~US_ECHO_PIN;

    // Timer1_A free-running @ 1 MHz
    TA1CTL = TASSEL_2 | MC_2 | TACLR;   // SMCLK, continuous, clear
}

// Returns raw echo width in microseconds using GPIO polling + TA1R
bool hal_ultra_measure_us_polling(uint16_t* us_out)
{
    uint32_t timeout;
    uint16_t t_start, t_end;

    if (!us_out) return false;

    // Send 10 Âµs trigger pulse
    ultrasonic_trigger_pulse_10us();

    // Wait for ECHO to go HIGH (rising edge), with timeout (~30 ms)
    for (timeout = 0UL; timeout < 30000UL; ++timeout) {
        if (P2IN & US_ECHO_PIN) break;
        __delay_cycles(1);
    }
    if (!(P2IN & US_ECHO_PIN)) return false;   // no rising edge detected

    t_start = TA1R;                             // latch start time

    // Wait for ECHO to go LOW (falling edge), with timeout (~30 ms)
    for (timeout = 0UL; timeout < 30000UL; ++timeout) {
        if (!(P2IN & US_ECHO_PIN)) break;
        __delay_cycles(1);
    }
    if (P2IN & US_ECHO_PIN) return false;      // still high â†’ timeout

    t_end = TA1R;                               // latch end time

    *us_out = (uint16_t)(t_end - t_start);      // handles wrap-around
    return true;
}

bool hal_ultra_measure_cm_polling(uint16_t* cm_out)
{
    uint16_t us;
    if (!cm_out) return false;
    if (!hal_ultra_measure_us_polling(&us)) return false;
    *cm_out = (uint16_t)((us + 29U) / 58U);     // â‰ˆ us/58, rounded
    return true;
}

/* ========= LDR (public = LDR1/A6 only) ========= */
void hal_ldr_begin(void)
{
    ldrs_adc_init_dual();
}

uint16_t hal_ldr_read_raw(void)
{
    uint16_t v[2];
    ldrs_adc_read_blocking(v);  /* v[0]=A6 (LDR1), v[1]=A7 (LDR2) */
    return v[0];
}

uint16_t hal_ldr_read_mv(uint16_t vcc_mv)
{
    uint16_t raw = hal_ldr_read_raw();
    uint32_t t   = (uint32_t)raw * vcc_mv + 511u;
    return (uint16_t)(t / 1023u);
}

uint16_t hal_ldr_read_avg(uint8_t samples)
{
    uint32_t acc = 0;
    uint8_t  n   = (samples ? samples : 1);
    uint8_t  i;

    for (i = 0; i < n; ++i) {
        acc += hal_ldr_read_raw();
        __delay_cycles(500); /* ~0.5 ms */
    }
    return (uint16_t)(acc / n);
}

// Simple init for both LDR channels (A5, A7)
void ldr_adc_init(void)
{
    // Configure pins as analog inputs
    P1DIR &= ~(BIT5 | BIT7);      // inputs
    ADC10AE0 |=  (BIT5 | BIT7);   // enable analog on A5, A7

    ADC10CTL0 &= ~ENC;            // disable before config

    // ADC on, 64-cycle S/H, V+ = Vcc, V- = GND
    ADC10CTL0 = ADC10SHT_3 | ADC10ON;
    // clock = SMCLK/4, single-channel (set per read)
    ADC10CTL1 = ADC10SSEL_3 | ADC10DIV_3;
}

// ---- Single-channel readers ----
uint16_t ldr_read_A5(void)
{
    uint16_t val;

    ADC10CTL0 &= ~ENC;
    ADC10CTL1 = (INCH_5 | ADC10SSEL_3 | ADC10DIV_3 | CONSEQ_0); // A5, single
    ADC10SA   = (uint16_t)&val;
    ADC10DTC1 = 1;

    ADC10CTL0 |= ENC | ADC10SC;
    while (ADC10CTL1 & ADC10BUSY) { }  // wait until done

    return val;
}

uint16_t ldr_read_A7(void)
{
    uint16_t val;

    ADC10CTL0 &= ~ENC;
    ADC10CTL1 = (INCH_7 | ADC10SSEL_3 | ADC10DIV_3 | CONSEQ_0); // A7, single
    ADC10SA   = (uint16_t)&val;
    ADC10DTC1 = 1;

    ADC10CTL0 |= ENC | ADC10SC;
    while (ADC10CTL1 & ADC10BUSY) { }

    return val;
}

void ldrs_read_blocking(uint16_t out2[2])
{
    ldrs_adc_read_blocking(out2);
}

// =================== UART (RX via ISR + ring buffer) ===================
#define UART_RX_BUF_SZ 16
static volatile unsigned char s_rx_buf[UART_RX_BUF_SZ];
static volatile unsigned char s_rx_head;
static volatile unsigned char s_rx_tail;
static inline unsigned char _rb_next(unsigned char i){ return (unsigned char)((i + 1U) % UART_RX_BUF_SZ); }

void hal_uart_enable_rx_irq(bool enable) {
    if (enable) IE2 |= UCA0RXIE;
    else        IE2 &= ~UCA0RXIE;
}

// Blocking TX helpers
void hal_uart_putc(char c) {
    while (!(IFG2 & UCA0TXIFG)) { /* wait */ }
    UCA0TXBUF = (unsigned char)c;
}
void hal_uart_puts(const char* s) {
    while (*s) hal_uart_putc(*s++);
}
void hal_uart_putu(unsigned int v) {
    char b[6], t[6]; unsigned int n=0U, k=0U, i;
    if (v == 0U) { hal_uart_putc('0'); return; }
    while (v) { t[k++] = (char)('0' + (v % 10U)); v /= 10U; }
    while (k) { b[n++] = t[--k]; }
    for (i=0U; i<n; ++i) hal_uart_putc(b[i]);
}

// Non-blocking single byte fetch from RX ring
bool hal_uart_getc(char *out_c) {
    if (s_rx_head == s_rx_tail) return false;
    *out_c = (char)s_rx_buf[s_rx_tail];
    s_rx_tail = _rb_next(s_rx_tail);
    return true;
}

// ---------------- Interrupt Service Routines ----------------
#pragma vector=PORT2_VECTOR
__interrupt void Port_2_ISR(void)
{
    if (P2IFG & BIT0) {
        P2IFG &= ~BIT0;     // clear flag

        pb0_pressed = true; // latch event

        // --- debounce: disable PB0 interrupt temporarily ---
        P2IE &= ~BIT0;
        __delay_cycles(20000);   // ~20ms at 1MHz
        P2IFG &= ~BIT0;          // clear again just in case
        P2IE |= BIT0;            // re-enable
    }
}

#pragma vector=PORT1_VECTOR
__interrupt void Port_1_ISR(void)
{
    if (P1IFG & BIT6) {   // PB1 (P1.6)
        P1IFG &= ~BIT6;
        pb1_pressed = true;
    }
}

// USCI_A0 RX ISR (MSP430G2553 uses USCIAB0RX_VECTOR for A0/B0 RX)
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void) {
    if (IFG2 & UCA0RXIFG) {
        unsigned char c = UCA0RXBUF;
        unsigned char next = _rb_next(s_rx_head);
        if (next != s_rx_tail) {        // store if not full
            s_rx_buf[s_rx_head] = c;
            s_rx_head = next;
        } else {
            volatile unsigned char drop = c; (void)drop; // drop on overflow
        }
    }
}



