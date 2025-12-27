#include "../header/bsp.h"

// ---------------- System ----------------
void clock_init_1mhz(void) {
    WDTCTL = WDTPW | WDTHOLD;                 // Stop WDT
    if (CALBC1_1MHZ != 0xFF) {
        BCSCTL1 = CALBC1_1MHZ;
        DCOCTL  = CALDCO_1MHZ;
    } else {
        // Fallback (approx ~1MHz)
        BCSCTL1 = XT2OFF | RSEL2;
        DCOCTL  = DCO1 | MOD1;
    }
}

void bsp_init(void) {
    clock_init_1mhz();

    // Buttons
    pb0_init_pullup_irq();
    pb1_init_pullup_irq();

    // Peripherals
    uart_init_9600();
    adc_init_ldr_dual();
    servo_pwm_init();
    ultrasonic_pins_init();

    // Leave GIE to app
}

void stop_all_timers(void) {
    TA0CTL = MC_0;
    TA1CTL = MC_0;
}

// ---------------- UART ----------------
/*
void uart_init_9600(void) {
    // P1.1=RXD, P1.2=TXD
    P1SEL  |= BIT1 | BIT2;
    P1SEL2 |= BIT1 | BIT2;

    UCA0CTL1 |= UCSWRST;            // Hold USCI in reset
    UCA0CTL1 |= UCSSEL_2;           // SMCLK
    UCA0BR0   = 104;                // 1MHz / 9600
    UCA0BR1   = 0;
    UCA0MCTL  = UCBRS0;             // Modulation (TI table)
    UCA0CTL1 &= ~UCSWRST;           // Release
}
*/
void uart_init_9600(void)
{
    UCA0CTL1 |= UCSWRST;                 /* hold in reset */
    UCA0CTL1 = UCSWRST | UCSSEL_2;       /* SMCLK */

    /* 1 MHz -> 9600 baud: BR=104, BRS=UCBRS0 */
    UCA0BR0  = 104;
    UCA0BR1  = 0x00;
    UCA0MCTL = UCBRS0;

    /* pins: P1.1=RXD, P1.2=TXD */
    P1SEL  |= BIT1 | BIT2;
    P1SEL2 |= BIT1 | BIT2;

    UCA0CTL1 &= ~UCSWRST;                /* release reset */

    __delay_cycles(3000);                /* ~3ms settle at 1MHz */

    /* prime a couple bytes so the very first real print is clean */
    while (!(IFG2 & UCA0TXIFG)) ;
    UCA0TXBUF = 0x00;
    while (!(IFG2 & UCA0TXIFG)) ;
    UCA0TXBUF = 'U';
    __delay_cycles(1000);
}

// ---------------- ADC (LDR) ----------------
void adc_init_ldr_dual(void) {
    // Highest channel = A7, sequence mode, clock source = SMCLK
    ADC10CTL1 = INCH_7 | CONSEQ_1 | ADC10SSEL_3;

    // Long sample-and-hold (good for LDRs), multiple conversions, ADC on
    ADC10CTL0 = ADC10SHT_3 | MSC | ADC10ON;

    // Enable analog function on P1.5 (A5) and P1.7 (A7)
    ADC10AE0 |= BIT5 | BIT7;

    // DTC will store 2 results (A5 then A7) per trigger
    ADC10DTC1 = 2;
}

// ---------------- Timer helpers ----------------
void timerA0_start_upmode(uint16_t period_ticks) {
    TA0CCR0  = period_ticks - 1;
    TA0CCTL0 = CCIE;
    TA0CTL   = TASSEL_2 | ID_0 | MC_1 | TACLR;
}

void timerA1_start_upmode(uint16_t period_ticks) {
    TA1CCR0  = period_ticks - 1;
    TA1CCTL0 = CCIE;
    TA1CTL   = TASSEL_2 | ID_0 | MC_1 | TACLR;
}

// ---------------- Servo (TA1.1@P2.1) ----------------
void servo_pwm_init(void) {
    P2SEL2 &= ~SERVO_PWM_PIN;              // <-- make sure alt-2 is off
    SERVO_PWM_PORT_SEL |=  SERVO_PWM_PIN;  // TA1.1 function on P2.1
    SERVO_PWM_PORT_DIR |=  SERVO_PWM_PIN;

    TA1CCR0  = SERVO_PWM_PERIOD_TICKS - 1; // 20 ms period @ 1 MHz
    TA1CCTL1 = OUTMOD_7;                   // reset/set
    TA1CCR1  = SERVO_PWM_MIN_US;           // safe start pulse
    TA1CTL   = TASSEL_2 | MC_1 | TACLR;    // SMCLK, up mode
}

void servo_pwm_enable(bool enable) {
    if (enable) TA1CCTL1 = OUTMOD_7;
    else        TA1CCTL1 = 0;
}

void servo_pwm_write_us(uint16_t pulse_us) {
    if (pulse_us < SERVO_PWM_MIN_US) pulse_us = SERVO_PWM_MIN_US;
    if (pulse_us > SERVO_PWM_MAX_US) pulse_us = SERVO_PWM_MAX_US;
    TA1CCR1 = pulse_us; // 1 tick = 1 µs @1MHz
}

// ---------------- Ultrasonic ----------------
void ultrasonic_pins_init(void) {
    // TRIG = P2.3 GPIO output low
    US_TRIG_PORT_SEL &= ~US_TRIG_PIN;
    P2SEL2           &= ~US_TRIG_PIN;  // ensure SEL2 off
    US_TRIG_PORT_DIR |=  US_TRIG_PIN;
    US_TRIG_PORT_OUT &= ~US_TRIG_PIN;

    // ECHO = P2.2 to TA1 capture input
    US_ECHO_PORT_DIR &= ~US_ECHO_PIN;
    US_ECHO_PORT_SEL |=  US_ECHO_PIN;  // route to TA1
    P2SEL2           &= ~US_ECHO_PIN;  // ensure SEL2 off
}

// ECHO on P2.2 => CCI1B, so CCIS_1. If you wire ECHO to P2.1, change CCIS_1 -> CCIS_0.
void ultrasonic_capture_init(void) {
    TA1CTL      = TASSEL_2 | MC_2 | TACLR;        // SMCLK, continuous
    US_CAP_CCTL = 0;                               // reset
    US_CAP_CCTL = CM_3 | CCIS_1 | SCS | CAP | CCIE;// both edges, CCI1B, IRQ
    TA1CTL     &= ~TAIFG;                          // clear overflow flag
    US_CAP_CCTL &= ~CCIFG;                         // clear capture flag
}

// 10 µs trigger pulse
void ultrasonic_trigger_pulse_10us(void) {
    US_TRIG_PORT_OUT |=  US_TRIG_PIN;
    __delay_cycles(10);                            // ≈10 µs @1MHz
    US_TRIG_PORT_OUT &= ~US_TRIG_PIN;
}

void ldrs_adc_init_dual(void)
{
    // Enable analog on P1.5 (A5) and P1.7 (A7)
    P1DIR    &= ~(LDR1_PIN | LDR2_PIN);
    ADC10AE0 |=  (LDR1_PIN | LDR2_PIN);

    ADC10CTL0 &= ~ENC;

    // Highest channel = A7, sequence-of-channels, SMCLK, /4 divider
    ADC10CTL1  = LDR2_ADC_CHANNEL | CONSEQ_1 | ADC10SSEL_3 | ADC10DIV_3; // INCH_7

    // 64-cycle sample, multiple sample & convert, ADC on
    ADC10CTL0  = ADC10SHT_3 | MSC | ADC10ON;

    // We need A7, A6, A5 (3 samples). We'll ignore A6 later.
    ADC10DTC1  = 3;
}

void ldrs_adc_read_blocking(uint16_t out2[2])
{
    static uint16_t dtc_buf[3];        // dtc_buf[0]=A7, [1]=A6, [2]=A5

    ADC10CTL0 &= ~ENC;
    ADC10CTL1  = LDR2_ADC_CHANNEL | CONSEQ_1 | ADC10SSEL_3 | ADC10DIV_3; // start @ A7
    ADC10DTC1  = 3;
    ADC10SA    = (uint16_t)dtc_buf;

    ADC10CTL0 |= ENC | ADC10SC;        // start sequence
    while (ADC10CTL1 & ADC10BUSY) { }  // wait until done

    // Map to API order: out2[0]=A5 (LDR1), out2[1]=A7 (LDR2)
    out2[0] = dtc_buf[2];  // A5
    out2[1] = dtc_buf[0]  ;// A7

    ADC10CTL0 &= ~ENC;
}

/* ======================= LCD 4-bit driver ======================= */
static inline void _lcd_delay_us(uint16_t us) {
    while (us--) __delay_cycles(1); // ~1us @1MHz
}
static inline void _lcd_delay_ms(uint16_t ms) {
    while (ms--) __delay_cycles(1000);
}
static inline void _lcd_en_pulse(void) {
    LCD_EN_OUT |=  LCD_EN_BIT;
    _lcd_delay_us(1);
    LCD_EN_OUT &= ~LCD_EN_BIT;
    _lcd_delay_us(40);
}
static inline void _lcd_put_nibble(uint8_t nib4) {
    LCD_D_OUT = (LCD_D_OUT & ~LCD_D_MASK) | ((nib4 & 0x0F) << 4);
    _lcd_en_pulse();
}
static void _lcd_write(uint8_t byte, bool rs) {
    if (rs) LCD_RS_OUT |=  LCD_RS_BIT; else LCD_RS_OUT &= ~LCD_RS_BIT;
    LCD_RW_OUT &= ~LCD_RW_BIT; // write mode
    _lcd_put_nibble(byte >> 4);
    _lcd_put_nibble(byte & 0x0F);
    if (byte == 0x01 || byte == 0x02) _lcd_delay_ms(2); else _lcd_delay_us(50);
}

void lcd_write_cmd(uint8_t cmd) { _lcd_write(cmd, false); }
void lcd_putc(char c)           { _lcd_write((uint8_t)c, true); }

void lcd_puts(const char* s) { while (*s) lcd_putc(*s++); }
void lcd_clear(void)         { lcd_write_cmd(0x01); }
void lcd_home(void)          { lcd_write_cmd(0x02); }

void lcd_goto(uint8_t row, uint8_t col) {
    if (row > 1) row = 1;
    if (col > 15) col = 15;
    uint8_t addr = (row == 0 ? 0x00 : 0x40) + col;
    lcd_write_cmd(0x80 | addr);
}

void lcd_init_4bit(void) {
    // Configure pins as GPIO outputs
    P1SEL  &= ~(LCD_EN_BIT);
    P1SEL2 &= ~(LCD_EN_BIT);

    LCD_RS_DIR |= LCD_RS_BIT;
    LCD_RW_DIR |= LCD_RW_BIT;
    LCD_EN_DIR |= LCD_EN_BIT;

    LCD_D_SEL  &= ~LCD_D_MASK;
    LCD_D_SEL2 &= ~LCD_D_MASK;
    LCD_D_DIR  |=  LCD_D_MASK;

    LCD_RS_OUT &= ~LCD_RS_BIT;
    LCD_RW_OUT &= ~LCD_RW_BIT; // we only write
    LCD_EN_OUT &= ~LCD_EN_BIT;
    LCD_D_OUT  &= ~LCD_D_MASK;

    _lcd_delay_ms(40);          // power-on wait

    // Init sequence to 4-bit mode
    _lcd_put_nibble(0x03); _lcd_delay_ms(5);
    _lcd_put_nibble(0x03); _lcd_delay_us(150);
    _lcd_put_nibble(0x03); _lcd_delay_us(150);
    _lcd_put_nibble(0x02); _lcd_delay_us(150); // 4-bit

    lcd_write_cmd(0x28); // Function set: 4-bit, 2 lines, 5x8
    lcd_write_cmd(0x08); // Display OFF
    lcd_clear();         // Clear
    lcd_write_cmd(0x06); // Entry mode: increment, no shift
    lcd_write_cmd(0x0C); // Display ON, cursor OFF, blink OFF
}


// ---------------- Buttons ----------------
static void pb_init_common(volatile unsigned char* dir,
                           volatile unsigned char* out,
                           volatile unsigned char* ren,
                           volatile unsigned char* sel,
                           volatile unsigned char* ies,
                           volatile unsigned char* ie,
                           volatile unsigned char* ifg,
                           uint8_t pin)
{
    *sel &= ~pin;         // GPIO
    *dir &= ~pin;         // input
    *ren |=  pin;         // pull enable
    *out |=  pin;         // pull-up
    *ies |=  pin;         // falling edge
    *ifg &= ~pin;         // clear pending
    *ie  |=  pin;         // enable port IRQ
}

void pb0_init_pullup_irq(void) { pb_init_common(&PB0_PORT_DIR, &PB0_PORT_OUT, &PB0_PORT_REN,
                                                &PB0_PORT_SEL, &PB0_PORT_IES, &PB0_PORT_IE,
                                                &PB0_PORT_IFG, PB0_PIN); }

void pb1_init_pullup_irq(void) { pb_init_common(&PB1_PORT_DIR, &PB1_PORT_OUT, &PB1_PORT_REN,
                                                &PB1_PORT_SEL, &PB1_PORT_IES, &PB1_PORT_IE,
                                                &PB1_PORT_IFG, PB1_PIN); }


