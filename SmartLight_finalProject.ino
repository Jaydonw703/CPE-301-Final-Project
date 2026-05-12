/* Author: Jaydon Wilson
 * Ambient Light Controller
 * Arduino IDE: direct register manipulation, no Arduino libs except millis() from the runtime.
 
 *
 * Pin Map
 * -------
 * Photoresistor  : A0  (ADC0 / PF0) — voltage divider with 10k to GND
 * PWM LED        : PH3 (OC4A, Timer4, Arduino Mega pin 6)
 * State LEDs     : PA0–PA3 (Arduino Mega pins 22–25)
 *                  PA0=OFF  PA1=IDLE  PA2=ACTIVE  PA3=ERROR
 * START button   : PE4 (Arduino Mega digital pin 2) — polled, active HIGH
 * OFF   button   : PE5 (Arduino Mega digital pin 3) — polled, active HIGH
 * RESET button   : PG5 (Arduino Mega digital pin 4) — polled, active HIGH
 *
 * LCD HD44780 — 4-bit parallel, RW tied to GND, V0 tied to GND
 *   RS → PH4 (D7)   EN → PH5 (D8)
 *   D4 → PH6 (D9)   D5 → PB4 (D10)   D6 → PB5 (D11)   D7 → PB6 (D12)
 *   BL+ → +5V via 220Ω (pin 15)       BL– → GND (pin 16)
 *
 * DS3231 RTC — hardware I2C
 *   SDA → PD1 (pin 20)   SCL → PD0 (pin 21)
 *   4.7 kΩ pull-ups on both lines to 5V
 *
 * UART0 — 9600 baud, TX only (pin 1)
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <string.h>

// ─── Pin / Port Definitions ──────────────────────────────────────────────────

// State LEDs — Port A, pins 22–25
#define LED_DDR    DDRA
#define LED_PORT   PORTA
#define LED_OFF    (1 << PA0)
#define LED_IDLE   (1 << PA1)
#define LED_ACTIVE (1 << PA2)
#define LED_ERROR  (1 << PA3)
#define LED_MASK   (LED_OFF | LED_IDLE | LED_ACTIVE | LED_ERROR)

// Buttons — all active HIGH with 10k pull-downs
#define BTN_START_DDR  DDRE
#define BTN_START_PIN  PINE
#define BTN_START_BIT  PE4   // D2

#define BTN_OFF_DDR    DDRE
#define BTN_OFF_PIN    PINE
#define BTN_OFF_BIT    PE5   // D3

#define BTN_RST_DDR    DDRG
#define BTN_RST_PIN    PING
#define BTN_RST_BIT    PG5   // D4      

// PWM LED — OC4A on PH3 (D6)
#define PWM_DDR  DDRH
#define PWM_BIT  PH3

// LCD — Port H: RS(PH4/D7), EN(PH5/D8), D4(PH6/D9)
//       Port B: D5(PB4/D10), D6(PB5/D11), D7(PB6/D12)
#define LCD_RS_DDR   DDRE
#define LCD_RS_PORT  PORTE
#define LCD_RS_BIT   PE7

#define LCD_EN_DDR   DDRH
#define LCD_EN_PORT  PORTH
#define LCD_EN_BIT   PH6

#define LCD_D4_DDR   DDRH
#define LCD_D4_PORT  PORTH
#define LCD_D4_BIT   PH5

#define LCD_D5_DDR   DDRB
#define LCD_D5_PORT  PORTB
#define LCD_D5_BIT   PB4

#define LCD_D6_DDR   DDRB
#define LCD_D6_PORT  PORTB
#define LCD_D6_BIT   PB5

#define LCD_D7_DDR   DDRB
#define LCD_D7_PORT  PORTB
#define LCD_D7_BIT   PB6

// DS3231
#define DS3231_ADDR  0x68

// ─── FSM ─────────────────────────────────────────────────────────────────────
typedef enum { STATE_OFF, STATE_IDLE, STATE_ACTIVE, STATE_ERROR } SystemState;

// ─── Globals ─────────────────────────────────────────────────────────────────
volatile SystemState currentState   = STATE_OFF;
static   uint32_t   lastLogTime     = 0;
static   uint32_t   idleEnterTime   = 0; 
static   uint8_t    idleSettling    = 0;   // 1 while waiting for ADC to settle
#define  IDLE_SETTLE_MS  600               // ms to wait after entering IDLE

// ─── Forward Declarations ────────────────────────────────────────────────────
void     uart_init(void);
void     uart_send_char(char c);
void     uart_send_str(const char *s);
void     uart_send_uint(uint16_t v);

void     adc_init(void);
uint16_t adc_read(uint8_t ch);

void     timer4_pwm_init(void);
void     pwm_set_duty(uint8_t duty);

void     lcd_init(void);
void     lcd_cmd(uint8_t cmd);
void     lcd_data(uint8_t data);
void     lcd_send_nibble(uint8_t nibble);
void     lcd_set_cursor(uint8_t col, uint8_t row);
void     lcd_print(const char *s);
void     lcd_clear(void);
void     lcd_print_uint(uint16_t v);
static   void lcd_update(const char *stateName, uint16_t lux);

void     twi_init(void);
uint8_t  twi_start(uint8_t addr_rw);
void     twi_stop(void);
uint8_t  twi_write(uint8_t data);
uint8_t  twi_read_ack(void);
uint8_t  twi_read_nack(void);
uint8_t  bcd2dec(uint8_t bcd);
void     rtc_read_time(uint8_t *h, uint8_t *m, uint8_t *s);

void     set_state_led(SystemState st);
uint8_t  poll_button(volatile uint8_t *pin, uint8_t bit);
void     check_buttons(void);
void     enter_idle(void);
void     run_state_machine(void);
void     log_data(uint16_t lux, uint8_t h, uint8_t m, uint8_t s);

// ═══════════════════════════════════════════════════════════════════════════
// UART — 9600 8N1, TX only
// ═══════════════════════════════════════════════════════════════════════════
void uart_init(void) {
    uint16_t ubrr = (F_CPU / (16UL * 9600)) - 1;
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)(ubrr);
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  // 8N1
}
void uart_send_char(char c) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}
void uart_send_str(const char *s)  { while (*s) uart_send_char(*s++); }
void uart_send_uint(uint16_t v) {
    char buf[6]; uint8_t i = 0;
    if (!v) { uart_send_char('0'); return; }
    while (v) { buf[i++] = '0' + (v % 10); v /= 10; }
    while (i--) uart_send_char(buf[i]);
}

// ═══════════════════════════════════════════════════════════════════════════
// ADC — AVcc ref, ch 0 (A0 / PF0)
// ═══════════════════════════════════════════════════════════════════════════
void adc_init(void) {
    ADMUX  = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}
uint16_t adc_read(uint8_t ch) {
    ADMUX = (ADMUX & 0xF0) | (ch & 0x0F);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

// ═══════════════════════════════════════════════════════════════════════════
// Timer4 — Fast PWM 8-bit on OC4A (PH3 / D6)
// millis() is left entirely to the Arduino runtime (Timer0).
// ═══════════════════════════════════════════════════════════════════════════
void timer4_pwm_init(void) {
    PWM_DDR |= (1 << PWM_BIT);
    TCCR4A   = (1 << COM4A1) | (1 << WGM40);
    TCCR4B   = (1 << WGM42)  | (1 << CS41);
    OCR4A    = 0;   
}
void pwm_set_duty(uint8_t duty) { OCR4A = duty; }

// ═══════════════════════════════════════════════════════════════════════════
// TWI (I2C) 
// ═══════════════════════════════════════════════════════════════════════════
#define TWI_TWBR_VAL ((F_CPU / 100000UL - 16) / 2)
void twi_init(void) {
    TWSR = 0x00;
    TWBR = (uint8_t)TWI_TWBR_VAL;
    TWCR = (1 << TWEN);
}
static void twi_wait(void) { while (!(TWCR & (1 << TWINT))); }
uint8_t twi_start(uint8_t addr_rw) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); twi_wait();
    TWDR = addr_rw;
    TWCR = (1 << TWINT) | (1 << TWEN);                twi_wait();
    return (TWSR & 0xF8);
}
void    twi_stop(void)         { TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN); while(TWCR&(1<<TWSTO)); }
uint8_t twi_write(uint8_t d)   { TWDR=d; TWCR=(1<<TWINT)|(1<<TWEN); twi_wait(); return(TWSR&0xF8); }
uint8_t twi_read_ack(void)     { TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWEA); twi_wait(); return TWDR; }
uint8_t twi_read_nack(void)    { TWCR=(1<<TWINT)|(1<<TWEN);           twi_wait(); return TWDR; }
uint8_t bcd2dec(uint8_t b)     { return ((b>>4)*10)+(b&0x0F); }
void rtc_read_time(uint8_t *h, uint8_t *m, uint8_t *s) {
    twi_start((DS3231_ADDR<<1)|0); twi_write(0x00);
    twi_start((DS3231_ADDR<<1)|1);
    *s = bcd2dec(twi_read_ack()  & 0x7F);
    *m = bcd2dec(twi_read_ack()  & 0x7F);
    *h = bcd2dec(twi_read_nack() & 0x3F);
    twi_stop();
}

// ═══════════════════════════════════════════════════════════════════════════
// LCD HD44780 — 4-bit, pins on Port H and Port B
// ═══════════════════════════════════════════════════════════════════════════
static inline void lcd_pin(volatile uint8_t *port, uint8_t bit, uint8_t v) {
    if (v) *port |= (1<<bit); else *port &= ~(1<<bit);
}
static void lcd_pulse_en(void) {
    LCD_EN_PORT |=  (1 << LCD_EN_BIT); _delay_us(5);
    LCD_EN_PORT &= ~(1 << LCD_EN_BIT); _delay_us(200);
}
void lcd_send_nibble(uint8_t n) {
    lcd_pin(&LCD_D4_PORT, LCD_D4_BIT, n & 0x01);
    lcd_pin(&LCD_D5_PORT, LCD_D5_BIT, n & 0x02);
    lcd_pin(&LCD_D6_PORT, LCD_D6_BIT, n & 0x04);
    lcd_pin(&LCD_D7_PORT, LCD_D7_BIT, n & 0x08);
    lcd_pulse_en();
}
void lcd_cmd(uint8_t cmd) {
    LCD_RS_PORT &= ~(1 << LCD_RS_BIT);
    lcd_send_nibble(cmd >> 4);
    lcd_send_nibble(cmd & 0x0F);
    if (cmd < 4) _delay_ms(2); else _delay_us(50);
}
void lcd_data(uint8_t d) {
    LCD_RS_PORT |= (1 << LCD_RS_BIT);
    lcd_send_nibble(d >> 4);
    lcd_send_nibble(d & 0x0F);
    _delay_us(50);
}
void lcd_init(void) {
    TCCR4A &= ~(1 << COM4A1);   
    // Set all LCD pins as outputs and drive low
    LCD_RS_DDR |= (1<<LCD_RS_BIT); LCD_RS_PORT &= ~(1<<LCD_RS_BIT);
    LCD_EN_DDR |= (1<<LCD_EN_BIT); LCD_EN_PORT &= ~(1<<LCD_EN_BIT);
    LCD_D4_DDR |= (1<<LCD_D4_BIT); LCD_D4_PORT &= ~(1<<LCD_D4_BIT);
    LCD_D5_DDR |= (1<<LCD_D5_BIT); LCD_D5_PORT &= ~(1<<LCD_D5_BIT);
    LCD_D6_DDR |= (1<<LCD_D6_BIT); LCD_D6_PORT &= ~(1<<LCD_D6_BIT);
    LCD_D7_DDR |= (1<<LCD_D7_BIT); LCD_D7_PORT &= ~(1<<LCD_D7_BIT);

    _delay_ms(50);  // wait for LCD power-on

    // HD44780 3-step 8-bit init then switch to 4-bit
    lcd_send_nibble(0x03); _delay_ms(5);
    lcd_send_nibble(0x03); _delay_ms(1);
    lcd_send_nibble(0x03); _delay_ms(1);
    lcd_send_nibble(0x02); _delay_ms(1);  // 4-bit mode

    lcd_cmd(0x28);  // 2 lines, 5x8
    lcd_cmd(0x0C);  // display on, cursor off
    lcd_cmd(0x06);  // auto-increment, no shift
    lcd_cmd(0x01);  // clear
    _delay_ms(2);

    _delay_ms(10);
}
void lcd_clear(void)                { lcd_cmd(0x01); _delay_ms(2); }
void lcd_set_cursor(uint8_t c, uint8_t r) {
    lcd_cmd(0x80 | (c + (r ? 0x40 : 0x00)));
}
void lcd_print(const char *s)       { while (*s) lcd_data(*s++); }
void lcd_print_uint(uint16_t v) {
    char buf[6]; uint8_t i = 0;
    if (!v) { lcd_data('0'); return; }
    while (v) { buf[i++] = '0' + (v%10); v /= 10; }
    while (i--) lcd_data(buf[i]);
}
static void lcd_update(const char *name, uint16_t lux) {
    lcd_set_cursor(0, 0); lcd_print("State:          ");
    lcd_set_cursor(7, 0); lcd_print(name);
    lcd_set_cursor(0, 1); lcd_print("Light:          ");
    lcd_set_cursor(7, 1); lcd_print_uint(lux);
}

// ═══════════════════════════════════════════════════════════════════════════
// State LED
// ═══════════════════════════════════════════════════════════════════════════
void set_state_led(SystemState st) {
    LED_PORT &= ~LED_MASK;
    switch (st) {
        case STATE_OFF:    LED_PORT |= LED_OFF;    break;
        case STATE_IDLE:   LED_PORT |= LED_IDLE;   break;
        case STATE_ACTIVE: LED_PORT |= LED_ACTIVE; break;
        case STATE_ERROR:  LED_PORT |= LED_ERROR;  break;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Button helper — debounce + wait for release, returns 1 on confirmed press
// ═══════════════════════════════════════════════════════════════════════════
uint8_t poll_button(volatile uint8_t *pin, uint8_t bit) {
    if (*pin & (1 << bit)) {
        _delay_ms(50);
        if (*pin & (1 << bit)) {
            while (*pin & (1 << bit));   // wait for release
            return 1;
        }
    }
    return 0;
}

// ═══════════════════════════════════════════════════════════════════════════
// enter_idle() — centralised helper so every path into IDLE is identical
// ═══════════════════════════════════════════════════════════════════════════
void enter_idle(void) {
    currentState  = STATE_IDLE;
    idleSettling  = 1;
    idleEnterTime = millis();
    lcd_update("IDLE  ", 0);
    uart_send_str("-> IDLE (settling)\r\n");
}

// ═══════════════════════════════════════════════════════════════════════════
// Button polling (called every loop tick)
// ═══════════════════════════════════════════════════════════════════════════
void check_buttons(void) {
    // START: OFF -> IDLE only
    if (currentState == STATE_OFF) {
        if (poll_button(&BTN_START_PIN, BTN_START_BIT)) {
    _delay_ms(50);
    if (poll_button(&BTN_START_PIN, BTN_START_BIT)) {
        enter_idle();
    }
}
        return;  // in STATE_OFF no other buttons matter
    }

    // OFF: any state -> STATE_OFF
    if (poll_button(&BTN_OFF_PIN, BTN_OFF_BIT)) {
        pwm_set_duty(0);
        currentState = STATE_OFF;
        uart_send_str("-> OFF\r\n");
        return;
    }

    // RESET: ERROR -> IDLE only
    if (currentState == STATE_ERROR) {
        if (poll_button(&BTN_RST_PIN, BTN_RST_BIT)) {
            enter_idle();
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Logging
// ═══════════════════════════════════════════════════════════════════════════
void log_data(uint16_t lux, uint8_t h, uint8_t m, uint8_t s) {
    const char *st;
    switch (currentState) {
        case STATE_OFF:    st = "OFF";    break;
        case STATE_IDLE:   st = "IDLE";   break;
        case STATE_ACTIVE: st = "ACTIVE"; break;
        default:           st = "ERROR";  break;
    }
    uart_send_str("[");
    uart_send_uint(h); uart_send_char(':');
    uart_send_uint(m); uart_send_char(':');
    uart_send_uint(s);
    uart_send_str("] State="); uart_send_str(st);
    uart_send_str(" Light=");  uart_send_uint(lux);
    uart_send_str("\r\n");
}

// ═══════════════════════════════════════════════════════════════════════════
// Main FSM
// ═══════════════════════════════════════════════════════════════════════════
void run_state_machine(void) {
    static SystemState prevState = STATE_OFF;
    uint16_t lux = 0;
    uint8_t  h = 0, m = 0, s = 0;

    check_buttons();

    switch (currentState) {

        // ── OFF ────────────────────────────────────────────────────────
        case STATE_OFF:
            if (prevState != STATE_OFF) {
                pwm_set_duty(0);
                lcd_update("OFF   ", 0);
            }
            break;

        // ── IDLE ───────────────────────────────────────────────────────
        case STATE_IDLE:
            // Non-blocking settle: wait IDLE_SETTLE_MS after entering IDLE
            // before making any threshold decision. Uses millis() so the
            // loop keeps running (buttons still polled) during the wait.
            if (idleSettling) {
                if ((millis() - idleEnterTime) < IDLE_SETTLE_MS) break;
                idleSettling = 0;
                uart_send_str("IDLE settled\r\n");
            }

            lux = adc_read(0);

            if (prevState != STATE_IDLE) {
                lcd_update("IDLE  ", lux);
            }

            if (lux < 512) {
                currentState = STATE_ACTIVE;
            }

            if (millis() - lastLogTime >= 60000UL) {
                rtc_read_time(&h, &m, &s);
                log_data(lux, h, m, s);
                lcd_update("IDLE  ", lux);
                lastLogTime = millis();
            }
            break;

        // ── ACTIVE ─────────────────────────────────────────────────────
        case STATE_ACTIVE:
            lux = adc_read(0);

            // Scale: lux 0 (dark) -> duty 255, lux 1023 (bright) -> duty 0
            pwm_set_duty((uint8_t)(255 - (lux >> 2)));

            if (prevState != STATE_ACTIVE) {
                lcd_update("ACTIVE", lux);
            } else {
                lcd_set_cursor(7, 1); lcd_print("     ");
                lcd_set_cursor(7, 1); lcd_print_uint(lux);
            }

            if (lux >= 512) {
                pwm_set_duty(0);
                // Return to IDLE without re-settling (sensor already warm)
                currentState = STATE_IDLE;
                idleSettling  = 0;
            }

            if (millis() - lastLogTime >= 60000UL) {
                rtc_read_time(&h, &m, &s);
                log_data(lux, h, m, s);
                lastLogTime = millis();
            }
            break;

        // ── ERROR ──────────────────────────────────────────────────────
        case STATE_ERROR:
            pwm_set_duty(0);
            if (prevState != STATE_ERROR) {
                lcd_clear();
                lcd_set_cursor(0, 0); lcd_print("  SENSOR ERROR  ");
                lcd_set_cursor(0, 1); lcd_print("Press RST button");
                uart_send_str("[ERROR] Fault detected\r\n");
            }
            break;
    }

    if (currentState != prevState) {
        set_state_led(currentState);
        prevState = currentState;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// setup()
// ═══════════════════════════════════════════════════════════════════════════
void setup(void) {
    // GPIO
    LED_DDR       |= LED_MASK;
    LED_PORT      &= ~LED_MASK;
    BTN_START_DDR &= ~(1 << BTN_START_BIT);
    BTN_OFF_DDR   &= ~(1 << BTN_OFF_BIT);
    BTN_RST_DDR   &= ~(1 << BTN_RST_BIT);
    _delay_ms(200); 
    // Peripherals
    uart_init();
    adc_init();
    timer4_pwm_init();
    twi_init();

    sei();  // enable interrupts BEFORE lcd_init() and any millis() use

    lcd_init(); 

    // Initial state
    currentState = STATE_OFF;
    set_state_led(STATE_OFF);
    lcd_clear();
    lcd_set_cursor(0, 0); lcd_print("Light Controller");
    lcd_set_cursor(0, 1); lcd_print("Press START btn ");
    uart_send_str("System initialized\r\n");

    lastLogTime = millis();
}

// ═══════════════════════════════════════════════════════════════════════════
// loop()
// ═══════════════════════════════════════════════════════════════════════════
void loop(void) {
    run_state_machine();
    _delay_ms(20);
