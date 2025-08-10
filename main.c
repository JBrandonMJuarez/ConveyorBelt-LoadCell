/*
  - LCD I2C (PCF8574)
  - HX711: DOUT=RD0, CLK=RD1
  - Servo: señal en RD2 (por temporización)
  - Motor PWM: CCP1 RC2
  - Botones: RB4(sensor), RB5(start), RB6(stop), RB7(calibrar)
  - EEPROM interna para guardar 'escala'
*/

#define _XTAL_FREQ 20000000UL

#include <xc.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>   // <-- necesario para sprintf

// CONFIG
#pragma config FOSC = HS       // Oscilador HS externo  (20 MHz)
#pragma config WDT = OFF
#pragma config LVP = OFF
#pragma config PBADEN = OFF
#pragma config MCLRE = ON

// ---------- Definiciones hardware ----------
/* I2C (PCF8574) usando MSSP hardware (SDA=RC4, SCL=RC3 en PIC18F4550 hardware) */
#define I2C_HW               1
#define PCF8574_ADDR         0x40  // 7-bit address (común para muchos módulos LCD)

// NOTA: en el código original las macros I2C_SDA_PIN etc estaban apuntando a RB0/RB1.
// Al usar MSSP hardware, el hardware usa RC3/RC4. No modifiqué el resto del programa
// salvo las funciones i2c_* — el resto del código mantiene sus macros originales.

/* HX711 pins */
#define HX711_DOUT      PORTDbits.RD0
#define HX711_SCK_LAT   LATDbits.LATD1
#define HX711_SCK_TRIS  TRISDbits.TRISD1
#define HX711_DOUT_TRIS TRISDbits.TRISD0

/* Servo pin (signal) */
#define SERVO_LAT LATDbits.LATD2
#define SERVO_TRIS TRISDbits.TRISD2

/* Motor PWM - CCP1 on RC2 */
#define MOTOR_PWM_TRIS TRISCbits.TRISC2

/* Motor enable pin mapped from Arduino pin7 -> RD3 (salida) */
#define MOTOR_EN_LAT LATDbits.LATD3
#define MOTOR_EN_TRIS TRISDbits.TRISD3

/* Botones (a partir de RB4) */
#define BTN_SENSOR PORTBbits.RB4  // sensor (equiv. digitalRead(1))
#define BTN_START  PORTBbits.RB5  // start (equiv. pin 2)
#define BTN_STOP   PORTBbits.RB6  // stop  (equiv. pin 3)
#define BTN_CALIB  PORTBbits.RB7  // calibrar (equiv. pin 4)

/* Flags (mantengo nombres semejantes a Arduino) */
volatile uint8_t detecta = 0;
volatile uint8_t limpia = 0;
volatile uint8_t arranca = 0;
volatile uint8_t presionado = 0;
volatile uint8_t presionado2 = 0;

/* Balanza (HX711) */
long escala = 1;   // se guarda en EEPROM
float peso = 0.0f;

/* -- Prototipos -- */
void setup_hw(void);
void loop_main(void);

/* I2C (MSSP hardware) */
void I2C_Init(void);
//void i2c_start(void);
//void i2c_stop(void);
//uint8_t i2c_write_byte(uint8_t b);
//uint8_t i2c_read_byte(uint8_t ack);

//void pcf_write(uint8_t data);

/* LCD 16x2 sobre PCF8574 (4-bit) */
void LCD_Init(void);
//void lcd_send_nibble(uint8_t nibble, uint8_t rs);
//void lcd_send_byte(uint8_t b, uint8_t rs);
void lcd_Cmd(uint8_t b);
//void lcd_putc(char c);
//void lcd_print(const char *s);
void lcd_clear(void);
void lcd_SetCursor(uint8_t col, uint8_t row);

/* HX711 */
long hx711_read_raw(void);
long hx711_get_value(uint8_t times);
void hx711_tare(uint8_t times);
float hx711_get_units(uint8_t times);

/* EEPROM (leer/escribir long en direcciones 0..3) */
void eeprom_write_long(uint16_t addr, long val);
long eeprom_read_long(uint16_t addr);

/* Servo (temporización) */
void servo_write_blocking(uint8_t angle); // genera pulses ~1s para mover/asegurar posicion

/* Calibración y pesaje */
void Calibrar(void);
void Pesa(void);

/* Interrupción RB change */
volatile uint8_t prevRB; // para detectar flancos en RB4..RB7
void __interrupt() isr(void);

/* PWM motor */
void pwm_init_ccp1(void);
void pwm_set_duty_percent(uint8_t duty);

/* UTIL */
static inline void delay_us(unsigned int us) { while(us--) __delay_us(1); }

/* ----------------- IMPLEMENTACIÓN ----------------- */

void setup_hw(void) {
    /* Pines */
    OSCCON = 0; // external HS 20MHz per pragma
    // Configure TRIS:
    TRISB = 0xFF;
    TRISD = 0xFF;
    TRISC = 0xFF;

    // HX711 pins:
    HX711_DOUT_TRIS = 1;         // DOUT input
    HX711_SCK_TRIS = 0;         // SCK output
    HX711_SCK_LAT = 0;

    // Servo pin:
    SERVO_TRIS = 0;
    SERVO_LAT = 0;

    // Motor enable pin as output (RD3)
    MOTOR_EN_TRIS = 0;
    MOTOR_EN_LAT = 0; // off

    // Botones: RB4..RB7 inputs
    TRISBbits.TRISB4 = 1;
    TRISBbits.TRISB5 = 1;
    TRISBbits.TRISB6 = 1;
    TRISBbits.TRISB7 = 1;

    // pull-ups on PORTB
    INTCON2bits.RBPU = 0; // enable PORTB pull-ups (if hardware supports)

    // PWM CCP1 init
    pwm_init_ccp1();

    // init I2C (hardware MSSP)
    I2C_Init();
    LCD_Init();
    
    
    // load escala from EEPROM
    escala = eeprom_read_long(0);

    /* Attach interrupts for RB change (RB4..RB7) */
    prevRB = PORTB & 0xF0; // guardar estado alto 4 bits
    INTCONbits.RBIF = 0;
    INTCONbits.RBIE = 1;
    INTCONbits.GIE = 1;
}

// ================= I2C ==================
void I2C_Init(void) {
    SSPCON1 = 0x28;
    SSPCON2 = 0x00;
    SSPADD = (_XTAL_FREQ/(4*100000))-1;
    SSPSTAT = 0;
    TRISB0 = 1; // SDA
    TRISB1 = 1; // SCL
}

void I2C_Wait(void) {
    while ((SSPCON2 & 0x1F) || (SSPSTAT & 0x04));
}

void I2C_Start(void) {
    I2C_Wait();
    SEN = 1;
}

void I2C_Stop(void) {
    I2C_Wait();
    PEN = 1;
}

void I2C_Write(uint8_t data) {
    I2C_Wait();
    SSPBUF = data;
}

// ================= LCD I2C ==================
#define LCD_ADDR 0x40

void LCD_Cmd(uint8_t cmd) {
    uint8_t hi = cmd & 0xF0;
    uint8_t lo = (cmd << 4) & 0xF0;
    I2C_Start();
    I2C_Write(LCD_ADDR);
    I2C_Write(hi | 0x0C);
    I2C_Write(hi | 0x08);
    I2C_Write(lo | 0x0C);
    I2C_Write(lo | 0x08);
    I2C_Stop();
    __delay_ms(2);
}

void lcd_clear(void) { LCD_Cmd(0x01); __delay_ms(2); }

void LCD_Char(char data) {
    uint8_t hi = data & 0xF0;
    uint8_t lo = (data << 4) & 0xF0;
    I2C_Start();
    I2C_Write(LCD_ADDR);
    I2C_Write(hi | 0x0D);
    I2C_Write(hi | 0x09);
    I2C_Write(lo | 0x0D);
    I2C_Write(lo | 0x09);
    I2C_Stop();
    __delay_us(50);
}

void LCD_Init() {
    __delay_ms(50);
    LCD_Cmd(0x33);
    LCD_Cmd(0x32);
    LCD_Cmd(0x28);
    LCD_Cmd(0x0C);
    LCD_Cmd(0x06);
    LCD_Cmd(0x01);
    __delay_ms(2);
}

void LCD_Text(const char* txt) {
    while(*txt) LCD_Char(*txt++);
}

void LCD_SetCursor(uint8_t col, uint8_t row) {
    uint8_t addr = (row == 0) ? 0x00 : 0x40;
    addr += col;
    LCD_Cmd(0x80 | addr);
}

/* ---------- HX711 (simple implementation) ---------- */
// Definiciones necesarias
#define HX711_DOUT      PORTDbits.RD0  // Pin de datos del HX711 (entrada)
#define HX711_SCK_LAT   LATDbits.LATD1 // Pin de reloj del HX711 (salida)
#define HX711_SCK_TRIS  TRISDbits.TRISD1
#define HX711_DOUT_TRIS TRISDbits.TRISD0

void hx711_init(void) {
    HX711_SCK_TRIS = 0; // SCK como salida
    HX711_DOUT_TRIS = 1; // DOUT como entrada
    HX711_SCK_LAT = 0; // Nivel bajo inicial
}

long hx711_read_raw(void) {
    unsigned long count = 0;
    unsigned int timeout = 0;

    // esperar a que DOUT baje (dato listo)
    //while (HX711_DOUT) {
    //    __delay_us(10);
    //    if (++timeout > 50000) return 0; // timeout
    //}

    // leer 24 bits
    for (uint8_t i = 0; i < 24; i++) {
        HX711_SCK_LAT = 1;
        __delay_us(1);
        count = (count << 1);
        if (HX711_DOUT) count++;
        HX711_SCK_LAT = 0;
        __delay_us(1);
    }

    // pulso extra para seleccionar ganancia 128 (canal A)
    HX711_SCK_LAT = 1; __delay_us(1);
    HX711_SCK_LAT = 0; __delay_us(1);

    // signo: si es negativo, extender el bit
    if (count & 0x800000) count |= 0xFF000000;

    return (long)count;
}

long hx711_get_value(uint8_t times) {
    long sum = 0;
    for (uint8_t i = 0; i < times; i++) {
        sum += hx711_read_raw();
    }
    return sum / times;
}

void hx711_tare(uint8_t times) {
    long val = hx711_get_value(times);
    (void) val;
}

float hx711_get_units(uint8_t times) {
    long value = hx711_get_value(times);
    if (escala == 0) return 0.0f;
    float units = (float)value / (float)escala;
    return units;
}

/* ---------- EEPROM (write/read long at addr) ---------- */
void eeprom_write_long(uint16_t addr, long val) {
    uint8_t *p = (uint8_t*)&val;
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t a = (uint8_t)(addr + i);
        while (EECON1bits.WR);
        EEADR = a;
        EEDATA = p[i];
        EECON1bits.EEPGD = 0;
        EECON1bits.CFGS = 0;
        EECON1bits.WREN = 1;
        INTCONbits.GIE = 0;
        EECON2 = 0x55;
        EECON2 = 0xAA;
        EECON1bits.WR = 1;
        INTCONbits.GIE = 1;
        EECON1bits.WREN = 0;
    }
}

long eeprom_read_long(uint16_t addr) {
    long val = 0;
    uint8_t *p = (uint8_t*)&val;
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t a = (uint8_t)(addr + i);
        EEADR = a;
        EECON1bits.EEPGD = 0;
        EECON1bits.CFGS = 0;
        EECON1bits.RD = 1;
        p[i] = EEDATA;
    }
    return val;
}

/* ---------- Servo (blocking pulses) ---------- */
void servo_write_blocking(uint8_t angle) {
    uint16_t pulse = 1000 + ((uint32_t)angle * 1000UL) / 180UL; // pulso en us: 1000..2000

    for (uint8_t i = 0; i < 200; i++) {
        SERVO_LAT = 1;
        delay_us(pulse);
        SERVO_LAT = 0;

        if (pulse < 20000) {
            uint16_t rest = 20000 - pulse;
            unsigned int rest_ms = rest / 1000;
            uint16_t rest_us = rest % 1000;

            while (rest_ms--) __delay_ms(1);
            if (rest_us) delay_us(rest_us);
        } else {
            __delay_ms(20);
        }
    }
}

/* ---------- Calibrar & Pesa (manteniendo lógica) ---------- */
void Calibrar(void) {
    uint8_t conf = 1;
    long adc_lecture;
    lcd_clear();
    LCD_SetCursor(0,0); LCD_Text("Calibrando base");
    LCD_SetCursor(4,1); LCD_Text("Balanza");
    __delay_ms(3000);
    hx711_read_raw();
    hx711_tare(20);
    lcd_clear();
    while (conf) {
        LCD_SetCursor(1,0); LCD_Text("Peso referencial:");
        LCD_SetCursor(1,1);
        char buf[20];
        sprintf(buf, "%ld g        ", (long)250); // <-- restaurado para evitar imprimir basura
        LCD_Text(buf);
        __delay_ms(3000);
        lcd_clear();
        LCD_SetCursor(1,0); LCD_Text("Ponga el Peso");
        LCD_SetCursor(1,1); LCD_Text("Referencial");
        __delay_ms(3000);
        adc_lecture = hx711_get_value(100);
        if (250 != 0) escala = adc_lecture / 250;
        if (escala == 0) escala = 1;
        eeprom_write_long(0, escala);
        __delay_ms(100);
        LCD_SetCursor(1,0); LCD_Text("Retire el Peso");
        LCD_SetCursor(1,1); LCD_Text("referencial");
        __delay_ms(3000);
        lcd_clear();
        LCD_SetCursor(1,0); LCD_Text("READY!!....");
        __delay_ms(3000);
        lcd_clear();
        conf = 0;
        lcd_clear();
    }
}

void Pesa(void) {
    peso = 0.0f;
    float referencia = 200.0f;
    __delay_ms(5);
    hx711_tare(10);
    while ((peso < referencia) && arranca) {
        peso = hx711_get_units(10);
        LCD_SetCursor(0,1);
        char buf[32];
        sprintf(buf, "Peso: %.0f g        ", peso);
        LCD_Text(buf);
        __delay_ms(5);
    }
}

/* ---------- PWM motor (CCP1) ---------- */
void pwm_init_ccp1(void) {
    // CCP1 on RC2
    TRISCbits.TRISC2 = 0;
    PR2 = 249; // period -> adjust as needed
    T2CON = 0x04; // TMR2 On, prescaler 1
    CCP1CON = 0x0C; // PWM mode
    CCPR1L = 0;
    PIR1bits.TMR2IF = 0;
}

void pwm_set_duty_percent(uint8_t duty) {
    if (duty > 100) duty = 100;
    uint16_t pwm_max = (PR2 + 1) * 4 - 1;
    uint16_t pwm_val = ((uint32_t)duty * pwm_max) / 100;
    CCPR1L = (pwm_val >> 2) & 0xFF;
    CCP1CONbits.DC1B = pwm_val & 0x03;
}

/* ---------- INTERRUPCIÓN RB change (arranque/paro) ---------- */
void __interrupt() isr(void) {
    if (INTCONbits.RBIF) {
        uint8_t cur = PORTB & 0xF0;
        uint8_t changed = prevRB ^ cur;
        // RB5 start falling edge
        if ((changed & (1<<5))) {
            if ((prevRB & (1<<5)) && !(cur & (1<<5))) {
                arranca = 1;
            }
        }
        // RB6 stop falling edge
        if ((changed & (1<<6))) {
            if ((prevRB & (1<<6)) && !(cur & (1<<6))) {
                arranca = 0;
            }
        }
        prevRB = cur;
        INTCONbits.RBIF = 0;
    }
}

/* ---------- main loop (estructura convertida del sketch) ---------- */
int main(void) {
    hx711_init();
    setup_hw();

    // Emular setup(): servo initial position 0
    servo_write_blocking(45);

    // motor enable low and motor off
    MOTOR_EN_LAT = 0;
    pwm_set_duty_percent(0);
    for (uint8_t i = 0; i < 200; i++) {
        SERVO_LAT = 1;
        __delay_us(1);
        SERVO_LAT = 0;
	__delay_ms(19);
       
    }
    __delay_ms(500);
    for (uint8_t i = 0; i < 200; i++) {
        SERVO_LAT = 1;
        __delay_ms(2);
        SERVO_LAT = 0;
	__delay_ms(18);
       
    }
    //while(1){
    //  hx711_get_value(2);
    //}
    // check calibrar button at startup (RB7)
    if (BTN_CALIB) {
        Calibrar();
    }
    if (escala <= 0) escala = 1;
    hx711_tare(20);

    while (1) {
        MOTOR_EN_LAT = 0;
        pwm_set_duty_percent(0);

        LCD_SetCursor(0,0);
        LCD_Text("PRESIONE START");
        LCD_SetCursor(0,1);
        LCD_Text("PARA INICIAR     ");

        while (!arranca) { /* idle */ }

        if (!limpia) {
            lcd_clear();
            limpia = 1;
        }

        while (arranca) {
            LCD_SetCursor(0,0);
            LCD_Text("EN PROCESO       ");
            MOTOR_EN_LAT = 0;
            pwm_set_duty_percent(100);

            if (!BTN_SENSOR) {
                detecta = 1;
            }

            if (detecta) {
                lcd_clear();
                MOTOR_EN_LAT = 0;
                pwm_set_duty_percent(0);
                LCD_SetCursor(0,0);
                LCD_Text("LLENANDO         ");
                __delay_ms(1000);
                servo_write_blocking(90);
                Pesa();
                __delay_ms(1000);
                servo_write_blocking(60);
                __delay_ms(1000);
                detecta = 0;
                MOTOR_EN_LAT = 0;
                pwm_set_duty_percent(100);
                while (!BTN_SENSOR && arranca) { }
                lcd_clear();
            }
        }
        limpia = 0;
    }
    return 0;
}

