// PIC18F4550 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1L
#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2 // System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator (HS))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = ON         // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting 2.05V)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = ON      // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)

#include <xc.h>
#include <stdio.h>

#define _XTAL_FREQ 8000000L
#define RANGE 15                 


//================================================================================

//================================================================================
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00
#define LCD_TYPE 2 // 4-bit, 2 d?ng
#define LCD_TURN_ON 0x0C
#define LCD_CLEAR 0x01
#define LCD_RETURN_HOME 0x02
#define LCD_ENTRY_MODE_SET 0x04
// ??nh ngh?a ch?n I2C ph?n m?m
#define I2C_SCL PORTBbits.RB1
#define I2C_SDA PORTBbits.RB0
#define I2C_SCL_DIR TRISBbits.RB1
#define I2C_SDA_DIR TRISBbits.RB0

// Khai b?o bi?n to?n c?c
unsigned char RS, i2c_add = 0x27; 
unsigned char BackLight_State = LCD_BACKLIGHT;
unsigned int SETPOINT = 0;
unsigned int count = 0;
unsigned char humidity1;
int mode;
typedef struct {
    int set_time;
    int time_delay;
    int SETPOINT;
    int mode;
} mode_value;

// Prototype cho c?c h?m
void I2C_Init(void);
void I2C_Wait(void);
void I2C_Start(void);
void I2C_Stop(void);
unsigned char I2C_Write(unsigned char data);
void I2C_SDA_High(void);
void I2C_SDA_Low(void);
void I2C_SCL_High(void);
void I2C_SCL_Low(void);
unsigned char I2C_Read_ACK(void);
void LCD_Init(void);
void LCD_CMD(unsigned char CMD);
void LCD_Write_Char(char Data);
void LCD_Write_String(char* Str);
void LCD_Set_Cursor(unsigned char ROW, unsigned char COL);
void LCD_Clear(void);
mode_value select_mode(void);
volatile unsigned long millis_counter = 0;
unsigned long millis() {
    return millis_counter;
}

 //-----------------------------------------------------------------------------
void ADC_Init() {
    ADCON0 = 0x01;         // B?t ADC, k?nh m?c ??nh AN0
    ADCON1 = 0x0C;         // AN0?AN3 l? analog, c?n l?i l? digital
    ADCON2 = 0xA9;         // Right justified, 8TAD, Fosc/8
    TRISAbits.TRISA0 = 1;  // AN0 input
    TRISAbits.TRISA1 = 1;  // AN1 input
    TRISAbits.TRISA2 = 1;  // AN2 input
    TRISAbits.TRISA3 = 1;  // AN3 input
    __delay_ms(2);         // ??i ?n ??nh
}


unsigned int ADC_Read(unsigned char channel) {
    if(channel > 13) return 0;   // PIC18F4550 c? AN0?AN13

    ADCON0 &= 0b11000001;        // X?a bit ch?n k?nh c? (CHS3:0)
    ADCON0 |= (channel << 2);    // G?n k?nh m?i v?o CHS3:0
    __delay_us(10);              // ??i m?t ch?t sau khi ch?n k?nh

    ADCON0bits.GO = 1;           // B?t ??u chuy?n ??i
    while (ADCON0bits.GO);       // ??i xong
    return ((unsigned int)ADRESH << 8) | ADRESL;

}

//------------------------------------------------------------------------------
void UART_Init(void) {
    TRISCbits.TRISC6 = 0;   // TX l? output
    TRISCbits.TRISC7 = 1;   // RX l? input

    SPBRG = 51;            // Baud rate 9600 v?i Fosc = 20MHz (cho BRGH = 1)
    TXSTAbits.BRGH = 1;     // High speed

    TXSTAbits.SYNC = 0;     // Ch? ?? kh?ng ??ng b?
    RCSTAbits.SPEN = 1;     // B?t Serial Port (TX/ RX)

    TXSTAbits.TXEN = 1;     // B?t truy?n
    RCSTAbits.CREN = 1;     // B?t nh?n
}

void UART_TxChar(char ch) {
    while (!TXSTAbits.TRMT);  // Ch? khi buffer truy?n r?ng
    TXREG = ch;               // G?i k? t?
}

void UART_TxString(const char* str) {
    while (*str) {
        UART_TxChar(*str++);
    }
}
unsigned char UART_RxChar(void) {
    while (!PIR1bits.RCIF);   // Ch? ??n khi c? RCIF = 1 (có d? li?u)
    if (RCSTAbits.OERR) {     // Ki?m tra l?i overrun
        RCSTAbits.CREN = 0;   // Reset b? nh?n
        RCSTAbits.CREN = 1;   // B?t l?i b? nh?n
    }
    return RCREG;             // Tr? v? byte d? li?u t? thanh ghi RCREG
}
unsigned char ADC_ToPercent(unsigned int value) {
    return (unsigned char)(100-(value * 100UL) / 1023);
}

void custom_delay_ms(unsigned long ms) {
    for (unsigned long i = 0; i < ms; i++) {
        __delay_ms(1); // Dùng macro delay có s?n
    }
}

//-------------------------------------------------------------------------------

void Control_Pump_PWM(int humidity1, int SETPOINT) {  
    TRISBbits.RB4=0;
    TRISBbits.RB3=0;
    LATBbits.LATB3 = 0;
    
    if (humidity1 < SETPOINT - RANGE && SETPOINT >= RANGE ) {
        
        //LCD_Set_Cursor(2, 1);
        //LCD_Write_String("Watering....   ");
        LATBbits.LATB4 = 1;
    } else if ( PORTBbits.RB4 == 1 || humidity1 >= SETPOINT|| SETPOINT <= RANGE){
        LATBbits.LATB4 = 0;
        //LCD_Set_Cursor(2, 1);
        //LCD_Write_String("Irrigated   ");
        //__delay_ms(1000);
        //LCD_Clear();
        }
    else LATBbits.LATB4 = 0;
        //PWM1_SetDuty(duty);   

}
void Control_Pump_Timer(int time_delay, int set_time) {
    TRISBbits.RB4=0;
    TRISBbits.RB3=0;
    LATBbits.LATB3 = 0;
    while (1) {
        if (time_delay > 0) {
            unsigned long start = millis();
            while (millis() - start < (unsigned long)time_delay * 3600 * 1000);
        }
        
        unsigned long startPump = millis();
        while (millis() - startPump < (unsigned long)set_time * 60 * 1000);
        LATBbits.LATB4 = 0;
       
    }
}
    //-----------------------------------------------------------------------------

void I2C_SDA_High(void) {
    I2C_SDA_DIR = 1; // ??t SDA l? input ?? pull-up k?o l?n cao
    __delay_us(15);
}

void I2C_SDA_Low(void) {
    I2C_SDA_DIR = 0; // ??t SDA l? output
    I2C_SDA = 0;     // K?o SDA xu?ng th?p
    __delay_us(15);
}

void I2C_SCL_High(void) {
    I2C_SCL_DIR = 1; // ??t SCL l? input ?? pull-up k?o l?n cao
    __delay_us(15);
}

void I2C_SCL_Low(void) {
    I2C_SCL_DIR = 0; // ??t SCL l? output
    I2C_SCL = 0;     // K?o SCL xu?ng th?p
    __delay_us(15);
}

void I2C_Init(void) {
    I2C_SCL_DIR = 1; // SCL (RB1) l? input (open-drain)
    I2C_SDA_DIR = 1; // SDA (RB0) l? input (open-drain)
    I2C_SCL = 1;
    I2C_SDA = 1;


}


void I2C_Start(void) {
    I2C_SDA_High();
    I2C_SCL_High();
    I2C_SDA_Low();  // SDA t? cao xu?ng th?p khi SCL cao ? Start condition
    I2C_SCL_Low();
}

void I2C_Stop(void) {
    I2C_SCL_Low();
    I2C_SDA_Low();
    I2C_SCL_High();
    I2C_SDA_High(); // SDA t? th?p l?n cao khi SCL cao ? Stop condition
}

unsigned char I2C_Read_ACK(void) {
    unsigned char ack;
    I2C_SDA_High();  // Th? SDA ?? slave ?i?u khi?n
    I2C_SCL_High();  // T?o xung clock ?? ??c ACK
    ack = I2C_SDA;   // ??c tr?ng th?i SDA (0 = ACK, 1 = NACK)
    I2C_SCL_Low();
    return ack;
}

unsigned char I2C_Write(unsigned char data) {
    unsigned char i;
    for (i = 0; i < 8; i++) {
        I2C_SCL_Low();
        if (data & 0x80) {
            I2C_SDA_High();
        } else {
            I2C_SDA_Low();
        }
        data <<= 1;
        I2C_SCL_High(); // T?o xung clock ?? g?i bit
        I2C_SCL_Low();
    }
    return I2C_Read_ACK(); // ??c ACK t? slave
}

//---------------[ LCD Routines ]----------------
void LCD_Init(void) {
    I2C_Start();
    if (I2C_Write((unsigned char)(i2c_add << 1)) != 0) { // G?i ??a ch? 8-bit
        I2C_Stop();
        return;
    }
    
    I2C_Write(0x00);
    I2C_Stop();
    __delay_ms(10);
    LCD_CMD(0x03);
    __delay_ms(2);
    LCD_CMD(0x03);
    __delay_ms(2);
    LCD_CMD(0x03);
    __delay_ms(2);
    LCD_CMD(LCD_RETURN_HOME);
    __delay_ms(2);
    LCD_CMD(0x20 | (LCD_TYPE << 2));
    __delay_ms(10);
    LCD_CMD(LCD_TURN_ON);
    __delay_ms(10);
    LCD_Clear();
    __delay_ms(150);
    LCD_CMD(LCD_ENTRY_MODE_SET | LCD_RETURN_HOME);
    __delay_ms(10);
}

void LCD_CMD(unsigned char CMD) {
    RS = 0; // Command Register Select
    unsigned char nibble = (CMD & 0xF0) | RS;
    I2C_Start();
    I2C_Write((unsigned char)(i2c_add << 1));
    I2C_Write((nibble | 0x04) | BackLight_State); // Pulse EN
    I2C_Stop();
    I2C_Start();
    I2C_Write((unsigned char)(i2c_add << 1));
    I2C_Write((nibble & 0xFB) | BackLight_State); // EN = 0
    I2C_Stop();
    __delay_us(300);

    nibble = ((CMD << 4) & 0xF0) | RS;
    I2C_Start();
    I2C_Write((unsigned char)(i2c_add << 1));
    I2C_Write((nibble | 0x04) | BackLight_State); // Pulse EN
    I2C_Stop();
    I2C_Start();
    I2C_Write((unsigned char)(i2c_add << 1));
    I2C_Write((nibble & 0xFB) | BackLight_State); // EN = 0
    I2C_Stop();
    __delay_us(300);
}

void LCD_Write_Char(char Data) {
    RS = 1; // Data Register Select
    unsigned char nibble = (Data & 0xF0) | RS;
    I2C_Start();
    I2C_Write((unsigned char)(i2c_add << 1));
    I2C_Write((nibble | 0x04) | BackLight_State); // Pulse EN
    I2C_Stop();
    I2C_Start();
    I2C_Write((unsigned char)(i2c_add << 1));
    I2C_Write((nibble & 0xFB) | BackLight_State); // EN = 0
    I2C_Stop();
    __delay_us(300);

    nibble = ((Data << 4) & 0xF0) | RS;
    I2C_Start();
    I2C_Write((unsigned char)(i2c_add << 1));
    I2C_Write((nibble | 0x04) | BackLight_State); // Pulse EN
    I2C_Stop();
    I2C_Start();
    I2C_Write((unsigned char)(i2c_add << 1));
    I2C_Write((nibble & 0xFB) | BackLight_State); // EN = 0
    I2C_Stop();
    __delay_us(300);
}

void LCD_Write_String(char* Str) {
    while (*Str) {
        LCD_Write_Char(*Str++);
        __delay_us(700);
    }
}

void LCD_Set_Cursor(unsigned char ROW, unsigned char COL) {
    switch (ROW) {
        case 2:
            LCD_CMD(0xC0 + COL - 1);
            break;
        case 3:
            LCD_CMD(0x94 + COL - 1);
            break;
        case 4:
            LCD_CMD(0xD4 + COL - 1);
            break;
        default:
            LCD_CMD(0x80 + COL - 1);
    }
}

void LCD_Clear(void) {
    RS = 0;
    unsigned char nibble;

    // G?i n?a cao c?a l?nh Clear (0x01 = 0b00000001)
    nibble = (LCD_CLEAR & 0xF0) | RS;
    I2C_Start();
    I2C_Write((unsigned char)(i2c_add << 1));
    I2C_Write((nibble | 0x04) | BackLight_State); // EN = 1 (pulse)
    I2C_Write((nibble & 0xFB) | BackLight_State); // EN = 0
    
    // G?i n?a th?p c?a l?nh Clear
    nibble = ((LCD_CLEAR << 4) & 0xF0) | RS;
    I2C_Write((nibble | 0x04) | BackLight_State); // EN = 1 (pulse)
    I2C_Write((nibble & 0xFB) | BackLight_State); // EN = 0
    
    I2C_Stop();

    __delay_ms(2);  // delay ~2ms cho l?nh Clear
}
//-----------------------------------------------------------------------------------
void init_timer0() {
    T0CON = 0b10000100;   // Timer0 ON, 8-bit, internal, prescaler 1:32
    TMR0L = 194;          // N?p l?i giá tr? ?? t?o ng?t m?i ~1ms v?i Fosc = 8MHz
    INTCONbits.TMR0IE = 1;
    INTCONbits.GIE = 1;
}

    
#define BTN_RD0 PORTDbits.RD0

unsigned char isPressed = 0;
unsigned long pressStart = 0;
unsigned long lastBurst = 0;
uint8_t setpoint = 0;

int update_setpoint_from_button(int setpoint) {
    unsigned long now = millis();

    // --------- T?NG b?ng RD0 ----------
    static uint8_t isPressedInc = 0;
    static unsigned long pressStartInc = 0;
    static unsigned long lastBurstInc = 0;

    if (PORTDbits.RD0 == 0 && isPressedInc == 0) {
        __delay_ms(10);
        if (PORTDbits.RD0 == 0) {
            isPressedInc = 1;
            pressStartInc = now;
            lastBurstInc = now;
        }
    }

    if (PORTDbits.RD0 == 0 && isPressedInc == 1) {
        unsigned long heldTime = now - pressStartInc;
        if (heldTime >= 1000 && (now - lastBurstInc) >= 500) {
            lastBurstInc = now;
            if (setpoint <= 90) setpoint += 10;
            else setpoint = 100;
        }
    }

    if (PORTDbits.RD0 == 1 && isPressedInc == 1) {
        __delay_ms(10);
        if (PORTDbits.RD0 == 1) {
            isPressedInc = 0;
            unsigned long duration = now - pressStartInc;
            if (duration < 1000) {
                if (setpoint < 100) setpoint += 1;
            }
        }
    }

    // --------- GI?M b?ng RD1 ----------
    static uint8_t isPressedDec = 0;
    static unsigned long pressStartDec = 0;
    static unsigned long lastBurstDec = 0;

    if (PORTDbits.RD1 == 0 && isPressedDec == 0) {
        __delay_ms(10);
        if (PORTDbits.RD1 == 0) {
            isPressedDec = 1;
            pressStartDec = now;
            lastBurstDec = now;
        }
    }

    if (PORTDbits.RD1 == 0 && isPressedDec == 1) {
        unsigned long heldTime = now - pressStartDec;
        if (heldTime >= 1000 && (now - lastBurstDec) >= 500) {
            lastBurstDec = now;
            if (setpoint >= 10) setpoint -= 10;
            else setpoint = 0;
        }
    }

    if (PORTDbits.RD1 == 1 && isPressedDec == 1) {
        __delay_ms(10);
        if (PORTDbits.RD1 == 1) {
            isPressedDec = 0;
            unsigned long duration = now - pressStartDec;
            if (duration < 1000) {
                if (setpoint > 0) setpoint -= 1;
            }
        }
    }

    return setpoint;
}



//----------------------------------------------------------------------------------------
void LCD_mode0(unsigned char humidity1, int SETPOINT,unsigned int count ){
    char buf_LCD[20];
        LCD_Set_Cursor(1, 1);
        LCD_Write_String("Humidity:");
        LCD_Set_Cursor(1, 11);
        sprintf(buf_LCD,"%d%%  ", humidity1);  // G?p chu?i
        LCD_Write_String(buf_LCD);  // G?i l?n LCD
    
        LCD_Set_Cursor(2, 1);
        LCD_Write_String("SP: ");
        LCD_Set_Cursor(2,4);
        sprintf(buf_LCD,"%d%%  ", SETPOINT);  // G?p chu?i
        LCD_Write_String(buf_LCD);  // G?i l?n LCD        
        LCD_Set_Cursor(2, 9);
        LCD_Write_String("ST: ");
        if (PORTBbits.RB4 == 1){
        LCD_Set_Cursor(2,12);
        LCD_Write_String("WATER");}
        else if (PORTBbits.RB4 == 0){
        LCD_Set_Cursor(2,12);
        LCD_Write_String("GOOD ");
        }
        
}
void LCD_mode1(int set_time, int time_delay){
char buf_LCD[20];
    LCD_Set_Cursor(1, 1);
    LCD_Write_String("Timer:");
    LCD_Set_Cursor(1, 7);
    sprintf(buf_LCD,"%d H",time_delay);
    LCD_Write_String(buf_LCD);
    LCD_Set_Cursor(2, 1);
    LCD_Write_String("Set time:");
    LCD_Set_Cursor(2, 10);
    sprintf(buf_LCD,"%d min ", set_time);
    LCD_Write_String(buf_LCD);
}
//===================================select_mode=============================================
void __interrupt() ISR(void) {
    if (INTCON3bits.INT2IF) {
        select_mode();
        INTCON3bits.INT2IF = 0;
    }
    if (INTCONbits.TMR0IF && INTCONbits.TMR0IE) {
        millis_counter++;
        TMR0L = 194; // ??ng b? v?i init_timer0
        INTCONbits.TMR0IF = 0;
    }
}
 mode_value select_mode(void) {
     mode_value result ={0,0,50,6};
     
    LCD_Set_Cursor(1, 1);
    LCD_Write_String("         "); //clear
    LCD_Set_Cursor(1, 1);
    LCD_Write_String("SELECT MODE ");
    LCD_Set_Cursor(2, 1);
    LCD_Write_String("Mode:");
    LCD_Set_Cursor(2, 7);
    LCD_Write_String("Set Point");
    char buf_LCD[20];
    while (1) {
        if (PORTDbits.RD2 == 0) {
            __delay_ms(200);  // debounce
            
            break; 
        }

        if (PORTDbits.RD0 == 0 || PORTDbits.RD1 == 0) {
            __delay_ms(200);  // debounce
            result.mode++; // T?ng mode trong struct
            if ( result.mode % 2 == 0) {
                LCD_Set_Cursor(2, 7);
                LCD_Write_String("         "); //clear
                LCD_Set_Cursor(2, 7);
                LCD_Write_String("Set Point"); //dk: even
                
            } else {
                LCD_Set_Cursor(2, 7);
                LCD_Write_String("         "); //clear
                LCD_Set_Cursor(2, 7);
                LCD_Write_String("Set Time"); //dk: old
            }
        }
    }
    if (result.mode %2==0){
    
    LCD_Clear();
    LCD_Set_Cursor(1, 1);
    LCD_Write_String("MODE 0");
    LCD_Set_Cursor(2, 1);
    LCD_Write_String("Set Point: ");
    
    
    while (1) {
    result.SETPOINT = update_setpoint_from_button(result.SETPOINT); 
    
    LCD_Set_Cursor(2, 11);
    sprintf(buf_LCD,"%d%%  ", result.   SETPOINT);
    LCD_Write_String(buf_LCD);
    if (PORTDbits.RD2 == 0){
        __delay_ms(100);
        break;
        }
    }
    } else{   
    LCD_Clear();
    LCD_Set_Cursor(1, 1);
    LCD_Write_String("MODE 1");
    LCD_Set_Cursor(2, 1);
    LCD_Write_String("Time delay: ");
    while (1) {
    result.time_delay = update_setpoint_from_button( result.time_delay);  
    LCD_Set_Cursor(2, 12);
    sprintf(buf_LCD,"%d H", result.time_delay);
    LCD_Write_String(buf_LCD);
    if (PORTDbits.RD2 == 0){
        __delay_ms(100);
        break;
        }
    } 
    LCD_Set_Cursor(2, 1);
    LCD_Write_String("                  ");
    LCD_Set_Cursor(2, 1);
    LCD_Write_String("Set time:");
    while (1) {
    result.set_time = update_setpoint_from_button(result.set_time);  
    LCD_Set_Cursor(2, 10);
    sprintf(buf_LCD,"%d min ", result.set_time);
    LCD_Write_String(buf_LCD);
    if (PORTDbits.RD2 == 0){
        __delay_ms(100);
        break;
        }
    } 
    }
    return result;
}

//================================================================================
void main(void) {
    
    TRISCbits.RC7=1;
    //button
    TRISDbits.TRISD0 = 1; // RD0 as input +
    TRISDbits.TRISD1 = 1; // RD0 as input -
    TRISDbits.TRISD2 = 1; // RD0 as input -
    //adc
    ADC_Init();  // Kh?i t?o ADC
    UART_Init();
    I2C_Init();   // Kh?i t?o I2C ph?n m?m
    LCD_Init();
    
    //mode qua ngat int2
    TRISBbits.TRISB2 = 1;

    // C?u hình c?nh ng?t INT2: c?nh lên (1) ho?c xu?ng (0)
    INTCON2bits.INTEDG2 = 1; // Ng?t khi RB2 chuy?n t? 0 -> 1

    // B?t ng?t INT2
    INTCON3bits.INT2IE = 1;  // Cho phép INT2
    INTCON3bits.INT2IF = 0;  // Xóa c? INT2

    // Cho phép ng?t ngo?i vi và toàn c?c
    INTCONbits.PEIE = 1;
    INTCONbits.GIE  = 1;
    
    mode_value result;
    char buf_UART[50]; 
    LCD_Clear();
    init_timer0();
    
    result = select_mode();
    
    if (result.mode % 2 == 0){
    while (1){
        result.SETPOINT=update_setpoint_from_button( result.SETPOINT); 
        //unsigned int button =  ADC_Read(0);
        //unsigned int count =ADC_ToPercent(button);
        unsigned int value1 = ADC_Read(1);
        unsigned char humidity1 = ADC_ToPercent(value1);
        
        sprintf(buf_UART, "Mode %u\n ", result.mode);
        UART_TxString(buf_UART);
        sprintf(buf_UART, "Humidity %u\n ", humidity1);
        UART_TxString(buf_UART);
        sprintf(buf_UART, "Set_point %u\n ", result.SETPOINT);
        UART_TxString(buf_UART);
        
        LCD_mode0(humidity1,result.SETPOINT,count);
        Control_Pump_PWM(humidity1,result.SETPOINT);
    } 
    }
    else {
         LCD_Clear();
         LCD_mode1(result.set_time, result.time_delay);
        while (1){
             sprintf(buf_UART, "Mode %u\n ", result.mode);
        UART_TxString(buf_UART);
        sprintf(buf_UART, "Set_time %u\n ", result.set_time);
        UART_TxString(buf_UART);
        sprintf(buf_UART, "Delay %u\n ", result.time_delay);
        UART_TxString(buf_UART);
           Control_Pump_Timer(result.time_delay, result.set_time);
           
       
    }   
}
    }
    