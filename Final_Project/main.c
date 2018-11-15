#include "msp.h"


/**
 * Jenna Stolzman and Gabrielle Green
 *
 */

void Initialize_Pins(void);

void delay_micro(unsigned microsec);
void delay_ms(unsigned ms);

void Initialize_LCD(void);
void PulseEnablePin(void);
void pushNibble(uint8_t nibble);
void pushByte(uint8_t byte);
void commandWrite(uint8_t command);
void dataWrite(uint8_t data);

void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
	Initialize_Pins();
	Initialize_LCD();

}


/*
 * void Initialize_Pins(void)
 *
 * The purpose of this function is to initialize pins for the LCD
 * and LED's used in alarm clock
 */
void Initialize_Pins(void)
{
    //LCD pins
    //Initialize P6.0 E and 6.1 RS
    P6->SEL0 &= ~(BIT0 | BIT1); //sets up P4.0 GPIO
    P6->SEL1 &= ~(BIT0 | BIT1); //sets up P4.0 as GPIO
    P6->DIR |= (BIT0 | BIT1); //sets up P4.0 as an output

    //Initialize P5.4 DB4, P5.5 DB5, P5.6 DB6, P5.7 DB7
    P5->SEL0 &= ~(BIT4 | BIT5 | BIT6 | BIT7); //sets as GPIO
    P5->SEL1 &= ~(BIT4 | BIT5 | BIT6 | BIT7); //sets as GPIO
    P5->DIR |= (BIT4 | BIT5 | BIT6 | BIT7); //sets as an output

    //LED pins as PWM with TimerA
    P2->SEL0 |= (BIT4 | BIT5); //set as Timer A
    P2->SEL1 &= ~(BIT4 | BIT5); //set as Timer A
    P2->DIR |= (BIT4 | BIT5); //set as output
    P2->OUT &= ~(BIT4 | BIT5);
}
/*
 * void Initialize_LCD(void)
 *
 * The purpose of this function is to initialize the LCD by going through the
 * initialization sequence outlined in Figure 4 of the prelab
 *
 * From figure 4 given in prelab
 */
void Initialize_LCD(void)
{

    commandWrite(3);
    delay_ms(100);
    commandWrite(3);
    delay_ms(100);
    commandWrite(3);
    delay_ms(100);

    commandWrite(2);
    delay_ms(100);
    commandWrite(2);
    delay_ms(100);

    commandWrite(8);
    delay_ms(100);
    commandWrite(0x0F);
    delay_ms(100);
    commandWrite(1);
    delay_ms(100);
    commandWrite(6);
    delay_ms(10);

}
/*
 * void PulseEnablePin(void)
 *
 * The purpose of this function is to pulse the enable so that bits
 * can be pushed to pins
 *
 * From flowchart (figure 6) given in prelab
 */

void PulseEnablePin(void)
{
    P6->OUT &= ~(BIT0); //Set E = 0
    delay_micro(10); //delay 10 microseconds
    P6->OUT |= BIT0; //Set E = 1
    delay_micro(10); //delay 10 microseconds
    P6->OUT &= ~(BIT0); //Set E = 0
    delay_micro(10); //delay 10 microseconds
}

/*
 * void pushNibble(uint8_t nibble)
 *
 * The purpose of this function is to push a nibble (4 bits)
 * onto DB4-DB7
 *
 * From flowchart (figure 5) given in prelab
 */

void pushNibble(uint8_t nibble)
{
    P5-> OUT &= ~(BIT4); //clear bits DB4-DB7
    P5-> OUT &= ~(BIT5);
    P5-> OUT &= ~(BIT6);
    P5-> OUT &= ~(BIT7);

    P5-> OUT |= ((nibble<< 4) & (BIT4 | BIT5 | BIT6 | BIT7)); //save nibble to pins
    PulseEnablePin(); //call pulse function so sent to LCD
}

/*
 * void pushByte(uint8_t byte)
 *
 * The purpose of this function is to push a byte (8 bits)
 * onto DB4-DB7 using pushNibble()
 *
 * From flowchart (figure 5) given in prelab
 */

void pushByte(uint8_t byte)
{
    uint8_t byte_ = 0x00;

   //send most significant digits
    byte_ = ((byte & 0xF0) >> 4);
    pushNibble(byte_);

    //send least significant digits
    byte_ = (byte & 0x0F);
     pushNibble(byte_);
    delay_micro(100);
}

/*
 * void commandWrite(uint8_t command)
 *
 * The purpose of this function is to write one byte of command using
 * the pushByte function with command parameter
 */

void commandWrite(uint8_t command)
{
    P6->OUT &= ~(BIT1); //Set RS to 0
    pushByte(command);
}

/*
 * void dataWrite(uint8_t data)
 *
 * The purpose of this function is to write one byte of data using
 * the pushByte function with data parameter
 */

void dataWrite(uint8_t data)
{
    P6->OUT |= BIT1; //Set RS to 1
    pushByte(data);
}
/*
 * void delay_micro(unsigned microsec)
 *
 * The purpose of this function is to delay the desired number of microseconds
 */

void delay_micro(unsigned microsec)
{
    SysTick->LOAD = ((microsec*3)-1);
    SysTick->VAL = 0;

    while((SysTick->CTRL & 0x00010000) == 0); //Bit 16 means complete
}

/*
 * void delay_ms(unsigned ms)
 *
 * The purpose of this function is to delay the desired number of milliseconds
 */

void delay_ms(unsigned ms)
{
    SysTick->LOAD = ((ms*3000)-1);
    SysTick->VAL = 0;

    while((SysTick->CTRL & 0x00010000) == 0); //Bit 16 means complete
}
