#include "msp.h"
#include <stdio.h>
//test 5

/**
 * Jenna Stolzman and Gabrielle Green
 * Final Project..
 *
 * New Comment
 */

enum states{HOURS,MINUTES,SECONDS};
void Initialize_Pins(void);

void delay_micro(unsigned microsec);
void delay_ms(unsigned ms);
void SysTick_Init();

void Timer_32_Init(void);
void T32_INT1_IRQHandler(void);

void Initialize_LCD(void);
void PulseEnablePin(void);
void pushNibble(uint8_t nibble);
void pushByte(uint8_t byte);
void commandWrite(uint8_t command);
void dataWrite(uint8_t data);

void LCD_CurrentTime(void);

volatile int current_second = 0, current_minute = 0, current_hour = 0;

volatile int flag,flag_up=0,flag_check_hms = 0;


volatile char current_day_status = 'A';


void SetupPort5Interrupts();

void PORT5_IRQHandler(void);

void Set_Time(void);


int j=0;
int k=1;
int l=1;
int n=1;

int num;
int sec;
int num1;
int sec1;

char hour_current[2];
char minute_current[2];
char second_current[2];
char minute_current_small[1];
char second_current_small[1];

void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

    __disable_irq();
    SetupPort5Interrupts();
    __enable_interrupt();

	Initialize_Pins();
	SysTick_Init();

	Initialize_LCD();
	Timer_32_Init();

	int i;

	while(1)
	{
	    LCD_CurrentTime();

	   enum states state = HOURS;

	   while(flag_check_hms)
	   {
	       if(flag_check_hms == 1)
	       {
	           state = HOURS;
	       }
	       if(flag_check_hms == 2)
	       {
	           state = MINUTES;
	       }
	       if(flag_check_hms == 3)
	       {
	           state = SECONDS;
	       }
	    switch (state)
	    {

	    case HOURS:
	        if(flag_up == 1)
	        {
	            current_hour++;
	            sprintf(hour_current,"%d",current_hour);
	            delay_ms(100);
	            commandWrite(0x80);
	            if(current_hour > 10)
	            {
	            for(i = 0; i < 2; i++)
	            {
	                dataWrite(hour_current[i]);
	            }
	                j = 1;
	            }
	            else
	            {
	                dataWrite(hour_current[0]);
	            }
	            dataWrite(0b00111010);
	            TIMER32_1 -> VALUE = 5062500000 - (current_hour*4218750);
	            flag_up = 0;
	        }
	        break;
	    case MINUTES:
	        if(flag_up == 1)
	        {
	            current_minute++;
	            if(current_minute<10)
	                {
	                    sprintf(minute_current_small,"%d",current_minute);
	                    delay_ms(100);
	                    commandWrite(0x82+j);
	                    dataWrite('0');
	                    for(i=0;i<1;i++)
	                    {
	                        dataWrite(minute_current_small[i]);
	                    }
	                    dataWrite(0b00111010);

	                }
	        else
	        {
	            sprintf(minute_current,"%d",current_minute);
	            delay_ms(100);
	            commandWrite(0x82+j);
	        for(i = 0; i < 2; i++)
	            {
	            dataWrite(minute_current[i]);

	            }
	        dataWrite(0b00111010);
	        }
	            flag_up = 0;
	        }
	        TIMER32_1 -> VALUE = (TIMER32_1 -> VALUE +(562500000 - (current_minute*703125)));
	        break;

	    case SECONDS:
	        if(flag_up == 1)
	        {
	            current_second++;
	        if(current_second<10)
	        {
	            sprintf(second_current_small,"%d",current_second);
	                delay_ms(100);
	                commandWrite(0x85+j);
	                dataWrite('0');
	                for(i=0;i<1;i++)
	                {
	                    dataWrite(second_current_small[i]);
	                }

	        }
	        else
	        {
	            sprintf(second_current,"%d",current_second);

	            delay_ms(100);
	            commandWrite(0x85+j);
	        for(i = 0; i < 2; i++)
	        {
	            dataWrite(second_current[i]); //print
	        }

	        }
	        flag_up = 0;
	        }
	        TIMER32_1 -> VALUE = (TIMER32_1 -> VALUE + (562500000 - (current_second*11719)));
	        break;
	    }

	   }
	}

}

void PORT5_IRQHandler(void)

{
    int status = P5 -> IFG;
    P5 -> IFG = 0;
    if(status & BIT1)
    {

        if(flag_check_hms == 3)
        {
            flag_check_hms = 0;
        }
        else
        {
            flag_check_hms++;

        }

    }
    if(status & BIT0)
    {
        flag_up = 1;
    }

}
void SetupPort5Interrupts()
{

    /*
     * Set Time Button
     */
    P5->SEL0 &= ~BIT1;                              // Setup the P5.1 on the Launchpad as Input, Pull Up Resistor
    P5->SEL1 &= ~BIT1;
    P5->DIR &= ~BIT1;
    P5->REN |= BIT1;
    P5->OUT |= BIT1;

    P5->IES |= BIT1;                                //Set pin interrupt to trigger when it goes from high to low (starts high due to pull up resistor)
    P5->IE |= BIT1;                                 //Set interrupt on for P5.1

    /*
     * Set On/Off/Up Button
     */
    P5->SEL0 &= ~BIT0;                              // Setup the P5.0 on the Launchpad as Input, Pull Up Resistor
    P5->SEL1 &= ~BIT0;
    P5->DIR &= ~BIT0;
    P5->REN |= BIT0;
    P5->OUT |= BIT0;
    P5->IES |= BIT0;                                //Set pin interrupt to trigger when it goes from high to low (starts high due to pull up resistor)
    P5->IE |= BIT0;                                 //Set interrupt on for P5.0

    /*
     * Set Alarm Button
     */
    P5->SEL0 &= ~BIT2;                              // Setup the P1.1 on the Launchpad as Input, Pull Up Resistor
    P5->SEL1 &= ~BIT2;
    P5->DIR &= ~BIT2;
    P5->REN |= BIT2;
    P5->OUT |= BIT2;

    P5->IES |= BIT2;                                //Set pin interrupt to trigger when it goes from high to low (starts high due to pull up resistor)
    P5->IE |= BIT2;

    P5->IFG = 0;                                    //Clear all interrupt flags
    NVIC_EnableIRQ(PORT5_IRQn);                     //Enable Port 1 interrupts.  Look at msp.h if you want to see what all these are called.
}

/*
 * void LCD_CurrentTime
 *
 * The purpose of this function is to print the current
 * time to the LCD
 */
void LCD_CurrentTime(void)
{
    int i;
    /*
     * Print Current Hour
     */
    current_hour = 5062500000 - TIMER32_1 -> VALUE;
    current_hour = (current_hour/42187500) +1;
    printf("%d", current_hour);
    sprintf(hour_current,"%d",current_hour);
    delay_ms(100);
    commandWrite(0x80);
    if(current_hour > 10)
    {
    for(i = 0; i < 2; i++)
    {
        dataWrite(hour_current[i]);
    }
        j = 1;
    }
    else
    {
        dataWrite(hour_current[0]);
    }
    dataWrite(0b00111010);

    /*
     * Print current minute
     */
    current_minute = 5062500000 - TIMER32_1 -> VALUE;
    if(current_minute <= 703125*l)
    {
        if(l>=60)
        {
            num1 = l/60;
            sec1 = l%(60*num1);
            current_second = sec1;
        }
        else
        {
            current_minute = (l-1);
        }
                if(current_minute<10)
                {
            sprintf(minute_current_small,"%d",current_minute);
            delay_ms(100);
            commandWrite(0x82+j);
            dataWrite('0');
            for(i=0;i<1;i++)
            {
                dataWrite(minute_current_small[i]);
            }
                dataWrite(0b00111010);

            }
            else
            {
            sprintf(minute_current,"%d",current_minute);

            delay_ms(100);
            commandWrite(0x82+j);
            for(i = 0; i < 2; i++)
            {
                dataWrite(minute_current[i]);

            }
            dataWrite(0b00111010);
            }
    }
    else
    {

        l++;
    }

    /*
     * Print current second
     */
    current_second = 5062500000 - TIMER32_1 -> VALUE;

    if(current_second<=11719*n)
    {
        if(n>=60)
        {
            num = n/60;
            sec = n%(60*num);
            current_second = sec;
        }
        else
        {
            current_second = (n-1);
        }
    if(current_second<10)
    {
        sprintf(second_current_small,"%d",current_second);
            delay_ms(100);
            commandWrite(0x85+j);
            dataWrite('0');
            for(i=0;i<1;i++)
            {
                dataWrite(second_current_small[i]);
            }

    }
    else
    {
        sprintf(second_current,"%d",current_second);

        delay_ms(100);
        commandWrite(0x85+j);
    for(i = 0; i < 2; i++)
    {
        dataWrite(second_current[i]); //print
    }


    }
    }
    else
    {
        n++;
    }

    /*
     * Print AM or PM
     */
        commandWrite(0x87+j);
        dataWrite(current_day_status);
        dataWrite('M');
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

    commandWrite(0x0C);
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
/*
 * void Timer_32_Init(void)
 *
 * The purpose of this function is to initialize Timer 32
 */
void Timer_32_Init(void)
{
    TIMER32_1 -> CONTROL = 0b11101011; //Enabled, periodic mode, interrupt enabled, 256 divider, 32 bit, one-shot
    TIMER32_1 -> LOAD = 5062500000; //12 hours
    TIMER32_1 -> INTCLR = 0;
    NVIC_EnableIRQ(T32_INT1_IRQn);
}
/*
 * void T32_INT1_IRQHandler(void)
 *
 * The purpose of this function is the interrupt handler
 * for Timer 32
 */
void T32_INT1_IRQHandler(void)
{
    if(current_day_status=='A')
    {
        current_day_status = 'P';
    }
    else
    {
        current_day_status = 'A';
    }
    TIMER32_1 -> INTCLR = 0;
}
void SysTick_Init()
{
    SysTick->CTRL=0x00; //counter off
    SysTick->VAL=0x00; //decimal 3000 (1 ms)
    SysTick->LOAD=0x00; //Resets the counter
    SysTick->CTRL=0x05; //turns counter on
}
