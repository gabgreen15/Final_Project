#include "msp.h"
#include <stdio.h>

/**
 * Jenna Stolzman and Gabrielle Green
 * Final Project..
 *
 *
 */

enum states{HOURS,MINUTES,SECONDS,HOURS_ALARM,MINUTES_ALARM,SECONDS_ALARM};
void Initialize_Pins(void);

void delay_micro(unsigned microsec);
void delay_ms(unsigned ms);
void SysTickInit(void);

void Timer_32_init();


void Initialize_LCD(void);
void PulseEnablePin(void);
void pushNibble(uint8_t nibble);
void pushByte(uint8_t byte);
void commandWrite(uint8_t command);
void dataWrite(uint8_t data);


volatile int flag,flag_up=0,flag_down = 0, flag_check_hms = 0, flag_check_hms_alarm = 0;


volatile char current_day_status = 'A';


void SetupPort5Interrupts(void);

void PORT5_IRQHandler(void);
void SetupPort3Interrupts(void);

void configRTC(void);
void printRTC(void);
void printAlarm(void);

void printRTC_SetTime(void);

int flag_time = 0;


uint8_t hr1 = 12, min1 = 30, sec1 = 0;
uint8_t hr_alarm = 0, min_alarm = 0;

// global struct variable called now
struct
{
    uint8_t sec;
    uint8_t min;
    uint8_t hour;
} now;

uint8_t RTC_flag = 0, RTC_alarm;


void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

    __disable_irq();
    SetupPort5Interrupts();
    SetupPort3Interrupts();

    configRTC();
    NVIC_EnableIRQ(RTC_C_IRQn);

    __enable_interrupt();

	Initialize_Pins();
	SysTickInit();
//	Timer_32_init();

	Initialize_LCD();

	printAlarm();

    while(1)
    {
        if(RTC_flag)
        {
            printRTC();
            RTC_flag = 0;
        }
        if(RTC_alarm)
        {
            printf("ALARM\n");
            RTC_alarm = 0;
        }

	   enum states state = HOURS;

	   while(flag_check_hms || flag_check_hms_alarm)
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
	       if(flag_check_hms_alarm == 1)
	       {
	           state = HOURS_ALARM;
	       }
	       if(flag_check_hms_alarm == 2)
	       {
	           state = MINUTES_ALARM;
	       }
	       if(flag_check_hms_alarm == 3)
	       {
	           state = SECONDS_ALARM;
	       }
	    switch (state)
	    {

	    case HOURS:
//	        commandWrite(0x08);
//	        delay_ms(1000);
//	        commandWrite(0x0C);
	        if(flag_up == 1)
	        {
	            hr1 = hr1 + 0b1;
                if(hr1 == 12)
                {
                    if(flag_time ==0)
                    {
                        flag_time = 1;
                    }
                    else
                    {
                        flag_time = 0;
                    }
                }
	            configRTC();
	            printRTC_SetTime();
                flag_up = 0;
	        }
	        if(flag_down == 1)
	        {
                hr1 = hr1 - 0b1;
                if(hr1 == 12)
                {
                    if(flag_time ==0)
                    {
                        flag_time = 1;
                    }
                    else
                    {
                        flag_time = 0;
                    }
                }
                configRTC();
                printRTC_SetTime();
                flag_down = 0;
	        }
	        break;
	    case MINUTES:
	        if(flag_up == 1)
	        {
	            min1 = min1 + 0b1;
	            configRTC();
	            printRTC_SetTime();
                flag_up = 0;
	        }

	        if(flag_down == 1)
	        {
	             min1 = min1 - 0b1;
	             configRTC();
	             printRTC_SetTime();
	             flag_down = 0;
	         }
	        break;
	    case SECONDS:
	        if(flag_up == 1)
	        {
	            sec1 = sec1 + 0b1;
                configRTC();
                printRTC_SetTime();
                flag_up = 0;
	        }
            if(flag_down == 1)
            {
                sec1 = sec1 - 0b1;
                configRTC();
                printRTC_SetTime();
                flag_down = 0;
             }
	        break;

	    case HOURS_ALARM:
	        if(flag_up == 1)
	        {
	            hr_alarm = hr_alarm + 0b1;

	            if(hr_alarm==12)
	            {
	                if(flag_time ==0)
	                {
	                    flag_time = 1;
	                }
	                else
	                {
	                    flag_time = 0;
	                }
	            }
	            configRTC();
	            printAlarm();
	            flag_up = 0;
            }
            if(flag_down == 1)
            {
                hr_alarm = hr_alarm - 0b1;

                if(hr_alarm==12)
                {
                    if(flag_time ==0)
                    {
                        flag_time = 1;
                    }
                    else
                    {
                        flag_time = 0;
                    }
                }
                 configRTC();
                 printAlarm();
                 flag_down = 0;
             }
	        break;

	    case MINUTES_ALARM:
	        if(flag_up == 1)
	        {
                min_alarm = min_alarm + 0b1;
                configRTC();
                printAlarm();
                flag_up = 0;
	        }
            if(flag_down == 1)
            {
                min_alarm = min_alarm - 0b1;
                 configRTC();
                 printAlarm();
                 flag_down = 0;
             }
            break;
	        }
	    }
	   }
}

void printAlarm(void)
{
    char minute_alarm[2];
    char hour_alarm[2];

    int j= 0, i = 0;

    sprintf(minute_alarm,"%d",min_alarm);
    sprintf(hour_alarm,"%d",hr_alarm);

    if(hr_alarm > 12)
    {
        hr_alarm = hr_alarm - 0b1100; //12 in binary
    }
    if(min_alarm > 59)
    {
        min_alarm = min_alarm - 0b111100; //60 in binary
    }


    commandWrite(0xC0);
    if(hr_alarm>=10)
    {
    for(i=0;i<2;i++)
    {
        dataWrite(hour_alarm[i]);
    }
        j=1;
    }
    else
    {
        commandWrite(0xC6);
        dataWrite(0b00100000);
        commandWrite(0xC0);
        dataWrite(hour_alarm[0]);
    }
    dataWrite(0b00111010);

    commandWrite(0xC2+j);
    if(min_alarm<10)
    {
        dataWrite('0');
        dataWrite(minute_alarm[0]);
    }
    else
    {
        for(i=0;i<2;i++)
        {
            dataWrite(minute_alarm[i]);
        }
    }
    commandWrite(0xC4+j);
    if(flag_time==0)
    {
        current_day_status = 'A';
    }
    else
    {
        current_day_status = 'P';
    }
    dataWrite(current_day_status);
    dataWrite('M');
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
    if(status & BIT2)
    {
        if(flag_check_hms_alarm == 2)
        {
            flag_check_hms_alarm = 0;
        }
        else
        {
            flag_check_hms_alarm++;
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
    P5->SEL0 &= ~BIT2;                              // Setup the P5.2 on the Launchpad as Input, Pull Up Resistor
    P5->SEL1 &= ~BIT2;
    P5->DIR &= ~BIT2;
    P5->REN |= BIT2;
    P5->OUT |= BIT2;

    P5->IES |= BIT2;                                //Set pin interrupt to trigger when it goes from high to low (starts high due to pull up resistor)
    P5->IE |= BIT2;

    P5->IFG = 0;                                    //Clear all interrupt flag
    NVIC_EnableIRQ(PORT5_IRQn);                     //Enable Port 5 interrupts.  Look at msp.h if you want to see what all these are called.
}

void SetupPort3Interrupts()
{
    /*
     * Set Snooze/Down Button
     */
    P3->SEL0 &= ~BIT0;                              // Setup the P3.0 on the Launchpad as Input, Pull Up Resistor
    P3->SEL1 &= ~BIT0;
    P3->DIR &= ~BIT0;
    P3->REN |= BIT0;
    P3->OUT |= BIT0;
    P3->IES |= BIT0;                                //Set pin interrupt to trigger when it goes from high to low (starts high due to pull up resistor)
    P3->IE |= BIT0;                                 //Set interrupt on for P3.0

    P3->IFG = 0;                                    //Clear all interrupt flags
    NVIC_EnableIRQ(PORT3_IRQn);                     //Enable Port 3 interrupts.  Look at msp.h if you want to see what all these are called.
}

void PORT3_IRQHandler(void)
{
    int status = P3 -> IFG;
    P3 -> IFG = 0;

    if(status & BIT0)
    {
        flag_down = 1;
    }
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

void configRTC(void)
{
    RTC_C->CTL0     =   0xA500;     //Write Code, IE on RTC Ready
    RTC_C->CTL13    =   0x0000;
    RTC_C->TIM0     = min1<<8 | sec1;
    RTC_C->TIM1     = 2<<8 | hr1;
    RTC_C->DATE     = 11<<8 | 26;
    RTC_C->YEAR     = 2018;
    RTC_C->PS1CTL   = 0b11010;

    RTC_C->AMINHR   = hr_alarm | min_alarm | BIT(15) | BIT(7);
    RTC_C->ADOWDAY = 0;
    RTC_C->CTL0     = ((0xA500) | BIT5);

}

void printRTC(void)
{

    int i;
    int j=0;

    char hourStr[2];
    char minStr[2];
    char secStr[2];


    if(now.hour > 12)
    {
        now.hour = now.hour - 0b1100; //12 in binary
    }
    if(now.min > 59)
    {
        now.min = now.min - 0b111100; //60 in binary
    }
    if(now.sec > 59)
    {
        now.sec = now.sec - 0b111100; //60 in binary
    }

    sprintf(hourStr, "%d", now.hour);
    sprintf(minStr, "%d", now.min);
    sprintf(secStr, "%d", now.sec);



    commandWrite(0x80);

    if(now.hour>=10)
    {
    for(i=0;i<2;i++)
    {
        dataWrite(hourStr[i]);
    }
        j=1;
    }
    else
    {
        commandWrite(0x89);
        dataWrite(0b00100000);
        commandWrite(0x80);
        dataWrite(hourStr[0]);
    }
    dataWrite(0b00111010);

    commandWrite(0x82+j);
    if(now.min<10)
    {
        dataWrite('0');
        dataWrite(minStr[0]);
        dataWrite(0b00111010);
    }
    else
    {
        for(i=0;i<2;i++)
        {
            dataWrite(minStr[i]);
        }
        dataWrite(0b00111010);
    }

    commandWrite(0x85+j);
    if(now.sec<10)
    {
        dataWrite('0');
        dataWrite(secStr[0]);
    }
    else
    {
        for(i=0;i<2;i++)
        {
            dataWrite(secStr[i]);
        }
    }
    commandWrite(0x87+j);
    if(flag_time==0)
    {
        current_day_status = 'A';
    }
    else
    {
        current_day_status = 'P';
    }
    dataWrite(current_day_status);
    dataWrite('M');
 //   printf("%02d:%02d:%02d %s %d %s %4d\n",now.hour, now.min, now.sec, dowStr, now.day, monthStr, now.year);

}

void printRTC_SetTime(void)
{

    int i;
    int j=0;

    char hourStr[2];
    char minStr[2];
    char secStr[2];


    if(hr1 > 12)
    {
        hr1 = hr1 - 0b1100; //12 in binary
    }
    if(min1 > 59)
    {
        min1 = min1 - 0b111100; //60 in binary
    }
    if(sec1 > 59)
    {
        sec1 = sec1 - 0b111100; //60 in binary
    }

    sprintf(hourStr, "%d", hr1);
    sprintf(minStr, "%d", min1);
    sprintf(secStr, "%d", sec1);



    commandWrite(0x80);

    if(hr1>=10)
    {
    for(i=0;i<2;i++)
    {
        dataWrite(hourStr[i]);
    }
        j=1;
    }
    else
    {
        commandWrite(0x89);
        dataWrite(0b00100000);
        commandWrite(0x80);
        dataWrite(hourStr[0]);
    }
    dataWrite(0b00111010);

    commandWrite(0x82+j);
    if(min1<10)
    {
        dataWrite('0');
        dataWrite(minStr[0]);
        dataWrite(0b00111010);
    }
    else
    {
        for(i=0;i<2;i++)
        {
            dataWrite(minStr[i]);
        }
        dataWrite(0b00111010);
    }

    commandWrite(0x85+j);
    if(sec1<10)
    {
        dataWrite('0');
        dataWrite(secStr[0]);
    }
    else
    {
        for(i=0;i<2;i++)
        {
            dataWrite(secStr[i]);
        }
    }
    commandWrite(0x87+j);
    if(flag_time == 0)
    {
        current_day_status = 'A';
    }
    else
    {
        current_day_status = 'P';
    }
    dataWrite(current_day_status);
    dataWrite('M');
 //   printf("%02d:%02d:%02d %s %d %s %4d\n",now.hour, now.min, now.sec, dowStr, now.day, monthStr, now.year);

}
void RTC_C_IRQHandler(void)
{
    if(RTC_C->CTL0 & BIT1) {
        RTC_alarm = 1;
        RTC_C->CTL0 = 0xA500;
    }
    if(RTC_C->PS1CTL & BIT0) {
        now.sec         =   RTC_C->TIM0>>0 & 0x00FF;
        now.min         =   RTC_C->TIM0>>8 & 0x00FF;
        now.hour        =   RTC_C->TIM1>>0 & 0x00FF;
        RTC_flag = 1;
        RTC_C->PS1CTL &= ~BIT0;
    }

}

void SysTickInit(void)
{
    SysTick->CTRL       =   0;
    SysTick->LOAD       =   0;        //Set interval for interrupt to occur at
    SysTick->VAL        =   0;                          //Reset value to zero
    SysTick->CTRL       =   0b101;                      //Set CLK, Set IE, Set Run
}

//void Timer_32_init()
//{
//    TIMER32_1 -> CONTROL = 0xC2;
//    TIMER32_1 -> LOAD = 30000000;
//    TIMER32_1 -> INTCLR = 0;
//    TIMER32_1 -> CONTROL |= 0x20;
//    NVIC_EnableIRQ(T32_INT1_IRQn);
//}
//
//void T32_INT1_IRQHandler()
//{
//
//    if(flag_screensaver == 0)
//    {
//        flag_screensaver = 1;
//    }
//
//    TIMER32_1 -> INTCLR = 0;
//}
