#include "msp.h"
#include <stdio.h>

/**
 * Jenna Stolzman and Gabrielle Green
 * Final Project...
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


volatile int flag,flag_up=0,flag_down = 0, flag_check_hms = 0, flag_check_hms_alarm = 0, flag_snooze;
volatile int flag_realtime=0,flag_faketime=0;
volatile int flag_alarm_no_snooze = 1;
volatile int flag_wake_lights = 0;

volatile char current_day_status = 'A';

void TimerA_Init_BLUE(void);
void TimerA_Init_GREEN(void);
void SetupPort5Interrupts(void);
void SetupPort1Interrupts();
void PORT5_IRQHandler(void);
void SetupPort3Interrupts(void);

void configRTC(void);
void printRTC(void);
void printAlarm(void);

void printRTC_SetTime(void);

int flag_time = 0;

int button_alarm = 0;

volatile int sec_lights = 0, min_lights = 0;
volatile int percent = 0;

uint8_t hr1 = 12, min1 = 34, sec1 = 55;
uint8_t hr_alarm = 12, min_alarm = 35;

int flag2 = 0;
int flag3 = 0;

// global struct variable called now
struct
{
    uint8_t sec;
    uint8_t min;
    uint8_t hour;
} now;

uint8_t RTC_flag = 0, RTC_alarm;

volatile uint16_t result;
float nADC;
float temp_C;
float temp_F;

char temp_Farenheit[5];

void Print_Temp(void);
void conversion(void);
void SysTick_Handler(void);
void ADC14init(void);
#define ADC_CONVERSION_RATE 30000  //3000*500ms = 2 per second or 1 every .5 seconds

void Alarm_Status(void);
#define C4 261.63
#define REST 0
#define WHOLE 4000000
#define MAX_NOTE 2
void SetupTimer32s();
#define BREATH_TIME 50000


int note = 0;       //The note in the music sequence we are on
int breath = 0;
float music_note_sequence[][2] = {
                                  {C4,WHOLE},
                                  {REST,WHOLE},

                                 };

void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

    __disable_irq();
    SetupPort5Interrupts();
    SetupPort1Interrupts();
    SetupPort3Interrupts();
    configRTC();
    NVIC_EnableIRQ(RTC_C_IRQn);

    __enable_interrupt();

	Initialize_Pins();
	SysTickInit();
//	Timer_32_init();

	Initialize_LCD();

	Alarm_Status();

	printAlarm();

	ADC14init();

    while(1)
    {
        if((hr_alarm == now.hour) && (min_alarm == now.min + 5))
        {
            flag_wake_lights = 1;
            TimerA_Init_BLUE();
            TimerA_Init_GREEN();
        }
        if(RTC_flag)
        {
            printRTC();
            RTC_flag = 0;
        }
        if(RTC_alarm && button_alarm)
        {
            if(flag_alarm_no_snooze == 0)
            {
                Alarm_Status();
                min_alarm = min_alarm + 10;

                hr1 = now.hour;
                min1 = now.min;
                sec1 = now.sec;

                configRTC();
                printAlarm();
//                flag_snooze = 0;
                RTC_alarm = 0;

                TIMER32_1->CONTROL = 0x0;                //Sets timer 1 for Enabled, Periodic, No Interrupt, No Prescaler, 32 bit mode, One Shot Mode.  See 589 of the reference manual
                TIMER32_2->CONTROL = 0x0;
            }
            if(flag_alarm_no_snooze == 1)
            {
                Alarm_Status();
                SetupTimer32s();

            }
        }
        else
        {
            Alarm_Status();

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
	        if(hr1 < 10)
	        {
	            commandWrite(0x80);
	            delay_ms(500);
	            dataWrite(0b00100000);
	            delay_ms(500);
	            printRTC_SetTime();
	        }
	        else
	        {
	            commandWrite(0x80);
	            delay_ms(500);
	            dataWrite(0b00100000);
	            dataWrite(0b00100000);
	            delay_ms(500);
	            printRTC_SetTime();
	        }

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
            if(hr1 < 10)
            {
                commandWrite(0x82);
                delay_ms(500);
                dataWrite(0b00100000);
                dataWrite(0b00100000);
                delay_ms(500);
                printRTC_SetTime();
            }
            else
            {
                commandWrite(0x83);
                delay_ms(500);
                dataWrite(0b00100000);
                dataWrite(0b00100000);
                delay_ms(500);
                printRTC_SetTime();
            }
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
            if(hr1 < 10)
            {
                commandWrite(0x85);
                delay_ms(500);
                dataWrite(0b00100000);
                dataWrite(0b00100000);
                delay_ms(500);
                printRTC_SetTime();
            }
            else
            {
                commandWrite(0x86);
                delay_ms(500);
                dataWrite(0b00100000);
                dataWrite(0b00100000);
                delay_ms(500);
                printRTC_SetTime();
            }
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
            if(hr_alarm < 10)
            {
                commandWrite(0xC0);
                delay_ms(500);
                dataWrite(0b00100000);
                delay_ms(500);
                printAlarm();
            }
            else
            {
                commandWrite(0xC0);
                delay_ms(500);
                dataWrite(0b00100000);
                dataWrite(0b00100000);
                delay_ms(500);
                printAlarm();
            }
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
            if(hr_alarm < 10)
            {
                commandWrite(0xC2);
                delay_ms(500);
                dataWrite(0b00100000);
                dataWrite(0b00100000);
                delay_ms(500);
                printAlarm();
            }
            else
            {
                commandWrite(0xC3);
                delay_ms(500);
                dataWrite(0b00100000);
                dataWrite(0b00100000);
                delay_ms(500);
                printAlarm();
            }
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
        if(flag_check_hms || flag_check_hms_alarm)
        {
            flag_up = 1;
        }
        else
        {
            if(button_alarm == 0)
            {
                button_alarm = 1;
//                if(RTC_alarm == 1)
//                {
//                    RTC_alarm = 0;
//                }
            }
            else
            {
                button_alarm = 0;
                flag_snooze = 0;
                if(RTC_alarm == 1)
                {
                    RTC_alarm = 0;
//                    TIMER32_1->CONTROL = 0b01000011;                //Sets timer 1 for Enabled, Periodic, No Interrupt, No Prescaler, 32 bit mode, One Shot Mode.  See 589 of the reference manual
//                    TIMER32_2->CONTROL = 0b01100011;
                }
            }
        }

        if(flag_wake_lights)
        {
            TIMER_A0->CCR[3] = 0;
            TIMER_A0->CCR[2] = 0;
            flag_wake_lights = 0;
        }
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

void SetupPort1Interrupts()
{
    P1->SEL0 &= ~(BIT1|BIT4);
    P1->SEL1 &= ~(BIT1|BIT4);
    P1->DIR &= ~(BIT1|BIT4);
    P1->OUT |= (BIT1|BIT4);
    P1->REN |= (BIT1|BIT4);
    P1->IES |= (BIT1|BIT4);                                //Set pin interrupt to trigger when it goes from high to low (starts high due to pull up resistor)
    P1->IE |= (BIT1|BIT4);                                 //Set interrupt on for P3.0

    P1->IFG = 0;                                    //Clear all interrupt flags
    NVIC_EnableIRQ(PORT1_IRQn);

}

void PORT3_IRQHandler(void)
{
    int status = P3 -> IFG;
    P3 -> IFG = 0;

    if(status & BIT0)
    {
        if(flag_check_hms || flag_check_hms_alarm)
        {
            flag_down = 1;
        }
        else
        {
            flag_snooze = 2;
            flag_alarm_no_snooze = 0;
        }
    }
}


void PORT1_IRQHandler(void)
{
    int status = P1 -> IFG;
    P1 -> IFG = 0;

    if(status & BIT1)
    {
            flag_faketime = 1;
            flag2 = 1;
            flag3 = 1;
    }
    if(status & BIT4)
    {
        flag_faketime = 0;
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



    //initializes P2.5 to be used for Timer A for Green LED
    P2->SEL0 |= BIT5; //set as Timer A
    P2->SEL1 &= ~(BIT5); //set as Timer A
    P2->DIR |= BIT5; //set as output
    P2->OUT &= ~(BIT5);

    //initializes P2.6 to be used for Timer A for Blue LED
    P2->SEL0 |= BIT6; //set as Timer A
    P2->SEL1 &= ~(BIT6); //set as Timer A
    P2->DIR |= BIT6; //set as output
    P2->OUT &= ~(BIT6);


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


    RTC_C->AMINHR   = hr_alarm<<8 | min_alarm | BIT(15) | BIT(7);
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
        flag_alarm_no_snooze = 1;
        RTC_C->CTL0 = 0xA500;
    }
    if(RTC_C->PS1CTL & BIT0)
    {

        if(flag_faketime == 1)
        {
            if(flag3 ==1)
            {
            sec1 = now.min;
            min1 = now.hour;

            configRTC();

            flag3 = 0;
            }

            now.sec         =   0;
            now.min         =   RTC_C->TIM0>>0 & 0x00FF;
            now.hour        =   RTC_C->TIM0>>8 & 0x00FF;

            RTC_flag = 1;
            RTC_C->PS1CTL &= ~BIT0;
        }
        else
        {
            if(flag2 == 1)
            {
            sec1 = 0;
            min1 = now.min;
            hr1 = now.hour;

            configRTC();

            flag2 = 0;
            }

            now.sec         =   RTC_C->TIM0>>0 & 0x00FF;
            now.min         =   RTC_C->TIM0>>8 & 0x00FF;
            now.hour        =   RTC_C->TIM1>>0 & 0x00FF;
            RTC_flag = 1;
            RTC_C->PS1CTL &= ~BIT0;
        }

    }
}

/*----------------------------------------------------------------
 * void SysTickInit(void)
 *
 * Description: Set SysTick interval.  Set CLK, IE, and Run
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/
void SysTickInit(void)
{
    SysTick->CTRL       =   0;
    SysTick->LOAD       =   3000000;        //Set interval for interrupt to occur at
    SysTick->VAL        =   0;                          //Reset value to zero
    SysTick->CTRL       =   0b101;                      //Set CLK, Set IE, Set Run
}
//
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





///*
// * Systick Interrupt for temperature sensor
// */
//void SysTick_Handler(void)
//{
//    ADC14->CTL0 |= ADC14_CTL0_SC; //start conversation
//}

void conversion(void)
{
    result = ADC14->MEM[0]; // get the value from the ADC
    nADC = (result * 3.3) / 16384; //Converts value to voltage
    nADC = nADC * 1000.0; //converts V to mV
    temp_C = (nADC/10); //converts mV to temperature reading in degrees Celsius
    temp_F = (temp_C * 1.8) + 32; //converts degrees Celsius to degrees Farenheit
    delay_ms(500);
    Print_Temp();
}
/*----------------------------------------------------------------
 * void Print_Temp(void)
 *
 * Description: Function will print "Current Temp. is" and
 * temperature value in degrees Farenheit to LCD.
 * Clock Source: SMCLK
 * Clock DIV:   32
 * Resolution: 10 bits
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/
void Print_Temp(void)
{
    int k;

    sprintf(temp_Farenheit , "%f" , temp_F);
   delay_ms(100);
    commandWrite(0xD0);
   delay_ms(100);
    for(k=0;k<4;k++)
    {
        dataWrite(temp_Farenheit[k]);
    }
    dataWrite(0b00100000);
    dataWrite('F');
}
/*----------------------------------------------------------------
 * void ADC14init(void)
 *
 * Description: Function will set up the ADC14 to run in single
 * measurement mode and to interrupt upon conversion.
 * Clock Source: SMCLK
 * Clock DIV:   32
 * Resolution: 10 bits
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/
void ADC14init(void)
{
    P4->SEL0            |=   BIT6;                      // Select ADC Operation
    P4->SEL1            |=   BIT6;                      // SEL = 11 sets to ADC operation

    ADC14->CTL0         =    0;                         // Disable ADC for setup

    // CTL0 Configuration
    // 31-30 = 10   to divide down input clock by 4X
    // 26    = 1    to sample only when told to
    // 21-19 = 100  for SMCLK
    // 11-8  = 0011 for 32 clk sample and hold time
    // 4     = 1    to turn on ADC
    ADC14->CTL0         =    0b10000100001000000000001100010000;

    ADC14->CTL1         =    0b110000;                  // Bits 5 and 4 = 11 to enable 14 bit conversion
    ADC14->MCTL[0]      =    7;                         // Default configuration for ADC Channel
    ADC14->IER0         |=   BIT0;                      // Interrupt on
    ADC14->CTL0         |=   0b10;                      // Enable Conversion
    NVIC->ISER[0]       |=   1<<ADC14_IRQn;             // Turn on ADC Interrupts in NVIC.  Equivalent to "NVIC_EnableIRQ(ADC14_IRQn);"

ADC14->CTL0 |= ADC14_CTL0_SC; //start conversation
}
/*----------------------------------------------------------------
 * void ADC14_IRQHandler(void)
 *
 * Description: Sets the Timer_A0 output to a period based on the
 * value of a potentiometer.  The period changes the frequency
 * which changes the pitch of a note.
 * Inputs: None
 * Outputs: None
----------------------------------------------------------------*/
void ADC14_IRQHandler(void)
{
    if(ADC14->IFGR0 & BIT0)                             // Table 20-14. ADC14IFGR0 Register Description of Reference Manual says interrupt flag will be at BIT0 for ADC14MEM0
    {
        // At 3MHz, 525 Hz is 3000000/525=5714
        // At 3MHz, 5000 Hz is 3000000/5000=600
        // An ADC value of 0 should result in CCR[0] set to 600
        // An ADC value of 16535 (2^14 for 14 bit ADC) should result in CCR[0] set to 5714
        conversion();
        ADC14->CLRIFGR0     &=  ~BIT0;                  // Clear MEM0 interrupt flag
    }
    ADC14->CLRIFGR1 &=  ~0b1111110;                 // Clear all IFGR1 Interrupts (Bits 6-1.  These could trigger an interrupt and we are checking them for now.)
}


void Alarm_Status(void)
{
    int i;

    char alarm_on[2] = "ON";
    char alarm_off[3] = "OFF";
    char alarm_snooze[6] = "SNOOZE";
    char alarm_status[7] = "ALARM: ";
    commandWrite(0x90);
    for(i=0;i<7;i++)
    {
        dataWrite(alarm_status[i]);
    }
    if(button_alarm == 1)
    {
        if((flag_snooze == 2))
        {
            for(i = 0; i<6; i++)
            {
                dataWrite(alarm_snooze[i]);
            }
        }
        else
        {
        for(i = 0; i<2 ; i++)
        {
            dataWrite(alarm_on[i]);
        }
        dataWrite(0b00100000);
        dataWrite(0b00100000);
        dataWrite(0b00100000);
        dataWrite(0b00100000);
        }
    }
    else
    {
        for(i = 0; i<3 ; i++)
        {
            dataWrite(alarm_off[i]);
        }
        dataWrite(0b00100000);
        dataWrite(0b00100000);
        dataWrite(0b00100000);
    }

}

void SetupTimer32s()
{
//    flag_snooze = 0;

    TIMER32_1->CONTROL = 0b11000011;                //Sets timer 1 for Enabled, Periodic, No Interrupt, No Prescaler, 32 bit mode, One Shot Mode.  See 589 of the reference manual
    TIMER32_2->CONTROL = 0b11100011;                //Sets timer 2 for Enabled, Periodic, With Interrupt, No Prescaler, 32 bit mode, One Shot Mode.  See 589 of the reference manual
    NVIC_EnableIRQ(T32_INT2_IRQn);                  //Enable Timer32_2 interrupt.  Look at msp.h if you want to see what all these are called.
    TIMER32_2->LOAD = 3000000 - 1;                  //Set to a count down of 1 second on 3 MHz clock

    TIMER_A0->CCR[0] = 0;                           // Turn off timerA to start
    TIMER_A0->CCTL[1] = 0b0000000011110100;         // Setup Timer A0_1 Reset/Set, Interrupt, No Output
    TIMER_A0->CCR[1] = 0;                           // Turn off timerA to start
//    TIMER_A0->CCTL[2] = 0b0000000011110100;         // Setup Timer A0_2 Reset/Set, Interrupt, No Output
//    TIMER_A0->CCR[2] = 0;                           // Turn off timerA to start
    TIMER_A0->CTL = 0b0000001000010100;             // Count Up mode using SMCLK, Clears, Clear Interrupt Flag

    NVIC_EnableIRQ(TA0_N_IRQn);                     // Enable interrupts for CCTL1-6 (if on)

    P2->SEL0 |= BIT4;                               // Setup the P2.4 to be an output for the song.  This should drive a sounder.
    P2->SEL1 &= ~BIT4;
    P2->DIR |= BIT4;

}

void TA0_N_IRQHandler()
{
    if(TIMER_A0->CCTL[1] & BIT0) {                  //If CCTL1 is the reason for the interrupt (BIT0 holds the flag)
    }
//    if(TIMER_A0->CCTL[2] & BIT0) {                  //If CCTL1 is the reason for the interrupt (BIT0 holds the flag)
//    }
}

void T32_INT2_IRQHandler()
{
    if((RTC_alarm == 1) && (button_alarm == 1) && (flag_snooze == 0))
    {
    TIMER32_2->INTCLR = 1;                                      //Clear interrupt flag so it does not interrupt again immediately.
                                                    //If not a breath (a note)
        TIMER32_2->LOAD = music_note_sequence[note][1] - 1;     //Load into interrupt count down the length of this note

        if(music_note_sequence[note][0] == REST) {              //If note is actually a rest, load in nothing to TimerA
            TIMER_A0->CCR[0] = 0;
            TIMER_A0->CCR[1] = 0;
//            TIMER_A0->CCR[2] = 0;
        }
        else {
            TIMER_A0->CCR[0] = 3000000 / music_note_sequence[note][0];  //Math in an interrupt is bad behavior, but shows how things are happening.  This takes our clock and divides by the frequency of this note to get the period.
            TIMER_A0->CCR[1] = 1500000 / music_note_sequence[note][0];  //50% duty cycle
//            TIMER_A0->CCR[2] = TIMER_A0->CCR[0];                        //Had this in here for fun with interrupts.  Not used right now
        }

        note = note + 1;                                                //Next note
        if(note >= MAX_NOTE) {                                          //Go back to the beginning if at the end
            note = 0;
        }


    }
    else                                                //disables interrupts
    {
        TIMER32_1->CONTROL = 0b0;               //Sets timer 1 for Enabled, Periodic, No Interrupt, No Prescaler, 32 bit mode, One Shot Mode.  See 589 of the reference manual
        TIMER32_2->CONTROL = 0b0;

        TIMER_A0->CCTL[1] = 0b0;         // Setup Timer A0_1 Reset/Set, Interrupt, No Output
    }
}

void TimerA_Init_BLUE(void)
{


    if(now.sec == sec_lights)
{
    percent = percent + 10;
    TIMER_A0->CCR[0] = 999;
    TIMER_A0->CCR[3] = percent;

    TIMER_A0->CCTL[3] = 0b0000000011100000;
    TIMER_A0->CTL = 0b0000001001010100;

    sec_lights = now.sec + 3;

}


}

void TimerA_Init_GREEN(void)
{
    TIMER_A0->CCR[0] = 999;
    TIMER_A0->CCR[2] = percent;

    TIMER_A0->CCTL[2] = 0b0000000011100000;
    TIMER_A0->CTL = 0b0000001001010100;
}
