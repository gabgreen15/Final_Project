#include "msp.h"
#include <stdio.h>
#include <string.h>

/**
 * Jenna Stolzman and Gabrielle Green
 * Final Project
 *
 * The purpose of this code is to create an alarm clock. The alarm clock
 * has 4 buttons that can be used to set the time, set the alarm,
 * enable/disable the alarm, and snooze the alarm. The alarm clock
 * displays the current time, alarm time, alarm status, and current
 * temperature. The alarm clock can also be set using serial commands.
 *
 * Credit given to Professor Zuidema and Professor Brakora for various functions
 * used in this program that were presented throughout the semester.
 */
//Initialize state machine
enum states{HOURS,MINUTES,SECONDS,HOURS_ALARM,MINUTES_ALARM,SECONDS_ALARM}; //state machine used to set the current time / alarm time

//Function used to initialize pins
void Initialize_Pins(void);

//Serial functions and variables
void setupSerial();
void writeOutput(char *string);
void readInput(char *string);
void EUSCIA0_IRQHandler(void);
char INPUT_BUFFER[BUFFER_SIZE];
uint16_t storage_location = 0;
uint16_t read_location = 0;
long int value;
#define BUFFER_SIZE 100

//SysTick functions
void SysTickInit(void);
void delay_micro(unsigned microsec);
void delay_ms(unsigned ms);

//Timer32 function
void Timer_32_init();
void SetupTimer32s();
#define C4 261.63
#define REST 0
#define WHOLE 3000000
#define MAX_NOTE 2
int note = 0;       //The note in the music sequence we are on
int breath = 0;
float music_note_sequence[][2] = {
                                  {C4,WHOLE},
                                  {REST,WHOLE},
                                 };

//LCD functions
void Initialize_LCD(void);
void PulseEnablePin(void);
void pushNibble(uint8_t nibble);
void pushByte(uint8_t byte);
void commandWrite(uint8_t command);
void dataWrite(uint8_t data);

//Initialize flag variables
volatile int flag,flag_up=0,flag_down = 0, flag_check_hms = 0, flag_check_hms_alarm = 0, flag_snooze=0, flag_disable_lights = 0;
volatile int flag_realtime=0,flag_faketime=0;
volatile int flag_alarm_no_snooze = 1;
volatile int flag_wake_lights = 0;
volatile int flag_LCD = 0;
volatile char current_day_status = 'A';
int flag_time = 0;
int button_alarm = 1;
int flag2 = 0;
int flag3 = 0;
uint8_t RTC_flag = 0, RTC_alarm;
volatile int flag_serial;
volatile int flag_first_alarm = 0;

//TimerA functions
void TimerA_Init_BLUE(void);
void TimerA_Init_GREEN(void);
void TimerA_Init_LCD(void);

//Initialize pin interrupts
void SetupPort5Interrupts(void);
void SetupPort1Interrupts();
void SetupPort3Interrupts(void);

//ADC functions, variables, and string
void ADC14init(void);
void ADC14init_LCD(void);
void conversion_lcd(void);
void conversion(void);
void Print_Temp(void);
volatile uint16_t result;
float nADC;
float temp_C;
float temp_F;
char temp_Farenheit[5];
#define ADC_CONVERSION_RATE 30000
volatile uint16_t result_lcd;
float nADC_lcd;

//Real Time Clock and Clock Display Functions
void configRTC(void);
void printRTC(void);
void printAlarm(void);
void printRTC_SetTime(void);
void Alarm_Status(void);

//initializing global variables to set time and alarm
uint8_t hr1 = 12, min1 = 34, sec1 = 55;
uint8_t hr_alarm = 12, min_alarm = 35;

// global struct variable called now
struct
{
    uint8_t sec;
    uint8_t min;
    uint8_t hour;
} now;

//Variables and arrays for serial commands
volatile int value_hr1 = 0, value_hr2 = 0, value_min1 = 0, value_min2 = 0, value_sec1 = 0, value_sec2 = 0;
volatile int value_hr1_A = 0, value_hr2_A = 0, value_min1_A = 0, value_min2_A = 0;
char hr1_array[1] , hr2_array[1], min1_array[1], min2_array[1], sec1_array[1], sec2_array[1];
char hr1_array_A[1], hr2_array_A[1], min1_array_A[1], min2_array_A[1];

/*
 * void main(void)
 *
 * The purpose of this function is to execute the main portion
 * of the code - this contains the state machine for setting the
 * time and the alarm as well as using serial commands
 */

void main(void)
{
    //Initialize strings to print to serial port
    char string[BUFFER_SIZE];
    char hr_string_print[3], min_string_print[3], sec_string_print[3];
    char colon_print[2] = ':';
    char zero_print[1] = '0';
    char hr_time_one[1], min_time_one[1], sec_time_one[1];
    char hr_time_A[3], min_time_A[3];

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
    __disable_irq();

    SetupPort5Interrupts();
    SetupPort1Interrupts();
    setupSerial();
    SetupPort3Interrupts();
    configRTC();
    NVIC_EnableIRQ(RTC_C_IRQn);
    INPUT_BUFFER[0] = '\0';

    __enable_interrupt();

    Initialize_Pins();
    SysTickInit();
    Initialize_LCD();
    Alarm_Status();
    printAlarm();
    printRTC_SetTime();
    ADC14init();
    delay_ms(500);

    while(1)
    {
        ADC14init_LCD();
        if(flag_serial == 1) //executes if button for serial commands has been pressed
        {
        readInput(string);
        if(string[0] != '\0')
        {
        if((string[0] == 'S') && (string[3] == 'T')) //if SETTIME command as input
        {
            //saves desired time as integers
            hr1_array[0] = string[8];
            hr2_array[0] = string[9];
            min1_array[0] = string[11];
            min2_array[0] = string[12];
            sec1_array[0] = string[14];
            sec2_array[0] = string[15];
            value_hr1 = atoi(&hr1_array[0]);
            value_hr2 = atoi(&hr2_array[0]);
            value_min1 = atoi(&min1_array[0]);
            value_min2 = atoi(&min2_array[0]);
            value_sec1 = atoi(&sec1_array[0]);
            value_sec2 = atoi(&sec2_array[0]);
            hr1 = (value_hr1 * 10) + value_hr2;
            min1 = (value_min1 * 10) + value_min2;
            sec1 = (value_sec1 * 10) + value_sec2;
            if(hr1 > 12)
            {
                hr1 = hr1 - 0b1100; //subtracts 12 from value of hr1 because it was input in military time
                flag_time = 1;      //changes to PM
            }
            configRTC();
            now.sec         =   RTC_C->TIM0>>0 & 0x00FF;
            now.min         =   RTC_C->TIM0>>8 & 0x00FF;
            now.hour        =   RTC_C->TIM1>>0 & 0x00FF;
            printRTC_SetTime();
            flag_serial = 0;
            flag_wake_lights = 1;
            flag_disable_lights = 0;
        }
        if((string[0] == 'S') && (string[3] == 'A'))    //if SETALARM command as input
        {
            //saves desired time as integers
            hr1_array_A[0] = string[9];
            hr2_array_A[0] = string[10];
            min1_array_A[0] = string[12];
            min2_array_A[0] = string[13];

            value_hr1_A = atoi(&hr1_array_A[0]);
            value_hr2_A = atoi(&hr2_array_A[0]);
            value_min1_A = atoi(&min1_array_A[0]);
            value_min2_A = atoi(&min2_array_A[0]);

            hr_alarm = (value_hr1_A * 10) + value_hr2_A;
            min_alarm = (value_min1_A * 10) + value_min2_A;
            if(hr_alarm > 12)
            {
                hr_alarm = hr_alarm - 0b1100;   //subtracts 12 from value of hr1 because it was input in military time
                flag_time = 1;                  //changes to PM
            }
            configRTC();
            printAlarm();
            flag_serial = 0;
            flag_wake_lights = 1;
            flag_disable_lights = 0;
        }
        if((string[0] == 'R') && (string[4] == 'T'))    //if READTIME command as input
        {
            if(flag_time == 1)
            {
                now.hour = now.hour + 0b1100;           //Adds 12 to convert back to military time
            }
                sprintf(hr_string_print, "%02d", now.hour);     //Prints hour
                writeOutput(hr_string_print);
                writeOutput(colon_print);

                sprintf(min_string_print, "%02d", now.min);     //Prints minute
                writeOutput(min_string_print);
                writeOutput(colon_print);

                sprintf(sec_string_print, "%02d", now.sec);     //Prints second
                writeOutput(sec_string_print);

            flag_serial = 0;
            flag_wake_lights = 1;
            flag_disable_lights = 0;
        }
        if((string[0] == 'R') && (string[4] == 'A'))    //if READALARM command as input
        {
            if(flag_time == 1)
            {
                hr_alarm = hr_alarm + 0b1100;           //Adds 12 to convert back to military time
            }
                sprintf(hr_time_A, "%02d", hr_alarm);   //Prints hour of alarm
                writeOutput(hr_time_A);
                writeOutput(colon_print);

                sprintf(min_time_A, "%02d", min_alarm); //Prints minute of alarm
                writeOutput(min_time_A);

            flag_serial = 0;
            flag_wake_lights = 1;
            flag_disable_lights = 0;
        }
       }
      }
      if((hr_alarm == now.hour) && (now.min < min_alarm) && (now.min >= (min_alarm - 5)) && (button_alarm == 1) && (flag_disable_lights == 0))  //Alarm is enabled and current time is equal to or less than 5 minutes from alarm time
        {
            flag_wake_lights = 1;   //Turn on wake up lights
            TimerA_Init_BLUE();     //PWM for blue LED
            TimerA_Init_GREEN();    //PWM for green LED
        }

        if(RTC_flag)            //If time set to update
        {
            printRTC();         //Prints to LCD screen
            RTC_flag = 0;
        }
        if(RTC_alarm && button_alarm)   //If alarm enabled and going off
        {
            if(flag_alarm_no_snooze == 0)   //If snooze button pressed
            {
                Alarm_Status();
                min_alarm = min_alarm + 10;     //Increments alarm time by 10 minutes
                hr1 = now.hour;
                min1 = now.min;
                sec1 = now.sec;
                configRTC();
                printAlarm();
                RTC_alarm = 0;                  //Turns off alarm currently going off
                TIMER32_1->CONTROL = 0x0;       //Disables TIMER32 interrupt
                TIMER32_2->CONTROL = 0x0;
            }
            if(flag_alarm_no_snooze == 1)       //If snooze not pressed
            {
                Alarm_Status();
                if(flag_first_alarm)            //If first time program goes into the interrupt (used to TIMER32 only initializes once)
                {
                    SetupTimer32s();            //Initialize TIMER32
                    flag_first_alarm = 0;
                }
            }
        }
        else
        {
            Alarm_Status();                 //Prints status of alarm
        }
       enum states state = HOURS;       //Default state to HOURS
       while(flag_check_hms || flag_check_hms_alarm)    //While either SET TIME or SET ALARM buttons have been pressed
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
            if(flag_up == 1)        //If UP button pressed
            {
                hr1 = hr1 + 0b1;    //Increment current hour
                if(hr1 == 12)       //Roll over from AM to PM or vice versa
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
            if(flag_down == 1)      //If DOWN button pressed
            {
                hr1 = hr1 - 0b1;    //Decrements current hour
                if(hr1 == 11)       //Roll over from AM to PM or vice versa
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
                min1 = min1 + 0b1;      //Increment current minute
                configRTC();
                printRTC_SetTime();
                flag_up = 0;
            }
            if(flag_down == 1)
            {
                if(min1 == 0)       //Causes minute to roll back to 59 once it reaches 0
                {
                    min1 = 59;
                }
                else
                {
                    min1 = min1 - 0b1;  //Decrement current minute
                }
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
                sec1 = sec1 + 0b1;      //Increment current second
                configRTC();
                printRTC_SetTime();
                flag_up = 0;
            }
            if(flag_down == 1)
            {
                if(sec1 == 0)       //Cause second to roll back to 59 if at 0
                {
                    sec1 = 59;
                }
                else
                {
                 sec1 = sec1 - 0b1;     //Decrement current second
                }
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
                hr_alarm = hr_alarm + 0b1;      //Increment hour for alarm
                if(hr_alarm==12)
                {
                    if(flag_time ==0)       //Change from AM to PM and vice versa
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
                hr_alarm = hr_alarm - 0b1;      //Decrement hour for alarm
                if(hr_alarm == 0)       //Cause time to rollover
                {
                    hr_alarm = 12;
                }
                if(hr_alarm==11)
                {
                    if(flag_time ==0)       //Change from AM to PM and vice versa
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
                min_alarm = min_alarm + 0b1;        //Increments minute for alarm
                configRTC();
                printAlarm();
                flag_up = 0;
            }
            if(flag_down == 1)
            {
                if(min_alarm == 0)      //Cause minute to roll back to 59 if at 0
                {
                   min_alarm = 59;
                }
                else
                {
                    min_alarm = min_alarm - 0b1;        //Decrement minute for alarm
                }
                 configRTC();
                 printAlarm();
                 flag_down = 0;
             }
            break;
            }
        }
    }
}
/*
 * void printAlarm(void)
 *
 * The purpose of this function is to print the current alarm time that is set
 * in the RTC to the LCD screen.
 *
 */
void printAlarm(void)
{
    char minute_alarm[2];
    char hour_alarm[2];
    int j= 0, i = 0;

    sprintf(minute_alarm,"%d",min_alarm);   //save value of integer 'min_alarm' into string
    sprintf(hour_alarm,"%d",hr_alarm);      //save value of integer 'hr_alarm' into string

    if(hr_alarm > 12)
    {
        hr_alarm = hr_alarm - 0b1100;       //Subtract 12 from 'hr_alarm' if greater than 12
    }
    if(hr_alarm == 0)
    {
        hr_alarm = 12;                  //Causes value to roll back to 12 if at 0
    }
    if(min_alarm > 59)                  //Causes value to rollover to 0 if greater than 59
    {
        min_alarm = 0;
    }
    configRTC();
    commandWrite(0xC0);
    if(hr_alarm>=10)
    {
        for(i=0;i<2;i++)
        {
            dataWrite(hour_alarm[i]);   //Prints value of 'hr_alarm' to LCD screen
        }
            j=1;                        //hr_alarm is 2 digits, so causes address of each to shift to right by 1
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
        dataWrite('0');             //Prints a '0' first if 'min_alarm' is only one digit
        dataWrite(minute_alarm[0]); //Prints value of 'min_alarm' to LCD screen
    }
    else
    {
        for(i=0;i<2;i++)
        {
            dataWrite(minute_alarm[i]);
        }
    }
    commandWrite(0xC4+j);
    if(flag_time==0)                //condition to check whether in AM or PM
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
/*
 * void PORT5_IRQHandler(void)
 *
 * This function is the interrupt handler for all interrupts occurring on P5. The purpose of this function is
 * to check each bit and carry out the specific action for that interrupt.
 *
 */
void PORT5_IRQHandler(void)
{
    int status = P5 -> IFG;     //Saves values of each bit
    P5 -> IFG = 0;              //Clears flag
    if(status & BIT1)
    {
        if(flag_check_hms == 3)
        {
            flag_check_hms = 0;     //Gets out of SET TIME state
        }
        else
        {
            flag_check_hms++;      //Increments SET TIME flag to switch from hours to minutes to seconds
        }
    }
    if(status & BIT2)
    {
        if(flag_check_hms_alarm == 2)
        {
            flag_check_hms_alarm = 0;   //Gets out of SET ALARM state
        }
        else
        {
            flag_check_hms_alarm++;     //Increments SET ALARM flag to switch from hours to minutes
        }
    }
    if(status & BIT0)
    {
        if(flag_check_hms || flag_check_hms_alarm)
        {
            flag_up = 1;            //if in SET TIME or SET ALARM, used to increment current time
        }
        else
        {
            if(button_alarm == 0)
            {
                button_alarm = 1;       //Turns alarm status ON
                flag_wake_lights = 1;   //Turns on wake lights
                flag_disable_lights = 0;
            }
            else
            {
                button_alarm = 0;       //Turns alarm status OFF
                flag_snooze = 0;
                if(RTC_alarm == 1)      //Turns alarm off
                {
                    RTC_alarm = 0;
                }
            }
        if((flag_wake_lights == 1))
        {
            flag_disable_lights = 1;    //Turns off wake lights
            button_alarm = 1;           //Keeps alarm enabled
            TIMER_A0->CCR[1] = 0;
            TIMER_A0->CCR[2] = 0;
            flag_wake_lights = 0;
        }
    }
    }
}
/*
 * void SetupPort5Interrupts()
 *
 * The purpose of this function is to setup the pins on Port 5 that have interrupts.
 *
 */
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
/*
 * void SetupPort3Interrupts()
 *
 * The purpose of this function is to setup the pins on Port 3 that have interrupts.
 *
 */
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
    /*
     * Serial Enable Button
     */
    P3->SEL0 &= ~BIT3;                              // Setup the P3.3 on the Launchpad as Input, Pull Up Resistor
    P3->SEL1 &= ~BIT3;
    P3->DIR &= ~BIT3;
    P3->REN |= BIT3;
    P3->OUT |= BIT3;
    P3->IES |= BIT3;                                //Set pin interrupt to trigger when it goes from high to low (starts high due to pull up resistor)
    P3->IE |= BIT3;                                 //Set interrupt on for P3.3
    P3->IFG = 0;                                    //Clear all interrupt flags
    NVIC_EnableIRQ(PORT3_IRQn);                     //Enable Port 3 interrupts.  Look at msp.h if you want to see what all these are called.
}
/*
 * void SetupPort1Interrupts()
 *
 * The purpose of this function is to setup the pins on Port 1 that have interrupts.
 *
 */
void SetupPort1Interrupts()
{
    /*
     * Buttons on MSP to switch between real time and fast time
     */
    P1->SEL0 &= ~(BIT1|BIT4);                               // Setup the P1.1 and P1.4 on the Launchpad as Input, Pull Up Resistor
    P1->SEL1 &= ~(BIT1|BIT4);
    P1->DIR &= ~(BIT1|BIT4);
    P1->OUT |= (BIT1|BIT4);
    P1->REN |= (BIT1|BIT4);
    P1->IES |= (BIT1|BIT4);                                //Set pin interrupts to trigger when it goes from high to low (starts high due to pull up resistor)
    P1->IE |= (BIT1|BIT4);                                 //Set interrupts on for P1.1 and P1.4
    P1->IFG = 0;                                          //Clear all interrupt flags
    NVIC_EnableIRQ(PORT1_IRQn);
}
/*
 * void PORT3_IRQHandler(void)
 *
 * This function is the interrupt handler for all interrupts occurring on P3. The purpose of this function is
 * to check each bit and carry out the specific action for that interrupt.
 *
 */
void PORT3_IRQHandler(void)
{
    int status = P3 -> IFG;         //Saves value of each bit
    P3 -> IFG = 0;                  //Clears flag
    if(status & BIT0)
    {
        if(flag_check_hms || flag_check_hms_alarm)
        {
            flag_down = 1;          //if in SET TIME or SET ALARM, used to decrement current time
        }
        else
        {
            flag_snooze = 2;            //Turn on SNOOZE
            flag_alarm_no_snooze = 0;
            flag_wake_lights = 1;       //Keep wake lights on
            flag_disable_lights = 0;
            music_note_sequence[note][0] == REST;       //Keep alarm silent
        }
    }
    if(status & BIT3)
    {
       flag_serial = 1;             //Enables the use of serial commands
    }
}
/*
 * void PORT1_IRQHandler(void)
 *
 * This function is the interrupt handler for all interrupts occurring on P1. The purpose of this function is
 * to check each bit and carry out the specific action for that interrupt.
 *
 */
void PORT1_IRQHandler(void)
{
    int status = P1 -> IFG;     //Saves values of each bit
    P1 -> IFG = 0;              //Clears interrupt flag
    if(status & BIT1)
    {
            flag_faketime = 1;        //Turns on flag for fast time
            flag2 = 1;
            flag3 = 1;
    }
    if(status & BIT4)
    {
        flag_faketime = 0;          //Turns off flag for fast time, returns to real time
    }
}
/*
 * void Initialize_Pins(void)
 *
 * The purpose of this function is to initialize pins for the LCD screen
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
    //initializes P2.4 to be used for Timer A for Blue LED
    P2->SEL0 |= BIT4; //set as Timer A
    P2->SEL1 &= ~(BIT4); //set as Timer A
    P2->DIR |= BIT4; //set as output
    P2->OUT &= ~(BIT4);
    //initializes P2.5 to be used for Timer A for Green LED
    P2->SEL0 |= BIT5; //set as Timer A
    P2->SEL1 &= ~(BIT5); //set as Timer A
    P2->DIR |= BIT5; //set as output
    P2->OUT &= ~(BIT5);
    //Set for PWM for backlight for LCD screen
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
    SysTick->LOAD = ((microsec*3)-1);       //Sets LOAD value to desired number of microseconds
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
    SysTick->LOAD = ((ms*3000)-1);      //Sets LOAD value to desired number of milliseconds
    SysTick->VAL = 0;
    while((SysTick->CTRL & 0x00010000) == 0); //Bit 16 means complete
}
/*
 * void configRTC(void)
 *
 * The purpose of this function is to configure the Real Time Clock using
 * global variables 'hr1', 'min1', 'sec1', 'hr_alarm', 'min_alarm'.
 */
void configRTC(void)
{
    RTC_C->CTL0     =   0xA500;             //Write Code, IE on RTC Ready
    RTC_C->CTL13    =   0x0000;
    RTC_C->TIM0     = min1<<8 | sec1;       //sets minutes,seconds
    RTC_C->TIM1     = 2<<8 | hr1;           //sets DOW, hour
    RTC_C->PS1CTL   = 0b11010;
    RTC_C->AMINHR   = hr_alarm<<8 | min_alarm | BIT(15) | BIT(7); //sets alarm
    RTC_C->ADOWDAY = 0;
    RTC_C->CTL0     = ((0xA500) | BIT5);
}
/*
 * void printRTC(void)
 *
 * The purpose of this function is to print the current time
 * to the LCD screen.
 */
void printRTC(void)
{
    int i;
    int j=0;
    char hourStr[2];            //character string for current hour
    char minStr[2];             //character string for current minute
    char secStr[2];             //character string for current second
    if(now.hour > 12)
    {
        now.hour = now.hour - 0b1100;   //subtracts 12 from current hour if greater than 12
    }
    if(now.min > 59)
    {
        now.min = now.min - 0b111100; //subtracts 60 from current minute if greater than 59
    }
    if(now.sec > 59)
    {
        now.sec = now.sec - 0b111100; //subtracts 60 from current second if greater than 59
    }
    sprintf(hourStr, "%d", now.hour);       //Copies now.hour into character string
    sprintf(minStr, "%d", now.min);         //Copies now.min into character string
    sprintf(secStr, "%d", now.sec);         //Copies now.sec into character string
    commandWrite(0x80);
    if(now.hour>=10)
    {
        for(i=0;i<2;i++)
        {
            dataWrite(hourStr[i]);      //Prints current hour to LCD screen
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
        dataWrite('0');             //If now.min is less than 10, prints a '0' before digit
        dataWrite(minStr[0]);
        dataWrite(0b00111010);
    }
    else
    {
        for(i=0;i<2;i++)
        {
            dataWrite(minStr[i]);       //Prints current minute to LCD screen
        }
        dataWrite(0b00111010);
    }
    commandWrite(0x85+j);
    if(now.sec<10)
    {
        dataWrite('0');         //If now.sec is less than 10, prints a '0' before digit
        dataWrite(secStr[0]);
    }
    else
    {
        for(i=0;i<2;i++)
        {
            dataWrite(secStr[i]);       //Prints current second to LCD screen
        }
    }
    commandWrite(0x87+j);
    if(flag_time==0)        //Condition to determine AM or PM and prints accordingly
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
/*
 * void printRTC_SetTime(void)
 *
 * The purpose of this function is to print the time that is currently
 * being set to LCD screen using global variables.
 */
void printRTC_SetTime(void)
{
    int i;
    int j=0;
    char hourStr[2];            //character string for current hour
    char minStr[2];             //character string for current minute
    char secStr[2];             //character string for current second
    if(hr1 > 12)
    {
        hr1 = hr1 - 0b1100;     //subtracts 12 from hour if hour being set is greater than 12
    }
    if(min1 > 59)
    {
        min1 = 0;               //if minute being set is greater than 59, resets it back to 0
    }
    if(sec1 > 59)
    {
        sec1 = sec1 - 0b111100; //subtracts 60 from second if second being set is greater than 60
    }
    if(hr1 == 0)            //If hour is equal to 0, rolls back to 12
    {
        hr1 = 12;
    }
    configRTC();            //configures RTC with new variables
    sprintf(hourStr, "%d", hr1);        //Copies hr1 into character string
    sprintf(minStr, "%d", min1);        //Copies min1 into character string
    sprintf(secStr, "%d", sec1);        //Copies sec1 into character string
    commandWrite(0x80);
    if(hr1>=10)
    {
        for(i=0;i<2;i++)
        {
            dataWrite(hourStr[i]);       //Prints hour to LCD screen
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
        dataWrite('0');                 //If min1 is less than 10, prints a '0' before digit
        dataWrite(minStr[0]);
        dataWrite(0b00111010);
    }
    else
    {
        for(i=0;i<2;i++)
        {
            dataWrite(minStr[i]);           //Prints minute to LCD screen
        }
        dataWrite(0b00111010);
    }
    commandWrite(0x85+j);
    if(sec1<10)
    {
        dataWrite('0');                 //If sec1 is less than 10, prints a '0' before digit
        dataWrite(secStr[0]);
    }
    else
    {
        for(i=0;i<2;i++)
        {
            dataWrite(secStr[i]);       //Prints second to LCD screen
        }
    }
    commandWrite(0x87+j);
    if(flag_time == 0)              //Condition to check AM or PM, prints accordingly
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
/*
 * void RTC_C_IRQHandler(void)
 *
 * This function is the interrupt handler for the RTC. The purpose of this function is to
 * check which interrupt is occurring and complete the action for that specific interrupt.
 */
void RTC_C_IRQHandler(void)
{
    if(RTC_C->CTL0 & BIT1)          //If alarm time matches current time
    {
        RTC_alarm = 1;              //Sets alarm flag to 1
        flag_alarm_no_snooze = 1;
        flag_first_alarm = 1;
        TimerA_Init_LCD();          //Sets LCD screen to full brightness
        RTC_C->CTL0 = 0xA500;
    }
    if(RTC_C->PS1CTL & BIT0)        //If one second has passed and time to update clock
    {
        if(flag_faketime == 1)      //If button has been pressed to change to fast time
        {
            if(flag3 ==1)       //Sets current variables to fast time variables
            {
                sec1 = now.min;
                min1 = now.hour;
                configRTC();
                flag3 = 0;
            }
            now.sec         =   0;                              //Current second set to 0
            now.min         =   RTC_C->TIM0>>0 & 0x00FF;        //Current minute updates as if current second
            now.hour        =   RTC_C->TIM0>>8 & 0x00FF;        //Current hour updates as if current minute

            RTC_flag = 1;
            RTC_C->PS1CTL &= ~BIT0;
        }
        else
        {
            if(flag2 == 1)      //Sets current variables to real time variables
            {
                sec1 = 0;
                min1 = now.min;
                hr1 = now.hour;
                configRTC();
                flag2 = 0;
            }
            now.sec         =   RTC_C->TIM0>>0 & 0x00FF;        //Current second set back to update as if current second
            now.min         =   RTC_C->TIM0>>8 & 0x00FF;        //Current minute set back to update as if current minute
            now.hour        =   RTC_C->TIM1>>0 & 0x00FF;        //Current hour set back to update as if current hour

            RTC_flag = 1;
            RTC_C->PS1CTL &= ~BIT0;
        }
    }
}
/*
 * void SysTickInit(void)
 *
 * The purpose of this function is to initialize the SysTick timer
 */
void SysTickInit(void)
{
    SysTick->CTRL       =   0;
    SysTick->LOAD       =   3000000;                    //Set interval for interrupt to occur at
    SysTick->VAL        =   0;                          //Reset value to zero
    SysTick->CTRL       =   0b101;                      //Set CLK, Set IE, Set Run
}
/*
 * void conversion(void)
 *
 * The purpose of this function is to complete the conversion
 * for the analog input from the temperature sensor.
 */
void conversion(void)
{
    result = ADC14->MEM[0];         // get the value from the ADC
    nADC = (result * 3.3) / 16384;  //Converts value to voltage
    nADC = nADC * 1000.0;           //converts V to mV
    temp_C = (nADC/10);             //converts mV to temperature reading in degrees Celsius
    temp_F = (temp_C * 1.8) + 32;   //converts degrees Celsius to degrees Farenheit
    delay_ms(500);
    Print_Temp();                   //Print temperature to LCD screen
}
/*
 * void Print_Temp(void)
 *
 * The purpose of this function is to print the current temperature value
 * to the LCD screen.
 */
void Print_Temp(void)
{
    int k;
    sprintf(temp_Farenheit , "%f" , temp_F);            //Copy temperature into character string
    delay_ms(100);
    commandWrite(0xD0);
    delay_ms(100);
    for(k=0;k<4;k++)
    {
        dataWrite(temp_Farenheit[k]);                   //Write character string to LCD screen
    }
    dataWrite(0b00100000);
    dataWrite('F');
}
/*
 * void ADC14init(void)
 *
 * The purpose of this function is to set up the ADC14 to run in single
 * measurement mode and to interrupt upon conversion.
 */
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
    ADC14->CTL0 |= ADC14_CTL0_SC;                       //start conversion
}
/*
 * void ADC14init_LCD(void)
 *
 * The purpose of this function is to set up the ADC14 for the LCD backlight to run in single
 * measurement mode and to interrupt upon conversion.
 */
void ADC14init_LCD(void)
{
    P4->SEL0            |=   BIT7;                      // Select ADC Operation
    P4->SEL1            |=   BIT7;                      // SEL = 11 sets to ADC operation
    ADC14->CTL0         =    0;                         // Disable ADC for setup
    // CTL0 Configuration
    // 31-30 = 10   to divide down input clock by 4X
    // 26    = 1    to sample only when told to
    // 21-19 = 100  for SMCLK
    // 11-8  = 0011 for 32 clk sample and hold time
    // 4     = 1    to turn on ADC
    ADC14->CTL0         =    0b10000100001000000000001100010000;
    ADC14->CTL1         =    0b110000;                  // Bits 5 and 4 = 11 to enable 14 bit conversion
    ADC14->MCTL[0]      =    6;                         // Default configuration for ADC Channel
    ADC14->IER0         |=   BIT0;                      // Interrupt on
    ADC14->CTL0         |=   0b10;                      // Enable Conversion
    NVIC->ISER[0]       |=   1<<ADC14_IRQn;             // Turn on ADC Interrupts in NVIC.  Equivalent to "NVIC_EnableIRQ(ADC14_IRQn);"
    ADC14->CTL0 |= ADC14_CTL0_SC;                       //start conversion
    flag_LCD = 1;
}
/*
 * void ADC14_IRQHandler(void)
 *
 * This is the interrupt handler for ADC14. The purpose of this function is to
 * begin the conversion when the interrupt occurs.
 */
void ADC14_IRQHandler(void)
{
    if(ADC14->IFGR0 & BIT0)                             // Table 20-14. ADC14IFGR0 Register Description of Reference Manual says interrupt flag will be at BIT0 for ADC14MEM0
    {
        // At 3MHz, 525 Hz is 3000000/525=5714
        // At 3MHz, 5000 Hz is 3000000/5000=600
        // An ADC value of 0 should result in CCR[0] set to 600
        // An ADC value of 16535 (2^14 for 14 bit ADC) should result in CCR[0] set to 5714
        if(flag_LCD == 1)
        {
            conversion_lcd();                       //Conversion for LCD backlight
            flag_LCD = 0;
        }
        else
        {
            conversion();                           //Conversion for temperature sensor
        }
        ADC14->CLRIFGR0     &=  ~BIT0;                  // Clear MEM0 interrupt flag
    }
    ADC14->CLRIFGR1 &=  ~0b1111110;                 // Clear all IFGR1 Interrupts (Bits 6-1.  These could trigger an interrupt and we are checking them for now.)
}
/*
 * void conversion_lcd(void)
 *
 * The purpose of this function is to complete the conversion
 * for the analog input from the potentiometer to a voltage
 * value to be used to set the PWM for the LCD backlight.
 */
void conversion_lcd(void)
{
    result_lcd = ADC14->MEM[0];             // get the value from the ADC
    nADC_lcd = (result_lcd * 3.3) / 16384; //Converts value to voltage
    delay_ms(500);
    TimerA_Init_LCD();                      //Calls TimerA function for LCD backlight
}
/*
 * void Alarm_Status(void)
 *
 * The purpose of this function is to print the current alarm status to the LCD screen.
 */
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
        dataWrite(alarm_status[i]);     //Writes "ALARM: " to screen
    }
    if(button_alarm == 1)               //if alarm enabled
    {
        if((flag_snooze == 2))          //if SNOOZE on
        {
            for(i = 0; i<6; i++)
            {
                dataWrite(alarm_snooze[i]); //Write "SNOOZE" to LCD screen
            }
        }
        else
        {
            for(i = 0; i<2 ; i++)
            {
                dataWrite(alarm_on[i]);     //Write "ON" to LCD screen
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
            dataWrite(alarm_off[i]);        //Write "OFF" to LCD screen
        }
        dataWrite(0b00100000);
        dataWrite(0b00100000);
        dataWrite(0b00100000);
    }
}
/*
 * void SetupTimer32s()
 *
 * The purpose of this function is to setup and initialize Timer32.
 */
void SetupTimer32s()
{
    TIMER32_2->CONTROL = 0b11100011;                //Sets timer 2 for Enabled, Periodic, With Interrupt, No Prescaler, 32 bit mode, One Shot Mode.  See 589 of the reference manual
    NVIC_EnableIRQ(T32_INT2_IRQn);                  //Enable Timer32_2 interrupt.  Look at msp.h if you want to see what all these are called.
    TIMER32_2->LOAD = 3000000 - 1;                  //Set to a count down of 1 second on 3 MHz clock
    TIMER_A2->CCR[0] = 0;                           // Turn off timerA to start
    TIMER_A2->CCTL[3] = 0b0000000011100100;         // Setup Timer A2_3
    TIMER_A2->CCR[3] = 0;                           // Turn off timerA to start
    TIMER_A2->CTL = 0b0000001000010100;             // Count Up mode using SMCLK, Clears, Clear Interrupt Flag
    P6->SEL0 |= BIT6;                               // Setup the P2.6 to be an output for the alarm.
    P6->SEL1 &= ~BIT6;
    P6->DIR |= BIT6;
}
/*
 * void SetupTimer32s()
 *
 * This is the interrupt handler for Timer32. The purpose of this function is to cause the alarm to sound
 * when the alarm is enabled and the alarm time is equal to the current time.
 */
void T32_INT2_IRQHandler()
{
    if((RTC_alarm == 1) && (button_alarm == 1) && ((flag_snooze == 2) || (flag_alarm_no_snooze == 1)) && (flag_disable_lights == 0))    //Checks conditions for alarm to sound
    {
    TIMER32_2->INTCLR = 1;                                      //Clear interrupt flag so it does not interrupt again immediately.
    TIMER32_2->LOAD = music_note_sequence[note][1] - 1;     //Load into interrupt count down the length of this note
    if(music_note_sequence[note][0] == REST) {              //If note is actually a rest, load in nothing to TimerA
            TIMER_A2->CCR[0] = 0;
            TIMER_A2->CCR[3] = 0;
        }
        else {
            TIMER_A2->CCR[0] = 3000000 / music_note_sequence[note][0];  //This takes our clock and divides by the frequency of this note to get the period.
            TIMER_A2->CCR[3] = 1500000 / music_note_sequence[note][0];  //50% duty cycle
        }
        note = note + 1;                                                //Next note
        if(note >= MAX_NOTE) {                                          //Go back to the beginning if at the end
            note = 0;
        }
    }
    else
    {
        TIMER_A2->CCR[0] = 0;               //Sets period to 0
        TIMER_A2->CCR[3] = 0;               //Sets duty cycle to 0
        TIMER32_2->CONTROL = 0b0;           //Turns off Timer32 (disables interrupt)
        TIMER_A2->CCTL[3] = 0b0;            //Turns off TimerA2_3
    }
}
/*
 * void TimerA_Init_BLUE(void)
 *
 * The purpose of this function is to initialize the TimerA used for the blue LED wake up light.
 */
void TimerA_Init_BLUE(void)
{
    float blue_dutycycle = 0;                                                    //float variable used in calculation of LED duty cycle
    int blue_dutycycle1 = 0;                                                     //integer variable used to set duty cycle in TimerA register
    blue_dutycycle = ((10/3)*now.sec)+((5-(min_alarm-now.min))*60*(10/3));       //Calculation done to set current duty cycle for wake up light blue LED
    blue_dutycycle1 = blue_dutycycle;
    if((blue_dutycycle1 > 0.0) && (blue_dutycycle1 <= 1000))                    //condition reduces flickering due to PWM
    {
    TIMER_A0->CCR[0] = 999;                     //Sets period of TimerA
    TIMER_A0->CCR[1] = blue_dutycycle1;         //Sets duty cycle
    TIMER_A0->CCTL[1] = 0b0000000011100000;
    TIMER_A0->CTL = 0b0000001001010100;
    }
    else
    {
        TIMER_A0->CCR[1] = 999;
    }
}
/*
 * void TimerA_Init_GREEN(void)
 *
 * The purpose of this function is to initialize the TimerA used for the green LED wake up light.
 */
void TimerA_Init_GREEN(void)
{
    float green_dutycycle = 0;                                              //float variable used in calculation of LED duty cycle
    int green_dutycycle1 = 0;                                               //integer variable used to set duty cycle in TimerA register
    green_dutycycle = ((10/3)*now.sec)+((5-(min_alarm-now.min))*60*(10/3)); //Calculation done to set current duty cycle for wake up light green LED
    green_dutycycle1 = green_dutycycle;
    if((green_dutycycle1 > 0.0) && (green_dutycycle1 <= 1000))      //condition reduces flickering due to PWM
    {
    TIMER_A0->CCR[0] = 999;                     //Sets period of TimerA
    TIMER_A0->CCR[2] = green_dutycycle1;        //Sets duty cycle
    TIMER_A0->CCTL[2] = 0b0000000011100000;
    TIMER_A0->CTL = 0b0000001001010100;
    }
    else
    {
        TIMER_A0->CCR[2] = 999;
    }
}
/*
 * void TimerA_Init_LCD(void)
 *
 * The purpose of this function is to initialize the TimerA used for PWM to control
 * the brightness of the backlight of the LCD screen.
 */
void TimerA_Init_LCD(void)
{
    float LCD_dutycycle = 0;                        //float variable used in calculation of LCD brightness duty cycle
    int LCD_dutycycle1 = 0;                         //integer variable used to set duty cycle in TimerA register
    LCD_dutycycle = (nADC_lcd/3.3) * 999;           //Percentage found using voltage from potentiometer, multiplied by period to get LCD screen brightness duty cycle
    LCD_dutycycle1 = LCD_dutycycle;
    if((RTC_alarm == 1) && (button_alarm == 1) && (flag_snooze == 0))   //If alarm enabled and going off
    {
        TIMER_A0->CCR[3] = 999;     //Brightness set to full brightness
    }
    else
    {
        if((LCD_dutycycle1 > 0.0) && (LCD_dutycycle1 <= 1000))      //condition reduces flickering due to PWM
        {
        TIMER_A0->CCR[0] = 999;                       //Sets period of TimerA
        TIMER_A0->CCR[3] = LCD_dutycycle1;            //Sets duty cycle
        TIMER_A0->CCTL[3] = 0b0000000011100000;
        TIMER_A0->CTL = 0b0000001001010100;
        }
        else
        {
            TIMER_A0->CCR[3] = 999;
        }
    }
}
/*
* void EUSCIA0_IRQHandler(void)
*
* This is the interrupt handler for serial communication on EUSCIA0.
* The purpose of this function is to store the data in the RXBUF into the INPUT_BUFFER global character
* array for reading in the main application
*/
void EUSCIA0_IRQHandler(void)
{
if (EUSCI_A0->IFG & BIT0)                               // Interrupt on the receive line
{
    INPUT_BUFFER[storage_location] = EUSCI_A0->RXBUF;   // store the new piece of data at the present location in the buffer
    EUSCI_A0->IFG &= ~BIT0;                             // Clear the interrupt flag right away in case new data is ready
    storage_location++;                                 // update to the next position in the buffer
    if(storage_location == BUFFER_SIZE)                 // if the end of the buffer was reached, loop back to the start
        storage_location = 0;
}
}
/*
* void readInput(char *string)
*
* The purpose of this function is to read in an input written to the serial port
* and save it in a character string. It also updates the global variables of locations in the
* INPUT_BUFFER that have been already read.
*/
void readInput(char *string)
{
int i = 0;        // Location in the char array "string" that is being written to
do
{
// If a new line hasn't been found yet, but we are caught up to what has been received, wait here for new data
   while(read_location == storage_location && INPUT_BUFFER[read_location] != '\n');
    string[i] = INPUT_BUFFER[read_location];    // Manual copy of valid character into "string"
    INPUT_BUFFER[read_location] = '\0';
    i++;                                        // Increment the location in "string" for next piece of data
    read_location++;                            // Increment location in INPUT_BUFFER that has been read
    if(read_location == BUFFER_SIZE)            // If the end of INPUT_BUFFER has been reached, loop back to 0
    read_location = 0;
}
while(string[i-1] != '\n');                     // If a \n was just read, break out of the while loop
string[i-1] = '\0';                             // Replace the \n with a \0 to end the string when returning this function
}
/*
* void writeOutput(char *string)
*
* The purpose of this function is to send a string to the serial
* port to be displayed on the console.
*/
void writeOutput(char *string)
{
int i = 0;                              // Location in the char array "string" that is being written to
while(string[i] != '\0')
    {
        EUSCI_A0->TXBUF = string[i];    //Writes string to console
        i++;
        while(!(EUSCI_A0->IFG & BIT1));
    }
}
/*
* void setupSerial()
* The purpose of this function is to Set up the serial port EUSCI_A0 as 9600 8N1 (8 bits, no parity,
* one stop bit.) This enables the interrupt so that received data will
* results in an interrupt.
*/
void setupSerial()
{
P1->SEL0 |= (BIT2 | BIT3);                      // P1.2 and P1.3 are EUSCI_A0 RX
P1->SEL1 &= ~(BIT2 | BIT3);                     // and TX respectively.
EUSCI_A0->CTLW0 = BIT0;                         // Disables EUSCI. Default configuration is 8N1
EUSCI_A0->CTLW0 |= BIT7;                        // Connects to SMCLK BIT[7:6] = 10
EUSCI_A0->CTLW0 &= ~((BIT(15)) | (BIT(11)));    //BIT15 = parity diabled, BIT11 = one stop bit
/* Baud Rate Configuration
3000000/(16*9600) = 19.53 (3 MHz at 9600 bps is fast enough to turn on over sampling (UCOS = /16))
UCOS16 = 1 (0ver sampling, /16 turned on)
UCBR = 1 (Whole portion of the divide)
UCBRF = .53125 * 16 = 8
UCBRS = 3000000/9600 remainder=0.5 -> 0xAA (look up table 22-4)*/
EUSCI_A0->BRW = 19;                             // UCBR Value from above
EUSCI_A0->MCTLW = 0xAA81;                       //UCBRS (Bits 15-8) & UCBRF (Bits 7-4) & UCOS16 (Bit 0)
EUSCI_A0->CTLW0 &= ~BIT0;                       // Enable EUSCI
EUSCI_A0->IFG &= ~BIT0;                         // Clear interrupt
EUSCI_A0->IE |= BIT0;                           // Enable interrupt
NVIC_EnableIRQ(EUSCIA0_IRQn);
}
