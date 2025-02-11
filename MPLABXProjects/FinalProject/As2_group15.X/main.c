/*
 * File:   main.c
 * Author: 
 * * Chiappe Andrea
 * * Di Donna Alberto
 * * Guelfi Fabio s5004782
 * * Utegaliyeva Aidana 
 *
 * Created on June 13, 2024, 12:36 PM
 */


#include "xc.h"
#include <stdio.h>
#include <math.h>
#include "timer.h"
#include "uartlib.h"
#include "adclib.h"
#include "pwmlib.h"
#include "scheduler.h"
#include "parser.h"
#include "circularbufferlib.h"

/* ################################################################
                        Parameter Definition
    ###################################################################*/

#define MAX_TASKS 5
#define WAIT_FOR_START 0
#define EXECUTE 1


/* ################################################################
                        Interrupt functions
###################################################################*/
void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt(void); // timer interrupt for debouncing
void __attribute__((__interrupt__, __auto_psv__)) _INT1Interrupt(void);  // button interrupt
void __attribute__((__interrupt__, __auto_psv__)) _U1TXInterrupt(void); // uart trasmission interrupt
void __attribute__((__interrupt__, __auto_psv__)) _U1RXInterrupt(void); // uart reception interrupt

void __attribute__((__interrupt__, __auto_psv__)) _INT2Interrupt(void);  // brake interrupt

/* ################################################################
                        functions definition
###################################################################*/
void taskBlinkLedA0 (void* ); // task for the led A0
void taskBlinkIndicators (void* ); // task for the indicators
void taskADCSensing(void* ); // task for the ADC sensing
void taskPrintBattery(void* ); // task for the battery print on circular buffer
void taskPrintInfrared (void* ); // task for the infrared print on circular buffer

// main functions
void printAck (char ); // task for the ack print on circular buffer


/* ################################################################
                        global variables
###################################################################*/
typedef struct {
    float battery_data; // battery value
    float infraRed_data; // Infrared value
    fifo fifo_write; // circular buffer for trasmission
} data;


heartbeat schedInfo[MAX_TASKS]; // task of the scheduler
int state; // state of the state machine
int return_parser; // return value of the parser

data data_values; // data structure for the values of the sensors
parser_state pstate; // parser state
fifo_pwm fifo_command; // circular buffer for the commands for motors

int counter_for_int2; // counter for the interrupt 2

int main(void) 
{
    /* ################################################################
                            Init Config
    ###################################################################*/
    // 0 = digital; 1 = analog
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;
       
    /* ################################################################
                                Timer setup
    ###################################################################*/
    tmr_setup_period(TIMER5, 1); // setup the period for the while loop
    
    /* ################################################################
                            State Initialization
    ###################################################################*/
    state = WAIT_FOR_START; // initial state
    data_values.battery_data = 0.0; // initial value for the battery
    data_values.infraRed_data = 0.0; // initial value for the infrared
    fifo_init(data_values.fifo_write); // initialization of the circular buffer

    pstate.state = STATE_DOLLAR; // initial state for the parser
	pstate.index_type = 0; // initial index for the type of parser
	pstate.index_payload = 0; // initial index for the payload of parser
    return_parser = 0; // initial value for the parser
    fifo_pwm_init(fifo_command); // initialization of the circular buffer for the commands
    char pwm_type[] = "PCCMD"; // correct value for the pwm type 

    counter_for_int2 = -1; // counter for the interrupt 2 for timing
    
    /* ################################################################
                        Defining all the tasks
    ###################################################################*/
    // LED A0 task config
    schedInfo[0].n = 0;
    schedInfo[0].N = 1000;
    schedInfo[0].f = taskBlinkLedA0;
    schedInfo[0].params = NULL;
    schedInfo[0].enable = 1;
    // indicator task config
    schedInfo[1].n = -2;
    schedInfo[1].N = 1000;
    schedInfo[1].f = taskBlinkIndicators;
    schedInfo[1].params = NULL;
    schedInfo[1].enable = 1;
    
    // ADC acquisition task
    schedInfo[2].n = -5;
    schedInfo[2].N = 1;
    schedInfo[2].f = taskADCSensing;
    schedInfo[2].params = (void*)(&data_values);
    schedInfo[2].enable = 1;
    
    // Print Battery task
    schedInfo[3].n = -60;
    schedInfo[3].N = 1000;
    schedInfo[3].f = taskPrintBattery;
    schedInfo[3].params = (void*)(&data_values);
    schedInfo[3].enable = 1;

    // Print Infrared task
    schedInfo[4].n = -10;
    schedInfo[4].N = 100;
    schedInfo[4].f = taskPrintInfrared;
    schedInfo[4].params = (void*)(&data_values);
    schedInfo[4].enable = 1;

    /* ################################################################
                        pin remap and setup of the led
    ###################################################################*/
    // LED A0 setup
    TRISAbits.TRISA0 = 0; // 0 = output, 1 = input 
    LATAbits.LATA0 = 1; // value as low, to write (to read the command is PORT)
    // indicatorts setup
    TRISBbits.TRISB8 = 0; // left indicator
    LATBbits.LATB8 = 1;
    TRISFbits.TRISF1 = 0; // right indicator
    LATFbits.LATF1 = 1;

    // brakes led setup
    TRISFbits.TRISF0 = 0; // set brakes led as output
    LATFbits.LATF0 = 0; // set the brakes led as low

    // low intensity lights setup
    TRISGbits.TRISG1 = 0; // set the low intensity lights as output
    LATGbits.LATG1 = 1; // set the low intensity lights as high

    // beam headlights setup
    TRISAbits.TRISA7 = 0; // set the beam headlights as output
    LATAbits.LATA7 = 1; // set the beam headlights as high
    
    /* ################################################################
                        pin remap and setup of the buttons
    ###################################################################*/
    // Mapping INT1 to RE8 pin of left button
    RPINR0bits.INT1R = 0x58; // 0x58 is 88 in hexadecimal
    TRISEbits.TRISE8 = 1; // set the pin button as input
    
    /* ################################################################
                        pin adc remap
    ###################################################################*/
    // Mapping battery (no need)
    // enable the infra-red sensor
    TRISBbits.TRISB9 = 0; // set the enable pin as output
    LATBbits.LATB9 = 1; //set the high value for enable the infra-red sensor
    
    /* ################################################################
                        Peripheral configuration
    ###################################################################*/
    
    adc_scanmode_config(); // setup for the ADC (battery and InfraRed)
    
    uart_config(); // uart trasmission and reception configuration

    pwmConfig(); // standard pwm configuration
    
    /* ################################################################
                    Adc mode activation
    ###################################################################*/
    // select the scan pin
    AD1CSSLbits.CSS11 = 1; // Enable AN11 for scan (battery)
    AD1CSSLbits.CSS14 = 1; // Enable AN14 for scan (infra-red)
    
    ANSELBbits.ANSB11 = 0x0001; // battery input as analog value
    ANSELBbits.ANSB14 = 0x0001; // IR input as analog value (controlla rapporto 15 e pin)
    
    //MUST BE LAST THING TO DO:
    AD1CON1bits.ADON = 1; // turn ADC on
    
    /* ################################################################
                        Interrupt activation
    ###################################################################*/
    INTCON2bits.GIE = 1; // set global interrupt enable 

    // Interrupt for the button to change the state
    IFS1bits.INT1IF = 0; // clear interrupt flag
    IEC1bits.INT1IE = 1; // enable interrupt
    IFS0bits.T1IF = 0; // reset interrupt flag timer 1

    // Interrupt for reception
    IFS0bits.U1RXIF = 0; // setting the flag for reception to 0
    IEC0bits.U1RXIE = 1; // enable interrupt for UART 1 receiver
    
    // Interrupt for trasmission
    IFS0bits.U1TXIF = 0; // resetting U1TX interrupt flag
    IEC0bits.U1TXIE = 1; // enable U1TX interrupt 
    
    // Interrupt for emergency stop motors pwm
    IFS1bits.INT2IF = 0; // clear interrupt flag for motors pwm
    IEC1bits.INT2IE = 1; // enable interrupt for motors pwm
    

    while (1)
    {
        if (state == WAIT_FOR_START)
        {
            whstop(); // stop the wheels
            schedInfo[1].enable = 1; // enable the indicators
            if (counter_for_int2 == 0 && fifo_command.tail != fifo_command.head)
            {
                counter_for_int2 = -1; // reset the counter
                fifo_command.tail = (fifo_command.tail + 1) % MAX_COMMANDS; // circular increment of the tail
            }
        }
        if (state == EXECUTE)
        {
            schedInfo[1].enable = 0; // disable the indicators
            
            // check the control buffer is not empty
            if (fifo_command.tail != fifo_command.head)
            {
                if (counter_for_int2 == -1) // need to setup the timer
                {
                    input_move (fifo_command.msg[fifo_command.tail][0]);
                    if (fifo_command.msg[fifo_command.tail][1] <= MAX_TIME)
                    {
                        if (fifo_command.msg[fifo_command.tail][1] == 0)
                        {
                            // probably error, so action ends immediately
                            IFS1bits.T4IF = 1; // set the flag to zero
                        } else 
                        {
                            // setup the timer (consider the timer
                            tmr_setup_period(TIMER4, fifo_command.msg[fifo_command.tail][1]);
                        }
                    }
                    else 
                    {
                        // setup the timer (consider the timer)
                        tmr_setup_period(TIMER4, MAX_TIME); // setup the timer (consider the timer)
                    }
                    // set the timer for the wheels motor
                     
                    counter_for_int2 = 0; // set the counter to 0                
                }      

                if (tmr_wait_period(TIMER4) == 1 && counter_for_int2 == 0) // wait for the end of the time of the motor
                {
                    // circular increment of the tail
                    fifo_command.tail = (fifo_command.tail + 1) % MAX_COMMANDS; // circular increment of the tail
                    counter_for_int2 = -1; // reset the counter
                }
            }
            else
            {
                whstop(); // stop the wheels
            }
        }
        
      
        scheduler(schedInfo, MAX_TASKS);
        
        // check if the parser has received a new message
        if (return_parser == NEW_MESSAGE)
        {
            return_parser = NO_MESSAGE;
            if (strcmp(pstate.msg_type, pwm_type) == 0)
            {
                // Saving the command in the circular buffer
                if ((fifo_command.head + 1) % MAX_COMMANDS != fifo_command.tail) // if the buffer is not full
                {
                    // extract the values
                    fifo_command.msg[fifo_command.head][0] = extract_integer(pstate.msg_payload);
                    fifo_command.msg[fifo_command.head][1] = extract_integer(pstate.msg_payload + next_value(pstate.msg_payload, 0));
                    
                    // print there is more space
                    printAck ('1'); 

                    // circular increment of the head of the buffer
                    fifo_command.head = (fifo_command.head + 1) % MAX_COMMANDS;
                }
                else // buffer is full
                {
                    printAck ('0'); 
                }
            }
        }
        
        while(!tmr_wait_period(TIMER5)); // wait for the end of the Hb  
    }
    return 0;
}


/* ################################################################
                            Interrupt functions
###################################################################*/
// uart1 interrupt trasmission function 
void __attribute__((__interrupt__, __auto_psv__)) _U1TXInterrupt(void)
{   
    //set the flag to zero
    IFS0bits.U1TXIF = 0;
    
    while((U1STAbits.UTXBF != 1) && (data_values.fifo_write.tail != data_values.fifo_write.head))
    {
        U1TXREG = data_values.fifo_write.msg[data_values.fifo_write.tail]; // write the value of circula buffer isndie the uart1 buffer
        
        // circular increment of the tail
        data_values.fifo_write.tail = (data_values.fifo_write.tail + 1) % DIMFIFOWRITE;
    }
}

// uart1 interrupt reception function
void __attribute__((__interrupt__, __auto_psv__)) _U1RXInterrupt(void)
{
    // interrupt to receive the data from the uart and save them in the circular buffer
    //set the flag to zero
    IFS0bits.U1RXIF = 0;
    char c = U1RXREG;
    return_parser = parse_byte(&pstate, c);
}

//interrupt associated to button 
void __attribute__((interrupt, auto_psv)) _INT1Interrupt(void){
    // Advertise button has been pressed
    // The change of the state will be done in T1 interrupt
    //interrupt code  
    IEC1bits.INT1IE = 0; // disable the interrupt
    IFS1bits.INT1IF = 0; //put to zero the flag of the interrupt
    
    // activating the interrupt
    tmr_setup_period(TIMER1, 10); // setup the timer (consider the timer)
    IEC0bits.T1IE = 1;
}

// interrupt for the timer 1
void __attribute__((interrupt, auto_psv)) _T1Interrupt(void) {
    // interrupt to change the status of state machine
    IFS0bits.T1IF = 0; // reset interrupt flag
    IEC0bits.T1IE = 0;
    T1CONbits.TON = 0; // stop the timer

    if (PORTEbits.RE8 == 1) // if the button is still pressed
    { // if not it is a contact bounce
        if (state == WAIT_FOR_START)
        {
            state = EXECUTE;
            LATBbits.LATB8 = 0; // turn off the indicators
            LATFbits.LATF1 = 0; // turn off the indicators
        }
        else 
        {
            state = WAIT_FOR_START;
        }
    } 
    // Reactivating the button interrupt
    IFS1bits.INT1IF = 0; //put to zero the flag of the interrupt
    IEC1bits.INT1IE = 1; // enable the interrupt
}

// Emergency interrupt for the brake
void __attribute__((__interrupt__, __auto_psv__)) _INT2Interrupt(void)
{
    // interrupt for the brake
    IFS1bits.INT2IF = 0; // reset interrupt flag
    // IEC1bits.INT2IE = 0; // disable interrupt (only for new state)
    whstop(); // stop the wheels
}

/* ################################################################
                            Scheduler functions
###################################################################*/

void taskBlinkLedA0 (void* param)
{
    LATAbits.LATA0 = !LATAbits.LATA0;
}

void taskBlinkIndicators (void* param)
{
    // left indicator
    LATBbits.LATB8 = !LATBbits.LATB8;
    // right indicator
    LATFbits.LATF1 = !LATFbits.LATF1;
}

void taskADCSensing(void* param)
{
    data* cd = (data*) param;
    while(!AD1CON1bits.DONE);
    int potBitsBatt = ADC1BUF0;
    int potBitsIr = ADC1BUF1;
    cd->battery_data = battery_conversion((float) potBitsBatt); 
    cd->infraRed_data = volt2cm((float) potBitsIr);

    if(cd->infraRed_data < EMERGENCY_STOP){
        IFS1bits.INT2IF = 1; // ativate the flag for he emergency brake
        LATFbits.LATF0 = 1; // set the brakes led as high
    }else
    {
        LATFbits.LATF0 = 0; // set the brakes led as low
    }
}

void taskPrintBattery (void* param)
{
    data* cd = (data*) param;
    char buff[20];

    sprintf(buff,"$MBATT,%.2f*", cd->battery_data); 
    
    // write the values of the battery register
    for(int i = 0; buff[i] != 0; i++)
    {
        // chech if the buffer is not full to write
        if ((cd->fifo_write.head + 1) % DIMFIFOWRITE != cd->fifo_write.tail)
        {
            cd->fifo_write.msg[cd->fifo_write.head] = buff[i];
            
            // circular increment of the head of the buffer
            cd->fifo_write.head = (cd->fifo_write.head + 1) % DIMFIFOWRITE;
        }
        // else to not lose the data (TO COMPLETE) 
    } 
    if (U1STAbits.UTXBF == 0)
    {
        IFS0bits.U1TXIF = 1;
    }
}

void taskPrintInfrared (void* param)
{
    data* cd = (data*) param;
    char buff[20];

    sprintf(buff,"$MDIST,%.0f*", cd->infraRed_data); 
    
    // write the values of the infrared register
    for(int i = 0; buff[i] != 0; i++)
    {
        // chech if the buffer is not full to write
        if ((cd->fifo_write.head + 1) % DIMFIFOWRITE != cd->fifo_write.tail)
        {
            cd->fifo_write.msg[cd->fifo_write.head] = buff[i];
            
            // circular increment of the head of the buffer
            cd->fifo_write.head = (cd->fifo_write.head + 1) % DIMFIFOWRITE;
        }
        // else to not lose the data (TO COMPLETE)
    }  
    if (U1STAbits.UTXBF == 0)
    {
        IFS0bits.U1TXIF = 1;
    }
}

/* ################################################################
                            Main functions
###################################################################*/

void printAck (char ack)
{
    char buff[9];
    buff[0] = '$'; // start of the message
    buff[1] = 'M'; // start of the message
    buff[2] = 'A'; // start of the message
    buff[3] = 'C'; // start of the message
    buff[4] = 'K'; // start of the message
    buff[5] = ','; // start of the message
    buff[6] = ack; // possible variable character
    buff[7] = '*'; // end of the message
    buff[8] = '\0'; // null character 

    for(int i = 0; buff[i] != 0; i++)
    {
        // chech if the buffer is not full to write
        if ((data_values.fifo_write.head + 1) % DIMFIFOWRITE != data_values.fifo_write.tail)
        {
            data_values.fifo_write.msg[data_values.fifo_write.head] = buff[i];
            
            // circular increment of the head of the buffer
            data_values.fifo_write.head = (data_values.fifo_write.head + 1) % DIMFIFOWRITE;
        }
        // else to not lose the data (TO COMPLETE)
    }  
    if (U1STAbits.UTXBF == 0)
    {
        IFS0bits.U1TXIF = 1;
    }

}
