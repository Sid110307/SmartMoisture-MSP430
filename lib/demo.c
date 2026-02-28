/*********************************************************
 * Oxygen Concentrator Dept of Materials Engineering
 * Ver : see the #define VERSION
 *
 * Author:  K. Bhaskar
 * Date : 24/10/20
 *
 * This Program by default runs on 2 x 3x 2  valve actuated by two solenoid
 * at  HalfCycleTime = 7 sec
 *     NoOfCycles      = 100 Cycles
 *     Relay1 dur      = 3 secs
 *     Delta         = 0.1 sec
 *     OxygenPoll    =  1750 5 secs
 *
 *
 * The timing is confirmed by Scope at 1 Hz with
 * TA0CCR0 register at 46750
 * If Crystal is replaced then check on the timings
 * of this register
 *
 * Uart programming of the variables is available
 * dynamically.  Buffer needs may be one or two
 * invocations.   Not intended for Field but
 * R&D Only
 *
 * Release : R&D
 *
 * Production Release needs to be #defined
 *
 *
 * *****************************************************/

#include "msp430i2040.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include "util.h"


// *******************************************
// * All Global Data and defines here
// *******************************************

// <--- ** WARNING**  this has to be disable for Production

// #define TESTING                      // All Testing and R&D code should go under this.  Just disable to get PRODUCTION RELEASE


#ifdef  TESTING
#define VERSION "0.3.0 (R&D)"
#else
#define VERSION "0.3.0 (PRODUCTION)"
#define BLE     1                     // To enable blue tooth in production
#endif

#define MAX_SAMPLES     256
#define SHIFT           8
#define OXYCMD_START   'T'
#define OXYCMD_END     '\r'
#define MAX_CMD_LEN     127

#define OXYCMD_ACK     'R'

// The following is used to Initialize TA0CCR0 to derive 1 Sec for 350 counts checked on Scope
// Will it be same for all the chips has to be tested before production release

#define TIMING_1_SEC_BY_SCOPE  46750
#define SECONDS 350


#define UPPER_LIMIT 5600
#define LOGGING_PERIOD 5    // once in 5 mins the usage is written onto info
                            // info has a million operation - so 10 continuous years of usage of controller lasts
                            // later provision to move the data to next block can be done in the code
                            // in current version ** NOT DONE **

#define UART       1
#define BLUETOOTH  2
#define DELAY 16000000  // delay for 1 sec at 16 MHz
#define RESET 16000 // 0.1 sec


#define RELAYONETWO    (P1OUT |= BIT0) ; \
                       (P1OUT &= ~BIT1)  ;

#define RELAYONEOFF    (P1OUT |= BIT0);   // 1.0 is Off ->  Relay is ON logic reversed because of Pull Up Resistor
#define RELAYONEON     (P1OUT &= ~BIT0);  // 1.0 is ON -->  Relay is off

#define RELAYTWOOFF    (P1OUT |= BIT1);   // 1.1 is Off -> Relay On
#define RELAYTWOON     (P1OUT &= ~BIT1);  // 1.1 is On  -> Relay Off


// All the logic of OFF and ON is reversed because it is wrt to Vcc than to GND by board design
// reason given by Ram Mohan, stability of µ Controller for first µ seconds

#define SLEEP(t)    __delay_cycles(t*16000) // t in 100 msec units so for t=1 => 100 msec delay

#define LED1OFF     (P1OUT |= BIT4)
#define LED1ON      (P1OUT &= ~BIT4)
#define LED1BLINK   LED1ON; SLEEP(1);LED1OFF; SLEEP(1);
#define CONTACTORON  LED1ON
#define CONTACTOROFF LED1OFF

#define LED2OFF     (P1OUT |= BIT5)

#define LED2ON      (P1OUT &= ~BIT5)
#define LED2TOGGLE  (P1OUT ^= BIT5)
#define BUZHIGHON    LED2ON
#define BUZHIGHOFF   LED2OFF

#define LED3OFF     (P1OUT |= BIT6)
#define LED3ON      (P1OUT &= ~BIT6)
#define BUZMEDON    LED3ON
#define BUZMEDOFF   LED3OFF

#define LED4OFF     (P1OUT |= BIT7)
#define LED4ON      (P1OUT &= ~BIT7)
#define BUZLOWON     LED4ON
#define BUZLOOFF     LED4OFF


#define LED5OFF     (P2OUT |= BIT0)
#define LED5ON      (P2OUT &= ~BIT0)
#define REDON       LED5ON
#define REDOFF      LED5OFF

#define LED6OFF     (P2OUT |= BIT1)
#define LED6ON      (P2OUT &= ~BIT1)
#define GREENON     LED6ON
#define GREENOFF    LED6OFF


#define LED7OFF     (P2OUT |= BIT2)
#define LED7ON      (P2OUT &= ~BIT2)
#define FANOFF      LED7OFF
#define FANON       LED7ON


#define LED8OFF     (P2OUT |= BIT3)
#define LED8ON      (P2OUT &= ~BIT3)
#define BLUEON      LED8ON
#define BLUEOFF     LEDOFF

// Defining SD24 Channels

#define CHANNEL0     SD24CCTL0
#define CHANNEL1     SD24CCTL1
#define CHANNEL2     SD24CCTL2
#define CHANNEL3     SD24CCTL3

#define OXYCHANNEL   CHANNEL1
#define PRECHANNEL   CHANNEL0
#define TMPCHANNEL   CHANNEL3

#define UARTWRITE(X)   temp=X;sprintf(numbers,"%d|",temp);UWriteString(numbers);


// **********************************
// * All external declarations here
// **********************************

extern unsigned short FANPERIOD_CNT;
extern unsigned short FANDUTYCYCLE_CNT;
extern unsigned short Ecount ;
extern unsigned short Enum    ;

// * All temporary buffers
volatile char OxyC  = '\0';         // used in ADC sampling
volatile char OxyCmd[MAX_CMD_LEN + 10] = {""} ;  // allocating extra for possible buffer overflow problems

static char buf[MAX_CMD_LEN + 10]; // temp variable for token parsing of cmds recd
static char numbers[12];

volatile unsigned int iOxyCnt = 0;
volatile bool       Pressure_Configure = false;


volatile bool datacanberxed, cmdrxed ;
unsigned volatile long  pressure,oxygen,intTemp,flow;

/***********************************
 * Application Logic Variables     *
 ***********************************/


unsigned short int Relay_1_StartFrom = 0;
unsigned short int Relay_1_Duration  = 3*SECONDS;
unsigned short int Relay_2_StartFrom = 3*SECONDS;
unsigned short int Relay_2_Duration  = 3*SECONDS;
unsigned short int Pressuriser       = 0;
unsigned short int Ticks = 0;           // used for finding out time elapsed in small units
unsigned long  int UsedTime = 0;        // this will hold the no of minutes m/c used has to be made persistent

unsigned short int HalfCycleTime  = 3500;   // multiples of 350
unsigned short int OxygenPollTime = 1750;     //

unsigned short  Delta          = 35;     // is 35=100 msec
unsigned long   StartExpt      = 1750;   // 3 secs
unsigned long   EndExpt        = 2100;   // 6 secs


#ifdef TESTING
static unsigned long  NoOfCycles    =  65000;         // No of ParmCycles particular expt should run
#else
static unsigned long  NoOfCycles    =  INT_MAX ;  // this is virtually infinite amounts to some 483 years
#endif

unsigned long  ParmCycles     = 0;
unsigned short int ledno = 1;
unsigned short  temp;

unsigned long  TotalCycles, NoOfSamples;

int  relaysetcount,Mode;
bool adcreadcomplete=false, ExptOver=false, TimeLocked=false,Verbose=true;

bool Connected=false;
int  ResetCount=100, Ack=0;
bool Waking=false;


// ***********************************************
// *  All Functions here
// ***********************************************

void UWriteData(char data) {
    while (!(UCA0IFG & UCTXIFG))
             ;
    UCA0TXBUF = data;
}

void UWriteString(const char *str)
{
    while ((*str) != '\0')
    {
        UWriteData(*str);
        str++;
    }
}

void Store_data_in_infomem(){
    //  ***************************************
    //  * write to info mem
    //  * Store the following variables
    //  *     Verbose    -   used for o/p
    //  *     Mode       -   UART/BLUETOOTH
    //  *     Relay_1_Duration
    //  *     Relay_2_StartFrom
    //  ***************************************

}

void Initialize()
{
    // All globals to sane value before starting the machine

    /*
    Relay_1_StartFrom =
    Relaly_1_Duration =
    Relay_2_StartFrom =
    Relay_2_Duration  =
*/
    HalfCycleTime = (Relay_1_Duration + Pressuriser)* 2;
    relaysetcount = 0;

    adcreadcomplete = false;
    TotalCycles = 0;
    ExptOver = false;
    P1OUT = 0x00;
    P1DIR |= BIT0|BIT1|BIT4|BIT5|BIT6|BIT7;     // New Controller Board - enabling 2 relays & all leds
    P2DIR |= BIT0|BIT1|BIT2|BIT3;
    LED8OFF; LED7OFF; LED6OFF; LED5OFF; LED4OFF; LED3OFF; LED2OFF; LED1ON;
    RELAYONEOFF
    RELAYTWOOFF
    //Pressuriser %= 350;  // Limiting to 1 sec

#ifdef BLE
    Mode = BLUETOOTH;
    Verbose = false; // by default O/p is minimal on Bluetooth
#else
    Mode = UART;
    Verbose = true; // in UART Mode and forced by command only
#endif
}

void StartAllOverAgain(){
    // Initialize everything to the beginning so that expt begins
    UWriteString("|E|<-- Expt Over --> Resetting>|\r\n\0");
    Relay_1_Duration = Relay_2_Duration = 1050;
    Initialize();
}

//***********************************
//* All bluetooth Responses handling
//***********************************

// the following 2 routines are for ble - celium commands

void handle_ble_responses(char *response){

    if (strncmp(response,"RSP",3)==0){
        //UWriteString("Yes Responding\r\n");
        Ack++;
    }
    if (Ack > 5) Ack=0; // send ony 5 init commands
}

// ******************************************
// All bluetooth Event Handlers
// ******************************************

void handle_ble_events(char *event){
    char Command[128];int i;
    // init the buf for unknown things and walk thru mem
    UWriteString("EVT Occured\r\n");
    for ( i=0;i<128;i++){
        Command[i]=0;
    }
    //UWriteString(event);UWriteString("\r\n");
    if (strncmp(event,"EVT+READY",9)==0){// do init after wakeup or power cycle
        if (Waking){
            ble_init();
            Waking=false;
        } else {
            // already woken up do nothing
            Ack = 1; // check for bool
        }
    }
    if (strncmp(event,"EVT+CON",7)==0){
        Connected = true;
        //__delay_cycles(DELAY*1);
    } else if (strncmp(event,"EVT+DATA",8)==0){
        // get the data here
        strncpy(Command,event[11],120); // parse and take the payload after , only..... now adhoc

    } else if (strncmp(event,"EVT+DISCON",10)==0){
        Connected=false; // disabling o/p
        //ble_init();
    }
}

void ble_send_cmd(char *msg){
    UWriteString(msg);
    __delay_cycles(DELAY);
}
#define WAIT_FOR_RESPONSE

void ble_init(){
    Ack=0;
    switch (Ack){
           case 0 : ble_send_cmd("CMD+RESET=0\r\n");    ;
           case 1 : ble_send_cmd("CMD+NAME=Prana\r\n"); ;
           case 2 : ble_send_cmd("CMD+RESET=0\r\n");    ;
           case 3 : ble_send_cmd("CMD+ADV=1\r\n");      ;break;
           case 4 : ble_send_cmd("CMD+NOTIFY=1\r\n");   break;
           default: break;
    }
}

//#endif

//************************************************************
//* This should run only in uart mode ble should be blocked
//************************************************************
bool SetParameters(mode){
    if (mode == BLUETOOTH)
        return false;
        // fmt => "T|start|delta|end|halfcycle|NoOfCycles|;|"
        // error handling of parms not done

       Relay_1_StartFrom =  atoi(strtok(NULL,"|"));
       Relay_1_Duration  =  atoi(strtok(NULL,"|"));
       Relay_2_StartFrom =  atoi(strtok(NULL,"|"));
       Relay_2_Duration  =  atoi(strtok(NULL,"|"));

       Relay_2_Duration  =  Relay_1_Duration;   // just overriding above line... Stupid though
       Pressuriser       =  atoi(strtok(NULL,"|"));

       Delta             =  atoi(strtok(NULL,"|"));
       EndExpt           =  atoi(strtok(NULL,"|"));   // would like to be curtailed here  but keeping it

       HalfCycleTime     =  atoi(strtok(NULL,"|"));   // again would like to auto compute will do it later

       NoOfCycles        =  atoi(strtok(NULL,"|"));
       OxygenPollTime    =  atoi(strtok(NULL,"|"));    //  this is meaningless some problem check the routines
       FANDUTYCYCLE_CNT  =  atoi(strtok(NULL,"|"));

       Stop_LED7PWM_FAN();
       init_LED7PWM_FAN();
       LED7PWM_Fan_Speed();


       // initializing the state of the board
       Initialize();                                // to start all over again dynamically
       UWriteString("|R|<-- Resetting Expt with new values --->|");

       UARTWRITE(Relay_1_StartFrom);
       UARTWRITE(Relay_1_Duration);
       UARTWRITE(Relay_2_StartFrom);
       UARTWRITE(Relay_2_Duration)
       UARTWRITE(Pressuriser);

       UARTWRITE(Delta);
       UARTWRITE(EndExpt);

       UARTWRITE(HalfCycleTime);
       UARTWRITE(NoOfCycles);
       UARTWRITE(OxygenPollTime);

       UARTWRITE(FANDUTYCYCLE_CNT);
       UWriteString(";|\r\n\0");
       ExptOver=false;
       return(true);
}


// ***********************************************************************
// * This routine can only be run within 1 min of starting the machine
// * To safeguard accidental runs due to any malfunctions
// * All the routine should be locked after 1 min
// ***********************************************************************

bool diagnose(char *parms){
    int ledno;
    if (TimeLocked){
        UWriteString("|LOG|** WARNING ** Diagnosis Started Wrongly---|;|\r\n");
        return false;
    }

    ledno = atoi(strtok(NULL,"|"));
    switch (ledno)
    {
          case 1: LED1ON; break;
          case 2: LED2ON; break;
          case 3: LED3ON; break;
          case 4: LED4ON; break;
          case 5: LED5ON; break;
          case 6: LED6ON; break;
          case 7: LED7ON; break;
          case 8: LED8ON; break;
    }
    sprintf(buf,"|X|<--- Version : %s led:  %d : Expt %d---->|;|\r\n\0",VERSION,ledno,iOxyCnt);

    UWriteString(buf); //test this out
    return true;
}
void main(void) {

    int  rxwaitingperiod=0;
    volatile  unsigned long  channelone, channeltwo,channelthree,channelfour;


       WDTCTL = WDTPW | WDTHOLD;   // Stop WDT
       __delay_cycles(3200);
       Initialize();

/*******************LED7 as PWM *********************/
/*
 * Timer_A1 configured for up/down mode. The value in CCR0, 128, defines the PWM
 * period/2 and the value in CCR2( 32) the PWM duty cycles.
 * A 75% duty cycle is on P2.2.
 */
       P2SEL0 |=  BIT2;                  //  P2.2 CCRx Function
       P2DIR  |=  BIT2;                  // Set  P2.2 as outputs
       TA1CCR0 = 256;
       TA1CCTL2 = OUTMOD_6;              // CCR2 PWM toggle/set
       TA1CCR2 = 32;
       TA1CTL = TASSEL_2 | MC_3 | ID_3 ; // SMCLK/8, Cont. Mode
/*****************************************************/
       init_LED7PWM_FAN();
       LED7PWM_Fan_Speed();
       P1SEL0 |= BIT2 | BIT3;                  // P1.2/3 eUSCI_A Function
       P1SEL1 &= ~(BIT2 | BIT3);
       TA0CCTL0 = CCIE;                        // enabling Interrupt
       TA0CCR0 = TIMING_1_SEC_BY_SCOPE;        // 46750 - this gave 350 = 1sec. No drift observed with this
       TA0CTL = TASSEL_2 | MC_1 | TACLR;       // this is where division is happenning...
       UCA0CTL1 |= UCSWRST;                    // Hold eUSCI in reset
       UCA0CTL1 |= UCSSEL_2;                   // SMCLK

#ifdef BLE
       // *******************************************************
       // * Send Data and Receive Any commands from Blue Tooth
       // * Only the following Commands will be allowed from BLE
       // *  Diag|p1.x|p2.x|;|\r\n
       // *  Diag|ALL|;\r\n
       // *  Settings|channels|ch1|ch2|ch3|ch4|;|\r\n
       // *  Settings|RelayTime|999999|Pressuriser|9999|
       // *  Settings|FanDuty|99|;|\r\n
       // *  Settings|Verbose|;\r\n
       // *  GetData|;|\r\n
       // *******************************************************
       UCA0BR0 = 142;                          // 115200 baud
       UCA0BR1 = 0x00;
       UCA0MCTLW = 0x2200;                     //16.384MHz/115200 = 142.22  (See UG)
#else
       // ****************************************************
       // Connect UART for data Acquisition from Computer
       // All Commands allowed.....
       // ****************************************************

       UCA0BR0 = 0xAA;                         // 9600 baud
       UCA0BR1 = 0x06;
       UCA0MCTLW = 0xD600;                     //16.384MHz/9600 = 1706.6667 (See UG)
#endif

       UCA0CTL1 &= ~UCSWRST;                   // release from reset
       UCA0IE   |= UCRXIE;                     // Enable Rx Interrupt

       SD24CTL     = SD24REFS;                     // Internal ref
       SD24CCTL0  |= SD24SNGL | SD24GRP | SD24DF;  // Group with CH1
       SD24CCTL1  |= SD24SNGL | SD24GRP | SD24DF;  // Group with CH2
       SD24CCTL2  |= SD24SNGL | SD24GRP | SD24DF;  // Group with CH3
       SD24CCTL3  |= SD24SNGL | SD24IE  | SD24DF;  // Enable Interrupt
       SD24INCTL2 |= SD24INCH_6;

       __delay_cycles(3200);                       // Delay ~200us for 1.2V ref to settle


    while(1)
    {
        // trying to wakeup the ble once in a while if it is not connected
        // This will wake up in case ble hangs down also

#ifdef BLE
        if (!Connected){
            ResetCount--;
            if (!ResetCount){
                ble_init();
                ResetCount=RESET;
            }
        }
#endif


         SD24CCTL3 |= SD24SC;
         int ij=0;

         SD24CCTL2 |= SD24SC;                    // Set bit to start conversion
         ij++;  // this is junk it is a silly nop
         SD24CCTL1 |= SD24SC;                    // Set bit to start conversion
         ij++;  // this is junk it is a silly nop
         SD24CCTL0 |= SD24SC;                    // Set bit to start conversion
         ij++;  // this is junk it is a silly nop

        __bis_SR_register(LPM0_bits | GIE);     // Enter LPM0 w/ interrupts

        if (!(UsedTime % LOGGING_PERIOD)){
            Store_data_in_infomem();
        }

        // **********************************
        // * Handle all Sensors here
        // * Currently implemented
        // *    Pressure Sensor :
        // *      connection : o/p 2 bar
        // *    Oxygen Sensor :
        // *       for percentage
        // **********************************

        if (adcreadcomplete && !ExptOver )
        {
            channelone   = (pressure >> SHIFT); // shifting right by SHIFT to divide by  MAX_SAMPLES
            channeltwo   = (oxygen >> SHIFT);
            channelthree = intTemp >> SHIFT;
            channelfour  = flow ;
            NoOfSamples  = pressure = oxygen = intTemp = flow = 0;  // initializing all of them for loop
            channelthree  = ((channelthree * 12000)/70711) - 2730;  // temp in C multiplied by 10
/**********************/
            ErrorCount();
            //channelone= VPRESSURE_NORMAL;
            GetPressureState(channelone);
            PressureAlarm();
            /**********************/
#ifdef BLE
            if (Connected){
                UWriteString("CMD+DATA=0,");

#endif
            UWriteString("|V|");
            UARTWRITE(channelone);
            UARTWRITE(channeltwo);
            UARTWRITE(channelthree);
            UARTWRITE(channelfour);

            UWriteString("C|");
            UARTWRITE(TotalCycles);
            UWriteString("S|");
            UARTWRITE(Relay_1_StartFrom);
            UARTWRITE(Relay_1_Duration);
            UARTWRITE(Relay_2_StartFrom);
            UARTWRITE(Relay_2_Duration);
            UARTWRITE(Pressuriser);
            UARTWRITE(Delta);
            UARTWRITE(EndExpt);
            UARTWRITE(NoOfCycles);
            UARTWRITE(HalfCycleTime);
            UARTWRITE(OxygenPollTime);
            UWriteString("F|");
            UARTWRITE(FANPERIOD_CNT);
            UARTWRITE(FANDUTYCYCLE_CNT)
            UWriteString("E|");
            UARTWRITE(Ecount);
            UARTWRITE(Enum);
            UWriteString(";|\r\n");  // terminate the line by Delimiter ; and CRLF NULL for any string op
#ifdef BLE
            } // conditional o/p
#endif
            adcreadcomplete = false;    // doing it last so that samples are not collected and possible corruption

            ErrorCount();
        }


        // *********************************************
        // * Process All Commands from the USB         *
        // *********************************************
        if(cmdrxed == true)
        {
            //UWriteString("WRITING =>");  // never ever put \n and write 2 continuous strings you will not receive it

            OxyCmd[MAX_CMD_LEN] = '\0';    // terminating for extreme case
            UWriteString("Controller Recd=>");
            UWriteString(OxyCmd);UWriteString("\r\n");
            char Cmd[128];
            strcpy(Cmd,OxyCmd);
            iOxyCnt = 0;
            cmdrxed = false ;
            strcpy(buf,strtok(OxyCmd,"|"));

            if (strcmp (buf, "T") == 0) {  // the very first token
                SetParameters(Mode);
            }
            else if (strcmp (buf, "Diag") == 0) {  // just retrieve version for checking firmware in field
                diagnose(Cmd);  // parse again

            } // parse for bluetooth ble commands nrf52810 celium modules
            else if (strcmp (buf, "BLE") == 0){
                char ble_cmd[6];
                strncpy(ble_cmd,Cmd,3);
                ble_cmd[3]='\0'; // doubly ensuring termination

                if (strcmp(ble_cmd,"EVT")==0){// handle all events
                    handle_ble_events(Cmd);
                }
                else if (strcmp (ble_cmd,"RSP")==0){ // handle all responses
                    handle_ble_responses(Cmd);
                }
            }
        }

        else {             // what is this code doing ?
            if(datacanberxed)
            {
                if(++rxwaitingperiod > 5000)
                {
                    rxwaitingperiod = 0 ;
                    datacanberxed = false ;
                }
            }
            else rxwaitingperiod = 0 ;
        }
    }
}// <----------------- End of Main Loop -------------------------------->

// *****************************
// *  All ISR Routines here
// *
// *****************************

// ***********************
// *  Timer A0 ISR       *
// ***********************

#pragma vector=TIMER0_A0_VECTOR
__interrupt
void TA0_ISR(void) {
    ++relaysetcount;


    if (++Ticks >350*60){
        UsedTime++;
        Ticks=0;
    }

    if (relaysetcount == Relay_1_StartFrom+1){
        RELAYONEON;
    }

    if (relaysetcount == Relay_2_StartFrom ){
        RELAYTWOON;
    }

    if (relaysetcount == Relay_1_Duration){
        RELAYONEOFF;
    }

    if (relaysetcount == Relay_2_StartFrom + Relay_2_Duration){
        RELAYTWOOFF;
    }

    if (relaysetcount >= HalfCycleTime){
        relaysetcount = 0;
        ++TotalCycles;
        ++ParmCycles;
    }

  if (Verbose){         // this is set only Lab Testing Conditions and when required in field testing
      if (ParmCycles >= NoOfCycles && !ExptOver){

        Relay_1_Duration += Delta;
        Relay_2_Duration  = Relay_1_Duration;

        Relay_2_StartFrom = Relay_1_Duration + Pressuriser;
        HalfCycleTime = 2 * (Relay_1_Duration + Pressuriser);
        // At this point Realy_1_StartFrom, Relay_1_Duration with Relay_2 should be identical  and HalfCycleTime = sum of 4 + 2 x Pressuriser
        ParmCycles=0;
        // Limit HalfCycleTime so that expt does not go awry
        if (HalfCycleTime > UPPER_LIMIT){
            StartAllOverAgain();
        }
     }

     if (Relay_1_Duration >= EndExpt){
        adcreadcomplete = true;
        StartAllOverAgain();    // restart expt from beginning to avoid pressure building
     }
   }  // <------------- End of Verbose -------------------

}

// ********************
// *   Handle UART
// ********************

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(UCA0IV,USCI_UART_UCTXCPTIFG)) {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:
             //   while (!(UCA0IFG&UCTXIFG)); // USCI_A0 TX buffer fready?
              //  UCA0TXBUF = UCA0RXBUF;      // TX -> RXed character

                OxyC = UCA0RXBUF;
                if(cmdrxed == false)
                {
                   OxyCmd[iOxyCnt] = OxyC;
                   if (OxyC){ // remove null chars due to noise
                       iOxyCnt = ( iOxyCnt <  MAX_CMD_LEN ) ?  (iOxyCnt + 1 ) : MAX_CMD_LEN;
                   }
                   cmdrxed = ( OxyC == OXYCMD_END || iOxyCnt >= MAX_CMD_LEN) ?  true : false;
                   ExptOver = false;    // this is to kick up Oxygen Generation after end of expt
                   Ack=1; // check here for bool op
                }
                OxyCmd[iOxyCnt+1]='\0'; //null terminating the string for future operations

                break;

        case USCI_UART_UCTXIFG: break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
        default: break;
    }
}   // <------------------------ End of UART ISR ----------------------->

// ************************
// * SD24 - Σ- Δ ADC  ISR
// ************************

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=SD24_VECTOR
__interrupt void SD24_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(SD24_VECTOR))) SD24_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch (__even_in_range(SD24IV,SD24IV_SD24MEM3)) {
        case SD24IV_NONE: break;
        case SD24IV_SD24OVIFG: break;
        case SD24IV_SD24MEM0: break;
        case SD24IV_SD24MEM1: break;
        case SD24IV_SD24MEM2: break;
        case SD24IV_SD24MEM3:
                pressure += SD24MEM0;       // Save CH0 results (clears IFG)
                oxygen   += SD24MEM1;       // Save CH1 results (clears IFG)
                intTemp  += SD24MEM2;       // Save CH2 results (clears IFG)
                flow     += SD24MEM3;       // save CH3 results (clears IFG)

                NoOfSamples++;

                if (NoOfSamples >=MAX_SAMPLES){
                    //adcreadcomplete =  (NoOfSamples >= MAX_SAMPLES) ?  true :  false;
                    adcreadcomplete = true;
                }

              __bic_SR_register_on_exit(LPM0_bits); // Wake up
                break;

        default: break;
    }
}

// <---------------------- End of Program ---------------------------->
