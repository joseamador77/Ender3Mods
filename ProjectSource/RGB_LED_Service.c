/****************************************************************************
 Module
   RGB_LED_Service.c

 Revision
   1.0.1

 Description
   This is a template file for implementing a simple service under the
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/16/12 09:58 jec      began conversion from TemplateFSM.c
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "RGB_LED_Service.h"
#include "dbprintf.h"
#include <sys/attribs.h>



/*----------------------------- Module Defines ----------------------------*/
#define PBCLK_RATE 20000000L
#define LED_PERIOD 33
#define HIGH_TIME 27
#define LOW_TIME 6

#define LOWTIME 15 // number of 48MHz cycles to be low for 0.35uS
#define HIGHTIME 65 // number of 48MHz cycles to be high for 1.65uS


/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
static void initLEDs(void);
static wsColor HSBtoRGB(float hue, float sat, float brightness);
static void ws2812b_setColor(wsColor * c, int numLEDs);

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;

static wsColor colorSet[4];


static uint32_t colorArray[5];
static volatile uint8_t numLEDsToUpdate = 0;




/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitTemplateService

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any
     other required initialization for this service
 Notes

 Author
     J. Edward Carryer, 01/16/12, 10:00
****************************************************************************/
bool InitRGB_LED_Service(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  /********************************************
   in here you write your initialization code
   *******************************************/
//    wsColor pink = HSBtoRGB(300,0.66,1);
//    wsColor red = HSBtoRGB(0,1,1);
//    wsColor orange = HSBtoRGB(26,1,1);
//    wsColor yellow = HSBtoRGB(61,1,1);
//    wsColor green = HSBtoRGB(112,1,1);
//    wsColor blue = HSBtoRGB(237,1,1);
//    wsColor purple = HSBtoRGB(270,0.82,1);
  
  initLEDs();
  
  
//  colorArray[0] = red.r << 16 | red.g <<8 | red.b;
//  //colorArray[1] = green.r << 16 | green.g <<8 | green.b;
//  
//  colorArray[0] = 0xFF0000;
  //colorArray[1] = 0x0000FF;
  // post the initial transition event
  ThisEvent.EventType = ES_INIT;
  if (ES_PostToService(MyPriority, ThisEvent) == true)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/****************************************************************************
 Function
     PostRGB_LED_Service

 Parameters
     EF_Event_t ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostRGB_LED_Service(ES_Event_t ThisEvent)
{ 
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunTemplateService

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes

 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event_t RunRGB_LED_Service(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  
  switch(ThisEvent.EventType){
        case ES_TIMEOUT: {
            
        }
        break;
        
        case ES_RED: {
            wsColor pink = HSBtoRGB(300,0.66,1);
            wsColor red = HSBtoRGB(0,1,1);
            wsColor orange = HSBtoRGB(26,1,1);
            wsColor yellow = HSBtoRGB(61,1,1);
            wsColor green = HSBtoRGB(112,1,1);
            wsColor blue = HSBtoRGB(237,1,1);
            wsColor purple = HSBtoRGB(270,0.82,1);
            colorSet[0] = red;
            colorSet[1] = green;
            colorSet[2] = yellow;
            colorSet[3] = green;
            
            DB_printf("COLORS\r\n");
            ws2812b_setColor(colorSet,4);
            //LATBINV = 0b100000000000; // invert B11
        }
        break;

    }
  
  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/
static void initLEDs(void){
    // --------------- Set LED pin as digital out ------------------ 
    TRISBbits.TRISB11 = 0;       // OC output
    
    LATBbits.LATB11 = 0;        //set to 0
    // ----------------------------------------------------------------
    
    // ------------------------- Timer 2 ------------------------------
    T2CONbits.ON = 0;           // turn off timer 2
    T2CONbits.TCS = 0;          // PBCLK source
    T2CONbits.TCKPS = 0b000;    // pre-scale 1
    //T2CONbits.TCKPS = 0b011;    // pre-scale 1
    TMR2 = 0;          // clear the timer register
    PR2 = 65535;           // set period to 2500 Hz
//    T2CONbits.ON = 1;           // turn on timer 2
    //PR2 = 10000;           // set period to 2500 Hz
    // ----------------------------------------------------------------

    // -------------------------- OC 2 --------------------------
//    OC2CONbits.ON = 0;          // turn off OC2
//    OC2CONbits.OCTSEL = 0;      // select timer 2
//    OC2CONbits.OCM = 0b101;     // Continuous output pulses
//    OC2CONbits.OC32 = 0;        // 16 bit mode
//    OC2R = 0;                   // Resets on 0
//    OC2RS = HIGH_TIME;                  // Timing one
//    RPB11R = 0b0101;             // map OC2 to RB11
    // ----------------------------------------------------------------

     //--------------------------CONFIG INTERRUPTS--------------------//
//    __builtin_disable_interrupts(); // disable global interrupts
//    INTCONbits.MVEC = 1;            // enable multivector mode
//    
//    //IPC2bits.OC2IP = 6;             //OC2 priority
//    //IPC2bits.T2IP = 6;              //T2 priority
//    
//    //IFS0CLR = _IFS0_OC2IF_MASK;     //clear OC flag
//    //IFS0CLR = _IFS0_T2IF_MASK;      //clear T2 flag
//    
//    //IEC0SET = _IEC0_OC2IE_MASK;     //enable OC2 interrupts
//    //IEC0SET = _IEC0_T2IE_MASK;      //enable T2 interrupts
//    
//    __builtin_enable_interrupts();  // enable global interrupts
    
    
    
}   


void __ISR(_TIMER_2_VECTOR,IPL6SOFT) Timer2Rollover(void){
    static int8_t index = 23;
    static uint8_t currLED = 0;
    
    IFS0CLR = _IFS0_T2IF_MASK; //clear interrupt flag
     //DB_printf("INDEX %d \r\n",index);     
    if (colorArray[currLED] & (1 << index)){ //1
         OC2RS = HIGH_TIME;
        // DB_printf("HIGH\r\n");
    }else{ //0
        OC2RS = LOW_TIME;
        //DB_printf("LOW\r\n");
    }
    
    index--;
    if (index < 0){
        currLED++;
        index = 23;
       // DB_printf("NEXT LED\r\n");
                  
        if (currLED >= numLEDsToUpdate){
            T2CONbits.ON = 0;           // turn off timer 2
            OC2CONbits.ON = 0;          // turn off OC2
            currLED = 0;                //reset current LED
            LATBbits.LATB11 = 0;
            DB_printf("END\r\n");
           
        }
    }
    
}


//static void ws2812b_setColor(wsColor * c, uint8_t numLEDs) {
//    numLEDsToUpdate = numLEDs;
//    
//    LATBbits.LATB11 = 0;
//    TMR2 = 0; //reset timer 2
//    //turn on modules to update the colors
//    DB_printf("MODULES ON\r\n");
//    DB_printf("num = %x\r\n",colorArray[0]);
//    DB_printf("num = %x\r\n",colorArray[1]);
//    DB_printf("num = %x\r\n",colorArray[2]);
//    OC2CONbits.ON = 1;          // turn on OC2
//    T2CONbits.ON = 1;           // turn on timer 2
//    
//}


// build an array of high/low times from the color input array, then output the high/low bits
void ws2812b_setColor(wsColor * c, int numLEDs) {
    int i = 0; int j = 0; // for loops
    int numBits = 2 * 3 * 8 * numLEDs; // the number of high/low bits to store, 2 per color bit
    volatile unsigned int delay_times[2*3*8 * 5]; // I only gave you 5 WS2812B, adjust this if you get more somewhere

    // start at time at 0
    delay_times[0] = 0;
    
    int nB = 1; // which high/low bit you are storing, 2 per color bit, 24 color bits per WS2812B
	
    // loop through each WS2812B
    for (i = 0; i < numLEDs; i++) {
        // loop through each color bit, MSB first
        for (j = 7; j >= 0; j--) {
            // if the bit is a 1
            if (c[i].r >> j == 1) {/* identify the bit in c[].r, is it 1 */
                // the high is longer
                delay_times[nB] = delay_times[nB - 1] + HIGHTIME;
                nB++;
                delay_times[nB] = delay_times[nB - 1] + LOWTIME;
                nB++;
            } 
            // if the bit is a 0
            else {
                // the low is longer
                delay_times[nB] = delay_times[nB - 1] + LOWTIME;
                nB++;
                delay_times[nB] = delay_times[nB - 1] + HIGHTIME;
                nB++;
            }
        }
        for (j = 7; j >= 0; j--) {
            // if the bit is a 1
            if (c[i].g >> j == 1) {/* identify the bit in c[].r, is it 1 */
                // the high is longer
                delay_times[nB] = delay_times[nB - 1] + HIGHTIME;
                nB++;
                delay_times[nB] = delay_times[nB - 1] + LOWTIME;
                nB++;
            } 
            // if the bit is a 0
            else {
                // the low is longer
                delay_times[nB] = delay_times[nB - 1] + LOWTIME;
                nB++;
                delay_times[nB] = delay_times[nB - 1] + HIGHTIME;
                nB++;
            }
        }
        for (j = 7; j >= 0; j--) {
            // if the bit is a 1
            if (c[i].b >> j == 1) {/* identify the bit in c[].r, is it 1 */
                // the high is longer
                delay_times[nB] = delay_times[nB - 1] + HIGHTIME;
                nB++;
                delay_times[nB] = delay_times[nB - 1] + LOWTIME;
                nB++;
            } 
            // if the bit is a 0
            else {
                // the low is longer
                delay_times[nB] = delay_times[nB - 1] + LOWTIME;
                nB++;
                delay_times[nB] = delay_times[nB - 1] + HIGHTIME;
                nB++;
            }
        }
        // do it again for green
		// do it again for blue
    }
    
//    DB_printf("stupid\r\n");
//    for (i = 1; i < numBits; i++) {
//        DB_printf("delay = %d\r\n",delay_times[i]);
//    }

    // turn on the pin for the first high/low
    LATBbits.LATB11 = 1;
    TMR2 = 0; // start the timer
    T2CONbits.ON = 1;
    for (i = 1; i < numBits; i++) {
        while (TMR2 < delay_times[i]) {
        }
        //DB_printf("delay = %d\r\n",delay_times[i]);
        LATBINV = 0b100000000000; // invert B11
    }
    
    LATBbits.LATB11 = 0;
    TMR2 = 0;
    while(TMR2 < 2400){} // wait 50uS, reset condition
}

// adapted from https://forum.arduino.cc/index.php?topic=8498.0
// hue is a number from 0 to 360 that describes a color on the color wheel
// sat is the saturation level, from 0 to 1, where 1 is full color and 0 is gray
// brightness sets the maximum brightness, from 0 to 1
static wsColor HSBtoRGB(float hue, float sat, float brightness) {
    float red = 0.0;
    float green = 0.0;
    float blue = 0.0;

    if (sat == 0.0) {
        red = brightness;
        green = brightness;
        blue = brightness;
    } else {
        if (hue == 360.0) {
            hue = 0;
        }

        int slice = hue / 60.0;
        float hue_frac = (hue / 60.0) - slice;

        float aa = brightness * (1.0 - sat);
        float bb = brightness * (1.0 - sat * hue_frac);
        float cc = brightness * (1.0 - sat * (1.0 - hue_frac));

        switch (slice) {
            case 0:
                red = brightness;
                green = cc;
                blue = aa;
                break;
            case 1:
                red = bb;
                green = brightness;
                blue = aa;
                break;
            case 2:
                red = aa;
                green = brightness;
                blue = cc;
                break;
            case 3:
                red = aa;
                green = bb;
                blue = brightness;
                break;
            case 4:
                red = cc;
                green = aa;
                blue = brightness;
                break;
            case 5:
                red = brightness;
                green = aa;
                blue = bb;
                break;
            default:
                red = 0.0;
                green = 0.0;
                blue = 0.0;
                break;
        }
    }

    unsigned char ired = red * 255.0;
    unsigned char igreen = green * 255.0;
    unsigned char iblue = blue * 255.0;

    wsColor c;
    c.r = ired;
    c.g = igreen;
    c.b = iblue;
    return c;
}


/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

