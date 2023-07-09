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
#define LED_PERIOD 34

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
static void initLEDs(void);
static void ws2812b_setColor(wsColor * c, int numLEDs);

/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;

static wsColor colorSet[1];
static volatile uint8_t numBitsToSend = 0;




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
  
  initLEDs();
  colorSet[0] = HSBtoRGB(0,1,1);
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
            ws2812b_setColor(colorSet,1)
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
    // ----------------------------------------------------------------
    
    // ------------------------- Timer 2 ------------------------------
    T2CONbits.ON = 0;           // turn off timer 2
    T2CONbits.TCS = 0;          // PBCLK source
    T2CONbits.TCKPS = 0b000;    // pre-scale 1
    TMR2 = 0;          // clear the timer register
    PR2 = 34;           // set period to 2500 Hz
    // ----------------------------------------------------------------

    // -------------------------- OC 2 --------------------------
    OC2CONbits.ON = 0;          // turn off OC2
    OC2CONbits.OCTSEL = 0;      // select timer 2
    OC2CONbits.OCM = 0b101;     // Continuous output pulses
    OC2CONbits.OC32 = 0;        // 16 bit mode
    OC2R = 0;                   // Resets on 0
    OC2RS = 7;                  // Timing one
    RPB11R = 0b0101;             // map OC2 to RB11
    // ----------------------------------------------------------------

     //--------------------------CONFIG INTERRUPTS--------------------//
    __builtin_disable_interrupts(); // disable global interrupts
    INTCONbits.MVEC = 1;            // enable multivector mode
    
    IPC2bits.OC2IP = 6;             //OC2 priority
    IPC2bits.T2IP = 5;              //T2 priority
    
    IFS0CLR = _IFS0_OC2IF_MASK;     //clear OC flag
    IFS0CLR = _IFS0_T2IF_MASK;      //clear T2 flag
    
    IEC0SET = _IEC0_OC2IE_MASK;     //enable OC2 interrupts
    IEC0SET = _IEC0_T2IE_MASK;      //enable T2 interrupts
    
    __builtin_enable_interrupts();  // enable global interrupts
    
     // Turn on OC2, and Timer 2
   //OC2CONbits.ON = 1;          // turn on OC2
   //T2CONbits.ON = 1;           // turn on timer 2
}   

void __ISR(_OUTPUT_COMPARE_2_VECTOR, IPL6SOFT) LEDControl(void){
    IFS0CLR = _IFS0_OC2IF_MASK;     //clear OC flag     
    

    

    while (numBitsToSend > 0){
        if (colorSet[0].r >> 7 == 0){
            OC2RS = 7;
        }else{
            OC2RS = 27;
        }
        
    }
    
}

void __ISR(_TIMER_2_VECTOR,IPL5SOFT) Timer2Rollover(void){
    IFS0CLR = _IFS0_T2IF_MASK;
    numBitsToSend--; //finished a bit
}


static void ws2812b_setColor(wsColor * c, int numLEDs) {
    static int i = 0; 
    static int j = 0; // for loops
    int numBits = 2 * 3 * 8 * numLEDs; // the number of high/low bits to store, 2 per color bit
     
    
    numBitsToSend = 3*8*numLEDs + 1 ;//determine bits to send
    TMR2 = 0; //reset timer 2
    
    //turn on modules to update the colors
    OC2CONbits.ON = 1;          // turn on OC2
    T2CONbits.ON = 1;           // turn on timer 2

	
    // loop through each WS2812B
    for (i = 0; i < numLEDs; i++) {
        // loop through each color bit, MSB first
  
       
    
       
    }
}

// adapted from https://forum.arduino.cc/index.php?topic=8498.0
// hue is a number from 0 to 360 that describes a color on the color wheel
// sat is the saturation level, from 0 to 1, where 1 is full color and 0 is gray
// brightness sets the maximum brightness, from 0 to 1
wsColor HSBtoRGB(float hue, float sat, float brightness) {
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

