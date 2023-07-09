/****************************************************************************
 Module
   XBeeCommSM.c

 Revision
   1.0.1

 Description
   This is a template file for implementing flat state machines under the
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/15/12 11:12 jec      revisions for Gen2 framework
 11/07/11 11:26 jec      made the queue static
 10/30/11 17:59 jec      fixed references to CurrentEvent in RunTemplateSM()
 10/23/11 18:20 jec      began conversion from SMTemplate.c (02/20/07 rev)
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "EnderCommSM.h"
#include "dbprintf.h"
#include <sys/attribs.h>
//#include "RGB_LED_Service.h"


/*----------------------------- Module Defines ----------------------------*/
#define STOP_CMD_LENGTH 5

#define READFILAMENT PORTBbits.RB10

/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/
static void InitUART2(void);
static void initFilament(void);
static void constructPauseMsg(void);

static void parseData(void);
static void storeData(uint8_t byte);

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static EnderCommState_t CurrentState;

static uint8_t TXMessageLength = 0;
static uint8_t TX_Index = 0;
static uint8_t TXArray[20];//length of max message size
static bool TXEnabled = false;

static uint8_t RXArray[30];
static uint8_t RXMsgLength = 0;

static bool prevFilamentStatus;

// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;



/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitXBeeCommSM

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 18:55
****************************************************************************/
bool InitEnderCommSM(uint8_t Priority)
{
  ES_Event_t ThisEvent;

  MyPriority = Priority;
  
  InitUART2();//init the uart communication
  //initFilament();
  
  
  // put us into the Initial PseudoState
  CurrentState = InitPState1;
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
     PostEnderCommSM

 Parameters
     EF_Event_t ThisEvent , the event to post to the queue

 Returns
     boolean False if the Enqueue operation failed, True otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostEnderCommSM(ES_Event_t ThisEvent)
{
  return ES_PostToService(MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunEnderCommSM

 Parameters
   ES_Event_t : the event to process

 Returns
   ES_Event_t, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes
   uses nested switch/case to implement the machine.
 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event_t RunEnderCommSM(ES_Event_t ThisEvent)
{
  ES_Event_t ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

  switch (CurrentState)
  {
    case InitPState1:        // If current state is initial Psedudo State
    {
      if (ThisEvent.EventType == ES_INIT)    // only respond to ES_Init
      {
        // now put the machine into the actual initial state
        CurrentState = Idle;
      }
    }
    break;

    case Idle:       
    {
        
      switch (ThisEvent.EventType) 
      {
        
        case ES_STORE_RX_DATA: //parse received data
        {   
            storeData((uint8_t)ThisEvent.EventParam);
        }
        break;
                  
        case ES_NO_FILAMENT: //Create a pause message
        {   
            DB_printf("SENDING STOP\r\n");
            constructPauseMsg();
            
        }
        break;
        
        case ES_TIMEOUT:
        {   
            //if (ThisEvent.EventParam == 0){//Timer timer expiration
            //               
            //}
        }
        break;
        
        
        
        default:
          ;
      }  
    }
    break;
    
    case Printing:       
    {
      switch (ThisEvent.EventType)
      {
        
        case ES_STORE_RX_DATA: //parse received data
        {   
            storeData((uint8_t)ThisEvent.EventParam);
        }
        break;
          
             
        case ES_TIMEOUT:
        {   
//            if (ThisEvent.EventParam == 0){//Timer timer expiration
//               
//            }
        }
        break;
        
        default:
          ;
      }  
    }
    break;
    
    default:
      ;
  }                                   // end switch on Current State
  return ReturnEvent;
}

/****************************************************************************
 Function
     QueryXBeeCommSM

 Parameters
     None

 Returns
     XBeeCommState_t The current state of the Template state machine

 Description
     returns the current state of the Template state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:21
****************************************************************************/
EnderCommState_t QueryEnderCommSM(void)
{
  return CurrentState;
}

/***************************************************************************
 private functions
 ***************************************************************************/

//-------------------------------INIT FUNCTIONS---------------------------//
static void InitUART2(void){
    //--------------------------CONFIG UART SETTINGS--------------------//
    U2MODEbits.ON = 0;          // turn UART off
    
    RPA3R = 0b0010;             // map TX to RA3 
    TRISAbits.TRISA3 = 0;       // output
    
    U2RXR = 0b0000;             // map RX to RA1 
    ANSELAbits.ANSA1 = 0;       // digital
    TRISAbits.TRISA1 = 1;       // input    
    
    U2MODE = 0;                 // clear the U1MODE register
    
    U2MODEbits.BRGH = 0;        // 16x baud clock
    U2MODEbits.STSEL = 0;       // 1 stop bit
    U2MODEbits.PDSEL = 0;       // 8 bit no parity
    
    U2STA = 0;                  // clear U1STA
    U2STAbits.URXISEL = 0b00;   // interrupt when RX buffer not empty
    U2STAbits.UTXEN = 1;        // enable TX pin
    U2STAbits.URXEN = 1;        // enable RX pin
    
    U2BRG = 129;                 // 9600 baud //TODO
    
    U2MODEbits.ON = 1;          // turn UART on
    
    //--------------------------CONFIG UART INTERRUPTS--------------------//
    __builtin_disable_interrupts(); // disable global interrupts
    INTCONbits.MVEC = 1;            // enable multivector mode
    
    IPC9bits.U2IP = 7;              // set priority of the interrupt
    
    IFS1CLR = _IFS1_U2RXIF_MASK;    // clear RX flag
    IFS1CLR = _IFS1_U2TXIF_MASK;    // clear TX flag
    
    IEC1SET = _IEC1_U2RXIE_MASK;    // enable U2 RX interrupts
    //IEC1SET = _IEC1_U2TXIE_MASK;    // enable U2 TX interrupts
    
    __builtin_enable_interrupts();  // enable global interrupts
}

//-------------------------------------CONSTRUCTING MSG FUNCTIONS--------------------------//
static void constructPauseMsg(void){
    TXArray[0] = 77; //M
    TXArray[1] = 48; //0
    TXArray[2] = 49; //1
    TXArray[3] = 13; // \r
    TXArray[4] = 10; // \n
    TXMessageLength = STOP_CMD_LENGTH;
    
    //setup to use TX interrupt
    U2TXREG = TXArray[TX_Index]; //preload the U2TXREG
    TX_Index++; //increment TX index
    IEC1SET = _IEC1_U2TXIE_MASK;    // enable U2 TX interrupts
    TXEnabled = true;
    
}

//-----------------------------------------------STORING AND PARSING MESSAGES----------------------------//
static void storeData(uint8_t byte){
    
    RXArray[RXMsgLength] = byte; //store incoming bytes
    RXMsgLength++; //increment message length
    
    if (byte == 10){//if end line
        parseData();//parse through data
    }
}

static void parseData(void){
    IEC1CLR = _IEC1_U2RXIE_MASK;    //disable U2 RX interrupts
    
    for(int i = 0; i < RXMsgLength-1;i++){//loop through the data(except new line) and parse    
        if (i == 2){
        }else if (i == 3){ 
        }
    }
    IEC1SET = _IEC1_U2RXIE_MASK;    //re-enable U2 RX interrupts
}

//---------------------------------------ISR's--------------------------------//
void __ISR(_UART_2_VECTOR, IPL7SOFT) UARTHandler(void){
    static ES_Event_t RXEvent;
    
    //if TX flag is set and we enabled the TX interrupt and RX idle
    if (IFS1bits.U2TXIF == 1 && TXEnabled && (U2STAbits.RIDLE == 1)){ 
        IFS1CLR = _IFS1_U2TXIF_MASK;    // clear TX flag
        
        while (U2STAbits.TRMT == 0){} //wait until the TX shift register is empty
        
        if (TX_Index >= TXMessageLength){ //if message is complete    
            IEC1CLR = _IEC1_U2TXIE_MASK; //turn off the transmit interrupt
            TX_Index = 0; //reset TX index
            TXEnabled = false; //disable transmits
        }else{
            U2TXREG = TXArray[TX_Index]; //load the U2TXREG
            TX_Index++; //increment TX index
        }
    }
    
    if (IFS1bits.U2RXIF == 1){ //if the RX flag is set
        IFS1CLR = _IFS1_U2RXIF_MASK;    // clear RX flag
        uint8_t RecvdMSG = U2RXREG;
        
        RXEvent.EventType = ES_STORE_RX_DATA; //setup new event
        RXEvent.EventParam = RecvdMSG;//read the incoming message
        PostEnderCommSM(RXEvent); //post event to self
        
    }
    IFS1CLR = _IFS1_U2TXIF_MASK;    // clear TX flag
    IFS1CLR = _IFS1_U2RXIF_MASK;    // clear RX flag
}

static void initFilament(void){
    TRISBbits.TRISB10 = 1;  // input
    prevFilamentStatus = READFILAMENT;
   
}

// FILAMENT EVENT CHECKER
bool check4Filament(void){ 
    bool currState = READFILAMENT;
    
    if (currState != prevFilamentStatus){
        if (!currState){ //button low
            //puts("bayt switch is low\r\n");
            //ES_Timer_InitTimer(BAYT_DEBOUNCE_TIMER, 500);
        }else{//button is high
            //puts("bayt switch is high\r\n");
            //ES_Timer_StopTimer(BAYT_DEBOUNCE_TIMER);
        }
    }
    prevFilamentStatus = currState;
   
    return false;
}

