/****************************************************************************

  Header file for template Flat Sate Machine
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef EnderCommSM_H
#define EnderCommSM_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum
{
  Idle = 0x00,
  Printing = 0x01,
  InitPState1
}EnderCommState_t;



// Public Function Prototypes

bool InitEnderCommSM(uint8_t Priority);
bool PostEnderCommSM(ES_Event_t ThisEvent);
ES_Event_t RunEnderCommSM(ES_Event_t ThisEvent);
EnderCommState_t QueryEnderCommSM(void);


// event checker
bool check4Filament(void);

#endif /* FSMTemplate_H */

