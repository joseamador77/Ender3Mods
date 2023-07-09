/****************************************************************************

  Header file for template service
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef RGB_LED_Service_H
#define RGB_LED_Service_H

#include "ES_Types.h"

// Public Function Prototypes

bool InitRGB_LED_Service(uint8_t Priority);
bool PostRGB_LED_Service(ES_Event_t ThisEvent);
ES_Event_t RunRGB_LED_Service(ES_Event_t ThisEvent);

// link three 8bit colors together
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} wsColor; 

#endif /* ServTemplate_H */

