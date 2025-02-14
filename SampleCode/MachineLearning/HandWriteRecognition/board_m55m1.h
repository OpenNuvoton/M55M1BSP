#ifndef __BOARD_M55M1_H__
#define __BOARD_M55M1_H__

//
// Includes
//
#include <stdint.h>
#include "fsa506.h"
#include "st1633i.h"
//
// Defines
//
#define SYSTEM_WIDTH                 (240)
#define SYSTEM_HEIGHT                (320)

/*For TFLM*/
#define MNIST_HANDWRITE_ARENA       (0x28000)

#define MNIST_BUTTON_X_THRESH       (286)
#define MNIST_BUTTON_Y_THRESH       (200)
#define MNIST_HANDWRITE_SIZE        (28)
#define MNIST_HANDWRITE_TYPE        (10)
#define MNIST_HANDWRITE_WIDTH       MNIST_HANDWRITE_SIZE
#define MNIST_HANDWRITE_HEIGHT      MNIST_HANDWRITE_SIZE

//UI related
#define CLEARDRAW_BUTTON_X_THRESH   (286)//Define Button Area on UI
#define CLEARDRAW_BUTTON_Y_THRESH   (200)//Define Button Area on UI

//TP related
#define PANEL_WIDTH                 (480)
#define PANEL_HEIGHT                (272)
#define TP_POS_X_MIN                (0)
#define TP_POS_X_MAX                (224)
#define TP_POS_Y_MIN                (0)
#define TP_POS_Y_MAX                (224)

#define FSA506_BLACK                0x0000
#define FSA506_BLUE                 0x001F
#define FSA506_RED                  0xF800
#define FSA506_GREEN                0x07E0
#define FSA506_CYAN                 0x07FF
#define FSA506_MAGENTA              0xF81F
#define FSA506_YELLOW               0xFFE0
#define FSA506_WHITE                0xFFFF
#define FSA506_LGRAY                0xC618

// default orientation
#define FSA506_WIDTH  480
#define FSA506_HEIGHT 272

//LED GPIO reltaed
#define LED_RED                     PH4
#define LED_YELLOW                  PH5
#define LED_GREEN                   PH6
#define LED_TRANSPARENT             PE13
#define LED_DONN                    LED_RED
#define LED_DOCCAP                  LED_YELLOW
#define LED_DOBUFUPDATE             LED_GREEN
#define LED_TIMER                   LED_TRANSPARENT
#define LED_INIT()                  (PH->MODE = (PH->MODE &(~(0x3ful << 4*2)) | (0x15ul << 4 *2)))

#define TIMER_PERIOD_VALHZ          16


//Enumeration

//Different state of
typedef enum
{
    Undefine_State = 0,
    Draw_State = 1,
    Release_State = 2,

} eTPState;

typedef enum
{
    ButtonPress_Intent = 0,
    DrawSomething_Intent = 1,
    NoIntent_Intent = 2,

} eUserIntent;
//Struct

//
// Functions
//
#ifdef  __cplusplus
extern  "C" {
#endif

#ifdef  __cplusplus
}
#endif
#endif  /* __BOARD_M55M1_H__ */
