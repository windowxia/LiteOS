#include "los_bsp_led.h"

#ifdef LOS_LPC54114
    #include "board.h"
    #define LED_NUM 1
#endif

/*****************************************************************************
 Function    : LOS_EvbLedInit
 Description : Led init
 Input       : None
 Output      : None
 Return      : None
 *****************************************************************************/
void LOS_EvbLedInit(void)
{
#ifdef LOS_LPC54114
    LED_RED_INIT(1);
#endif

    return;
}

/******************************************************************************
 Function   £ºLOS_EvbLedControl
 Discription: Control led on or off
 Input      : index Led's index
              cmd   Led on or off
 Return     : None
*******************************************************************************/
void LOS_EvbLedControl(int index, int cmd)
{
#ifdef LOS_LPC54114
    
    if(index >= LED_NUM)
    {
        return;
    }
    switch (cmd)
    {
        case LED_ON:
        {
            LED_RED_ON();
            break;
        }
        case LED_OFF:
        {
            LED_RED_OFF();
            break;
        }
        default:
        {
            break;
        }
    }
#endif
    return ;
}


