#include "los_sys.h"
#include "los_tick.h"
#include "los_task.ph"
#include "los_config.h"

#include "los_bsp_led.h"
#include "los_bsp_key.h"
#include "los_bsp_uart.h"
#include "los_inspect_entry.h"
#include "los_demo_entry.h"

#include <string.h>

#include "board.h"

extern void LOS_EvbSetup(void);

static UINT32 g_uwboadTaskID;
static LITE_OS_SEC_TEXT VOID LOS_BoardExampleTskfunc(VOID)
{
    while (1)
    {
        LOS_EvbLedControl(LOS_LED1, LED_ON);
        LOS_EvbUartWriteStr("Board Test\r\n");
        (void)LOS_TaskDelay(500);
        LOS_EvbLedControl(LOS_LED1, LED_OFF);
        (void)LOS_TaskDelay(500);
    }
}

void LOS_BoardExampleEntry(void)
{
    UINT32 uwRet;
    TSK_INIT_PARAM_S stTaskInitParam;

    (VOID)memset((void *)(&stTaskInitParam), 0, sizeof(TSK_INIT_PARAM_S));
    stTaskInitParam.pfnTaskEntry = (TSK_ENTRY_FUNC)LOS_BoardExampleTskfunc;
    stTaskInitParam.uwStackSize = LOSCFG_BASE_CORE_TSK_IDLE_STACK_SIZE;
    stTaskInitParam.pcName = "BoardDemo";
    stTaskInitParam.usTaskPrio = 10;
    uwRet = LOS_TaskCreate(&g_uwboadTaskID, &stTaskInitParam);

    if (uwRet != LOS_OK)
    {
        return;
    }
    return;
}

/*****************************************************************************
Function    : main
Description : Main function entry
Input       : None
Output      : None
Return      : None
 *****************************************************************************/
LITE_OS_SEC_TEXT_INIT
int main(void)
{
    UINT32 uwRet;
    /*
        add you hardware init code here
        for example flash, i2c , system clock ....
     */
    //HAL_init();....
    /* Board pin, clock, debug console init */
    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
    /* enable clock for GPIO*/
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Gpio1);

    BOARD_InitPins();
    BOARD_BootClockFROHF48M();
    
    /*Init LiteOS kernel */
    uwRet = LOS_KernelInit();
    if (uwRet != LOS_OK) {
        return LOS_NOK;
    }
    /* Enable LiteOS system tick interrupt */
    uwRet = LOS_EnableTick();
    if (uwRet != LOS_OK) {
        return LOS_NOK;
    }
   
    /*
        Notice: add your code here
        here you can create task for your function 
        do some hw init that need after systemtick init
     */
    LOS_EvbSetup();//init the device on the dev baord

    //LOS_Demo_Entry();

    //LOS_Inspect_Entry();

    LOS_BoardExampleEntry();

    /* Kernel start to run */
    (void)LOS_Start();
    for (;;);
    /* Replace the dots (...) with your own code. */
}
