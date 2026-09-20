/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the RTC periodic wake-up Code Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
********************************************************************************
* (c) 2026, Infineon Technologies AG, or an affiliate of Infineon
* Technologies AG. All rights reserved.
* This software, associated documentation and materials ("Software") is
* owned by Infineon Technologies AG or one of its affiliates ("Infineon")
* and is protected by and subject to worldwide patent protection, worldwide
* copyright laws, and international treaty provisions. Therefore, you may use
* this Software only as provided in the license agreement accompanying the
* software package from which you obtained this Software. If no license
* agreement applies, then any use, reproduction, modification, translation, or
* compilation of this Software is prohibited without the express written
* permission of Infineon.
*
* Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
* IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
* THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
* SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
* Infineon reserves the right to make changes to the Software without notice.
* You are responsible for properly designing, programming, and testing the
* functionality and safety of your intended application of the Software, as
* well as complying with any legal requirements related to its use. Infineon
* does not guarantee that the Software will be free from intrusion, data theft
* or loss, or other breaches ("Security Breaches"), and Infineon shall have
* no liability arising out of any Security Breaches. Unless otherwise
* explicitly approved by Infineon, the Software may not be used in any
* application where a failure of the Product or any consequences of the use
* thereof can reasonably be expected to result in personal injury.
*******************************************************************************/


/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

/*******************************************************************************
* Macros
*******************************************************************************/

#define MAX_ATTEMPTS             (500u)  /* Maximum number of attempts for RTC operation */
#define INIT_DELAY_MS             (5u)    /* delay 5 milliseconds before trying again */

/* Constants to define LONG and SHORT presses on User Button (x10 = ms) */
#define SHORT_PRESS_COUNT           10u     /* 100 ms < press < 2 sec */
#define LONG_PRESS_COUNT            200u    /* press > 2 sec */

/* Glitch delays */
#define SHORT_GLITCH_DELAY_MS       10u     /* in ms */
#define LONG_GLITCH_DELAY_MS        100u    /* in ms */

/* RTC Alarm initial date/time values. All field-enable bits are set to
   DISABLE so the alarm fires once (all comparisons pass) when the alarm
   is armed via Cy_RTC_SetAlarmDateAndTime(). The values below represent
   the absolute calendar date at which the alarm is programmed. */
#define RTC_ALARM_INITIAL_DATE_SEC      10u    /* Alarm seconds value (0-59) */
#define RTC_ALARM_INITIAL_DATE_MIN      0u    /* Initial minutes value */
#define RTC_ALARM_INITIAL_DATE_HOUR     10u    /* Initial hours value */
#define RTC_ALARM_INITIAL_DATE_DAY      6u    /* Initial day of the month */
#define RTC_ALARM_INITIAL_DATE_DOW      6u    /* Initial day of the week. It's Friday (6u)*/
#define RTC_ALARM_INITIAL_DATE_MONTH    9u    /* Initial month */
#define RTC_INITIAL_DATE_YEAR           24u /* Initial year */
#define RTC_ALARM_INTERRUPT_PRIORITY    3u   /* Alarm Interrupt priority level */
#define STRING_BUFFER_SIZE              80u  /* RTC time values buffer size*/

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* User button was pressed for how long */
typedef enum
{
    SWITCH_NO_EVENT     = 0u,
    SWITCH_SHORT_PRESS  = 1u,
    SWITCH_LONG_PRESS   = 2u,
} en_switch_event_t;

/* Alarm configuration structure: all field-enable bits are disabled so
   the comparison is bypassed and the alarm fires once when programmed.
   Only almEn = CY_RTC_ALARM_ENABLE keeps the alarm globally armed. */
cy_stc_rtc_alarm_t alarm_config =
{
    .sec            = RTC_ALARM_INITIAL_DATE_SEC,
    .secEn          = CY_RTC_ALARM_DISABLE,
    .min            = RTC_ALARM_INITIAL_DATE_MIN,
    .minEn          = CY_RTC_ALARM_DISABLE,
    .hour           = RTC_ALARM_INITIAL_DATE_HOUR,
    .hourEn         = CY_RTC_ALARM_DISABLE,
    .dayOfWeek      = RTC_ALARM_INITIAL_DATE_DOW,
    .dayOfWeekEn    = CY_RTC_ALARM_DISABLE,
    .date           = RTC_ALARM_INITIAL_DATE_DAY,
    .dateEn         = CY_RTC_ALARM_DISABLE,
    .month          = RTC_ALARM_INITIAL_DATE_MONTH,
    .monthEn        = CY_RTC_ALARM_DISABLE,
    .almEn          = CY_RTC_ALARM_ENABLE
};
uint8_t alarm_flag = 0u;

char buffer[STRING_BUFFER_SIZE];

/* Debug UART context */
cy_stc_scb_uart_context_t  DEBUG_UART_context;
/* Debug UART HAL object */
mtb_hal_uart_t DEBUG_UART_hal_obj;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
 cy_en_rtc_status_t rtc_init(void);
 cy_en_rtc_status_t rtc_alarmconfig(void);
 en_switch_event_t get_switch_event(void);
 void debug_printf(const char *str);
 void handle_error(void);
 void convert_date_to_string(cy_stc_rtc_config_t *dateTime);
 void rtc_interrupt_handler(void);
 en_switch_event_t get_switch_event(void);


/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* Summary:
* User defined error handling function
*
* Parameters:
*  uint32_t status - status indicates success or failure
*
* Return:
*  void
*
*******************************************************************************/
void handle_error(void)
{
    __disable_irq();
    CY_ASSERT(0);

}


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for RTC periodic wakeup.
*    1. Initialize the retarget-io and RTC blocks.
*    2. Check the reset reason, if it is wakeup from Hibernate power mode, then
*       set RTC initial time and date.
*    Do Forever loop:
*    3. Check if User button was pressed and for how long.
*    4. If short pressed 1s, set the RTC alarm and then go to DeepSleep mode.
*    5. If long pressed 2s, set the RTC alarm and then go to Hibernate mode.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    cy_en_rtc_status_t rtcSta;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }

    /* Check the IO freeze status: after Hibernate wakeup, IOs are frozen
       to retain their last driven state. Unfreeze them before driving IOs. */
    if (Cy_SysPm_GetIoFreezeStatus())
     {
        /* Unfreeze the system */
        Cy_SysPm_IoUnfreeze();
     }

    /* Initialize and enable the debug UART peripheral */
    result = Cy_SCB_UART_Init(DEBUG_UART_HW, &DEBUG_UART_config, &DEBUG_UART_context);
       if (result != CY_RSLT_SUCCESS)
       {
           CY_ASSERT(0);
       }
    Cy_SCB_UART_Enable(DEBUG_UART_HW);
       /* Set up the HAL wrapper around the UART peripheral */
    result = mtb_hal_uart_setup(&DEBUG_UART_hal_obj, &DEBUG_UART_hal_config, &DEBUG_UART_context, NULL);
       if (result != CY_RSLT_SUCCESS)
       {
           CY_ASSERT(0);
       }
       /* Redirect printf() to the debug UART port via retarget-io */
    result = cy_retarget_io_init(&DEBUG_UART_hal_obj);
       /* retarget-io init failed. Stop program execution */
       if (result != CY_RSLT_SUCCESS)
       {
           CY_ASSERT(0);
       }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("************************************************************\r\n");
    printf("PSOC Control C3M/P8: RTC periodic wakeup\r\n");
    printf("************************************************************\r\n\n");
    printf("Short press 'SW3' key to DeepSleep mode.\r\n\r\n");
    printf("Long press 'SW3' key to Hibernate mode.\r\n\r\n");

    /* Initialize the User Button */
    Cy_GPIO_Pin_SecFastInit(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_PIN, CY_GPIO_DM_PULLUP, 1UL, HSIOM_SEL_GPIO);

    /* Configure the RTC interrupt */
    cy_stc_sysint_t rtc_intr_config = {
                    .intrSrc = srss_interrupt_backup_IRQn,
                    .intrPriority = RTC_ALARM_INTERRUPT_PRIORITY
                    };

    /* Enable RTC interrupt */
    Cy_RTC_SetInterruptMask(CY_RTC_INTR_ALARM2);

    /* Configure RTC interrupt ISR */
    Cy_SysInt_Init(&rtc_intr_config, rtc_interrupt_handler);
    /* Clear pending interrupt */
    NVIC_ClearPendingIRQ(rtc_intr_config.intrSrc);
    /* Enable RTC interrupt */
    NVIC_EnableIRQ(rtc_intr_config.intrSrc);

    /* Check the reset reason */
    if(CY_SYSLIB_RESET_HIB_WAKEUP == (Cy_SysLib_GetResetReason() & CY_SYSLIB_RESET_HIB_WAKEUP))
      {
          /* The reset has occurred on a wakeup from Hibernate power mode */
          debug_printf("Wakeup from the Hibernate mode\r\n\n");
      }

    /* Initialize RTC */
    rtcSta = rtc_init();
    if (rtcSta != CY_RTC_SUCCESS)
        {
            handle_error();
        }

    /* Print the current date and time by UART */
    debug_printf("Current date and time\r\n");

    /* Enable global interrupts */
    __enable_irq();

    for (;;)
    {
    switch (get_switch_event())
           {
                case SWITCH_SHORT_PRESS:
                    debug_printf("Go to DeepSleep mode\r\n");

                    /* Set the RTC generate alarm after 1 second */
                    rtc_alarmconfig();
                    Cy_SysLib_Delay(LONG_GLITCH_DELAY_MS);

                    /* Go to deep sleep */
                    Cy_SysPm_CpuEnterDeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
                    debug_printf("Wakeup from DeepSleep mode\r\n");
                    break;

                case SWITCH_LONG_PRESS:
                    debug_printf("Go to Hibernate mode\r\n");

                    /*Set the RTC generate alarm after 1 second */
                    rtc_alarmconfig();
                    Cy_SysLib_Delay(LONG_GLITCH_DELAY_MS);

                   /*Go to hibernate and configure the RTC alarm as wakeup source*/
                    Cy_SysPm_SetHibernateWakeupSource(CY_SYSPM_HIBERNATE_RTC_ALARM);
                    if(CY_SYSPM_SUCCESS != Cy_SysPm_SystemEnterHibernate())
                                {
                                    printf("The CPU did not enter Hibernate mode\r\n\r\n");
                                    CY_ASSERT(0);
                                }
                    break;

                default:
                    break;
           }
     }
}

/*******************************************************************************
* Function Name: rtc_init
********************************************************************************
* Summary:
*  This functions implement the USER_RTC initialize
*
* Parameter:
*
*  function
*
* Return:
*  void
*******************************************************************************/
 cy_en_rtc_status_t rtc_init(void)
{
    uint32_t attempts = MAX_ATTEMPTS;
    cy_en_rtc_status_t rtc_result;

    /* Setting the time and date can fail. For example the RTC might be busy.
       Check the result and try again, if necessary.  */
    do
    {
        rtc_result = Cy_RTC_Init(&USER_RTC_config);
        attempts--;

        Cy_SysLib_Delay(INIT_DELAY_MS);
    } while(( rtc_result != CY_RTC_SUCCESS) && (attempts != 0u));

    return (rtc_result);

}

/******************************************************************************
* Function Name: rtc_alarmconfig
*******************************************************************************
*
* Summary:
*  This function schedules the alarm by configuring the date and time on the RTC
*
* Parameters:
*  None
*
* Return:
*  cy_en_rtc_status_t returns the RTC status and following are the states
*       CY_RTC_SUCCESS      : Time and date configuration is successfully done.
*       CY_RTC_BAD_PARAM    : Date values are not valid.
*       CY_RTC_TIMEOUT      : Timeout occurred.
*       CY_RTC_INVALID_STATE: RTC is busy state.
*       CY_RTC_UNKNOWN      : Unknown failure.
*
******************************************************************************/
cy_en_rtc_status_t rtc_alarmconfig(void)
{
    uint32_t attempts = MAX_ATTEMPTS;
    cy_en_rtc_status_t rtc_result;

    /* Print the RTC alarm time by UART */
    debug_printf("RTC alarm will be generated after 1 second\r\n");

    /* Setting the alarm can fail. For example the RTC might be busy.
       Check the result and try again, if necessary. */
    do
    {
        rtc_result = Cy_RTC_SetAlarmDateAndTime((cy_stc_rtc_alarm_t *)&alarm_config, CY_RTC_ALARM_2);
        attempts--;
        Cy_SysLib_Delay(INIT_DELAY_MS);
    } while(( rtc_result != CY_RTC_SUCCESS) && (attempts != 0u));

    return (rtc_result);
}

/*******************************************************************************
* Function Name: get_switch_event
********************************************************************************
* Summary:
*  Returns how the User button was pressed:
*  - SWITCH_NO_EVENT: No press
*  - SWITCH_SHORT_PRESS: Short press was detected
*  - SWITCH_LONG_PRESS: Long press was detected
*
* Parameter:
*  void
*
* Return:
*  Switch event that occurred, if any.
*
*******************************************************************************/
en_switch_event_t get_switch_event(void)
{
    en_switch_event_t event = SWITCH_NO_EVENT;
    uint32_t press_count = 0;

    /* Check if User button is pressed */
    while (Cy_GPIO_Read(CYBSP_USER_BTN_PORT,CYBSP_USER_BTN_PIN) == CYBSP_BTN_PRESSED)
    {
        /* Wait for 10 ms */
        Cy_SysLib_Delay(SHORT_GLITCH_DELAY_MS);

        /* Increment counter. Each count represents 10 ms */
        press_count++;
    }

    /* Check for how long the button was pressed */
    if (press_count > LONG_PRESS_COUNT)
    {
        event = SWITCH_LONG_PRESS;
    }
    else if (press_count > SHORT_PRESS_COUNT)
    {
        event = SWITCH_SHORT_PRESS;
    }

    /* Add a delay to avoid glitches */
    Cy_SysLib_Delay(SHORT_GLITCH_DELAY_MS);

    return event;
}

/*******************************************************************************
* Function Name: debug_printf
********************************************************************************
* Summary:
* This function prints out the current date time and user string.
*
* Parameters:
*  str      Point to the user print string.
*
* Return:
*  void
*
*******************************************************************************/
 void debug_printf(const char *str)
{
    cy_stc_rtc_config_t dateTime;

    /* Get the current time and date from the RTC peripheral */
    Cy_RTC_GetDateAndTime(&dateTime);

    /*Convert RTC int values to string*/
    convert_date_to_string(&dateTime);

    /* Print the current date and time and user string */
    printf("%s: %s\r\n", buffer, str);
}

/*******************************************************************************
* Function Name: convert_date_to_string
********************************************************************************
* Summary:
*  This functions get the RTC time values from 'dateTime', convert the uint32_t
*  values to chars, then combine all chars to one string and save in 'buffer'
*
* Parameter:
*  cy_stc_rtc_config_t *dateTime : the RTC configure struct pointer
*  function
*
* Return:
*  void
*******************************************************************************/
 void convert_date_to_string(cy_stc_rtc_config_t *dateTime)
{
    /* Read out RTC time values */
    uint32_t sec, min, hour, day, month, year;
    sec = dateTime->sec;      /*value range is 0-59*/
    min = dateTime->min;      /*value range is 0-59*/
    hour = dateTime->hour;     /*0-23 or 1-12*/
    day = dateTime->date;     /*date of month, 1-31*/
    month = dateTime->month;      /*1-12*/
    year = dateTime ->year;     /*base value is 2000. value range is 0-99*/

    /* Convert uint32_t values to chars */
    char secbuf[2], minbuf[2], hourbuf[2], daybuf[2], monthbuf[2], yearbuf[2];
    sprintf(secbuf, "%d", (int)sec);
    sprintf(minbuf, "%d", (int)min);
    sprintf(hourbuf, "%d", (int)hour);
    sprintf(daybuf, "%d", (int)day);
    sprintf(monthbuf, "%d", (int)month);
    sprintf(yearbuf, "%d", (int)year);

    /* Merge all chars to one string */
    snprintf(buffer, sizeof(buffer), "%s %s %s %s %s %s %s %s %s %s %s",
            hourbuf, ":", minbuf, ":", secbuf,"", yearbuf, "-", monthbuf, "-", daybuf);


}

 /******************************************************************************
 * Function Name: rtc_interrupt_handler
 *******************************************************************************
 *
 * Summary:
 *  This is the general RTC interrupt handler in CPU NVIC.
 *  It calls the Alarm2 interrupt handler if that is the interrupt that occurs.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
 ******************************************************************************/
 void rtc_interrupt_handler(void)
 {
     /* No DST parameters are required for the custom tick. */
     Cy_RTC_Interrupt(NULL, false);

 }

 /******************************************************************************
 * Function Name: Cy_RTC_Alarm2Interrupt
 *******************************************************************************
 *
 * Summary:
 *  The function overrides the __WEAK Cy_RTC_Alarm2Interrupt() in cy_rtc.c to
 *  handle CY_RTC_ALARM_2 interrupt.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
 ******************************************************************************/
 void Cy_RTC_Alarm2Interrupt(void)
 {
     /* the interrupt has fired, meaning time expired and the alarm went off */
     alarm_flag = 1u;
 }

/* [] END OF FILE */
