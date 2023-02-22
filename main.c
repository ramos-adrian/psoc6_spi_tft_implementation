/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty Application Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
********************************************************************************
* Copyright 2021-2022, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cy_pdl.h"
#include "GUI.h"
#include "LCDConf.h"
#include <stdio.h>

/*******************************************************************************
* Macros
*******************************************************************************/
#define GPIO_INTERRUPT_PRIORITY (7u)


/*******************************************************************************
* Function Prototypes
*******************************************************************************/

static void button_isr(void *handler_arg, cyhal_gpio_event_t event);


/*******************************************************************************
* Global Variables
*******************************************************************************/

const SPI_pins pins =
{
    .MOSI = P5_0,  //SDA
    .MISO = P5_1,  //MISO
    .SCK = P5_2,   //SCL
    .SS = P5_3,    //CS
    .dc = P9_7,    //RS
	.rst= P9_4,    //RST
};

/* GPIO callback initialization structure */
cyhal_gpio_callback_data_t cb_data =
{
.callback = button_isr,
.callback_arg = NULL
};

int segundos, minutos, horas;

bool pause = false;

cyhal_pwm_t pwm_obj;
/*******************************************************************************
* Function Definitions
*******************************************************************************/

/* Interrupt callback function */
static void button_isr(void *handler_arg, cyhal_gpio_event_t event)
{
	pause = !pause;

	cyhal_pwm_set_duty_cycle(&pwm_obj, 20, 700);
  	cyhal_pwm_start(&pwm_obj);
	CyDelay(100);
  	cyhal_pwm_stop(&pwm_obj);
	CyDelay(25);
  	cyhal_pwm_start(&pwm_obj);
	CyDelay(100);
  	cyhal_pwm_stop(&pwm_obj);

	cyhal_pwm_set_duty_cycle(&pwm_obj, 20, 1000);
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CPU. It...
*    1.
*    2.
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

#if defined (CY_DEVICE_SECURE)
    cyhal_wdt_t wdt_obj;

    /* Clear watchdog timer so that it doesn't trigger a reset */
    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(CY_RSLT_SUCCESS == result);
    cyhal_wdt_free(&wdt_obj);
#endif

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    result = SPI_init8(&pins);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    GUI_Init();
    GUI_SetBkColor(GUI_WHITE);
    GUI_Clear();
    GUI_SetColor(GUI_BLACK);
    GUI_DispStringAt("Cronometro", 5, 30);
    GUI_SetFont(GUI_FONT_10_ASCII);
    GUI_DispStringAt("Arroyo, Ramos", 80, 90);
    GUI_SetFont(GUI_FONT_32B_ASCII);


    cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT,
    		CYHAL_GPIO_DRIVE_STRONG, 0);

	/* Initialize PWM on the supplied pin and assign a new clock */
    cyhal_pwm_init(&pwm_obj, P8_0, NULL);
	cyhal_pwm_set_duty_cycle(&pwm_obj, 20, 500);

  	cyhal_pwm_start(&pwm_obj);
	CyDelay(200);
  	cyhal_pwm_stop(&pwm_obj);
	CyDelay(100);
  	result = cyhal_pwm_start(&pwm_obj);
	CyDelay(200);
  	cyhal_pwm_stop(&pwm_obj);

	result = cyhal_pwm_set_duty_cycle(&pwm_obj, 20, 1000);

    /* Initialize the button and setup the interrupt */
     cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT,
    CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);

     cyhal_gpio_register_callback(CYBSP_USER_BTN, &cb_data);
     cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL,
    GPIO_INTERRUPT_PRIORITY, true);

     __enable_irq();

	segundos = 0;
	minutos = 0;
	horas = 0;

    while(1)
    {
    	  if (pause) {
		      cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
    		  cyhal_pwm_stop(&pwm_obj);
    		  continue;
    	  }
		  if (segundos == 60){
			 minutos++;
			 segundos = 0;
		  }

		  if (minutos == 60){
			 horas++;
			 minutos = 0;
		  }

		  char buffer[16];
		  sprintf(buffer, "%02d:%02d:%02d", horas, minutos, segundos);
		  GUI_DispStringAt(buffer, 5, 53);

	      cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
		  cyhal_pwm_stop(&pwm_obj);

		  CyDelay(500);

		  if (!pause) {
			  segundos++;
		      cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
			  CyDelay(500);
			  cyhal_pwm_start(&pwm_obj);
		  }

    }

}

/* [] END OF FILE */
