/***************************************************************************//**
* \file main.c
* \version 1.0
*
* \brief
* Compatible Kits:
*   CY8CKIT-062-BLE
*   CY8CKIT-062-WIFI-BT
*	GPIO and UART configuration files are not included
*	
*	This code is tested on CY8CKIT-062-WIFI-BT using Modus toolbox and PSoC creator
*	
*******************************************************************************/

#include "cy_pdl.h"
#include "cycfg.h"
#include <stdio.h>
#include "DHT22.h"

/* Variable used for storing UART context */
static cy_stc_scb_uart_context_t KP_UART_context;

static DHT_config DHTsensor;

void ERROR_handler (void);

int main(void)
{
	char outputstring[100];

	/* Set up internal routing, pins, and clock-to-peripheral connections */
    init_cycfg_all();

    /* enable interrupts */
    __enable_irq();

	/* You need to configure the UART port connected with Kitprog */
	Cy_SCB_UART_Init(KP_UART_HW, &KP_UART_config, &KP_UART_context);
	Cy_SCB_UART_Enable(KP_UART_HW);

	Cy_SCB_UART_PutString(KP_UART_HW, "PSoC 6 MCU: DHT22\r\n");

	/* DHT sensor is connected at Arduino header pin D4 or P5.4 of PSoC62 */
	/* You need to configure the GPIO pin as Resistor Pullup with Input Buffer*/
	DHTsensor.pinNum = DHT22_PIN;
	DHTsensor.portNum = DHT22_PORT;
	DHTsensor.CtoF = 0;
	DHTsensor.sensor_type = DHT22;
	DHTsensor.MAX_cycles = 2000;

	if (DHT_begin(&DHTsensor) != DHT_SUCCESS)
		ERROR_handler();

    CyDelay(1000/*msec*/);
    for (;;)
    {
    	read_Temperature_Humidity (&DHTsensor);

    	sprintf(outputstring,"Temperature=%.2f C Humidity= %.2f %% \r\n" ,DHTsensor.temperature, DHTsensor.humidity);
    	Cy_SCB_UART_PutString(KP_UART_HW, outputstring);

    	CyDelay(2000/*msec*/);
    }
}

void ERROR_handler (void){
	Cy_GPIO_Inv(LED_RED_PORT, LED_RED_PIN); /* toggle the pin */
    CyDelay(500/*msec*/);
}

