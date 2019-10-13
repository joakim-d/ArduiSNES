/*
             LUFA Library
     Copyright (C) Dean Camera, 2017.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2017  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the Joystick demo. This file contains the main tasks of the demo and
 *  is responsible for the initial application hardware configuration.
 */

#include "Joystick.h"

#define CONTROLLER_1_CLOCK_ON   PORTB |= (1 << PB6) //10 (output)
#define CONTROLLER_1_CLOCK_OFF  PORTB &= ~(1 << PB6) //10 (output)
#define CONTROLLER_1_STROBE_ON  PORTB |= (1 << PB5) //9 (output)
#define CONTROLLER_1_STROBE_OFF PORTB &= ~(1 << PB5) //9 (output)
#define CONTROLLER_1_DATA       (PINB & (1 << PB4)) //8 (input)

#define CONTROLLER_2_CLOCK_ON   PORTC |= (1 << 7) //13 (output)
#define CONTROLLER_2_CLOCK_OFF  PORTC &= ~(1 << 7) //13 (output)
#define CONTROLLER_2_STROBE_ON  PORTD |= (1 << 6) //12 (output)
#define CONTROLLER_2_STROBE_OFF PORTD &= ~(1 << 6) //12 (output)
#define CONTROLLER_2_DATA       PINB & (1 << 7) //11 (input)

#define CONTROLLER_3_CLOCK_ON   PORTE |= (1 << PE6) //7 (output)
#define CONTROLLER_3_CLOCK_OFF  PORTE &= ~(1 << PE6) //7 (output)
#define CONTROLLER_3_STROBE_ON  PORTD |= (1 << PD7) //6 (output)
#define CONTROLLER_3_STROBE_OFF PORTD &= ~(1 << PD7) //6 (output)
#define CONTROLLER_3_DATA       PINC & (1 << PC6) //5 (input)

#define CONTROLLER_4_CLOCK_ON   PORTD |= (1 << PD4) //4 (output)
#define CONTROLLER_4_CLOCK_OFF  PORTD &= ~(1 << PD4) //4 (output)
#define CONTROLLER_4_STROBE_ON  PORTD |= (1 << PD0) //3 (output)
#define CONTROLLER_4_STROBE_OFF PORTD &= ~(1 << PD0) //3 (output)
#define CONTROLLER_4_DATA       PIND & (1 << PD1) //2 (input)


static USB_JoystickReport_Data_t controllers_data[4];

/** Main program entry point. This routine configures the hardware required by the application, then
 *  enters a loop to run the application tasks in sequence.
 */
int main(void)
{
	SetupHardware();

	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
	GlobalInterruptEnable();

	for (;;)
	{
		HID_Task();
		USB_USBTask();
        _delay_ms(16);
	}
}

void IOs_Init(void) {
    //Configure IO directions
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
    PORTE = 0;
    DDRB = (1 << PB5) | (1 << PB6);
    DDRC = (1 << PC7);
    DDRD = (1 << PD0) | (1 << PD4) | (1 << PD6) | (1 << PD7);
    DDRE = (1 << PE6);
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
#if (ARCH == ARCH_AVR8)
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);
#elif (ARCH == ARCH_XMEGA)
	/* Start the PLL to multiply the 2MHz RC oscillator to 32MHz and switch the CPU core to run from it */
	XMEGACLK_StartPLL(CLOCK_SRC_INT_RC2MHZ, 2000000, F_CPU);
	XMEGACLK_SetCPUClockSource(CLOCK_SRC_PLL);

	/* Start the 32MHz internal RC oscillator and start the DFLL to increase it to 48MHz using the USB SOF as a reference */
	XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);
	XMEGACLK_StartDFLL(CLOCK_SRC_INT_RC32MHZ, DFLL_REF_INT_USBSOF, F_USB);

	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
#endif

	/* Hardware Initialization */
	LEDs_Init();
	USB_Init();
    IOs_Init();
}


/** Event handler for the USB_Connect event. This indicates that the device is enumerating via the status LEDs and
 *  starts the library USB task to begin the enumeration and USB management process.
 */
void EVENT_USB_Device_Connect(void)
{
	/* Indicate USB enumerating */
	LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the USB_Disconnect event. This indicates that the device is no longer connected to a host via
 *  the status LEDs and stops the USB management and joystick reporting tasks.
 */
void EVENT_USB_Device_Disconnect(void)
{
	/* Indicate USB not ready */
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the USB_ConfigurationChanged event. This is fired when the host set the current configuration
 *  of the USB device after enumeration - the device endpoints are configured and the joystick reporting task started.
 */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	/* Setup HID Report Endpoint */
    ConfigSuccess &= Endpoint_ConfigureEndpoint(JOYSTICK_EPADDR_1, EP_TYPE_INTERRUPT, JOYSTICK_EPSIZE, 1);
    ConfigSuccess &= Endpoint_ConfigureEndpoint(JOYSTICK_EPADDR_2, EP_TYPE_INTERRUPT, JOYSTICK_EPSIZE, 1);
    ConfigSuccess &= Endpoint_ConfigureEndpoint(JOYSTICK_EPADDR_3, EP_TYPE_INTERRUPT, JOYSTICK_EPSIZE, 1);
    ConfigSuccess &= Endpoint_ConfigureEndpoint(JOYSTICK_EPADDR_4, EP_TYPE_INTERRUPT, JOYSTICK_EPSIZE, 1);


	/* Indicate endpoint configuration success or failure */
    LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

void getController1State(USB_JoystickReport_Data_t* data){
    cli();
    CONTROLLER_1_STROBE_ON; //latch pulse for 12 microseconds
    _delay_us(12);
    CONTROLLER_1_STROBE_OFF;

    data->nes_buttons = CONTROLLER_1_DATA ? 0 : 1;
    _delay_us(6);

    CONTROLLER_1_CLOCK_ON;
    _delay_us(6);
    data->nes_buttons |= CONTROLLER_1_DATA ? 0 : 2;
    CONTROLLER_1_CLOCK_OFF;
    _delay_us(6);

    CONTROLLER_1_CLOCK_ON;
    _delay_us(6);
    data->nes_buttons |= CONTROLLER_1_DATA  ? 0 : 4;
    CONTROLLER_1_CLOCK_OFF;
    _delay_us(6);

    CONTROLLER_1_CLOCK_ON;
    _delay_us(6);
    data->nes_buttons |= CONTROLLER_1_DATA  ? 0 : 8;
    CONTROLLER_1_CLOCK_OFF;
    _delay_us(6);

    CONTROLLER_1_CLOCK_ON;
    _delay_us(6);
    data->nes_buttons |= CONTROLLER_1_DATA ? 0 : 0x10;
    CONTROLLER_1_CLOCK_OFF;
    _delay_us(6);

    CONTROLLER_1_CLOCK_ON;
    _delay_us(6);
    data->nes_buttons |= CONTROLLER_1_DATA ? 0 : 0x20;
    CONTROLLER_1_CLOCK_OFF;
    _delay_us(6);

    CONTROLLER_1_CLOCK_ON;
    _delay_us(6);
    data->nes_buttons |= CONTROLLER_1_DATA ? 0 : 0x40;
    CONTROLLER_1_CLOCK_OFF;
    _delay_us(6);

    CONTROLLER_1_CLOCK_ON;
    _delay_us(6);
    data->nes_buttons |= CONTROLLER_1_DATA ? 0 : 0x80;
    CONTROLLER_1_CLOCK_OFF;
    _delay_us(6);

    CONTROLLER_1_CLOCK_ON;
    _delay_us(6);
    CONTROLLER_1_CLOCK_OFF;
    sei();
}

void getController2State(USB_JoystickReport_Data_t* data){
    cli();
    CONTROLLER_2_STROBE_ON; //latch pulse for 12 microseconds
    _delay_us(12);
    CONTROLLER_2_STROBE_OFF;

    data->nes_buttons = CONTROLLER_2_DATA ? 0 : 1;
    _delay_us(6);

    CONTROLLER_2_CLOCK_ON;
    _delay_us(6);
    data->nes_buttons |= CONTROLLER_2_DATA ? 0 : 2;
    CONTROLLER_2_CLOCK_OFF;
    _delay_us(6);

    CONTROLLER_2_CLOCK_ON;
    _delay_us(6);
    data->nes_buttons |= CONTROLLER_2_DATA  ? 0 : 4;
    CONTROLLER_2_CLOCK_OFF;
    _delay_us(6);

    CONTROLLER_2_CLOCK_ON;
    _delay_us(6);
    data->nes_buttons |= CONTROLLER_2_DATA  ? 0 : 8;
    CONTROLLER_2_CLOCK_OFF;
    _delay_us(6);

    CONTROLLER_2_CLOCK_ON;
    _delay_us(6);
    data->nes_buttons |= CONTROLLER_2_DATA ? 0 : 0x10;
    CONTROLLER_2_CLOCK_OFF;
    _delay_us(6);

    CONTROLLER_2_CLOCK_ON;
    _delay_us(6);
    data->nes_buttons |= CONTROLLER_2_DATA ? 0 : 0x20;
    CONTROLLER_2_CLOCK_OFF;
    _delay_us(6);

    CONTROLLER_2_CLOCK_ON;
    _delay_us(6);
    data->nes_buttons |= CONTROLLER_2_DATA ? 0 : 0x40;
    CONTROLLER_2_CLOCK_OFF;
    _delay_us(6);

    CONTROLLER_2_CLOCK_ON;
    _delay_us(6);
    data->nes_buttons |= CONTROLLER_2_DATA ? 0 : 0x80;
    CONTROLLER_2_CLOCK_OFF;
    _delay_us(6);

    CONTROLLER_2_CLOCK_ON;
    _delay_us(6);
    CONTROLLER_2_CLOCK_OFF;
    sei();
}

void getController3State(USB_JoystickReport_Data_t* data){
    cli();
    CONTROLLER_3_STROBE_ON; //latch pulse for 12 microseconds
    _delay_us(12);
    CONTROLLER_3_STROBE_OFF;

    data->nes_buttons = CONTROLLER_3_DATA ? 0 : 1;
    _delay_us(6);

    CONTROLLER_3_CLOCK_ON;
    _delay_us(6);
    data->nes_buttons |= CONTROLLER_3_DATA ? 0 : 2;
    CONTROLLER_3_CLOCK_OFF;
    _delay_us(6);

    CONTROLLER_3_CLOCK_ON;
    _delay_us(6);
    data->nes_buttons |= CONTROLLER_3_DATA  ? 0 : 4;
    CONTROLLER_3_CLOCK_OFF;
    _delay_us(6);

    CONTROLLER_3_CLOCK_ON;
    _delay_us(6);
    data->nes_buttons |= CONTROLLER_3_DATA  ? 0 : 8;
    CONTROLLER_3_CLOCK_OFF;
    _delay_us(6);

    CONTROLLER_3_CLOCK_ON;
    _delay_us(6);
    data->nes_buttons |= CONTROLLER_3_DATA ? 0 : 0x10;
    CONTROLLER_3_CLOCK_OFF;
    _delay_us(6);

    CONTROLLER_3_CLOCK_ON;
    _delay_us(6);
    data->nes_buttons |= CONTROLLER_3_DATA ? 0 : 0x20;
    CONTROLLER_3_CLOCK_OFF;
    _delay_us(6);

    CONTROLLER_3_CLOCK_ON;
    _delay_us(6);
    data->nes_buttons |= CONTROLLER_3_DATA ? 0 : 0x40;
    CONTROLLER_3_CLOCK_OFF;
    _delay_us(6);

    CONTROLLER_3_CLOCK_ON;
    _delay_us(6);
    data->nes_buttons |= CONTROLLER_3_DATA ? 0 : 0x80;
    CONTROLLER_3_CLOCK_OFF;
    _delay_us(6);

    CONTROLLER_3_CLOCK_ON;
    _delay_us(6);
    CONTROLLER_3_CLOCK_OFF;
    sei();
}

void getController4State(USB_JoystickReport_Data_t* data){
    cli();
    CONTROLLER_4_STROBE_ON; //latch pulse for 12 microseconds
    _delay_us(12);
    CONTROLLER_4_STROBE_OFF;

    data->nes_buttons = CONTROLLER_4_DATA ? 0 : 1;
    _delay_us(6);

    CONTROLLER_4_CLOCK_ON;
    _delay_us(6);
    data->nes_buttons |= CONTROLLER_4_DATA ? 0 : 2;
    CONTROLLER_4_CLOCK_OFF;
    _delay_us(6);

    CONTROLLER_4_CLOCK_ON;
    _delay_us(6);
    data->nes_buttons |= CONTROLLER_4_DATA  ? 0 : 4;
    CONTROLLER_4_CLOCK_OFF;
    _delay_us(6);

    CONTROLLER_4_CLOCK_ON;
    _delay_us(6);
    data->nes_buttons |= CONTROLLER_4_DATA  ? 0 : 8;
    CONTROLLER_4_CLOCK_OFF;
    _delay_us(6);

    CONTROLLER_4_CLOCK_ON;
    _delay_us(6);
    data->nes_buttons |= CONTROLLER_4_DATA ? 0 : 0x10;
    CONTROLLER_4_CLOCK_OFF;
    _delay_us(6);

    CONTROLLER_4_CLOCK_ON;
    _delay_us(6);
    data->nes_buttons |= CONTROLLER_4_DATA ? 0 : 0x20;
    CONTROLLER_4_CLOCK_OFF;
    _delay_us(6);

    CONTROLLER_4_CLOCK_ON;
    _delay_us(6);
    data->nes_buttons |= CONTROLLER_4_DATA ? 0 : 0x40;
    CONTROLLER_4_CLOCK_OFF;
    _delay_us(6);

    CONTROLLER_4_CLOCK_ON;
    _delay_us(6);
    data->nes_buttons |= CONTROLLER_4_DATA ? 0 : 0x80;
    CONTROLLER_4_CLOCK_OFF;
    _delay_us(6);

    CONTROLLER_4_CLOCK_ON;
    _delay_us(6);
    CONTROLLER_4_CLOCK_OFF;
    sei();
}

/** Event handler for the USB_ControlRequest event. This is used to catch and process control requests sent to
 *  the device from the USB host before passing along unhandled control requests to the library for processing
 *  internally.
 */
void EVENT_USB_Device_ControlRequest(void)
{
	/* Handle HID Class specific requests */

	switch (USB_ControlRequest.bRequest)
	{
		case HID_REQ_GetReport:
			if (USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				/* Create the next HID report to send to the host */
                uint8_t* report_data;
				Endpoint_ClearSETUP();
                switch(USB_ControlRequest.wIndex){
                    case 0: report_data = (uint8_t*)&controllers_data[0].nes_buttons; break;
                    case 1: report_data = (uint8_t*)&controllers_data[1].nes_buttons; break;
                    case 2: report_data = (uint8_t*)&controllers_data[2].nes_buttons; break;
                    case 3: report_data = (uint8_t*)&controllers_data[3].nes_buttons; break;
                    default: return;
                }

				/* Write the report data to the control endpoint */
                Endpoint_Write_Control_Stream_LE(report_data, 1);
				Endpoint_ClearOUT();
			}

			break;
	}
}

/** Function to manage HID report generation and transmission to the host. */
void HID_Task(void)
{
	/* Device must be connected and configured for the task to run */
	if (USB_DeviceState != DEVICE_STATE_Configured)
	  return;

    getController1State(&controllers_data[0]);
    getController2State(&controllers_data[1]);
    getController3State(&controllers_data[2]);
    getController4State(&controllers_data[3]);

	/* Select the Joystick Report Endpoint */
    Endpoint_SelectEndpoint(JOYSTICK_EPADDR_1);

	/* Check to see if the host is ready for another packet */
	if (Endpoint_IsINReady())
	{
		/* Write Joystick Report Data */
        Endpoint_Write_Stream_LE(&controllers_data[0].nes_buttons, 1, NULL);

		/* Finalize the stream transfer to send the last packet */
		Endpoint_ClearIN();
	}

    /* Select the Joystick Report Endpoint */
    Endpoint_SelectEndpoint(JOYSTICK_EPADDR_2);

    /* Check to see if the host is ready for another packet */
    if (Endpoint_IsINReady())
    {
        /* Write Joystick Report Data */
        Endpoint_Write_Stream_LE(&controllers_data[1].nes_buttons, 1, NULL);

        /* Finalize the stream transfer to send the last packet */
        Endpoint_ClearIN();
    }

    /* Select the Joystick Report Endpoint */
    Endpoint_SelectEndpoint(JOYSTICK_EPADDR_3);

    /* Check to see if the host is ready for another packet */
    if (Endpoint_IsINReady())
    {
        /* Write Joystick Report Data */
        Endpoint_Write_Stream_LE(&controllers_data[2].nes_buttons, 1, NULL);

        /* Finalize the stream transfer to send the last packet */
        Endpoint_ClearIN();
    }

    /* Select the Joystick Report Endpoint */
    Endpoint_SelectEndpoint(JOYSTICK_EPADDR_4);

    /* Check to see if the host is ready for another packet */
    if (Endpoint_IsINReady())
    {
        /* Write Joystick Report Data */
        Endpoint_Write_Stream_LE(&controllers_data[3].nes_buttons, 1, NULL);

        /* Finalize the stream transfer to send the last packet */
        Endpoint_ClearIN();
    }

}

