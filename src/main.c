// SPDX-License-Identifier: Apache-2.0
// Copyright: Gabriel Marcano, 2023

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#include <string.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include <uart.h>
#include <adc.h>
#include <spi.h>
#include <lora.h>
#include <gpio.h>

#define CHECK_ERRORS(x)\
	if ((x) != AM_HAL_STATUS_SUCCESS)\
	{\
		error_handler(x);\
	}

static void error_handler(uint32_t error)
{
	(void)error;
	for(;;)
	{
		am_devices_led_on(am_bsp_psLEDs, 0);
		am_util_delay_ms(500);
		am_devices_led_off(am_bsp_psLEDs, 0);
		am_util_delay_ms(500);
	}
}

static struct uart uart;
static struct adc adc;
static struct gpio lora_power;
static struct lora lora;

int main(void)
{
	// Prepare MCU by init-ing clock, cache, and power level operation
	am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);
	am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
	am_hal_cachectrl_enable();
	am_bsp_low_power_init();
	am_hal_sysctrl_fpu_enable();
	am_hal_sysctrl_fpu_stacking_enable(true);

	// Init UART, registers with SDK printf
	uart_init(&uart, UART_INST0);

	// Initialize the ADC.
	adc_init(&adc);

	// After init is done, enable interrupts
	am_hal_interrupt_master_enable();

	// Print the banner.
	am_util_stdio_terminal_clear();
	am_util_stdio_printf("Hello World!\r\n\r\n");

	// Trigger the ADC to start collecting data
	adc_trigger(&adc);

	gpio_init(&lora_power, 10, false);
	//lora_receive_mode(&lora);

	// Wait here for the ISR to grab a buffer of samples.
	while (1)
	{
		// Print the battery voltage and temperature for each interrupt
		//
		uint32_t data = 0;
		if (adc_get_sample(&adc, &data))
		{
			// The math here is straight forward: we've asked the ADC to give
			// us data in 14 bits (max value of 2^14 -1). We also specified the
			// reference voltage to be 1.5V. A reading of 1.5V would be
			// translated to the maximum value of 2^14-1. So we divide the
			// value from the ADC by this maximum, and multiply it by the
			// reference, which then gives us the actual voltage measured.
			const double reference = 1.5;
			double voltage = data * reference / ((1 << 14) - 1);

			double temperature = 5.506 - sqrt((-5.506)*(-5.506) + 4 * 0.00176 * (870.6 - voltage*1000));
			temperature /= (2 * -.00176);
			temperature += 30;

			gpio_set(&lora_power, true);
			// The documentation for the SX1276 states that it takes 10 ms for
			// the radio to come online from coldboot.
			am_util_delay_ms(10);
			// Only continue if we initialize
			// FIXME what if we never initialize?
			while(!lora_init(&lora, 915000000));
			lora_standby(&lora);
			lora_set_spreading_factor(&lora, 7);
			lora_set_coding_rate(&lora, 1);
			lora_set_bandwidth(&lora, 0x7);

			unsigned char buffer[64];
			int magnitude = 10000;
			snprintf((char*)buffer, sizeof(buffer),
				"{ \"temperature\": %i, \"magnitude\": %i }",
				(int)(temperature * magnitude),
				magnitude);
			lora_send_packet(&lora, buffer, strlen((char*)buffer));
			if (lora_rx_amount(&lora))
			{
				am_util_stdio_printf("length %i\r\n", lora_rx_amount(&lora));
				lora_receive_packet(&lora, buffer, 32);
				am_util_stdio_printf("Data: %s\r\n", buffer);
			}
			lora_destroy(&lora);
			gpio_set(&lora_power, false);
		}

		// Sleep here until the next ADC interrupt comes along.
		am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
	}
}
