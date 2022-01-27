/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */
#include <asf.h>
#define TEST_LED PIO_PA3_IDX

int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */

	board_init();
	sysclk_init();
	ioport_init();
	
	// Disable watchdog timer to stop the MCU from resetting
	WDT->WDT_MR |= WDT_MR_WDDIS;
	
	ioport_set_pin_dir(TEST_LED, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(TEST_LED, 1);

	/* Insert application code here, after the board has been initialized. */
	for(;;)
	{
		delay_ms(1000);
		ioport_toggle_pin_level(TEST_LED);
	}
	
}
