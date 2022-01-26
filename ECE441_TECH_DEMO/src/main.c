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

int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */

	board_init();
	sysclk_init();
	
	// Disable watchdog timer to stop the MCU from resetting
	WDT->WDT_MR |= WDT_MR_WDDIS;
	
	pmc_enable_periph_clk(ID_PIOA);
	pio_set_output(PIOA, PIO_PA3, HIGH, DISABLE, ENABLE);
	pio_set(PIOA, PIO_PA3);
	

	/* Insert application code here, after the board has been initialized. */
	bool is_on = 1;
	for(;;)
	{
		if(is_on)
			pio_clear(PIOA, PIO_PA3);
		else
			pio_set(PIOA, PIO_PA3);
		
		is_on = !is_on;
		
		delay_ms(1000);
	}
	
}
