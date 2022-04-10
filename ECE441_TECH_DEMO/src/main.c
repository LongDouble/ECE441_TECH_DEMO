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
#include <stdio.h>
#include <string.h>

// LED defines
#define TEST_LED				PIO_PA3_IDX

void led_setup(void);
void rtt_setup(void);

int main (void)
{
	
	/* Insert system clock initialization code here (sysclk_init()). */
	board_init();
	sysclk_init();
	
	// Disable watchdog timer to stop the MCU from resetting
	WDT->WDT_MR |= WDT_MR_WDDIS;
	
	/* Insert application code here, after the board has been initialized. */
	ioport_init();
	
	// Call setup routines (leaves LED off)
	led_setup();
	
	// Set up RTT
	rtt_setup();
	
	// Set wakeup mode to use RTT alarm
	supc_set_wakeup_mode(SUPC, SUPC_WUMR_RTTEN);
		
	// Go into backup mode
	supc_enable_backup_mode(SUPC);
	
	for(;;)
	{
		
	}
	
}

/* USER DEFINED FUNCTIONS */

// Sets up the LED
void led_setup(void)
{
	// Configure TEST_LED I/O pin and set high
	ioport_set_pin_dir(TEST_LED, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(TEST_LED, 0);
}

void rtt_setup(void)
{
	// Enable interrupt
	NVIC_EnableIRQ(RTT_IRQn);
	
	// Setup RTT to run at 1hz from RTC
	rtt_init(RTT, RTT_MR_RTC1HZ);
	
	// 10 seconds? (it's more like 13?)
	rtt_write_alarm_time(RTT, 4);
	
	// Enable RTT interrupt
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
}

/* INTERRUPTS */

void RTT_Handler( void) {
    // reading status register will clear interrupt flags
    uint32_t status = rtt_get_status(RTT);
	
    if ((status & RTT_SR_ALMS) >= 1){//ALMS generated an interrupt		
        /*
        Run the Code that you want to run every interrupt
        */
		
		ioport_toggle_pin_level(TEST_LED); // Turn LED on
		delay_ms(200);
		ioport_toggle_pin_level(TEST_LED); // Turn LED off
		delay_ms(200);
		ioport_toggle_pin_level(TEST_LED); // Turn LED on
		delay_ms(200);
		ioport_toggle_pin_level(TEST_LED); // Turn LED off
		delay_ms(200);
		ioport_toggle_pin_level(TEST_LED); // Turn LED on
		delay_ms(200);
		ioport_toggle_pin_level(TEST_LED); // Turn LED off
		delay_ms(200);
		
        // reset RTT counter
        REG_RTT_MR |= RTT_MR_RTTRST;
		
        // upon interrupt REG_RTT_AR.RTT_AR_ALMV gets set to the maximum value
        // we need to disable alarm to set a new value in ALMV
        REG_RTT_MR &= ~RTT_MR_ALMIEN;
		rtt_write_alarm_time(RTT, 4);  // 10 seconds? (it's more like 13?)
		
        //turn interrupt back on
        REG_RTT_MR |= RTT_MR_ALMIEN;
	}	
}