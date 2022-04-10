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
#define SQUARE_PIN				PIO_PA0_IDX

// UART defines
#define UART_SERIAL_BAUDRATE	115200
#define UART_SERIAL_MODE		UART_MR_PAR_NO
#define BUFFER_SIZE				256

// ADC defines
#define ADC_CLOCK				1000000

// Globals
uint32_t adc_result;

// User-defined functions
void uart_print(Uart *p_uart, char* message);
void led_setup(void);
void square_setup(void);
void uart_setup(void);
void adc_setup(void);
void rtt_setup(void);

int main (void)
{
	char buffer[BUFFER_SIZE];	// UART message to transmit
	float voltage;				// Voltage measured by ADC
	uint32_t binary[2];			// Binary representation of ADC count
	
	/* Insert system clock initialization code here (sysclk_init()). */
	board_init();
	sysclk_init();
	
	// Disable watchdog timer to stop the MCU from resetting
	WDT->WDT_MR |= WDT_MR_WDDIS;
	
	/* Insert application code here, after the board has been initialized. */
	ioport_init();
	
	// Call setup routines
	led_setup();
	rtt_setup();
	//square_setup();
	//uart_setup();
	//adc_setup();
	
	for(;;)
	{
		
		// Toggle the LED
		delay_ms(1000);
		
		
		//pmc_switch_mck_to_sclk(PMC_MCKR_PRES_CLK_1);
		//pmc_enable_backupmode();
	}
	
}

/* USER DEFINED FUNCTIONS */

// Sets up the LED
void led_setup(void)
{
	// Configure TEST_LED I/O pin and set high
	ioport_set_pin_dir(TEST_LED, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(TEST_LED, 1);
}

void rtt_setup(void)
{
	NVIC_EnableIRQ(RTT_IRQn);
	//setup rtt to run at 1hz from RTC
	rtt_init(RTT, RTT_MR_RTC1HZ);
	// 10 seconds?
	rtt_write_alarm_time(RTT, 4);
	// enable interrupt
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
}

void square_setup(void)
{
	// Configure TEST_LED I/O pin and set high
	ioport_set_pin_dir(SQUARE_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(SQUARE_PIN, 1);
}

// Sets up the UART0
void uart_setup(void)
{
	// Defines master clock rate, baud rate, and mode (parity or no)
	const sam_uart_opt_t uart0_settings = {sysclk_get_cpu_hz(), UART_SERIAL_BAUDRATE, UART_SERIAL_MODE};
		
	// Set pins to use the UART peripheral
	ioport_set_port_mode(IOPORT_PIOA,  PIO_PA9A_URXD0 | PIO_PA10A_UTXD0, IOPORT_MODE_MUX_A);
	ioport_disable_port(IOPORT_PIOA, PIO_PA9A_URXD0 | PIO_PA10A_UTXD0);
		
	// Enable UART0 clock
	pmc_enable_periph_clk(ID_UART0);
		
	// Initialize UART and enable TX
	uart_init(UART0, &uart0_settings); 
	uart_enable_tx(UART0);
}

// Sets up the ADC1
void adc_setup(void)
{
	// Enable ADC clock
	pmc_enable_periph_clk(ID_ADC);
	
	// Initialize ADC with 1 MHz clock
	adc_init(ADC, sysclk_get_main_hz(), ADC_CLOCK, 8);
	
	// Configure ADC timing
	adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);
	
	// Configure ADC resolution
	adc_set_resolution(ADC, ADC_MR_LOWRES_BITS_12);
	
	// Enable ADC1
	adc_enable_channel(ADC, ADC_CHANNEL_1);
	
	// Enable Data Ready Interrupt
	adc_enable_interrupt(ADC, ADC_IER_DRDY);
	
	// Set ADC to use software trigger
	adc_configure_trigger(ADC, ADC_TRIG_SW, 0);
	
	// Enable ADC_Handler interrupt
	NVIC_EnableIRQ(ADC_IRQn);
}

// Sends every character of message to the UART
void uart_print(Uart *p_uart, char* message){
	
	// For every character in the message...
	for(uint32_t i = 0; i < strlen(message); i++)
	{
		// Wait for UART to be ready
		while (!(p_uart->UART_SR & UART_SR_TXRDY));
		
		uart_write(p_uart, (uint8_t)(message[i]));
	}
}

/* INTERRUPTS */

void RTT_Handler( void) {
    //reading status register will clear interrupt flags
    uint32_t status = REG_RTT_SR;
    if ((status & RTT_SR_ALMS) >= 1){//ALMS generated an interrupt		
        /*
        Run the Code that you want to run every interrupt
        */
		ioport_toggle_pin_level(TEST_LED); // Toggle
		
        //reset RTT counter
        REG_RTT_MR |= RTT_MR_RTTRST;
        //upon interrupt REG_RTT_AR.RTT_AR_ALMV gets set to the maximum value
        //we need to disable alarm to set a new value in ALMV
        REG_RTT_MR &= ~RTT_MR_ALMIEN;
        REG_RTT_AR = 4;
        //turn interrupt back on
        REG_RTT_MR |= RTT_MR_ALMIEN;
	}	
}

// Called during any enabled ADC interrupt
void ADC_Handler(void)
{
	// Check the ADC conversion status
	if ((adc_get_status(ADC) & ADC_ISR_DRDY) == ADC_ISR_DRDY)
	{
		// Get latest digital data value from ADC and can be used by application
		adc_result = adc_get_latest_value(ADC);
	}
}




