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
void uart_setup(void);
void adc_setup(void);

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
	uart_setup();
	adc_setup();
	
	for(;;)
	{
		// Start a single ADC conversion
		adc_start(ADC);
		
		// Wait 100 ms
		delay_ms(100);
		
		// Convert ADC result into a voltage
		voltage = ((float)(adc_result))/(4095.0) * 3.3;
		
		// Get upper 6 bits of ADC result
		binary[1] = ((adc_result >> 10) & 1)*100000;
		binary[1] += ((adc_result >> 9) & 1)*10000;
		binary[1] += ((adc_result >> 8) & 1)*1000;
		binary[1] += ((adc_result >> 7) & 1)*100;
		binary[1] += ((adc_result >> 6) & 1)*10;
		binary[1] += ((adc_result >> 5) & 1)*1;
		
		// Get lower 5 bits of ADC result
		binary[0] = ((adc_result >> 4) & 1)*10000;
		binary[0] += ((adc_result >> 3) & 1)*1000;
		binary[0] += ((adc_result >> 2) & 1)*100;
		binary[0] += ((adc_result >> 1) & 1)*10;
		binary[0] += ((adc_result) & 1)*1;
		
		// Format a message to send over UART0
		snprintf(buffer, BUFFER_SIZE, "ADC Count[(%lu)], Binary[0b%06lu%05lu], Voltage[%f]\r\n", adc_result, binary[1], binary[0], voltage);
		
		// Send the message over UART0
		uart_print(UART0, buffer);
		
		// Toggle the LED
		ioport_toggle_pin_level(TEST_LED);
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


