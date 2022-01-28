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

#define TEST_LED PIO_PA3_IDX
#define BUFFER_SIZE 256

// UART defines
#define UART_SERIAL_BAUDRATE			115200
#define UART_SERIAL_CHANNEL_MODE		UART_MR_CHMODE_NORMAL	// Normal channel mode
#define UART_SERIAL_MODE				UART_MR_PAR_NO			// No parity bit
#define PINS_UART0          (PIO_PA9A_URXD0 | PIO_PA10A_UTXD0)	// Easier name for UART pins
#define PINS_UART0_FLAGS    (PIO_PERIPH_A | PIO_DEFAULT)		// Use periph A with no PU, filtering, or open drain
#define PINS_UART0_MASK     (PIO_PA9A_URXD0 | PIO_PA10A_UTXD0)
#define PINS_UART0_PIO      PIOA
#define PINS_UART0_ID       ID_PIOA
#define PINS_UART0_TYPE     PIO_PERIPH_A
#define PINS_UART0_ATTR     PIO_DEFAULT

// ADC defines
#define ADC_CLOCK 1000000

// Globals
uint32_t adc_result;

void ADC_Handler(void)
{
	// Check the ADC conversion status
	if ((adc_get_status(ADC) & ADC_ISR_DRDY) == ADC_ISR_DRDY)
	{
		// Get latest digital data value from ADC and can be used by application
		adc_result = adc_get_latest_value(ADC);
	}
}

void uart_print(Uart *p_uart, char* message);
void adc_setup(void);

int main (void)
{
	char buffer[BUFFER_SIZE];
	/* Insert system clock initialization code here (sysclk_init()). */
	
	// Initialize board stuff
	board_init();
	sysclk_init();
	
	// Disable watchdog timer to stop the MCU from resetting
	WDT->WDT_MR |= WDT_MR_WDDIS;
	
	// Configure TEST_LED I/O pin and set high
	ioport_init();
	ioport_set_pin_dir(TEST_LED, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(TEST_LED, 1);
	
	/* Configure UART0 (PA9 = RX, PA10 = TX) */
	// Set the pins to use the UART peripheral
	pio_configure(PINS_UART0_PIO, PINS_UART0_TYPE, PINS_UART0_MASK, PINS_UART0_ATTR);
	
	// Enable UART0 clock
	pmc_enable_periph_clk(ID_UART0);
	pmc_enable_periph_clk(ID_ADC);
	
	// Defines master clock rate, baud rate, and mode (parity or no)
	const sam_uart_opt_t uart0_settings = {sysclk_get_cpu_hz(), UART_SERIAL_BAUDRATE, UART_SERIAL_MODE};
		
	uart_init(UART0, &uart0_settings);      // Init UART0
	uart_enable_tx(UART0);
	
	// Setup and start ADC
	adc_setup();

	/* Insert application code here, after the board has been initialized. */
	for(;;)
	{
		adc_start(ADC);
		delay_ms(100);

		snprintf(buffer, BUFFER_SIZE, "Hello world! (%lu)\r\n", adc_result);
		uart_print(UART0, buffer);

		ioport_toggle_pin_level(TEST_LED);
	}
	
}

void adc_setup(void)
{
	adc_init(ADC, sysclk_get_main_hz(), ADC_CLOCK, 8);
	adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);
	adc_set_resolution(ADC, ADC_MR_LOWRES_BITS_12);
	adc_enable_channel(ADC, ADC_CHANNEL_1);
	adc_enable_interrupt(ADC, ADC_IER_DRDY);
	adc_configure_trigger(ADC, ADC_TRIG_SW, 0);
	NVIC_EnableIRQ(ADC_IRQn);
}

void uart_print(Uart *p_uart, char* message){
	
	// For every character in the message...
	for(uint32_t i = 0; i < strlen(message); i++)
	{
		// Wait for UART to be ready
		while (!(p_uart->UART_SR & UART_SR_TXRDY));
		
		uart_write(p_uart, (uint8_t)(message[i]));
	}
}


