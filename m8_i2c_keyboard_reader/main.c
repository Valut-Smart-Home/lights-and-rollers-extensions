// avrdude -p m8 -c usbasp -U flash:w:m8_i2c_keyboard_reader.hex:i
// avrdude -p m8 -c usbasp -U lfuse:w:0xe2:m

#define F_CPU 2000000UL
// Have to be > 16*F_scl, for 100 kHz it should be greather than 1.6 MHz

#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>
#include <avr/interrupt.h>

#define broken_communication_time_ms 200
#define filter_count 5

#define P_SCL PC5
#define P_SDA PC4

uint8_t input_no = 0;
volatile uint8_t inputs[8];
uint8_t input_keyb_no = 0;
uint8_t input_count[8][8];

volatile unsigned long currentTimeMs = 0;
volatile unsigned long lastCommunicationTimeMs = 0;
volatile unsigned long lastCommunicationResetTimeMs = 0;

void update_address()
{
	uint8_t address = PINC & ((1 << PC1) | (1 << PC2) | (1 << PC3));	// Get only PC1..PC3
	address ^= ((1 << PC1) | (1 << PC2) | (1 << PC3));					// Toggle states PC1..PC3
	address >>= 1;														// Make it fit to LSB
	address += 0x20;													// Add offset
	TWAR = (address << 1);												// Set TWI address
}

void init_twi_slave()
{
	DDRC  = (1 << PC0);						// Set input SDA, SCL, without PC0
	PORTC = ~((1 << P_SCL) | (1 << P_SDA));	// Set high impedance SDA, SCL - rest pull-up (PC0 high)
	_delay_loop_1(40);						// = 20 us with F_OSC = 2 MHz
	update_address();
	TWCR = (1 << TWEN)						// TWI Enable
		 | (1 << TWIE)						// TWI Interrupt Enable
		 | (1 << TWEA);						// TWI Acknowledge Enable
}

void inputs_handler_reset()
{
	input_no = 1;
	TWDR = inputs[0];
}

void inputs_handler()
{
	if (input_no >= 8)
	{
		TWDR = 0xFF;
		return;
	}
	
	TWDR = inputs[input_no];
	input_no++;
}

ISR(TWI_vect)
{
	switch (TW_STATUS)
	{
		case TW_ST_SLA_ACK:
			inputs_handler_reset();
			lastCommunicationTimeMs = currentTimeMs;
			break;
		case TW_ST_DATA_ACK:
			inputs_handler();
			lastCommunicationTimeMs = currentTimeMs;
			break;
		default:
		break;
	}

	TWCR |= (1 << TWINT);		// Clear TWI Interrupt flag
}

void init_keyboard()
{
	DDRD  = 0x00;	//Port D Input
	PORTD = 0xFF;	//PORT D Pull-up
	PORTB = 0x00;	//PORT B LOW
	DDRB  = 0xFF;	//PORT B Output
	
	for (uint8_t i = 0; i < 8; i++)
	{
		inputs[i] = 0;
		for (uint8_t j = 0; j < 8; j++)
		input_count[i][j] = 0;
	}
}

void handle_keyboard()
{
	input_keyb_no++;
	if (input_keyb_no >= 8)
		input_keyb_no = 0;
	
	PORTB = 1 << input_keyb_no;
	_delay_loop_1(40); // = 20 us with F_OSC = 2 MHz
	
	uint8_t new_inputs = 0x00;
	for (uint8_t i = 0; i < 8; i++)
	{
		if (PIND & (1 << i))
		{
			input_count[input_keyb_no][i] = 0;
		}
		else
		{
			if (input_count[input_keyb_no][i] < filter_count)
				input_count[input_keyb_no][i]++;
			else
				new_inputs |= 1 << i;
		}
	}
	inputs[input_keyb_no] = new_inputs;
}

void timer1_init()
{
	TCCR1B |= (1 << WGM12); // Set Timer1 to CTC mode
	TCCR1B |= (1 << CS11);	// Set prescaler to 8
	TCNT1 = 0;
	OCR1A = 249;			// Set compare value for 1ms interval
	TIMSK |= (1 << OCIE1A); // Enable Timer1 compare match interrupt
}

ISR(TIMER1_COMPA_vect)
{
	currentTimeMs++;
}

void check_lastCommunicationTime()
{
	if (currentTimeMs - lastCommunicationTimeMs < broken_communication_time_ms)
	{
		PORTC &= ~(1 << PC0);
		return;
	}
	
	PORTC |= (1 << PC0);
	if (currentTimeMs - lastCommunicationResetTimeMs > broken_communication_time_ms)
	{
		TWCR |= (1 << TWSTO);
		lastCommunicationResetTimeMs = currentTimeMs;
	}
}

int main(void)
{
	init_keyboard();		// Initialize keyboard
	init_twi_slave();		// Initialize TWI
	timer1_init();
	sei();					// Enable interrupts
	
	while(1)
	{
		handle_keyboard();	// Read keyboard, set next to read
		check_lastCommunicationTime();	// Set PC0 state, update address if needed
	}
}

