// This code drives a ATMega328 Atmel microcontroller.
// The system implements a phase cutting mains AC dimmer array.
// A user can set a channel with a 2 byte command.

#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU 20000000
#define USART_BAUDRATE 9600
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

// Messages
#define ACK1    1
#define ACK2    2
#define ERROR   0

// Commands
#define SET_ALL     0
#define SET_CH0     1
#define SET_CH1     2
#define SET_CH2     3
#define SET_CH3     4
#define SET_CH4     5
#define SET_CH5     6
#define SET_CH6     7
#define SET_CH7     8
#define COMMENCE    9
#define FREQUENCY   10

// Channels
#define HW_CH0      4
#define HW_CH1      5
#define HW_CH2      6
#define HW_CH3      7
#define HW_CH4      0
#define HW_CH5      1
#define HW_CH6      2
#define HW_CH7      3

// Globals
volatile uint8_t channel[8];    // The actual amount of phase cutting. 0 = full phase, about 200 = no phase
volatile uint8_t setting[8];    // The setting channel will have after COMMENCE command
volatile uint8_t counter = 0;   // How many timer ticks we had after last 0 crossing
volatile uint8_t mains_frequency = 0;   // Maximum of timer ticks after previous crossing
volatile uint8_t data_recieved = 0;     // The number of bytes recieved from serial, goes back to 0 after 2
volatile uint8_t data[2];               // Data recieved from serial

void USART0SendByte(uint8_t u8Data) // Send a byte by serial
{
    while(!(UCSR0A&(1<<UDRE0))){};  // Wait while previous byte is completed
    UDR0 = u8Data;                  // Transmit data
}

ISR(TIMER1_COMPA_vect) // Happens at 10kHz
{
    counter++;
    if (channel[0] == counter) PORTD |= 1 << HW_CH0;    // Turn on the light
    if (channel[1] == counter) PORTD |= 1 << HW_CH1;    // Turn on the light
    if (channel[2] == counter) PORTD |= 1 << HW_CH2;    // Turn on the light
    if (channel[3] == counter) PORTD |= 1 << HW_CH3;    // Turn on the light
    if (channel[4] == counter) PORTC |= 1 << HW_CH4;    // Turn on the light
    if (channel[5] == counter) PORTC |= 1 << HW_CH5;    // Turn on the light
    if (channel[6] == counter) PORTC |= 1 << HW_CH6;    // Turn on the light
    if (channel[7] == counter) PORTC |= 1 << HW_CH7;    // Turn on the light
}

ISR(USART_RX_vect) // Happens after byte recieved from serial
{
    while(!(UCSR0A&(1<<RXC0))){};   // Wait for byte to be received
    //return UDR0;                    // Return received data
    data[data_recieved] = UDR0;
    data_recieved++;

    if (data_recieved == 1) USART0SendByte(ACK1);

    if (data_recieved > 1)
    {
        switch(data[0])
        {
            case SET_ALL:
                setting[0] = data[1];
                setting[1] = data[1];
                setting[2] = data[1];
                setting[3] = data[1];
                setting[4] = data[1];
                setting[5] = data[1];
                setting[6] = data[1];
                setting[7] = data[1];
                USART0SendByte(ACK2);
                break;

            case SET_CH0:
                setting[0] = data[1];
                //channel[0] = setting[0];
                USART0SendByte(ACK2);
                break;

            case SET_CH1:
                setting[1] = data[1];
                //channel[1] = setting[1];
                USART0SendByte(ACK2);
                break;

            case SET_CH2:
                setting[2] = data[1];
                //channel[2] = setting[2];
                USART0SendByte(ACK2);
                break;

            case SET_CH3:
                setting[3] = data[1];
                //channel[3] = setting[3];
                USART0SendByte(ACK2);
                break;

            case SET_CH4:
                setting[4] = data[1];
                //channel[4] = setting[4];
                USART0SendByte(ACK2);
                break;

            case SET_CH5:
                setting[5] = data[1];
                //channel[5] = setting[5];
                USART0SendByte(ACK2);
                break;

            case SET_CH6:
                setting[6] = data[1];
                //channel[6] = setting[6];
                USART0SendByte(ACK2);
                break;

            case SET_CH7:
                setting[7] = data[1];
                //channel[7] = setting[7];
                USART0SendByte(ACK2);
                break;

            case COMMENCE:
                channel[0] = setting[0];
                channel[1] = setting[1];
                channel[2] = setting[2];
                channel[3] = setting[3];
                channel[4] = setting[4];
                channel[5] = setting[5];
                channel[6] = setting[6];
                channel[7] = setting[7];
                USART0SendByte(ACK2);
                break;

            default:
                USART0SendByte(ERROR);
                break;
        }
        data_recieved = 0;
    }
}

ISR (INT0_vect) // Happens when mains does a zero crossing (about 100 Hz)
{
    mains_frequency = counter;  // Store so we can awnser user if he asks for it
    counter = 0;
    PORTD   = 0;  // Turn off the lights
    PORTC   = 0;
    if (channel[0] == counter) PORTD |= 1 << HW_CH0;    // Turn on the light
    if (channel[1] == counter) PORTD |= 1 << HW_CH1;    // Turn on the light
    if (channel[2] == counter) PORTD |= 1 << HW_CH2;    // Turn on the light
    if (channel[3] == counter) PORTD |= 1 << HW_CH3;    // Turn on the light
    if (channel[4] == counter) PORTC |= 1 << HW_CH4;    // Turn on the light
    if (channel[5] == counter) PORTC |= 1 << HW_CH5;    // Turn on the light
    if (channel[6] == counter) PORTC |= 1 << HW_CH6;    // Turn on the light
    if (channel[7] == counter) PORTC |= 1 << HW_CH7;    // Turn on the light
}

int main(void)
{
//  Init Timer
    OCR1A = 1000;               // 10.00kHz
    TCCR1B |= (1 << WGM12);     // Mode 4, CTC on OCR1A
    TIMSK1 |= (1 << OCIE1A);    // Set interrupt on compare match
    TCCR1B |= (1 << CS10);      // Set prescaler to 1 and start the timer

//  Init USART
    UBRR0H = (uint8_t)(UBRR_VALUE>>8);  // Set baud rate
    UBRR0L = (uint8_t)UBRR_VALUE;
    UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);              // Set frame format to 8 data bits, no parity, 1 stop bit
    UCSR0B |= (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);    // Enable transmission and reception and interrupt on recieve

//  Init External interrupt
    EICRA |= (1 << ISC01);  // set INT0 to trigger on falling edge
    EIMSK |= (1 << INT0);   // Turns on INT0

//  Set ports
    PORTD = 0;          // All channels down on outputs
    PORTC = 0;
    DDRD = 0b11111000;  //PD2 as zero crossing input, PD3 output for LED, PD4..7 output for channel 0..3
	DDRC = 0b00001111;  //PC0..3 output for channel 4..7

	int x;              // All channels down on software
	for (x=0; x<8; x++)
	{
        channel[x] = 255;
        setting[x] = 255;
	}

	sei();  // enable global interrupts

    while(1)
    {
        // Do not work unless you have to (proverb)
    }
}
