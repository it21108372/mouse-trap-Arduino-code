# mouse-trap-Arduino-code

#ifndef F_CPU
#define F_CPU 16000000UL // 16 MHz clock speed
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

// Define baud rate
#define BAUD 9600
#define BAUD_PRESCALE ((F_CPU / (16UL * BAUD)) - 1)

void USART_Init(void)
{
  // Set baud rate
  UBRR0H = (uint8_t)(BAUD_PRESCALE >> 8);
  UBRR0L = (uint8_t)(BAUD_PRESCALE);

  // Enable receiver and transmitter
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);

  // Set frame format: 8 data bits, 1 stop bit, no parity
  UCSR0C = (3 << UCSZ00);
}

void USART_Transmit(unsigned char data)
{
  // Wait for empty transmit buffer
  while (!(UCSR0A & (1 << UDRE0)))
    ;

  // Put data into buffer, sends the data
  UDR0 = data;
}

int main(void)
{
  DDRB = 0x03; // Set PB0 and PB1 as output pins for servo motors
  PORTB = 0x00;

  DDRD |= (1 << PD4); // Set PD4 as output pin for trigger
  DDRD &= ~(1 << PD3); // Set PD3 as input pin for echo
  PORTD |= (1 << PD3); // Enable pull-up resistor for PD3

  // Initialize USART
  USART_Init();

  while (1)
  {
    // Generate 10us pulse on trigger pin
    PORTD &= ~(1 << PD4);
    _delay_us(2);
    PORTD |= (1 << PD4);
    _delay_us(10);
    PORTD &= ~(1 << PD4);

    // Wait for echo pulse
    while (!(PIND & (1 << PD3)))
      ;
    // Measure pulse width on echo pin
    uint16_t pulse_width = 0;
    while (PIND & (1 << PD3))
     { pulse_width++;
      _delay_us(1);
     }

    // Calculate distance in cm using pulse width
    float distance = (float)pulse_width / 58.0;

    // Check if IR sensor detects an object
    if (!(PIND & (1 << PD2)))
    {
      // Rotate first motor to 120 degrees
      PORTB = 0x01;
      _delay_us(1800);
      PORTB = 0x00;

      // Delay for 1 second
      _delay_ms(1000);

      // Rotate first motor back to previous position
      PORTB = 0x01;
      _delay_us(600);
      PORTB = 0x00;

      // Rotate second motor to 120 degrees
      PORTB |= (1 << PB1);
      _delay_us(1800);
      PORTB &= ~(1 << PB1);

      // Delay for 1 second
      _delay_ms(1000);

      // Rotate second motor back to previous position
      PORTB |= (1 << PB1);
      _delay_us(600);
      PORTB &= ~(1 << PB1);

      // Send motor positions to serial
      char buffer[10];
      itoa(120, buffer, 10);
      USART_Transmit('P');
      USART_Transmit(buffer[0]);
      USART_Transmit(buffer[1]);
      USART_Transmit('\n');
    }

      // Initialize USART
  USART_Init();

  
  
  /*  // Send ultrasonic pulse
    PORTD |= (1 << PD4);
    _delay_us(10);
    PORTD &= ~(1 << PD4);

    // Wait for echo
    while (!(PIND & (1 << PD3)))
      ;

    // Measure pulse duration
    uint16_t pulse_start = micros();
    while (PIND & (1 << PD3))
      ;
    uint16_t pulse_duration = micros() - pulse_start;

    // Calculate distance
    float distance = pulse_duration * 0.034 / 2.0;
*/
    // Check if door is open or closed
    if (distance > 10.0)
    {
      USART_Transmit('D');
      USART_Transmit('o');
      USART_Transmit('o');
      USART_Transmit('r');
      USART_Transmit(' ');
      USART_Transmit('i');
      USART_Transmit('s');
      USART_Transmit(' ');
      USART_Transmit('o');
      USART_Transmit('p');
      USART_Transmit('e');
      USART_Transmit('n');
      USART_Transmit('e');
      USART_Transmit('d');
      USART_Transmit('\n');
    }
    else
    {
      USART_Transmit('D');
      USART_Transmit('o');
      USART_Transmit('o');
      USART_Transmit('r');
      USART_Transmit(' ');
      USART_Transmit('i');
      USART_Transmit('s');
      USART_Transmit(' ');
      USART_Transmit('c');
      USART_Transmit('l');
      USART_Transmit('o');
      USART_Transmit('s');
      USART_Transmit('e');
      USART_Transmit('d');
      USART_Transmit('\n');
    }

    // Send distance measurement to serial
    char buffer[10];
    itoa((int)distance, buffer, 10);
    USART_Transmit('D');
    USART_Transmit(buffer[0]);
    USART_Transmit(buffer[1]);
    USART_Transmit(' ');
    USART_Transmit('c');
    USART_Transmit('m');
    USART_Transmit('\n');


  }
  
  

}




/*  // Send ultrasonic pulse
    PORTD |= (1 << PD4);
    _delay_us(10);
    PORTD &= ~(1 << PD4);

    // Wait for echo
    while (!(PIND & (1 << PD3)))
      ;

    // Measure pulse duration
    uint16_t pulse_start = micros();
    while (PIND & (1 << PD3))
      ;
    uint16_t pulse_duration = micros() - pulse_start;

    // Calculate distance
    float distance = pulse_duration * 0.034 / 2.0;
*/
