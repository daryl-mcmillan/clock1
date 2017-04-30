#include <avr/io.h>
#include <util/delay.h>
#include "adc.h"

// 0-1023
unsigned short analogRead( char pin ) {
  ADMUX = (0 << REFS0) | (1 << ADLAR) | (pin << MUX0);
  ADCSRA = (1 << ADEN) | (1 << ADSC) | (0b110 << ADPS0);
  while( ADCSRA & (1 << ADSC) );
  return ADC;
}

void setOutput( char pin ) {
  DDRB |= ( 1 << pin );
}

void setPin( char pin, char value ) {
  PORTB = ( PORTB & ~(1<<pin) ) | (value<<pin);
}

void setInput( char pin ) {
  DDRB &= ~( 1 << pin );
  PORTB &= ~( 1 << pin );
}

unsigned char getPin( char pin ) {
  return ( PINB >> pin ) & 1;
}

int main(void) {

  // comparator setup
  ADCSRB &= ~( 1 << ACME ); // no multiplex
  ACSR = ( 3 << ACIS0 ) | ( 1 << ACBG ); // detect rising edge, bandgap as positive input
  // DIDR0 |= 0b11; // disable digital

  unsigned char mux = (0 << REFS0) | (1 << ADLAR) | (0b10 << MUX0);
  unsigned char start = (1 << ADEN) | (1 << ADSC) | (0b110 << ADPS0);

  unsigned long step = 0x0FFFF;
  unsigned long duty = 32000;

  setOutput( 0 );

  unsigned long tick = 0;

  for( ;; ) {

    adc_refresh();
    step = adc_read( 2 );

    if( ACSR & ( 1 << ACI ) ) {
      tick = 0;
      ACSR |= 1 << ACI; // clear by writing 1 (!)
    }

    if( ADCSRA & ( 1 << ADSC ) ) {
      
    } else {
      // read adc
      duty = ADC;
      duty = duty << 16;
      // restart adc
      ADMUX = mux;
      ADCSRA = start;
    }
    tick += step;
    if( tick < duty ) {
      setPin( 0, 1 );
    } else {
      setPin( 0, 0 );
    }
  }

}
