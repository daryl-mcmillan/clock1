#include <avr/io.h>
#include <util/delay.h>
#include "adc.h"

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

  unsigned long step = 0x0FFFF;
  unsigned long duty = 32000;

  setOutput( 0 );

  unsigned long tick = 0;

  for( ;; ) {

    adc_refresh();
    step = adc_read( 3 );
    step = step << 4;
    duty = adc_read( 2 );
    duty = duty << 16;

    // hard sync
    if( ACSR & ( 1 << ACI ) ) {
      tick = 0;
      ACSR |= 1 << ACI; // clear by writing 1 (!)
    }

    tick += step;
    if( tick < duty ) {
      setPin( 0, 1 );
    } else {
      setPin( 0, 0 );
    }
  }

}
