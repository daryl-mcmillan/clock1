#include <avr/io.h>
#include <util/delay.h>

// 0-1023
unsigned short analogRead( char pin ) {
  ADMUX = (0 << REFS0) | (1 << ADLAR) | (0b10 << MUX0);
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

int main(void) {

  unsigned char mux = (0 << REFS0) | (1 << ADLAR) | (0b10 << MUX0);
  unsigned char start = (1 << ADEN) | (1 << ADSC) | (0b110 << ADPS0);

  unsigned long step = 0x4FFFF;
  unsigned long duty = 32000;

  setOutput( 3 );

  unsigned long tick = 0;

  for( ;; ) {
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
      setPin( 3, 1 );
    } else {
      setPin( 3, 0 );
    }
  }

}
