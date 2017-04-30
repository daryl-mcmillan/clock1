.data

adc_index:
.byte 0

adc_values:
.hword 0, 0, 0, 0

.text

#define ADCL 0x04
#define ADCH 0x05
#define ADCSRA 0x06
#define ADMUX 0x07

.global adc_refresh
.global adc_read
adc_refresh: ; (no args)
  ret

adc_read: ; ( unsigned char pin )
  ldi r24, 0x8F
  ldi r25, 0x8F
  ret
