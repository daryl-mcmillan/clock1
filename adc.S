.data

adc_index:
.byte 0

adc_values:
.hword 0xFF00, 0x8F00, 0x4F00, 0x0000

.text

.global __do_copy_data

#define ADCL 0x04
#define ADCH 0x05
#define ADCSRA 0x06
#define ADMUX 0x07

.global adc_refresh
.global adc_read
adc_refresh: ; (no args)
  
  ret

adc_read: ; ( unsigned char pin )

  ldi r30, lo8(adc_values)
  ldi r31, hi8(adc_values)
  lsl r24 // pin * 2 for address
  add r30, r24
  adc r31, r1 // r1=0
  ld r24, Z+
  ld r25, Z+
  ret
