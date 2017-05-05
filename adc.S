.data

adc_values:
.hword 0xFF00, 0x8F00, 0x1F00, 0x0000

.text

.global __do_copy_data

#define ADCL 0x04
#define ADCH 0x05
#define ADCSRA 0x06
#define ADMUX 0x07

.global adc_refresh
.global adc_read

adc_refresh_skip:

  ; nop to match the adc read time
  .rept 19
  nop
  .endr
  ret

adc_refresh: ; (no args)

  ; conversion in progress -- return
  sbic ADCSRA, 6
  rjmp adc_refresh_skip

  ; if a value is ready

  ; get the value
  in r18, ADCL
  in r19, ADCH
  in r20, ADMUX
  andi r20, 0b11
  mov r21, r20 ; index to store data
  subi r20, 1
  andi r20, 0b11 ; new index
  ori r20, 0b00100000 ; admux + adlar
  out ADMUX, r20
  ldi r20, 0b11000110 ; aden, adsc, clock_div=64
  out ADCSRA, r20

  ; store adc value
  lsl r21 ; offset of data
  ldi r30, lo8(adc_values)
  ldi r31, hi8(adc_values)
  add r30, r21
  adc r31, r1 // r1=0
  st Z+, r18
  st Z+, r19
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
