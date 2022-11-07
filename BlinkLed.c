#include <16F887.h>
#fuses INTRC            //Internal RC Oscillator
#use delay(CLOCK=8MHz)

#define Led    PIN_D0
#define SW     PIN_C7

void main()
{
set_tris_c(0b10000000);
set_tris_d(0b11111110);
while(TRUE)
{
if(input(SW)==0) 
{
output_toggle(Led);
delay_ms(500);
}
else output_high(Led);
}
}
