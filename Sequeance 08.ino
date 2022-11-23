#include "IOhelper.h" 
#include "Control.h"
boolean counter, pl;
unsigned long count;
void setup() {  //최초 한 번만 실행된다. 
    IOSetup();
    counter = OFF;
    pl = OFF;
    count = 0;
}

void loop() {  //반복 해서 실행된다. 
  delay(1);
  CheckButtons();
  if( counter == ON ) count = count + 1;
  //컨베이어 작동중 금속 재질이 감지 되면 pl변수에 저장한다. 
  if( Read(S4) == ON ) pl = ON;
  
  if( count == 5 * SEC ) {  //컨베이어가 어느정도 작동되면
      if( pl == ON ) PL1_on(); //pl에 ON이 저장되어있으면 PL1을 켜고,
      else PL2_on();  //pl에 ON이 저장되더있지 않다면 PL2를 켠다.
      CONV_off();
      count = 0;
      counter = OFF;  //카운터를 초기화하고 정지시킨다.
  }
} 
void StartButton() {
    PL1_off();
    PL2_off();
    pl = OFF;
    CONV_on();
    count = 0; 
    counter = ON; 
}  //Button_Start
void StopButton() {}  //Button_Stop
void ResetButton() {}  //Button_Reset
