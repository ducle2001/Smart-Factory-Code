#include "IOhelper.h"
void setup() {  //최초 한 번만 실행된다.
    IOSetup();
}

void loop() {  //반복 해서 실행된다.
    CheckButtons();
} 
void StartButton() {
    PL1_on();
}  //Button_Start
void StopButton() {
    PL2_on();
}  //Button_Stop
void ResetButton() {
    PL1_off();
    PL2_off();
}  //Button_Reset
