#include <LobotServoController.h>

LobotServoController myse(Serial3);

void setup() {
  pinMode(13,OUTPUT);
  Serial3.begin(9600);
  while(!Serial3);
  digitalWrite(13,HIGH);
  angleTest();
  //
  
  

  
}

//角度标定
void angleTest(){
  short angle = 0;
    while (1)
  {
    angle = (angle + 500) % 2500;
    myse.moveServo(1,angle,1000);
    delay(2000);
  }

}


int i = 0;
void loop() {
  break;
  
}