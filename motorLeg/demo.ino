#include <LobotServoController.h>
#include <MsTimer2.h>



int pin = 2;//外部中断功能引脚 2 3 21 20 19 18

long tm = 0;//时间变量,用来计时

boolean flag = true;//标记位,用来开启与关闭MsTimer2中断服务


LobotServoController myse(Serial3);

void setup() {
  
  pinMode(13,OUTPUT);
  Serial3.begin(9600);
  //while(!Serial3);
  digitalWrite(13,HIGH);
  angleTest();

  Serial.begin(9600);
  pinMode(pin, INPUT);
  attachInterrupt(0, isr, RISING);//当int.0电平时,升高时触发中断函数blink
  MsTimer2::set(10,count);//设置中断服务,每100ms运行一次
  
  
  

  
}  

//角度标定
void angleTest(){
  short angle = 500;
    while (1)
  {
    myse.moveServo(1,angle,1000);
    angle = (angle + 1000) % 2500;
    if (!angle)
    {
      angle = 500;
    }
    
    delay(2000);
  }

}


void loop()
{
  
  
}
//定时中断函数
void count(){
  Serial.println(tm++);
  //tm++;
}
//外部中断函数
void isr()
{
  if(flag){
    MsTimer2::start();
    flag=false;
  }
  else{
    flag=true;
    MsTimer2::stop();
  }
  
}//
void test(){
  
}