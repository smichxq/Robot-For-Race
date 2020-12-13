#include <MsTimer2.h>

int pin = 2;//外部中断功能引脚 2 3 21 20 19 18

long tm = 0;//时间变量,用来计时

boolean flag = true;//标记位,用来开启与关闭MsTimer2中断服务

void setup()
{
  Serial.begin(9600);
  pinMode(pin, INPUT);
  attachInterrupt(0, isr0, RISING);//当int.0电平时,升高时触发中断函数blink
  attachInterrupt(1, isr1, RISING);//当int.1电平时,升高时触发中断函数blink
  MsTimer2::set(10,count);//设置中断服务,每100ms运行一次
}

void loop()
{
  
  
}
//定时中断函数
void count(){
  Serial.println(tm++);
  //tm++;
}
void isr1(){
  MsTimer2::stop();
}
//外部中断函数
void isr0()
{
  MsTimer2::start();

}