// #include <LobotServoController.h>
#include <MsTimer2.h>
//方向
#define goStraight 0
#define goBack 1
#define turnLeft 2
#define turnRight 3
#define stp 4

//当前方向
short currentState;




//传感器接口
#define SensorPinF_1 52
#define SensorPinF_2 51 
#define SensorPinF_3 50

#define SensorPinR_1 49
#define SensorPinR_2 48 
#define SensorPinR_3 47 

#define SensorPinB_1 46
#define SensorPinB_2 45 
#define SensorPinB_3 44 

#define SensorPinL_1 43
#define SensorPinL_2 42 
#define SensorPinL_3 41 


//电机接口
#define CodeA1 18
#define MotorPin1 4
#define Motor1Ain2 A1
#define Motor1Ain1 A2

#define CodeA2 19
#define MotorPin2 5
#define Motor2Ain1 A3
#define Motor2Ain2 A4

#define CodeB1 20
#define MotorPin3 6
#define Motor3Ain1 A8
#define Motor3Ain2 A7

#define CodeB2 21
#define MotorPin4 7
#define Motor4Ain1 A9
#define Motor4Ain2 A10

/*
* 电机旋转方向
* S     P     N
* 反    正    停
*/
#define S 0
#define P 1
#define N 2



//路径结构体
typedef struct mission
{
  /*
  * 前期只有四个方向
  * 
  * 任务结构体
  *  A   B   C 3bit
  *  AB两位为方向　　  
  * goStraight  goBack  turnLeft  turnRight
  *    00　       01      10         11
  * Ｃ为模式
  * 方格模式  直线模式
  *   _ _ 0    _ _ 1
  */
  bool A;
  bool B;
  bool C;
  struct mission* next;
}mission;

//任务序列
typedef struct missions
{
  //每一个任务具体任务指针,无头节点
  mission* head;
  //任务序列指针域
  //struct missions* next;

  //多路径区分 默认为true
  bool flag;
  
}missions;

/*=====================声明一个全局的任务序列missions[N]=================*/

missions currentMissions[14];

/*=====================声明一个全局的任务序列missions[N]=================*/

/*=====================测试路径==================*/

mission* testMission;
mission* p  = testMission;
bool onceFlag = true;

/*=====================测试路径==================*/


/*
 * 外部中断功能引脚 2 3 21 20 19 18
 *               0 1  2  3 4  5
 */
 
//时间变量,用来计时(已弃用，原因：中断优先级未知，外部中断引脚不足)
//long tm = 0;

//boolean is1 = true;//标记位,用来判定是哪一侧先触线

//朝向左侧传感器
short is2;
//朝向右侧传感器
short is3;


//朝向左侧传感器
short is20;
//朝向右侧传感器
short is21;

//朝向右侧传感器
short isA12_F_flag = 0;
short isA13_R_flag = 0;
short isA14_B_flag = 0;
short isA15_L_flag = 0;

//全局时间变量
long int T1 = 0;
long int T2 = 0;


//全局速度变量
short spdA1 = 0;
volatile long spdA1Count = 0;
short spdA2 = 0;
volatile long spdA2Count = 0;
short spdB1 = 0;
volatile long spdB1Count = 0;
short spdB2 = 0;
volatile long spdB2Count = 0;


//
bool flagA = false;
bool flagB = false;
bool flagC = true;


// LobotServoController myse(Serial3);//舵机控制

void setup() {
  //测试路径初始化
  test();
  sensorInit();
  //MsTimer2::set(5,stateFix);
  motorInit();
  setSpdA1(80);
  setSpdA2(80);
  setSpdB1(80);
  setSpdB2(80);

}  
void loop()
{
    

  /*
  先使用路径规划pathPlan
  在命令电机移动directions
  */
  pathPlan();
  directions();
  
}


/*
* 返回任务码
*
*/
char* getTarget(){
  char* targets = NULL;
  targets = (char*)malloc(sizeof(3));
  /*
  * 与openmv通信
  * 得到串口返回任务码
  * 
  *targets = 'R';
  targets++;
  *targets = 'G';
  targets++;
  *targets = 'B';
  targets-=2;
  */
  return NULL;

}
/* 
* 传感器初始化
*/
void sensorInit(){

    pinMode(SensorPinF_1,INPUT);
    pinMode(SensorPinF_2,INPUT);
    pinMode(SensorPinF_3,INPUT);

    pinMode(SensorPinB_1,INPUT);
    pinMode(SensorPinB_2,INPUT);
    pinMode(SensorPinB_3,INPUT);

    pinMode(SensorPinL_1,INPUT);
    pinMode(SensorPinL_2,INPUT);
    pinMode(SensorPinL_3,INPUT);

    pinMode(SensorPinR_1,INPUT);
    pinMode(SensorPinR_2,INPUT);
    pinMode(SensorPinR_3,INPUT);

}
/*
* 传感器计数服务初始化============================================废弃====================================
*/
void sensorTimerInit(){

  /*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!一旦進入中断程序 就會自動禁止中斷服务!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
  
  //当int.N电平时,升高时触发中断函数blink,2号引脚 3号引脚
//  
//  attachInterrupt(0, isr0, FALLING);
//  attachInterrupt(1, isr1, FALLING);

  //当int.N电平时,升高时触发中断函数blink,2号引脚 3号引脚
//  attachInterrupt(2, isr2, FALLING);
//  attachInterrupt(3, isr3, FALLING);
  //设置中断服务,每1ms运行一次,使用c的tick
  //MsTimer2::set(1,count);
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!一旦進入中断程序 就會自動禁止中斷服务!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
  
}

/*
* 电机接口初始化
*
*
*/

void motorInit(){

  pinMode(A1,OUTPUT);
  pinMode(A2,OUTPUT);
  pinMode(A3,OUTPUT);
  pinMode(A4,OUTPUT);
  pinMode(A5,OUTPUT);
  pinMode(A6,OUTPUT);
  pinMode(A7,OUTPUT);
  pinMode(A8,OUTPUT);
  pinMode(A9,OUTPUT);
  pinMode(A10,OUTPUT);

  //编码器外部中断
//   pinMode(CodeA1,INPUT);
//   pinMode(CodeA2,INPUT);
//   pinMode(CodeB1,INPUT);
//   pinMode(CodeB2,INPUT);

/*
* 外部中断口服务初始化
* int.2 int.3 int.4 int.5
*   21   20     19    18
*/
//   attachInterrupt(2, isr0, CHANGE);
//   attachInterrupt(3, isr1, CHANGE);
//   attachInterrupt(4, isr2, CHANGE);
//   attachInterrupt(5, isr3, CHANGE);
  

  
}

//中断服务初始化

void IntServiceInit(){
    MsTimer2::set(5,count);
    MsTimer2::start();
}



/*
* 电机正反转控制
*  P   N   S
* 正  反  停
*/


//A1
void motorA1PNS(short mode)
{

  switch (mode)
  {
  case 0:
    //Serial.print("stop");
    digitalWrite(Motor1Ain1,LOW);
    digitalWrite(Motor1Ain2,LOW);
    break;
  case 1:
    //Serial.print("go Straight");
    digitalWrite(Motor1Ain1,LOW);
    digitalWrite(Motor1Ain2,HIGH);
    break;
  case 2:
    //Serial.print("go back");
    digitalWrite(Motor1Ain1,HIGH);
    digitalWrite(Motor1Ain2,LOW);
    break;
  
  default:
    break;
  }

}
void motorA2PNS(short mode)
{
  switch (mode)
  {
  case 0:
    //Serial.print("stop");;
    digitalWrite(Motor2Ain1,LOW);
    digitalWrite(Motor2Ain2,LOW);
    break;
  case 1:
    //Serial.print("go Straight");
    digitalWrite(Motor2Ain1,HIGH);
    digitalWrite(Motor2Ain2,LOW);
    break;
  case 2:
    //Serial.print("go back");
    digitalWrite(Motor2Ain1,LOW);
    digitalWrite(Motor2Ain2,HIGH);
    break;
  
  default:
    break;
  }

}
void motorB1PNS(short mode)
{
  switch (mode)
  {
  case 0:
    //printf("stop");
    digitalWrite(Motor3Ain1,LOW);
    digitalWrite(Motor3Ain2,LOW);
    break;
  case 1:
    //printf("go Straight");
    digitalWrite(Motor3Ain1,HIGH);
    digitalWrite(Motor3Ain2,LOW);
    break;
  case 2:
    //printf("go back");
    digitalWrite(Motor3Ain1,LOW);
    digitalWrite(Motor3Ain2,HIGH);
    break;
  
  default:
    break;
  }

}
void motorB2PNS(short mode)
{
  switch (mode)
  {
  case 0:
    //printf("stop");
    digitalWrite(Motor4Ain1,LOW);
    digitalWrite(Motor4Ain2,LOW);
    break;
  case 1:
    //printf("go Straight");
    digitalWrite(Motor4Ain1,HIGH);
    digitalWrite(Motor4Ain2,LOW);
    break;
  case 2:
    //printf("go back");
    digitalWrite(Motor4Ain1,LOW);
    digitalWrite(Motor4Ain2,HIGH);
    break;
  
  default:
    break;
  }

}

/*
* 速度函数
*/
void setSpdA1(short spd){
  spdA1 = spd;
}
void setSpdA2(short spd){
  spdA2 = spd;
}
void setSpdB1(short spd){
  spdB1 = spd;
}
void setSpdB2(short spd){
  spdB2 = spd;
}

/*
* 电机命令函数 
* A1 A2 
* B1 B2
* 
*/

void motorA1(){
  if (spdA1<0)
  {
    analogWrite(MotorPin1,0);
  }
  else if(spdA1<255)
  {
    analogWrite(MotorPin1,spdA1);
  }
  else
  {
    analogWrite(MotorPin1,255);
  }
  
}
void motorA2(){
  if (spdA2<0)
  {
    analogWrite(MotorPin2,0);
  }
  else if(spdA2<255)
  {
    analogWrite(MotorPin2,spdA2);
  }
  else
  {
    analogWrite(MotorPin2,255);
  }
  
}
void motorB1(){
  if (spdB1<0)
  {
    analogWrite(MotorPin3,0);
  }
  else if(spdB1<255)
  {
    analogWrite(MotorPin3,spdB1);
  }
  else
  {
    analogWrite(MotorPin3,255);
  }
  
}
void motorB2(){
  if (spdB2<0)
  {
    analogWrite(MotorPin4,0);
  }
  else if(spdB2<255)
  {
    analogWrite(MotorPin4,spdB2);
  }
  else
  {
    analogWrite(MotorPin4,255);
  }
  
}

/*
* 电机移动命令
* goStraight 前
* goBack 后
* turnLeft 左平移
* turnRight 右平移
*
* 参数为方向,速度
* 目前只有四个方向
* 后期增加其他方向
*/
void directions(){
  short dirct = currentState;

  //电机换向保护
    // motorA1PNS(S);
    // motorA2PNS(S);
    // motorB1PNS(S);
    // motorB2PNS(S);

    switch (dirct)
    {
    case goStraight:
        //printf("goStraight\n");
        motorA1PNS(P);
        motorA1();

        motorA2PNS(P);
        motorA2();

        motorB1PNS(P);
        motorB1();

        motorB2PNS(P);
        motorB2();
        
        break;
    case goBack:
        //printf("goBack\n");
        motorA1PNS(N);
        motorA1();

        motorA2PNS(N);
        motorA2();

        motorB1PNS(N);
        motorB1();

        motorB2PNS(N);
        motorB2();
        break;
    case turnLeft:
        //printf("turnLeft\n");
        motorA1PNS(N);
        motorA1();

        motorA2PNS(P);
        motorA2();

        motorB1PNS(P);
        motorB1();

        motorB2PNS(N);
        motorB2();

        break;
    case turnRight:
        //printf("turnRight\n");
        motorA1PNS(P);
        motorA1();

        motorA2PNS(N);
        motorA2();

        motorB1PNS(N);
        motorB1();

        motorB2PNS(P);
        motorB2();
        break;

    case stp:
      motorA1PNS(S);
      motorA2PNS(S);
      motorB1PNS(S);
      motorB2PNS(S);
    }
  
}



/*
* 功能: 增加一个路径序列项
* 参数: 任务结构体mission的每一项标记位
*/
mission* addMission(bool a,bool b,bool c)
{
  mission* currentMission = NULL;

  currentMission = (mission*)malloc(sizeof(mission));

  currentMission->A = a;
  currentMission->B = b;
  currentMission->C = c;

  currentMission->next = NULL;

  return currentMission;
}

/*
* 功能: 合成一个路径序列,并返回该序列的头指针
* 参数: 每个路径序列的数量 , 路径序列数组
* 
*/
mission* createMissionList(short num,mission* missArry)
{
  mission* head = NULL;
  mission* p = NULL;
  short flag = 1;
  for (int i = 0; i < num; i++)
  {
    if (flag)
    {
      head = addMission(missArry[i].A,missArry[i].B,missArry[i].C);
      p = head;
      flag = 0;
      continue;
    }

    p->next = addMission(missArry[i].A,missArry[i].B,missArry[i].C);
    p = p->next;
  }

  return head;

}


/*
* 功能: 遍历路径序列mission
* 
*
*/
void traverseMission(){
  mission* p = NULL;
  p = currentMissions[0].head;
  while (p)
  {
    //printf("%d  %d  %d \n",p->A,p->B,p->C);

    p = p->next;

  }

}

/*
* 功能: 初始化任务序列
* 将每一个任务的路径序列填入该函数
* 参数: 无,使用全局变量任务序列
*/
void missionsInit(){
  //路径序列,每一个任务的路径,共有14个
  mission missionAry[12];

  //mission* p = NULL;

  for (int i = 0; i < 12; i++)
  {
    missionAry[i].A = i;
    missionAry[i].B = i;
    missionAry[i].C = i;
  }

  //任务序列


  //扫码
  currentMissions[0].head = createMissionList(12,missionAry);
  currentMissions[0].flag = true;


  //原料区1
  currentMissions[1].head = createMissionList(12,missionAry);
  currentMissions[1].flag = true;


  //粗加工区1
  currentMissions[2].head = createMissionList(12,missionAry);
  currentMissions[2].flag = true;
  //粗加工区2
  currentMissions[3].head = createMissionList(12,missionAry);
  currentMissions[3].flag = false;


  //半成品区1
  currentMissions[4].head = createMissionList(12,missionAry);
  currentMissions[4].flag = true;
  //半成品区2
  currentMissions[5].head = createMissionList(12,missionAry);
  currentMissions[5].flag = false;


  //原料区1
  currentMissions[6].head = createMissionList(12,missionAry);
  currentMissions[6].flag = true;
  //原料区2
  currentMissions[7].head = createMissionList(12,missionAry);
  currentMissions[7].flag = false;



  //粗加工区1
  currentMissions[8].head = createMissionList(12,missionAry);
  currentMissions[8].flag = true;
  //粗加工区2
  currentMissions[9].head = createMissionList(12,missionAry);
  currentMissions[9].flag = false;



  //半成品区1
  currentMissions[10].head = createMissionList(12,missionAry);
  currentMissions[10].flag = true;
  //半成品区2
  currentMissions[11].head = createMissionList(12,missionAry);
  currentMissions[11].flag = false;

  //终点区1
  currentMissions[12].head = createMissionList(12,missionAry);
  currentMissions[12].flag = true;
  //终点区2
  currentMissions[13].head = createMissionList(12,missionAry);
  currentMissions[13].flag = false;


}


/*
* 路径规划
* 规划当前方向
* 
*/
void pathPlan()
{




  
  //判断是否经过方格
  isA12_F_flag = digitalRead(SensorPinF_2);
  isA14_B_flag = digitalRead(SensorPinB_2);

  isA15_L_flag = digitalRead(SensorPinL_2);
  isA13_R_flag = digitalRead(SensorPinR_2);

  

 

  //经过方格，移动到下一个路径
  if (flagB && flagA)
  {
    p = p->next;
    flagA = false;
    flagB = false;
    flagC = true;
    T1 = T2 = 0;
  }
  

  
  
  

  // while (p)
  // {
    //Serial.println(p->A + p->B + p->C);

    //前
    if (!(p->A) && !(p->B))
    {
      //更新当前方向
      currentState = goStraight;
      
    }
    //后
    if (!(p->A) && p->B)
    {
      //更新当前方向
      currentState = goBack;
    }
    //左
    if (p->A && !(p->B))
    {
      //更新当前方向
      currentState = turnLeft;
    }
    //右
    if (p->A && p->B)
    {
      //更新当前方向
      currentState = turnRight;
    }

    if (p->C)
    {
        currentState = stp;
    }
  // }

   //判断是否经过方格
  switch (currentState)
  {
  case goStraight:
    if (isA12_F_flag == 0)
    {
      flagA = true;
    }
    if (isA14_B_flag == 0)
    {
        //tick-pwm表
        
        T2 = getTickTime();

        if (flagC)
        {
            T1 = T2;
            flagC = false;
        }
        
        if (T2 - T1 > 500)
        {
            flagB = true;
        }

    }
    break;
  case goBack:
    if (isA14_B_flag == 0)
    {
      flagB = true;
    }
    if (isA12_F_flag == 0)
    {
        T2 = getTickTime();

        if (flagC)
        {
            T1 = T2;
            flagC = false;
        }
        
        if (T2 - T1 > 500)
        {
            flagB = true;
        }
    }
    break;

  case turnLeft:
    if (isA15_L_flag == 0)
    {
      flagA = true;
    }
    if (isA13_R_flag == 0)
    {
        T2 = getTickTime();

        if (flagC)
        {
            T1 = T2;
            flagC = false;
        }
        
        if (T2 - T1 > 500)
        {
            flagB = true;
        }
    }
    break;
    
    
  case turnRight:
    if (isA13_R_flag == 0)
    {
      flagA = true;
    }
    if (isA15_L_flag == 0)
    {
        T2 = getTickTime();

        if (flagC)
        {
            T1 = T2;
            flagC = false;
        }
        
        if (T2 - T1 > 500)
        {
            flagB = true;
        }
    }
    break;
  case stp:
    break;


  default:
    break;
  }

}

/*
* 路径序列测试函数
* 将规划好的路径存入该函数
*/

void test(){
  mission demo[6];

  demo[0].A = true;
  demo[0].B = false;
  demo[0].C = false;

  demo[1].A = true;
  demo[1].B = false;
  demo[1].C = false;


  demo[2].A = false;
  demo[2].B = true;
  demo[2].C = false;


  demo[3].A = true;
  demo[3].B = false;
  demo[3].C = false;


  demo[4].A = false;
  demo[4].B = true;
  demo[4].C = false;


  demo[5].A = true;
  demo[5].B = false;
  demo[5].C = true;





  testMission =  createMissionList(6,demo);

  
}


/*
* 偏移检测
* 根据时间来确定偏移度(已弃用，外部中断引脚不够)
* 对当前状态进行纠正
* 使用定时中断
*/

void stateFix()
{
  //传感器正常情况下是1 遇到黑色为0

  //朝向左侧传感器
  is2 = digitalRead(2);
  //朝向右侧传感器
  is3 = digitalRead(3);
  
  

  //朝向左侧传感器
  is20 = digitalRead(20);
  //朝向右侧传感器
  is21 = digitalRead(21);



  switch (currentState)
  {
  case goStraight:
    //当前方向 偏右(左侧先触碰)
    /*
    20  21
    1    1
    0    1
    0    0<-这一状态可能要加入调整
    1    1
    */
    

    if (!is20 &&  is21)
    {
      //short diff;
      
      spdA2 += 2;

      //spdA2Count++;
      
    }

    //如果方向为goStraight 偏左(右侧先触碰)

    /*
    20  21
    1    1
    1    0
    0    0<-这一状态可能要加入调整
    1    1
    */
    if (!is21 &&  is20)
    {
      //short diff;
      
      spdA1 += 2;

      //spdA2Count++;
      
    }


    //恢复正常
    if (is21 && is20)
    {
      spdA1 = spdA2 = spdB1;
    }
    
    break;
  case goBack:
    //当前方向偏右(左侧先触)
    /*
    20  21
    1    1
    0    1
    0    0
    1    1
    */

   if (!is20 && is21)
   {
     spdB1 += 2;
   }

   //当前方向偏左(右侧先触)
    /*
    20  21
    1    1
    1    0
    0    0<-这一状态可能要加入调整
    1    1
    */
   if (!is21 && is20)
   {
     spdB2 += 2;
   }

   //恢复正常
   if (is20 && is21)
   {
     spdB1 = spdB2 = spdA2;
   }

    break;
  case turnLeft:
    //当前方向偏左(右侧先触)
    /*
    2    3
    1    1
    1    0
    0    0<-这一状态可能要加入调整
    1    1
    */

   if (!is3 && is2)
   {
     spdB1 += 2;
   }

   //当前方向偏右(左侧先触)
    /*
    2    3
    1    1
    0    1
    0    0<-这一状态可能要加入调整
    1    1
    */
   if (!is2 && is3)
   {
     spdA1 += 2;
   }

   //恢复正常
   if (is2 && is3)
   {
     spdA1 = spdB1 = spdA2;
   }
    
    break;
  case turnRight:
    //当前方向偏右(左侧先触)
    /*
    2    3
    1    1
    0    1
    0    0<-这一状态可能要加入调整
    1    1
    */

   if (!is2 && is3)
   {
     spdB2 += 2;
   }

   //当前方向偏左(右侧先触)
    /*
    2    3
    1    1
    1    0
    0    0<-这一状态可能要加入调整
    1    1
    */
   if (!is3 && is2)
   {
     spdA2 += 2;
   }

   if (is2 && is3)
   {
     spdA2 = spdB2 = spdB1;
   }
   
    break;
  
  default:
    break;
  }
/*=============================早期版本可能后期会有用========================*/
/*
  //如果方向为goStraight 偏右(左侧先触碰)
  if (!is20 &&  is21)
  {
    //short diff;
    
    spdA2 += 2;

    //spdA2Count++;
    
  }

  //如果当前朝向偏左(右侧先触碰)
  if (!is21 && is20)
  {
    spdA1 += 2;
  }
  
  //如果位置纠正了
  if (!is20 && !is21)
  {
    //速度恢复
    spdA2 = spdA1;

    //spdA2Count = 0;
  }




if (currentState == turnLeft || currentState == turnRight)
{
  
    //如果当前朝向偏右(左侧先触)
  if (!is2 &&  is3)
  {
    //short diff;
    
    spdA1 += 2;

    //spdA2Count++;
    
  }
  //如果当前朝向偏左(右侧先触)
  if (!is3 && is2)
  {
    
  }
  


  //如果位置纠正了,或者同时触碰
  if (!is2 && !is3)
  {
    //速度恢复
    spdA2 = spdA1;

    //spdA2Count = 0;
  }
*/

}

  
  




/*
* 舵机角度标定
* 分别在500 1500 2500三个位置
*/

void angleTest(){
  short angle = 500;
    while (1)
  {
    //myse.moveServo(1,angle,1000);
    angle = (angle + 1000) % 2500;
    if (!angle)
    {
      angle = 500;
    }
    
    delay(2000);
  }

}



/*
 * 外部中断函数isr1~4
 */
void isr0()
{
    spdA1Count++;


}


void isr1()
{
    spdA2Count++;


}

void isr2()
{
    spdB1Count++;


}

void isr3()
{
    spdB2Count;

}



void count(){


    detachInterrupt(2);
    detachInterrupt(3);
    detachInterrupt(4);
    detachInterrupt(5);

    // Serial.print("A1:   ");
    // Serial.print(spdA1Count);
    // Serial.print("A2:   ");
    // Serial.print(spdA2Count);
    // Serial.print("B1:   ");
    // Serial.print(spdB1Count);
    // Serial.print("B2:   ");
    // Serial.print(spdB2Count);

    spdA1Count = spdA2Count = spdB1Count = spdB2Count = 0;

    attachInterrupt(2,isr0,CHANGE);
    attachInterrupt(2,isr1,CHANGE);
    attachInterrupt(2,isr2,CHANGE);
    attachInterrupt(2,isr3,CHANGE);




}

long int getTickTime(){
    return millis();
}


