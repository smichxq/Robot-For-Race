// #include <LobotServoController.h>
#include <MsTimer2.h>
//方向
#define goStraight 0
#define goBack 1
#define Left 2
#define Right 3
#define stp 4

//直线微调前压线
#define micro_line_F 5
//直线微调后压线
#define micro_line_B 6
//直线微调左压线
#define micro_line_L 7
//直线微调右压线
#define micro_line_R 8


//中间压线前
#define mid_line_F 9
//中间压线后
#define mid_line_B 10
//中间压线左
#define mid_line_L 11
//中间压线右
#define mid_line_R 12

//越线阈值 不超150 前后
#define tickThreshold 10

//速度
int targetSpd = 100;

//取样周期
#define smpT 100

//当前方向
short currentStates;
short lastStates = Left;

//PID控制器
float setPoint_A1 = 140.0;
float setPoint_A2 = 140.0;
float setPoint_B1 = 140.0;
float setPoint_B2 = 140.0;

 
float KP_A1 = 1.35, KI_A1 = 0.00, KD_A1 = 0.01;


float KP_A2 = 1.29, KI_A2 = 0.00, KD_A2 = 0.01;

float KP_B1 = 1.30, KI_B1 = 0.00, KD_B1 = 0.01;

float KP_B2 = 1.30, KI_B2 = 0.00, KD_B2 = 0.01;


float Ek_A1, Ek_A2, Ek_B1, Ek_B2;                       //当前误差
float Ek1_A1, Ek1_A2, Ek1_B1, Ek1_B2;                      //前一次误差 e(k-1)
float Ek2_A1, Ek2_A2, Ek2_B1 ,Ek2_B2;                      //再前一次误差 e(k-2)





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
  *  A   B   C   D   3bit
  *  AB两位为方向　　  
  * goStraight  goBack  Left  Right
  *    00　       01      10         11
  * Ｃ为模式
  * 方格模式  直线模式
  *   _ _ 0    _ _ 1
  */
  bool A;
  bool B;
  bool C;
  bool D;
  bool E;
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




/*===================================================UART===================================================*/
// byte REQ_CODE[] = [65,66];//服务请求码

byte UART_DATABUF[50];//数据缓冲区

byte UART_DECODE[8] = {97,98,99,100,101,102,103,104};//请求服务确认码

byte UART_TARGET[6];//任务码

byte UART_DELTA[2];//当前误差
/*===================================================UART===================================================*/





/*=====================声明一个全局的任务序列missions[N]=================*/

missions currentMissions[14];

/*=====================声明一个全局的任务序列missions[N]=================*/

/*=====================测试路径==================*/

mission* testMission;
mission* p  = NULL;//p指针在其实例化后无法动态绑定
bool onceFlag = true;

/*=====================测试路径==================*/


/*
 * 外部中断功能引脚 2 3 21 20 19 18
 *               0 1  2  3 4  5
 */
 
//时间变量,用来计时(已弃用，原因：中断优先级未知，外部中断引脚不足)
//long tm = 0;

//boolean is1 = true;//标记位,用来判定是哪一侧先触线

//其他传感器
short Sensor_F_L = 1;
short Sensor_F_R = 1;

short Sensor_R_L = 1;
short Sensor_R_R = 1;

short Sensor_B_L = 1;
short Sensor_B_R = 1;

short Sensor_L_L = 1;
short Sensor_L_R = 1;

//中间传感器
short Sensor_F_M = 1;
short Sensor_R_M = 1;
short Sensor_B_M = 1;
short Sensor_L_M = 1;

/*========================================================================偏移修正======================================================*/

//偏移修正因子
//前后 0.150
//左右 0.1
float FixFctr_GB = 0.920;
float FixFctr_LR = 0.92;
//偏移修正控制因子
bool ctrlFlag = true;
bool SensorFlagL = false;
bool FixCtrlFlag = false;
bool SensorFlagR = false;
long Fix_T1 = 0;
long Fix_T2 = 0;

/*=========================================================================偏移修正=======================================================*/


/*=========================================================================路径规划=======================================================*/
//全局时间变量(路径规划)
long int T1 = 0;
long int T2 = 0;
/*=========================================================================路径规划=======================================================*/

//全局速度变量
short spdA1 = 0;
volatile long spdA1Count = 0;
short spdA2 = 0;
volatile long spdA2Count = 0;
short spdB1 = 0;
volatile long spdB1Count = 0;
short spdB2 = 0;
volatile long spdB2Count = 0;


//状态控制因子
bool flagA = false;
bool flagB = false;
bool flagC = true;
bool flagD = false;


bool micro_line_flag = false;
//微调
bool micro_flag_F = false;
bool micro_flag_B = false;
bool micro_flag_L = false;
bool micro_flag_R = false;


bool mid_line_flag = false;
//中线
bool mid_flag_F = false;
bool mid_flag_B = false;
bool mid_flag_L = false;
bool mid_flag_R = false;


// LobotServoController myse(Serial3);//舵机控制

void setup() {
  //测试路径初始化
  test();
  p = testMission;
  sensorInit();
//  IntServiceInit();
  //前后 A2 +3 B2 +3
  //左 A1 +7.2 B1 +4 B2 +7.2
  //右 A1 +7.3 B1 +4.9 B2 +10.7
  motorInit();
  setSpdA1(targetSpd);
  setSpdA2(targetSpd + 3);
  setSpdB1(targetSpd);
  setSpdB2(targetSpd + 3);
//  Serial.begin(9600);
//  Serial.println("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=");
  delay(500);

}  
void loop()
{    

  /*
  先使用路径规划pathPlan
  再命令电机移动directions
  */
//  if (p){
//    Serial.print(p->A);
//    Serial.print(p->B);
//    Serial.print(p->C);
//    Serial.println(p->D);
//    Serial.println("=======================");
//    p = p->next;
//  }
  
  
  pathPlan();
  directions();
  stateFix();
//  delay(500);
//   Sensor_F_M = digitalRead(SensorPinF_2);
//   if (p->C && Sensor_F_M){
//       spdA1 = spdA2 = spdB1 = spdB2 = 65;
//       while(Sensor_F_M){
//           currentStates = goStraight;
//           directions();
//          
//       }
//       spdA1 = spdA2 = spdB1 = spdB2 = 0;
//       currentStates = stp;
//   }
  

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
  pinMode(CodeA1,INPUT);
  pinMode(CodeA2,INPUT);
  pinMode(CodeB1,INPUT);
  pinMode(CodeB2,INPUT);

/*
* 外部中断口服务初始化
* int.2 int.3 int.4 int.5
*   21   20     19    18
*/
  attachInterrupt(5, isr0, CHANGE);
  attachInterrupt(4, isr1, CHANGE);
  attachInterrupt(3, isr2, CHANGE);
  attachInterrupt(2, isr3, CHANGE);
  

  
}

//中断服务初始化

void IntServiceInit(){
    MsTimer2::set(smpT,acc);
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
* Left 左平移
* Right 右平移
*
* 参数为方向,速度
* 目前只有四个方向
* 后期增加其他方向
*/
void directions(){
    
    short dirct = currentStates;
    //电机换向保护
//    if (dirct != lastStates){
//        setSpdA1(0);
//        setSpdA2(0);
//        setSpdB1(0);
//        setSpdB2(0);
//        motorA1PNS(S);
//        motorA2PNS(S);
//        motorB1PNS(S);
//        motorB2PNS(S);
//        delay(200);
//        setSpdA1(spdA1);
//        setSpdA2(spdA2);
//        setSpdB1(spdB1);
//        setSpdB2(spdB2);
//    }
//    lastStates = dirct;


  


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
      case micro_line_F:
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

        motorA1PNS(N);
        motorA1();

        motorA2PNS(N);
        motorA2();

        motorB1PNS(N);
        motorB1();

        motorB2PNS(N);
        motorB2();
        break;

    case micro_line_B:
        //printf("goStraight\n");
        motorA1PNS(N);
        motorA1();

        motorA2PNS(N);
        motorA2();

        motorB1PNS(N);
        motorB1();

        motorB2PNS(N);
        motorB2();
        
        break;
    case Left:

    //左 A1 +7.2 B1 +4 B2 +7.2
        //printf("Left\n");

        motorA1PNS(N);
        motorA1();

        motorA2PNS(P);
        motorA2();

        motorB1PNS(P);
        motorB1();

        motorB2PNS(N);
        motorB2();

        break;
     case micro_line_L:
        //printf("goStraight\n");
   
        motorA1PNS(N);
        motorA1();

        motorA2PNS(P);
        motorA2();

        motorB1PNS(P);
        motorB1();

        motorB2PNS(N);
        motorB2();
        
        break;
        
    case Right:
    //右 A1 +7.3 B1 +4.9 B2 +10.7
//        setSpdA1(targetSpd + 7.3);
//        setSpdA2(targetSpd);
//        setSpdB1(targetSpd + 4.9);
//        setSpdB2(targetSpd + 10.7);
        //printf("Right\n");
        motorA1PNS(P);
        motorA1();

        motorA2PNS(N);
        motorA2();

        motorB1PNS(N);
        motorB1();

        motorB2PNS(P);
        motorB2();
        break;
   case micro_line_R:
        //printf("goStraight\n");
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
      setSpdA1(0);
      setSpdA2(0);
      setSpdB1(0);
      setSpdB2(0);

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
mission* addMission(bool a,bool b,bool c,bool d,bool e)
{
  mission* currentMission = NULL;

  currentMission = (mission*)malloc(sizeof(mission));

  currentMission->A = a;
  currentMission->B = b;
  currentMission->C = c;
  currentMission->D = d;
  currentMission->E = e;
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
  bool flag = true;
  for (int i = 0; i < num; i++)
  {
    if (flag)
    {
      head = addMission(missArry[i].A,missArry[i].B,missArry[i].C,missArry[i].D,missArry[i].E);
      p = head;
      flag = false;
      continue;
    }

    p->next = addMission(missArry[i].A,missArry[i].B,missArry[i].C,missArry[i].D,missArry[i].E);
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
    //状态更新控制因子
    bool Listflag = true;





  //判断是否经过方格
    Sensor_F_L = digitalRead(SensorPinF_1);
    Sensor_F_R = digitalRead(SensorPinF_3);

    Sensor_B_L = digitalRead(SensorPinB_1);
    Sensor_B_R = digitalRead(SensorPinB_3);

    Sensor_L_L = digitalRead(SensorPinL_1);
    Sensor_L_R = digitalRead(SensorPinL_3);

    Sensor_R_L = digitalRead(SensorPinR_1);
    Sensor_R_R = digitalRead(SensorPinR_3);

    Sensor_F_M = digitalRead(SensorPinF_2);
    Sensor_B_M = digitalRead(SensorPinB_2);

    Sensor_L_M = digitalRead(SensorPinL_2);
    Sensor_R_M = digitalRead(SensorPinR_2);

  

 

  //经过方格，移动到下一个路径
  if (flagA && flagD)
  {
    p = p->next;
    flagA = false;
    flagB = false;
    flagC = true;
    flagD = false;
    T1 = T2 = 0;
    if (p == NULL){
        Listflag = false;
        currentStates = stp;
    }
  }
  

  
  
  
//封装为函数

    //前
    if (!(p->A) && !(p->B) && Listflag)
    {
      //更新当前方向
      currentStates = goStraight;
      micro_line_flag = false;
      mid_line_flag = false;
      
    }
    //后
    if (!(p->A) && p->B && Listflag)
    {
      //更新当前方向
      currentStates = goBack;
      micro_line_flag = false;
      mid_line_flag = false;
    }
    //左
    if (p->A && !(p->B) && Listflag)
    {
      //更新当前方向
      currentStates = Left;
       micro_line_flag = false;
       mid_line_flag = false;
    }
    //右
    if (p->A && p->B && Listflag)
    {
      //更新当前方向
      currentStates = Right;
       micro_line_flag = false;
       mid_line_flag = false;
    }
    //停止
    if (p->C && Listflag)
    {
        currentStates = stp;
         micro_line_flag = false;
         mid_line_flag = false;
    }



    //线条模式 前压线

    if (p->D && Listflag && !(p->A) && !(p->B))
    {
        micro_flag_F = true;
        currentStates = micro_line_F;
    }
    
    //线条模式 后压线
    if (p->D && Listflag && !(p->A) && p->B)
    {
        micro_flag_B = true;
        currentStates = micro_line_B;
    }
    //线条模式 左压线
    if (p->D && Listflag && p->A && !(p->B))
    {
        micro_flag_L = true;
        currentStates = micro_line_L;
    }
    //线条模式 右压线
    if (p->D && Listflag && p->A && p->B)
    {
        micro_flag_R = true;
        currentStates = micro_line_R;
    }


    //中线模式 前

    if (p->E && !(p->A) && !(p->B)){
      mid_flag_F = true;
      currentStates = mid_line_F;
    }
    //中线模式 后
    if (p->E && !(p->A) && p->B){
      mid_flag_B = true;
      currentStates = mid_line_B;
    }
    //中线模式 左
    if (p->E && p->A && !(p->B)){
      mid_flag_L = true;
      currentStates = mid_line_L;
    }
    //中线模式 右
    if (p->E && p->A && p->B){
      mid_flag_R = true;
      currentStates = mid_line_R;
    }
    










   //判断是否经过方格
  switch (currentStates)
  {


  case goStraight:
  
  //上次状态为特殊模式
    if (micro_line_flag || mid_line_flag){
        flagA = true;
        micro_flag_B = micro_flag_F = micro_flag_L = micro_flag_R = false;
        mid_flag_B = mid_flag_F = mid_flag_R = mid_flag_L = false;
    }


    if (Sensor_F_M == 0)
    {
      flagA = true;
    }
    if (Sensor_B_M == 0 && flagA)
    {
        //tick-pwm表
        flagB = true;

    }

    if (Sensor_B_M == 1 && flagB)
    {
        T2 = getTickTime();

        if (flagC)
        {
            T1 = T2;
            flagC = false;
        }
        
        if (T2 - T1 > tickThreshold)
        {
            flagD = true;
        }
    }
    break;
  case goBack:

  //上次状态为特殊模式
    if (micro_line_flag || mid_line_flag){
        flagA = true;
        micro_flag_B = micro_flag_F = micro_flag_L = micro_flag_R = false;
        mid_flag_B = mid_flag_F = mid_flag_R = mid_flag_L = false;
    }

    if (Sensor_B_M == 0)
    {
      flagA = true;
    }
    if (Sensor_F_M == 0 && flagA)
    {
        flagB = true;

    }

    if(Sensor_F_M == 1 && flagB)
    {
        T2 = getTickTime();

        if (flagC)
        {
            T1 = T2;
            flagC = false;
        }
        
        if (T2 - T1 > tickThreshold)
        {
            flagD = true;
        }
    }
    break;

  case Left:

  //上次状态为特殊模式
    if (micro_line_flag || mid_line_flag){
        flagA = true;
        micro_flag_B = micro_flag_F = micro_flag_L = micro_flag_R = false;
        mid_flag_B = mid_flag_F = mid_flag_R = mid_flag_L = false;
    }
    if (Sensor_L_M == 0)
    {
      flagA = true;
    }
    if (Sensor_R_M == 0 && flagA)
    {
        flagB = true;
    }
    if (Sensor_R_M == 1 && flagB)
    {
        T2 = getTickTime();

        if (flagC)
        {
            T1 = T2;
            flagC = false;
        }
        
        if (T2 - T1 > tickThreshold + 30)
        {
            flagD = true;
        }
    }
    break;
    
    
  case Right:

  //上次状态为特殊模式
    if (micro_line_flag || mid_line_flag){
        flagA = true;
        micro_flag_B = micro_flag_F = micro_flag_L = micro_flag_R = false;
        mid_flag_B = mid_flag_F = mid_flag_R = mid_flag_L = false;
    }
    if (Sensor_R_M == 0)
    {
      flagA = true;
    }
    if (Sensor_L_M == 0 && flagA)
    {
        flagB =true;
    }

    if (Sensor_L_M == 1 && flagB)
    {
        T2 = getTickTime();

        if (flagC)
        {
            T1 = T2;
            flagC = false;
        }
        
        if (T2 - T1 > tickThreshold + 30)
        {
            flagD = true;
        }
    }
    
    break;
  case stp:
    Listflag = false;
    flagA = false;
    flagB = false;
    flagC = true;
    flagD = false;
    break;




  //压线模式
  case micro_line_F:
  //碰线,状态切换
    if (!Sensor_F_M){
      flagA = flagD = true;
      
    }
    break;
  case micro_line_B:
  //碰线,路径切换
    if (!Sensor_B_M){
      flagA = flagD = true;
    }
    break;
    
  case micro_line_L:
  //碰线,切换下一个状态
    if (!Sensor_L_L){
      flagA = flagD = true;
      
      
    }
     break;
  case micro_line_R:
  //碰线,切换下一个状态
    if (!Sensor_R_M){
      flagA = flagD = true;
      
    }
    break;


    //中线模式
  case mid_line_F:
  //碰线,路径切换
    if (!Sensor_L_M && !Sensor_R_M){
      flagA = flagD = true;
      
    }
    break;
  case mid_line_B:
  //碰线,路径切换
    if (!Sensor_L_M && !Sensor_R_M){
      flagA = flagD = true;
    }
    break;
    
  case mid_line_L:
  //碰线,切换下一个状态
    if (!Sensor_F_M && !Sensor_B_M){
      flagA = flagD = true;
      
    }
     break;
  case mid_line_R:
  //碰线,切换下一个状态
    if (!Sensor_B_M && !Sensor_F_M){
      flagA = flagD = true;
    }
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
  mission demo[7];
  //L
  demo[0].A = true;
  demo[0].B = false;
  demo[0].C = false;
  demo[0].D = false;
  demo[0].E = false;
  //L
  demo[1].A = true;
  demo[1].B = false;
  demo[1].C = false;
  demo[1].D = false;
  demo[1].E = false;
  //L
  demo[2].A = true;
  demo[2].B = false;
  demo[2].C = false;
  demo[2].D = false;
  demo[2].E = false;
  //B
  demo[3].A = true;
  demo[3].B = false;
  demo[3].C = false;
  demo[3].D = false;
  demo[3].E = false;


  //L
  demo[4].A = false;
  demo[4].B = true;
  demo[4].C = false;
  demo[4].D = false;
  demo[4].E = false;

  
////后
//  demo[5].A = false;
//  demo[5].B = true;
//  demo[5].C = false;
//  demo[5].D = true;
//  demo[5].E = false;

//左
  demo[5].A = false;
  demo[5].B = false;
  demo[5].C = false;
  demo[5].D = true;
  demo[5].E = false;
//
////右
//  demo[5].A = false;
//  demo[5].B = false;
//  demo[5].C = false;
//  demo[5].D = true;
//  demo[5].E = false;

  demo[6].A = false;
  demo[6].B = false;
  demo[6].C = true;
  demo[6].D = false;
  demo[6].E = false;


  

  testMission =  createMissionList(7,demo);

  
}


/*
* 偏移检测
* 根据时间来确定偏移度
* 对当前状态进行纠正
* 使用定时中断
*/

void stateFix()
{
  if (currentStates >4 && currentStates<9){
    return;
  }

//  MsTimer2::stop();

    
    Sensor_F_L = digitalRead(SensorPinF_1);
    Sensor_F_R = digitalRead(SensorPinF_3);

    Sensor_B_L = digitalRead(SensorPinB_1);
    Sensor_B_R = digitalRead(SensorPinB_3);

    Sensor_L_L = digitalRead(SensorPinL_1);
    Sensor_L_R = digitalRead(SensorPinL_3);

    Sensor_R_L = digitalRead(SensorPinR_1);
    Sensor_R_R = digitalRead(SensorPinR_3);

    Sensor_F_M = digitalRead(SensorPinF_2);
    Sensor_B_M = digitalRead(SensorPinB_2);
    Sensor_L_M = digitalRead(SensorPinL_2);
    Sensor_R_M = digitalRead(SensorPinR_2);
    






//前后左右
    switch (currentStates)
    {

    case goStraight: 
        //偏右
        if (!Sensor_F_L && Sensor_F_R && ctrlFlag)
        {
//          Serial.println("===偏右===");

            SensorFlagL = true;//先触碰一侧
            ctrlFlag = false;//计时控制因子
            Fix_T1 = getTickTime();
            FixCtrlFlag = true;
        }

        //偏左
        if (!Sensor_F_R && Sensor_F_L && ctrlFlag) 
        {
//          Serial.println("===偏左===");
            SensorFlagR = true;//标记先触碰一侧
            ctrlFlag = false;//计时控制因子
            Fix_T1 = getTickTime();
            FixCtrlFlag = true;
        }


        //偏左未修正状态
        if (SensorFlagR)
        {
//          Serial.println("===偏左修正中===");

            Fix_T2 = FixFctr_GB*(getTickTime() - Fix_T1);
            //减速修正
            setSpdA1(targetSpd + Fix_T2);
            setSpdB1(targetSpd + Fix_T2);
            //加速修正
            setSpdA2(targetSpd - Fix_T2);
            setSpdB2(targetSpd - Fix_T2);

        }

        


        //偏左已修正
        if (SensorFlagR && Sensor_F_L && Sensor_F_R)
        {
//          Serial.println("偏已左修正");
            // FixCtrlFlag = false;
            // SensorFlagL = false;
            Fix_T1 = Fix_T2 = 0;
            ctrlFlag = true;
            SensorFlagR = false;
            //减速修正
            setSpdA2(targetSpd + 3);
            setSpdB1(targetSpd);
            //加速修正
            setSpdA1(targetSpd);
            setSpdB2(targetSpd + 3);

        }
        
        
        

        //偏右未纠正
        if (SensorFlagL)
        {   //注意类型 
//          Serial.println("===偏右修正中===");
            Fix_T2 = FixFctr_GB*(getTickTime() - Fix_T1);
            //减速修正
            setSpdA1(targetSpd - Fix_T2);
            setSpdB1(targetSpd - Fix_T2);
            //加速修正
            setSpdA2(targetSpd + Fix_T2);
            setSpdB2(targetSpd + Fix_T2);

        }


        //偏右已纠正
        if (SensorFlagL && Sensor_F_R && Sensor_F_L)
        {
//          Serial.println("===偏右已修正===");

            SensorFlagL = false;
            ctrlFlag = true;
            Fix_T1 = Fix_T2 = 0;
            //减速恢复
            setSpdA1(targetSpd);
            setSpdB2(targetSpd + 3);
            //加速恢复
            setSpdA2(targetSpd + 3);
            setSpdB1(targetSpd);

        }

        break;
        
    case goBack:
        // Serial.print("左传感器: ");
        // Serial.print(Sensor_B_L);
        // Serial.print("  右传感器:    ");
        // Serial.println(Sensor_B_R);
        //偏右
        
        if (!Sensor_B_L && Sensor_B_R && ctrlFlag)
        {

            SensorFlagL = true;//标记先触碰一侧
            ctrlFlag = false;//计时控制因子
            Fix_T1 = getTickTime();
            // Serial.println(" =========== 偏右  =============");

        }

        //偏左
        if (!Sensor_B_R && Sensor_B_L && ctrlFlag) 
        {
            SensorFlagR = true;//标记先触碰一侧
            ctrlFlag = false;//计时控制因子
            Fix_T1 = getTickTime();
            // Serial.println(" ============ 偏左  =============");
        }


        //偏左未修正

        if (SensorFlagR)
        {
            Fix_T2 = FixFctr_GB*(getTickTime() - Fix_T1);
            //减速修正
            setSpdA2(targetSpd + Fix_T2);
            setSpdB1(targetSpd - Fix_T2);
            //加速修正
            setSpdA1(targetSpd - Fix_T2);
            setSpdB2(targetSpd + Fix_T2);
            // Serial.println(" ============= 偏左修正中 ================= ");

            //Serial.print("A2+++ B2+++ A1--- B1---   ");
        }


        //偏左已修正
        if (SensorFlagR && Sensor_B_L && Sensor_B_R && Sensor_B_M)
        {
            SensorFlagR = false;
            ctrlFlag = true;
            // Fix_T1 = Fix_T2 = 0;
            //Serial.println("=================偏左恢复正常===================");
            //减速修正
            setSpdA2(targetSpd + 3);
            setSpdB1(targetSpd);
            //加速修正
            setSpdA1(targetSpd);
            setSpdB2(targetSpd + 3);

        }
        
        
        

        //偏右未纠正
        if (SensorFlagL)
        {   //注意类型 
            Fix_T2 = FixFctr_GB*(getTickTime() - Fix_T1);
            //减速修正
            setSpdA1(targetSpd + Fix_T2);
            setSpdB2(targetSpd - Fix_T2);
            //加速修正
            setSpdA2(targetSpd - Fix_T2);
            setSpdB1(targetSpd + Fix_T2);

        }
        //偏右已纠正
        if (SensorFlagL && Sensor_B_R && Sensor_B_L && Sensor_B_M)
        {
            SensorFlagL = false;
            ctrlFlag = true;
            // Fix_T1 = Fix_T2 = 0;

            //减速恢复
            setSpdA1(targetSpd);
            setSpdB2(targetSpd + 3);
            //加速恢复
            setSpdA2(targetSpd + 3);
            setSpdB1(targetSpd);

        }
        break;

    case Left:

        //偏右
        if (!Sensor_L_L && Sensor_L_R && ctrlFlag)
        {
//          Serial.println("===偏右===");
          

            SensorFlagL = true;//标记偏向
            ctrlFlag = false;//计时控制因子
            Fix_T1 = getTickTime();
            // Serial.println("偏右");
        }

        //偏左
        if (!Sensor_L_R && Sensor_L_L && ctrlFlag) 
        {
//          Serial.println("===偏左===");
            SensorFlagR = true;//标记偏向哪一侧
            ctrlFlag = false;//计时控制因子
            Fix_T1 = getTickTime();
            // Serial.println("偏左");
        }


        //偏左未修正

        if (SensorFlagR)
        {
//          Serial.println("===偏左修正中===");
            Fix_T2 = FixFctr_LR*(getTickTime() - Fix_T1);
            //减速修正
            setSpdA1(targetSpd - Fix_T2);
            setSpdB2(targetSpd + Fix_T2);
            //加速修正
            setSpdA2(targetSpd - Fix_T2);
            setSpdB1(targetSpd + Fix_T2);
            // Serial.println("偏左修正中");
        }


        //偏左已修正
        if (SensorFlagR && Sensor_L_L && Sensor_L_R && Sensor_L_M)
        {
//          Serial.println("===偏左已修正===");
            SensorFlagR = false;
            ctrlFlag = true;
            // Fix_T1 = Fix_T2 = 0;
            //减速修正
            setSpdA1(targetSpd + 7.2);
            setSpdB2(targetSpd + 7.2);
            //加速修正
            setSpdA2(targetSpd);
            setSpdB1(targetSpd + 4);
            

        }
        
        
        

        //偏右未纠正
        if (SensorFlagL)
        {   //注意类型 
//          Serial.println("===偏右修正中===");
            Fix_T2 = FixFctr_LR*(getTickTime() - Fix_T1);
            //减速修正
            setSpdA2(targetSpd + Fix_T2);
            setSpdB1(targetSpd - Fix_T2);
            //加速修正
            setSpdA1(targetSpd + Fix_T2);
            setSpdB2(targetSpd - Fix_T2);
            // Serial.println("偏右修正中");

        }


        //偏右已纠正
        if (SensorFlagL && Sensor_L_R && Sensor_L_L && Sensor_L_M)
        {
//          Serial.println("===偏右已修正===");
            SensorFlagL = false;
            ctrlFlag = true;
            // Fix_T1 = Fix_T2 = 0;

            //减速恢复
            setSpdA2(targetSpd + 7.2);
            setSpdB1(targetSpd + 4);
            //加速恢复
            setSpdA1(targetSpd + 7.3);
            setSpdB2(targetSpd);

        }
        break;

    case Right:
        //偏右
        if (!Sensor_R_L && Sensor_R_R && ctrlFlag)
        {
//          Serial.println("===偏右===");

            SensorFlagL = true;//标记偏向
            ctrlFlag = false;//计时控制因子
            Fix_T1 = getTickTime();
        }

        //偏左
        if (!Sensor_R_R && Sensor_R_L && ctrlFlag) 
        {
//          Serial.println("===偏左===");
            SensorFlagR = true;//标记偏向哪一侧
            ctrlFlag = false;//计时控制因子
            Fix_T1 = getTickTime();
        }


        //偏左未修正

        if (SensorFlagR)
        {
//          Serial.println("===偏左修正中===");
            Fix_T2 = FixFctr_LR*(getTickTime() - Fix_T1);
            //减速修正
            setSpdA1(targetSpd + Fix_T2);
            setSpdB2(targetSpd - Fix_T2);
            //加速修正
            setSpdA2(targetSpd + Fix_T2);
            setSpdB1(targetSpd - Fix_T2);
        }


        //偏左已修正
        if (SensorFlagR && Sensor_R_L && Sensor_R_R && Sensor_R_M)
        {
//          Serial.println("===偏左已修正===");
            SensorFlagR = false;
            ctrlFlag = true;
            Fix_T1 = Fix_T2 = 0;
            //减速修正
            setSpdA1(targetSpd + 7.3);
            setSpdB2(targetSpd + 10.7);
            //加速修正
            setSpdA2(targetSpd);
            setSpdB1(targetSpd + 4.9);

        }
        
        
        

        //偏右未纠正
        if (SensorFlagL)
        {   //注意类型 
//          Serial.println("===偏右修正中==");
            Fix_T2 = FixFctr_LR*(getTickTime() - Fix_T1);
            //减速修正
            setSpdA2(spdA2 - Fix_T2);
            setSpdB1(spdB1 + Fix_T2);
            //加速修正
            setSpdA1(spdA1 - Fix_T2);
            setSpdB2(spdB2 + Fix_T2);

        }
        //偏右已纠正
        if (SensorFlagL && Sensor_R_R && Sensor_R_L && Sensor_R_M)
        {
//          Serial.println("===偏右已修正===");
            SensorFlagL = false;
            ctrlFlag = true;
            Fix_T1 = Fix_T2 = 0;

            //减速恢复
            setSpdA2(targetSpd);
            setSpdB1(targetSpd + 4.9);
            //加速恢复
            setSpdA1(targetSpd + 7.3);
            setSpdB2(targetSpd + 10.7);
        }
        break;
    //微调前模式
    case micro_line_F:
        setSpdA1(targetSpd-20);
        setSpdA2(targetSpd-20);
        setSpdB1(targetSpd-20);
        setSpdB2(targetSpd-20);
        break;
        
    //微调左模式
    case micro_line_L:
        //车身左前偏移(当前朝向右偏)
        if (Sensor_F_L){
          SensorFlagL = true;
        }
        //车身右前偏移(当前朝向左偏)
        if (Sensor_F_R){
          SensorFlagR = true;
        }
        //车身正
        if (!Sensor_F_R && !Sensor_F_M && !Sensor_F_L){
          SensorFlagR = false;
          SensorFlagL = false;
          setSpdA2(targetSpd-20);
          setSpdB1(targetSpd-20);
          //加速恢复
          setSpdA1(targetSpd-20);
          setSpdB2(targetSpd-20);
        }
        //右偏未校正
        if (SensorFlagL){
            Fix_T2 = FixFctr_LR*(getTickTime() - Fix_T1);
            //减速修正
            setSpdA2(targetSpd + Fix_T2);
            setSpdB1(targetSpd - Fix_T2);
            //加速修正
            setSpdA1(targetSpd + Fix_T2);
            setSpdB2(targetSpd - Fix_T2);
        }
        //左偏未校正
        if (SensorFlagR){
            Fix_T2 = FixFctr_LR*(getTickTime() - Fix_T1);
            //减速修正
            setSpdA1(targetSpd - Fix_T2);
            setSpdB2(targetSpd + Fix_T2);
            //加速修正
            setSpdA2(targetSpd - Fix_T2);
            setSpdB1(targetSpd + Fix_T2);
        }
        break;

    
    default:
        break;
    }


//    MsTimer2::start();



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
*获取通信数据
*阻塞
*/
bool get_data(){
    int i = 0;
    byte temp;
    bool temp_flag = false;

    if (Serial.available()<0){
        return false;
    }

    while (Serial.available()>0){
        // 每次只读取一个字节    
        temp = Serial.read();
        if (temp_flag || temp == 0x23)
        {
            UART_DATABUF[i] = temp;
        }
//        Serial.write(UART_DATABUF[i]);
//        Serial.write("  ");
        i++;
     }
     Serial.flush();
     return true;
 }

/*
*数据解码
*
*/
bool decodes(){
    short i = 0;

    
    while (UART_DATABUF[i] != 36 && i<51){
        
        
        //解码#a123321$
        if (UART_DATABUF[i] == UART_DECODE[0] && UART_DATABUF[i+7] == 36){
            //confmCode:a
            decode_a(i);//数据解码存储
            
            return false;

        }
        //          b$             b@&x&y$            b&x&y&xxx$
        else if (UART_DATABUF[i] == UART_DECODE[1]){
            //扫描下一个
            if (UART_DATABUF[i+1] == 36){
                //切换状态,车辆移动至下一格子
            }
            //抓取当前的
            if (UART_DATABUF[i+2] == 64){
                //解码,将数据存放至UART_DELTA[2]中
                //区分b@& - 1 2 & + 2 2$

            }
            //抓取并存放路径规划
            if (UART_DATABUF[i+2] == 38){
                //解码,更新路径,存放偏差
            }
            //confmCode:b

        }
        else if (UART_DATABUF[i] == UART_DECODE[2]){
            //confmCode:c
        }
        else if (UART_DATABUF[i] == UART_DECODE[3]){
            //confmCode:d
        }
        else if (UART_DATABUF[i] == UART_DECODE[4]){
            //confmCode:e
        }
        else if (UART_DATABUF[i] == UART_DECODE[5]){
            //confmCode:f
        }
        else if (UART_DATABUF[i] == UART_DECODE[6]){
            //confmCode:g
        }
        else{
            //confmCodeERROR
        }
        i++;
    }
    return true;
}



//请求服务,参数为请求码,反馈码
void reqs(char* req_code,char* resp_code){
    int i = 0;
    bool uart_flag = true;

    while (uart_flag)
    {
        Serial.flush();

        Serial.write(req_code,3);

        // Serial.write("#B$");
        //没有收到数据就跳出
        if (!get_data()){
            continue;
        }

        uart_flag = respServices(decodes(),resp_code);
        
    }

}

//任务码本地存储
void decode_a(short i){
    i++;
    int j = 0;
    while (UART_DATABUF[i]!=36){
      UART_TARGET[j] = UART_DATABUF[i];
      i++;
      j++;
    }

    // delay(100000);
    // delay(2);
}

//反馈服务
bool respServices(bool flag,char* resp_code){
    int i = 0;

    if(!flag){
        while (i < 20)
        {      
            Serial.write(resp_code,4);
            i++;
            //中断服务加入特殊情况,如果收到openmv特殊请求,重新运行请求服务
        }
        

        return flag;
    }
    
    return flag;
}


    

//数据解码服务
void decode_b(int i){
    
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
    spdB2Count++;

}

void acc(){
  if (micro_line_flag){
    targetSpd = 50;
  }
  else{
    targetSpd = 100;
  }
  if(!(p->C) && spdA1 <= targetSpd){
    spdA1 += 1;
    spdA2 += 1;
    spdB1 += 1;
    spdB2 += 1;
  }
  if (p->C){
    spdA1 -= 1;
    spdA2 -= 1;
    spdB1 -= 1;
    spdB2 -= 1;
  }
}

void count(){


    detachInterrupt(2);
    detachInterrupt(3);
    detachInterrupt(4);
    detachInterrupt(5);

    

    setSpdA1(spdController_A1(spdA1Count,1));
//    Serial.println(spdA1Count);
//    Serial.print(",");
//    Serial.println(spdA1);


    setSpdA2(spdController_A2(spdA2Count,2));
//
//    Serial.println(spdA2Count);
//    Serial.print(",");
//    Serial.println(spdA2);
    
    setSpdB1(spdController_B1(spdB1Count,2));

//    Serial.println(spdB1Count);
//    Serial.print(",");
//    Serial.println(spdB1);

    setSpdB2(spdController_B2(spdB2Count,2));

//    Serial.println(spdB2Count);
//    Serial.print(",");
//    Serial.println(spdB2);





    spdA1Count = spdA2Count = spdB1Count = spdB2Count = 0;

  attachInterrupt(5, isr0, CHANGE);
  attachInterrupt(4, isr1, CHANGE);
  attachInterrupt(3, isr2, CHANGE);
  attachInterrupt(2, isr3, CHANGE);




}

long int getTickTime(){
    return millis();
}



/*
参    数 ： setPoint:设置值 
            ActualValue:反馈值 
            Mode: 1 2 3 4 四个轮子
返 回 值 ： PIDInc:本次PID增量(+/-)
*/

float spdController_A1(float ActualValue, short Mode)
{

      float PIDInc;  //增量
  
  
        Ek_A1 = setPoint_A1 - ActualValue;
        
        PIDInc = (KP_A1 * Ek_A1) - (KI_A1 * Ek1_A1) + (KD_A1 * Ek2_A1);

        Ek2_A1 = Ek1_A1;
        Ek1_A1 = Ek_A1;  

                            
  
  return PIDInc;
}

float spdController_A2(float ActualValue, short Mode)
{

      float PIDInc;  //增量
  
  
        Ek_A2 = setPoint_A2 - ActualValue;
        
        PIDInc = (KP_A2 * Ek_A2) - (KI_A2 * Ek1_A2) + (KD_A2 * Ek2_A2);

        Ek2_A2 = Ek1_A2;
        Ek1_A2 = Ek_A2;  

                            
  
  return PIDInc;
}
float spdController_B1(float ActualValue, short Mode)
{

      float PIDInc;  //增量
  
  
        Ek_B1 = setPoint_B1 - ActualValue;
        
        PIDInc = (KP_B1 * Ek_B1) - (KI_B1 * Ek1_B1) + (KD_B1 * Ek2_B1);

        Ek2_B1 = Ek1_B1;
        Ek1_B1 = Ek_B1;  

                            
  
  return PIDInc;
}
float spdController_B2(float ActualValue, short Mode)
{

      float PIDInc;  //增量
  
  
        Ek_B2 = setPoint_B2 - ActualValue;
        
        PIDInc = (KP_B2 * Ek_B2) - (KI_B2 * Ek1_B2) + (KD_B2 * Ek2_B2);

        Ek2_B2 = Ek1_B2;
        Ek1_B2 = Ek_B2;  

                            
  
  return PIDInc;
}