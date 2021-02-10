// #include <LobotServoController.h>
#include <MsTimer2.h>
//方向
#define goStraight 0
#define goBack 1
#define turnLeft 2
#define turnRight 3
#define stp 4

//越线阈值 不超150 前后
#define tickThreshold 40

//速度
#define targetSpd 50

//取样周期
#define smpT 5

//当前方向
short currentStates;

//PID控制器
float setPoint = 9.0;





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

/*=======偏移修正========*/

//偏移修正因子
//前后 0.150
//左右 0.1
float FixFctr_GB = 0.150;
float FixFctr_LR = 0.1;
//偏移修正控制因子
bool ctrlFlag = true;
bool SensorFlagL = false;
bool FixCtrlFlag = false;
bool SensorFlagR = false;
long Fix_T1 = 0;
long Fix_T2 = 0;

/*=======偏移修正========*/


/*=======路径规划========*/
//全局时间变量
long int T1 = 0;
long int T2 = 0;
/*=======路径规划========*/

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


// LobotServoController myse(Serial3);//舵机控制

void setup() {
  //测试路径初始化
  test();
  p = testMission;
  sensorInit();
//   IntServiceInit();
  //前后 A2 +3 B2 +3
  //左右 A1 +7.2 B1 +4 B2 +7.2
  motorInit();
  setSpdA1(targetSpd);
  setSpdA2(targetSpd);
  setSpdB1(targetSpd);
  setSpdB2(targetSpd);
//   Serial.begin(9600);
  delay(500);

}  
void loop()
{

    
    

  /*
  先使用路径规划pathPlan
  在命令电机移动directions
  */
//   pathPlan();//检查路径规划
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
    MsTimer2::set(smpT,count);
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
  short dirct = currentStates;

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

      setSpdA1(0);
      setSpdA2(0);
      setSpdB1(0);
      setSpdB2(0);
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
  bool flag = true;
  for (int i = 0; i < num; i++)
  {
    if (flag)
    {
      head = addMission(missArry[i].A,missArry[i].B,missArry[i].C);
      p = head;
      flag = false;
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
    //状态更新控制因子
    bool Listflag = true;





  //判断是否经过方格
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
  

  
  
  


    //前
    if (!(p->A) && !(p->B) && Listflag)
    {
      //更新当前方向
      currentStates = goStraight;
      
    }
    //后
    if (!(p->A) && p->B && Listflag)
    {
      //更新当前方向
      currentStates = goBack;
    }
    //左
    if (p->A && !(p->B) && Listflag)
    {
      //更新当前方向
      currentStates = turnLeft;
    }
    //右
    if (p->A && p->B && Listflag)
    {
      //更新当前方向
      currentStates = turnRight;
    }

    if (p->C && Listflag)
    {
        currentStates = stp;
    }

   //判断是否经过方格
  switch (currentStates)
  {
  case goStraight:
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

  case turnLeft:
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
    
    
  case turnRight:
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

  default:
    break;
  }

}
/*
* 路径序列测试函数
* 将规划好的路径存入该函数
*/

void test(){
  mission demo[9];
    //S
  demo[0].A = false;
  demo[0].B = false;
  demo[0].C = false;
    //S
  demo[1].A = false;
  demo[1].B = false;
  demo[1].C = false;
    //L
  demo[2].A = true;
  demo[2].B = false;
  demo[2].C = false;
  //L
  demo[3].A = true;
  demo[3].B = false;
  demo[3].C = false;

    //Right
  demo[4].A = true;
  demo[4].B = true;
  demo[4].C = false;
      //Right
  demo[5].A = true;
  demo[5].B = true;
  demo[5].C = false;

      //Back
  demo[6].A = false;
  demo[6].B = true;
  demo[6].C = false;
      //Back
  demo[7].A = false;
  demo[7].B = true;
  demo[7].C = false;

    //stp
  demo[8].A = false;
  demo[8].B = false;
  demo[8].C = true;


  testMission =  createMissionList(9,demo);

  
}


/*
* 偏移检测
* 根据时间来确定偏移度
* 对当前状态进行纠正
* 使用定时中断
*/

void stateFix()
{

    
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

            SensorFlagL = true;//先触碰一侧
            ctrlFlag = false;//计时控制因子
            Fix_T1 = getTickTime();
            FixCtrlFlag = true;
        }

        //偏左
        if (!Sensor_F_R && Sensor_F_L && ctrlFlag) 
        {
            SensorFlagR = true;//标记先触碰一侧
            ctrlFlag = false;//计时控制因子
            Fix_T1 = getTickTime();
            FixCtrlFlag = true;
        }


        //偏左未修正状态
        if (SensorFlagR)
        {

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

    case turnLeft:

        //偏右
        if (!Sensor_L_L && Sensor_L_R && ctrlFlag)
        {

            SensorFlagL = true;//标记偏向
            ctrlFlag = false;//计时控制因子
            Fix_T1 = getTickTime();
            // Serial.println("偏右");
        }

        //偏左
        if (!Sensor_L_R && Sensor_L_L && ctrlFlag) 
        {
            SensorFlagR = true;//标记偏向哪一侧
            ctrlFlag = false;//计时控制因子
            Fix_T1 = getTickTime();
            // Serial.println("偏左");
        }


        //偏左未修正

        if (SensorFlagR)
        {
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
            SensorFlagL = false;
            ctrlFlag = true;
            // Fix_T1 = Fix_T2 = 0;

            //减速恢复
            setSpdA2(targetSpd);
            setSpdB1(targetSpd);
            //加速恢复
            setSpdA1(targetSpd);
            setSpdB2(targetSpd);

        }
        break;

    case turnRight:
        //偏右
        if (!Sensor_R_L && Sensor_R_R && ctrlFlag)
        {

            SensorFlagL = true;//标记偏向
            ctrlFlag = false;//计时控制因子
            Fix_T1 = getTickTime();
        }

        //偏左
        if (!Sensor_R_R && Sensor_R_L && ctrlFlag) 
        {
            SensorFlagR = true;//标记偏向哪一侧
            ctrlFlag = false;//计时控制因子
            Fix_T1 = getTickTime();
        }


        //偏左未修正

        if (SensorFlagR)
        {
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
            SensorFlagR = false;
            ctrlFlag = true;
            Fix_T1 = Fix_T2 = 0;
            //减速修正
            setSpdA1(targetSpd);
            setSpdB2(targetSpd);
            //加速修正
            setSpdA2(targetSpd);
            setSpdB1(targetSpd);

        }
        
        
        

        //偏右未纠正
        if (SensorFlagL)
        {   //注意类型 
            Fix_T2 = FixFctr_LR*(getTickTime() - Fix_T1);
            //减速修正
            setSpdA2(targetSpd - Fix_T2);
            setSpdB1(targetSpd + Fix_T2);
            //加速修正
            setSpdA1(targetSpd - Fix_T2);
            setSpdB2(targetSpd + Fix_T2);

        }
        //偏右已纠正
        if (SensorFlagL && Sensor_R_R && Sensor_R_L && Sensor_R_M)
        {
            SensorFlagL = false;
            ctrlFlag = true;
            Fix_T1 = Fix_T2 = 0;

            //减速恢复
            setSpdA2(targetSpd);
            setSpdB1(targetSpd);
            //加速恢复
            setSpdA1(targetSpd);
            setSpdB2(targetSpd);
        }
        break;
    
    default:
        break;
    }



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
    spdB2Count++;

}



void count(){


    detachInterrupt(2);
    detachInterrupt(3);
    detachInterrupt(4);
    detachInterrupt(5);

 //    setSpdA1(spdController(spdA1Count,1) + 50);
    // Serial.println(spdA1Count);
//    Serial.print(",");
//    Serial.println(spdA1);


    // Serial.print("A1:   ");
    // Serial.print(spdA1Count);
    // Serial.print("A2:   ");
    // Serial.print(spdA2Count);
    // Serial.print("B1:   ");
    // Serial.print(spdB1Count);
    // Serial.print("B2:   ");
    // Serial.print(spdB2Count);

    spdA1Count = spdA2Count = spdB1Count = spdB2Count = 0;

  attachInterrupt(5, isr0, CHANGE);
  attachInterrupt(4, isr1, CHANGE);
  attachInterrupt(3, isr2, CHANGE);
  attachInterrupt(2, isr3, CHANGE);




}

long int getTickTime(){
    return millis();
}
