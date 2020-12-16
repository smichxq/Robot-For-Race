#include <LobotServoController.h>
#include <MsTimer2.h>
//方向
#define goStraight 0
#define goBack 1
#define turnLeft 2
#define turnRight 3

//传感器外部中断接口
#define Isrpin0 2
#define Isrpin1 3
#define Isrpin2 21

//电机接口
#define MotorPin1 8
#define Pin1Ain1 1
#define Pin1Ain2 1
#define MotorPin2 9
#define Pin2Ain1 1
#define Pin2Ain2 1
#define MotorPin3 10
#define Pin3Ain1 1
#define Pin3Ain2 1
#define MotorPin4 11
#define Pin4Ain1 1
#define Pin4Ain2 1
/*
* 电机旋转方向
* S     P     N
* 反    正    停
*/
#define S 0
#define P 1
#define N 2

/*=====================声明一个全局的任务序列missions[N]=================*/

missions currentMissions[14];

/*=====================声明一个全局的任务序列missions[N]=================*/


//任务结构体
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
  struct missions* next;
  
}missions;


/*
 * 外部中断功能引脚 2 3 21 20 19 18
 *               0 1  2  3 4  5
 */
 

long tm = 0;//时间变量,用来计时

boolean flag = true;//标记位,用来开启与关闭MsTimer2中断服务


LobotServoController myse(Serial3);//舵机控制

void setup() {
  
  pinMode(13,OUTPUT);
  Serial3.begin(9600);
  //while(!Serial3);
  digitalWrite(13,HIGH);
  //angleTest();


  //传感器初始化
  sensorInit();


}  
void loop()
{
  
  
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
* 初始化支持外中断引脚位输入模式
* 设定外部中断服务
* 开启计数中断
*/
void sensorInit(){
  Serial.begin(9600);
  pinMode(Isrpin0, INPUT);
  pinMode(Isrpin1, INPUT);
  //中间引脚
  pinMode(Isrpin2, INPUT);

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!一旦進入中断程序 就會自動禁止中斷服务!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
  //当int.N电平时,升高时触发中断函数blink,
  attachInterrupt(0, isr0, FALLING);
  attachInterrupt(1, isr1, FALLING);
  //设置中断服务,每100ms运行一次,使用time2的tick
  MsTimer2::set(10,count);
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!一旦進入中断程序 就會自動禁止中斷服务!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/

}
/*
* 传感器计数服务初始化
*/
void sensorTimerInit(){
  
}

/*
* 电机正反转控制
*  P   N   S
* 正  反  停
*/
void motorPNS(short mode)
{
  switch (mode)
  {
  case 0:
    //printf("stop");
    
    break;
  case 1:
    //printf("go Straight");
    break;
  case 2:
    //printf("go back");
    break;
  
  default:
    break;
  }

}

/*
* 电机命令函数 
* A1 A2 
* B1 B2
* 
*/

void motorA1(short spd){
  if (spd<0)
  {
    analogWrite(0);
  }
  else if(spd<255)
  {
    analogWrite(spd);
  }
  else
  {
    analogWrite(255);
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
void directions(short dirct,short spd){

    switch (dirct)
    {
    case goStraight:
        //printf("goStraight\n");
        
        break;
    case goBack:
        //printf("goBack\n");
        break;
    case turnLeft:
        //printf("turnLeft\n");
        break;
    case turnRight:
        //printf("turnRight\n");
        break;
    }
  

}


/*
* 功能: 增加一个路径序列项
* 参数: 任务结构体mission的每一项标记位
*/
mission* addMission(int a,int b,int c){
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
mission* createMissionList(short num,mission* missArry){
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
* 功能: 遍历任务序列mission
* 
*
*/
void traverseMission(){
  mission* p = NULL;
  p = currentMissions[0].head;
  while (p)
  {
    printf("%d  %d  %d \n",p->A,p->B,p->C);
    p = p->next;

  }


}

/*
* 功能: 初始化任务序列
* 将每一个任务的路径序列填入该函数
* 参数: 无,使用全局变量任务序列
*/
void missionsInit(){
  //路径序列
  mission missionAry[12];

  mission* p = NULL;

  for (int i = 0; i < 12; i++)
  {
    missionAry[i].A = i;
    missionAry[i].B = i;
    missionAry[i].C = i;
  }

  //任务序列


  //扫码
  currentMissions[0].head = createMissionList(12,missionAry);


  //原料区1
  currentMissions[1].head = createMissionList(12,missionAry);


  //粗加工区1
  currentMissions[2].head = createMissionList(12,missionAry);
  //粗加工区2
  currentMissions[3].head = createMissionList(12,missionAry);


  //半成品区1
  currentMissions[4].head = createMissionList(12,missionAry);
  //半成品区2
  currentMissions[5].head = createMissionList(12,missionAry);


  //原料区1
  currentMissions[6].head = createMissionList(12,missionAry);
  //原料区2
  currentMissions[7].head = createMissionList(12,missionAry);



  //粗加工区1
  currentMissions[8].head = createMissionList(12,missionAry);
  //粗加工区2
  currentMissions[0].head = createMissionList(12,missionAry);



  //半成品区1
  currentMissions[9].head = createMissionList(12,missionAry);
  //
  currentMissions[10].head = createMissionList(12,missionAry);
  currentMissions[11].head = createMissionList(12,missionAry);
  currentMissions[12].head = createMissionList(12,missionAry);
  currentMissions[13].head = createMissionList(12,missionAry);

  
}


/*
* 路径规划
* 规划当前方向
* 
*/
void pathPlan(){
  

}

/*
* 偏移检测
* 根据时间来确定偏移度
* 对当前状态进行纠正
* 使用定时中断
*/

void stateFix(){
  

}

/*
* 舵机角度标定
* 分别在500 1500 2500三个位置
*/

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

/*
 * MsTimer(N)
 * 定时中断函数
 * 如果影响,
 * 用来计数
 */
void count(){
  Serial.println(tm++);
  //tm++;
}


/*
 * 外部中断函数isr0~2
 * 只运行一次
 * 用来触发定时中断
 */
void isr1(){
  MsTimer2::stop();
}


void isr0()
{
  MsTimer2::start();

}

