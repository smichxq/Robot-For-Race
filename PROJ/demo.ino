/*
* 路径 1743
*/

#include <LCDWIKI_GUI.h> //Core graphics library
#include <LCDWIKI_SPI.h> //Hardware-specific library

//paramters define
#define MODEL ILI9225
#define CS   A14
#define RST  A13
#define CD   A12
#define SDA  A11

#define SCK  A6
#define LED  A5   //if you don't need to control the LED pin,you should set it to -1 and set it to 3.3V

//the definiens of software spi mode as follow:
//if the IC model is known or the modules is unreadable,you can use this constructed function
LCDWIKI_SPI mylcd(MODEL,CS,CD,-1,SDA,RST,SCK,LED); //model,cs,dc,sdo,sda,reset,sck,led

#define  BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF



short Left_FixA1 = 23;
short Left_FixA2 = 0;
short Left_FixB1 = 13;
short Left_FixB2 = 30;




short Right_FixA1 = 16;
short Right_FixA2 = 0;
short Right_FixB1 = 16.5;
short Right_FixB2 = 27.5;


short Go_FixA1 = 0;
short Go_FixA2 = 4;
short Go_FixB1 = 0;
short Go_FixB2 = 4;



short Back_FixA1 = 0;
short Back_FixA2 = 4.2;
short Back_FixB1 = 0;
short Back_FixB2 = 4.2;


// short Left_FixA1 = 0;
// short Left_FixA2 = 0;
// short Left_FixB1 = 0;
// short Left_FixB2 = 0;




// short Right_FixA1 = 0;
// short Right_FixA2 = 0;
// short Right_FixB1 = 0;
// short Right_FixB2 = 0;


// short Go_FixA1 = 0;
// short Go_FixA2 = 0;
// short Go_FixB1 = 0;
// short Go_FixB2 = 0;



// short Back_FixA1 = 0;
// short Back_FixA2 = 0;
// short Back_FixB1 = 0;
// short Back_FixB2 = 0;







#include <LobotServoController.h>
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
#define tickThreshold 5

//速度
int targetSpd = 80;

//取样周期
#define smpT 5

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
// #define CodeA1 18
#define MotorPin1 4
#define Motor1Ain2 A1
#define Motor1Ain1 A2

// #define CodeA2 19
#define MotorPin2 5
#define Motor2Ain1 A3
#define Motor2Ain2 A4

// #define CodeB1 20
#define MotorPin3 6
#define Motor3Ain1 A8
#define Motor3Ain2 A7

// #define CodeB2 21
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
  bool M;
  bool M_1;
  bool M_2;
  bool M_3;
  struct mission* next;
}mission;

//任务序列
typedef struct missions
{
  //每一个任务具体任务指针,无头节点
  mission* head;
  //任务序列指针域
  struct missions* next;

  //多路径区分 默认为true
  bool flag;
  
}missions;




/*===================================================UART===================================================*/
// byte REQ_CODE[] = [65,66];//服务请求码

byte UART_DATABUF[50];//数据缓冲区

byte UART_DECODE[8] = {0x61,0x62,0x63,0x64,0x65,0x66,0x67,0x68};//请求服务确认码

int UART_TARGET[6] = {0,0,0,0,0,0};//任务码
char* CODE1 = NULL;
char* CODE2 = NULL;

int UART_DELTA[3] = {0,0,0};//当前误差

int UART_DELTA_VALUE[3];

int UART_PLAN[3] = {0,0,0};//规划


//服务标记位
bool UART_SERVICE_A_FLAG = true;
bool UART_SERVICE_B_FLAG = true;
bool UART_SERVICE_C_FLAG = true;
bool UART_SERVICE_D_FLAG = true;

/*===================================================UART===================================================*/


//蓝牙缓冲区
byte BTData[50];



/*=====================声明一个全局的任务序列missions[N]=================*/

missions currentMissions[14];

//任务序列指针
missions* mps  = NULL;

//任务序列索引
short index_mission = 0;

//任务指针
mission* mp = NULL;

/*=====================声明一个全局的任务序列missions[N]=================*/

/*=====================测试路径==================*/

// mission* testMission;
// mission* p  = NULL;//p指针在其实例化后无法动态绑定
// bool onceFlag = true;

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
float FixFctr_GB = 0.40;
float FixFctr_LR = 0.490;
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

bool ctrl_flag = true;

bool onef = true;

short grab_plan_count = 0;



LobotServoController myse(Serial3);//舵机控制

void setup() {
    delay(20);
    //显示屏初始化
    LEDInit();
    LEDCtrl("Spark",0,20,5,"krapS",0,90,5);
    sensorInit();

    // Serial.begin(9600);
    // Serial.println("Datatrans Ready!");

    mps = missionsInit();
    mp = mps->head;

    // mp = T_Obj();
    
    // Serial.println("Datatrans Ready!");

    // // while(mps){
    //     while(mp){

    //         if (!mp->next){

    //             break;
    //         }
    //         Serial.println(mp->A,BIN);
    //         Serial.println(mp->B,BIN);
    //         Serial.println(mp->C,BIN);
    //         Serial.println(mp->D,BIN);
    //         Serial.println(mp->E,BIN);
    //         Serial.println(mp->M,BIN);
    //         Serial.println(mp->M_1,BIN);
    //         Serial.println(mp->M_2,BIN);
    //         Serial.println(mp->M_3,BIN);
    //         Serial.println("===========================");
    //         mp = mp->next;
    //         delay(1111);
    //     }

    //     dynamicGrabPlan();
    //     while(mp){

    //         Serial.println(mp->A,BIN);
    //         Serial.println(mp->B,BIN);
    //         Serial.println(mp->C,BIN);
    //         Serial.println(mp->D,BIN);
    //         Serial.println(mp->E,BIN);
    //         Serial.println(mp->M,BIN);
    //         Serial.println(mp->M_1,BIN);
    //         Serial.println(mp->M_2,BIN);
    //         Serial.println(mp->M_3,BIN);
    //         Serial.println("===========================");
    //         mp = mp->next;

    //     }
    //     Serial.println("Dnoe");
        
    //     Serial.println("<><><><><><><><><><><><><><><><><<><<><<>");
    //     mps = mps->next;
    //     mp = mps->head;
    // }

    

    

    // Serial.println("Datatrans Done!");


    // mp = scanTest();
    
    //BT
    Serial.begin(115200);
    //openmv通信
    Serial2.begin(19200);


    //机械臂通信
    Serial3.begin(9600);
    //机械臂初始化
    //openmv控制
    myse.runActionGroup(0,1);
    


    // delay(2000000);

//  IntServiceInit();
  //前后 A2 +3 B2 +3
  //左 A1 +7.2 B1 +4 B2 +7.2
  //右 A1 +7.3 B1 +4.9 B2 +10.7
  motorInit();
  setSpdA1(targetSpd);
  setSpdA2(targetSpd);
  setSpdB1(targetSpd);
  setSpdB2(targetSpd);



//   currentStates = goStraight;
//   directions();
//   delay(1000);

//   currentStates = stp;
//   directions();

//   currentStates = goBack;
//   directions();
//   delay(1000);

//   currentStates = stp;
//   directions();
//   delay(1000);

//   currentStates = Left;
//   directions();
//   delay(1000);

//   currentStates = stp;
//   directions();
//   delay(1000);

//   currentStates = Right;
//   directions();
//   delay(1000);

//   currentStates = stp;
//   directions();
//   delay(1000);



//   BTCtrl();
  Serial.println("-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=");
  delay(500);

}  

void loop()
{    

  /*
  先使用路径规划pathPlan
  再命令电机移动directions
  */
    BTCtrl();
    
    // delay(1000);
    // pathPlan();
//   Serial.println(currentStates);
// currentStates = goStraight;

    // if (mp->C){
    //     Serial.println("STP!!!!!");
    // }

  
    if (!ctrl_flag){

    pathPlan();
    directions();

    stateFix();
    
        
    }



  

}

//LED服务初始化
void LEDInit(){
    mylcd.Init_LCD();
    mylcd.Fill_Screen(BLACK);
    mylcd.Set_Text_Mode(0);
}
/*
* LED显示服务
* 参数为两个任务码(字符串)
* 不可重复调用
*/
void LEDCtrl(char* MISSION_CODE_1, short x_1 ,short y_1, short size_1 ,char* MISSION_CODE_2, short x_2 ,short y_2, short size_2) 
{
    
  
  mylcd.Set_Text_Mode(0);
  //display 1 times string
  mylcd.Fill_Screen(0x0000);


  mylcd.Set_Text_colour(RED);
  mylcd.Set_Text_Back_colour(BLACK);
  mylcd.Set_Text_Size(size_1);
  mylcd.Print_String(MISSION_CODE_1, x_1, y_1);
 // mylcd.Print_Number_Float(01234.56789, 2, 0, 8, '.', 0, ' ');  
 // mylcd.Print_Number_Int(0xDEADBEF, 0, 16, 0, ' ',16);
  
  //display 2 times string
//   mylcd.Set_Text_colour(GREEN);
//   mylcd.Set_Text_Size(8);//大小
//   mylcd.Print_String("+", 50, 80);//50左右距离，80上下距离
//   mylcd.Print_Number_Float(01234.56789, 2, 0, 44, '.', 0, ' ');  
//   mylcd.Print_Number_Int(0xDEADBEF, 0, 60, 0, ' ',16);
 
 //display 3 times string
  mylcd.Set_Text_colour(BLUE);
  mylcd.Set_Text_Size(size_2);
  mylcd.Print_String(MISSION_CODE_2, x_2, y_2);
//   mylcd.Print_Number_Float(01234.56789, 2, 0, 104, '.', 0, ' ');  
//   mylcd.Print_Number_Int(0xDEADBEF, 0, 128, 0, ' ',16);
  
//   delay(3000);
//   while(1);
    // Serial.println("LED1===================");
    // delay(20);
    // Serial.println(*MISSION_CODE_1);
    // delay(20);
    // MISSION_CODE_1++;
    // Serial.println(*MISSION_CODE_1);
    // delay(20);
    // MISSION_CODE_1++;
    // Serial.println(*MISSION_CODE_1);
    // delay(20);
    // MISSION_CODE_1++;
    // Serial.println(*MISSION_CODE_1);
    // delay(20);
    // MISSION_CODE_1++;
    // Serial.println(*MISSION_CODE_1);
    // delay(20);
    // Serial.println("LED1===================");
    // delay(20);

    // Serial.println("LED2===================");
    // delay(20);
    // Serial.println(*MISSION_CODE_2);
    // delay(20);
    // MISSION_CODE_2++;
    // Serial.println(*MISSION_CODE_2);
    // delay(20);
    // MISSION_CODE_2++;
    // Serial.println(*MISSION_CODE_2);
    // delay(20);
    // MISSION_CODE_2++;
    // Serial.println(*MISSION_CODE_2);
    // delay(20);
    // MISSION_CODE_2++;
    // Serial.println(*MISSION_CODE_2);
    // delay(20);
    // Serial.println("LED2===================");
    // delay(20);
  
}




/*
* 处理误差转化为移动数据(垂直面移动)
* 参数1：y(图像误差)+x(待测固定值)
* 参数2：实际测试后为定值
* 输出：UART_DELTA_VALUE[3] 舵机移动参数
*/
//x=216 h = 62
void UART_DELTA_PROC(double x,double h){

    double L1 = 20.0;
    double L2 = 20.0;
    double L3;

    double Ang1,Ang2,Ang3;
    double Ang_a;
    double Anga;
    double Ang__1,Ang__2,Ang__3;
    // double A,B,C,D,E;

    if (L1 + L2 < pow(pow(x,2) + pow(h,2),0.5)){
        //无法到达
        // printf("无法到达");
        return;
    }

    if (!x){
        L3 = h;
    }
    
    else{
        L3 = pow(pow(x,2) + pow(h,2),0.5);
    }

    // A = pow(L1,2);
    // B = pow(L2,2);
    // C = pow(L3,2);
    // D = (2.0*L1*L2);
    // E = A + B - C / D;
    Ang2 = acos((pow(L1,2) + pow(L2,2) - pow(L3,2)) / (2.0*L1*L2));

    Ang2 = Ang2*180/PI;

    Ang_a = acos((pow(L1,2)  +pow(L3,2) - pow(L2,2)) / (2*L1*L3));

    Ang_a = Ang_a * 180 / PI;

    if (!x){
        Anga = 90;
    }

    else{
        Anga = atan(h/x) * 180 / PI;
    }

    if (!x){
        Ang1 = Ang_a + Anga;
    }

    else if (x > 0){
        Ang1 = Ang_a + Anga;
    }

    else if (x < 0){
        if (h > 0){
            Ang1 = Ang_a + 180 + Anga;
        }
        else if (h < 0){
            //Error place
            // printf("Error place");
            return;
        }
    }

    Ang1 = Ang1;

    Ang3 = 360-Ang1-Ang2;

    Ang__1 = 100.0 / 9.0 * (Ang1 + 45.0);

    Ang__2 = 200.0 / 27.0 * (382.5 - Ang2);

    Ang__3 = 100.0 / 9.0 * (315.0 - Ang3);

    if (Ang__1 > 2000){
            if (Anga < 0){
        Ang1 = 180 + Anga - Ang_a;
    }
        else{
        Ang1 = Anga - Ang_a;
    }
        Ang1 = Ang1;

    Ang3 = Ang3 - 2.0 * (180 - Ang_a - Ang2);

    Ang2 = 360-Ang2;

    Ang__1 = 100.0 / 9.0 * (Ang1 + 45.0);
    
    Ang__2 = 200.0 / 27.0 * (382.5 - Ang2);

    Ang__3 = 100.0 / 9.0 * (315.0 - Ang3);

    }













    if (Ang__1 < 500 || Ang__2 < 500 || Ang__3 < 500 || Ang__1 > 2500 || Ang__2 > 2500|| Ang__3 > 2500){
        //无法到达
        // printf("无法到达");
        return;
    } 

    else{

    }

    UART_DELTA_VALUE[0] = Ang__1;
    UART_DELTA_VALUE[1] = Ang__2;
    UART_DELTA_VALUE[2] = Ang__3;


    // int* p = NULL;

    // p = (int*)malloc(sizeof(3 * int));
    // *p = Ang__1;
    // p++;
    // *p = Ang__2;
    // p++;
    // *p = Ang__3;



}

/*
* 输入：水平误差x(图像误差)
* 误差进行函数拟合
* 输出：舵机水平移动参数
*/
int UART_DELTA_PROC_V(int x){

    if (!x){
        //误差为零
    }

    else{
        //误差在10以内
        if (abs(x) <= 10){
            return;

        }
        //误差在10~20
        else if (abs(x) <= 20){
            return;
        }
        //误差在20~30
        else if (abs(x) <= 30){
            return;

        }
        //误差在30~40
        else if (abs(x) <= 40){
            return;


        }
        //误差在40~50
        else if (abs(x) <= 50){
            return;

        }
        //误差在50~60
        else if (abs(x) <= 60){
            return;
        }
    }
    
}

//测试路径2
// mission* test2(){
//     mission demo[];
// }



//测试路径
mission* test(){
  mission demo[39];
    //左直线模式
  demo[0].A = true;
  demo[0].B = false;
  demo[0].C = false;
  demo[0].D = true;
  demo[0].E = false;
  demo[0].M = false;
  demo[0].M_1 = false;
  demo[0].M_2 = false;
  demo[0].M_3 = false;
    //前
  demo[1].A = false;
  demo[1].B = false;
  demo[1].C = false;
  demo[1].D = false;
  demo[1].E = false;
  demo[1].M = false;
  demo[1].M_1 = false;
  demo[1].M_2 = false;
  demo[1].M_3 = false;  

      //前
  demo[2].A = false;
  demo[2].B = false;
  demo[2].C = false;
  demo[2].D = false;
  demo[2].E = false;
  demo[2].M = false;
  demo[2].M_1 = false;
  demo[2].M_2 = false;
  demo[2].M_3 = false;  

      //前
  demo[3].A = false;
  demo[3].B = false;
  demo[3].C = false;
  demo[3].D = false;
  demo[3].E = false;
  demo[3].M = false;
  demo[3].M_1 = false;
  demo[3].M_2 = false;
  demo[3].M_3 = false;
      //前
  demo[4].A = false;
  demo[4].B = false;
  demo[4].C = false;
  demo[4].D = false;
  demo[4].E = false;
  demo[4].M = false;
  demo[4].M_1 = false;
  demo[4].M_2 = false;
  demo[4].M_3 = false;  
      //前
  demo[5].A = false;
  demo[5].B = false;
  demo[5].C = false;
  demo[5].D = false;
  demo[5].E = false;
  demo[5].M = false;
  demo[5].M_1 = false;
  demo[5].M_2 = false;
  demo[5].M_3 = false;
      //前
  demo[6].A = false;
  demo[6].B = false;
  demo[6].C = false;
  demo[6].D = false;
  demo[6].E = false;
  demo[6].M = false;
  demo[6].M_1 = false;
  demo[6].M_2 = false;
  demo[6].M_3 = false; 
      //左
  demo[7].A = true;
  demo[7].B = false;
  demo[7].C = false;
  demo[7].D = false;
  demo[7].E = false;
  demo[7].M = false;
  demo[7].M_1 = false;
  demo[7].M_2 = false;
  demo[7].M_3 = false;  

        //左
  demo[8].A = true;
  demo[8].B = false;
  demo[8].C = false;
  demo[8].D = false;
  demo[8].E = false;
  demo[8].M = false;
  demo[8].M_1 = false;
  demo[8].M_2 = false;
  demo[8].M_3 = false;  

        //左
  demo[9].A = true;
  demo[9].B = false;
  demo[9].C = false;
  demo[9].D = false;
  demo[9].E = false;
  demo[9].M = false;
  demo[9].M_1 = false;
  demo[9].M_2 = false;
  demo[9].M_3 = false;  

        //左
  demo[10].A = true;
  demo[10].B = false;
  demo[10].C = false;
  demo[10].D = false;
  demo[10].E = false;
  demo[10].M = false;
  demo[10].M_1 = false;
  demo[10].M_2 = false;
  demo[10].M_3 = false;  

        //左
  demo[11].A = true;
  demo[11].B = false;
  demo[11].C = false;
  demo[11].D = false;
  demo[11].E = false;
  demo[11].M = false;
  demo[11].M_1 = false;
  demo[11].M_2 = false;
  demo[11].M_3 = false;  

        //后
  demo[12].A = false;
  demo[12].B = true;
  demo[12].C = false;
  demo[12].D = false;
  demo[12].E = false;
  demo[12].M = false;
  demo[12].M_1 = false;
  demo[12].M_2 = false;
  demo[12].M_3 = false; 

        //后
  demo[13].A = false;
  demo[13].B = true;
  demo[13].C = false;
  demo[13].D = false;
  demo[13].E = false;
  demo[13].M = false;
  demo[13].M_1 = false;
  demo[13].M_2 = false;
  demo[13].M_3 = false; 

        //后
  demo[14].A = false;
  demo[14].B = true;
  demo[14].C = false;
  demo[14].D = false;
  demo[14].E = false;
  demo[14].M = false;
  demo[14].M_1 = false;
  demo[14].M_2 = false;
  demo[14].M_3 = false; 

     //右
  demo[15].A = true;
  demo[15].B = true;
  demo[15].C = false;
  demo[15].D = false;
  demo[15].E = false;
  demo[15].M = false;
  demo[15].M_1 = false;
  demo[15].M_2 = false;
  demo[15].M_3 = false; 

     //前
  demo[16].A = false;
  demo[16].B = false;
  demo[16].C = false;
  demo[16].D = false;
  demo[16].E = false;
  demo[16].M = false;
  demo[16].M_1 = false;
  demo[16].M_2 = false;
  demo[16].M_3 = false;
     //右
  demo[17].A = true;
  demo[17].B = true;
  demo[17].C = false;
  demo[17].D = false;
  demo[17].E = false;
  demo[17].M = false;
  demo[17].M_1 = false;
  demo[17].M_2 = false;
  demo[17].M_3 = false; 
    //后
  demo[18].A = false;
  demo[18].B = true;
  demo[18].C = false;
  demo[18].D = false;
  demo[18].E = false;
  demo[18].M = false;
  demo[18].M_1 = false;
  demo[18].M_2 = false;
  demo[18].M_3 = false; 

     //右
  demo[19].A = true;
  demo[19].B = true;
  demo[19].C = false;
  demo[19].D = false;
  demo[19].E = false;
  demo[19].M = false;
  demo[19].M_1 = false;
  demo[19].M_2 = false;
  demo[19].M_3 = false; 

     //前
  demo[20].A = false;
  demo[20].B = false;
  demo[20].C = false;
  demo[20].D = false;
  demo[20].E = false;
  demo[20].M = false;
  demo[20].M_1 = false;
  demo[20].M_2 = false;
  demo[20].M_3 = false; 

     //右
  demo[21].A = true;
  demo[21].B = true;
  demo[21].C = false;
  demo[21].D = false;
  demo[21].E = false;
  demo[21].M = false;
  demo[21].M_1 = false;
  demo[21].M_2 = false;
  demo[21].M_3 = false; 

    //后
  demo[22].A = false;
  demo[22].B = true;
  demo[22].C = false;
  demo[22].D = false;
  demo[22].E = false;
  demo[22].M = false;
  demo[22].M_1 = false;
  demo[22].M_2 = false;
  demo[22].M_3 = false; 

     //右
  demo[23].A = true;
  demo[23].B = true;
  demo[23].C = false;
  demo[23].D = false;
  demo[23].E = false;
  demo[23].M = false;
  demo[23].M_1 = false;
  demo[23].M_2 = false;
  demo[23].M_3 = false; 

       //前
  demo[24].A = false;
  demo[24].B = false;
  demo[24].C = false;
  demo[24].D = false;
  demo[24].E = false;
  demo[24].M = false;
  demo[24].M_1 = false;
  demo[24].M_2 = false;
  demo[24].M_3 = false; 

       //前
  demo[25].A = false;
  demo[25].B = false;
  demo[25].C = false;
  demo[25].D = false;
  demo[25].E = false;
  demo[25].M = false;
  demo[25].M_1 = false;
  demo[25].M_2 = false;
  demo[25].M_3 = false; 

       //前
  demo[26].A = false;
  demo[26].B = false;
  demo[26].C = false;
  demo[26].D = false;
  demo[26].E = false;
  demo[26].M = false;
  demo[26].M_1 = false;
  demo[26].M_2 = false;
  demo[26].M_3 = false; 



      //左
  demo[27].A = true;
  demo[27].B = false;
  demo[27].C = false;
  demo[27].D = false;
  demo[27].E = false;
  demo[27].M = false;
  demo[27].M_1 = false;
  demo[27].M_2 = false;
  demo[27].M_3 = false;  

        //左
  demo[28].A = true;
  demo[28].B = false;
  demo[28].C = false;
  demo[28].D = false;
  demo[28].E = false;
  demo[28].M = false;
  demo[28].M_1 = false;
  demo[28].M_2 = false;
  demo[28].M_3 = false;  
  
        //左
  demo[29].A = true;
  demo[29].B = false;
  demo[29].C = false;
  demo[29].D = false;
  demo[29].E = false;
  demo[29].M = false;
  demo[29].M_1 = false;
  demo[29].M_2 = false;
  demo[29].M_3 = false;  

        //左
  demo[30].A = true;
  demo[30].B = false;
  demo[30].C = false;
  demo[30].D = false;
  demo[30].E = false;
  demo[30].M = false;
  demo[30].M_1 = false;
  demo[30].M_2 = false;
  demo[30].M_3 = false;  

        //左
  demo[31].A = true;
  demo[31].B = false;
  demo[31].C = false;
  demo[31].D = false;
  demo[31].E = false;
  demo[31].M = false;
  demo[31].M_1 = false;
  demo[31].M_2 = false;
  demo[31].M_3 = false;  

    //后
  demo[32].A = false;
  demo[32].B = true;
  demo[32].C = false;
  demo[32].D = false;
  demo[32].E = false;
  demo[32].M = false;
  demo[32].M_1 = false;
  demo[32].M_2 = false;
  demo[32].M_3 = false; 


    //后
  demo[33].A = false;
  demo[33].B = true;
  demo[33].C = false;
  demo[33].D = false;
  demo[33].E = false;
  demo[33].M = false;
  demo[33].M_1 = false;
  demo[33].M_2 = false;
  demo[33].M_3 = false; 

    //后
  demo[34].A = false;
  demo[34].B = true;
  demo[34].C = false;
  demo[34].D = false;
  demo[34].E = false;
  demo[34].M = false;
  demo[34].M_1 = false;
  demo[34].M_2 = false;
  demo[34].M_3 = false; 

      //后
  demo[35].A = false;
  demo[35].B = true;
  demo[35].C = false;
  demo[35].D = false;
  demo[35].E = false;
  demo[35].M = false;
  demo[35].M_1 = false;
  demo[35].M_2 = false;
  demo[35].M_3 = false; 

      //后
  demo[36].A = false;
  demo[36].B = true;
  demo[36].C = false;
  demo[36].D = false;
  demo[36].E = false;
  demo[36].M = false;
  demo[36].M_1 = false;
  demo[36].M_2 = false;
  demo[36].M_3 = false; 

      //后
  demo[37].A = false;
  demo[37].B = true;
  demo[37].C = false;
  demo[37].D = false;
  demo[37].E = false;
  demo[37].M = false;
  demo[37].M_1 = false;
  demo[37].M_2 = false;
  demo[37].M_3 = false; 

      //stp
  demo[38].A = false;
  demo[38].B = false;
  demo[38].C = true;
  demo[38].D = false;
  demo[38].E = false;
  demo[38].M = false;
  demo[38].M_1 = false;
  demo[38].M_2 = false;
  demo[38].M_3 = false; 








  return createMissionList(39,demo);
  
}
//蓝牙控制
void BTCtrl(){
    // Serial.println("standby!!!!!!!!!!!!!!!!!!!!!!!!!!");
    delay(2);
    int i = 0;
    if (Serial.available()<0){
        return;
    }
        while (Serial.available()>0)
        {
            BTData[i] = Serial.read();
            i++;
        }
        
        for(int j = 0 ; j < i+1 ; j++){
            //ol(online)
            // Serial.println("解析!!!!!!!!!!!!!");
            if (BTData[j]==111 && BTData[j+1] ==108){
                Serial.println("online!!!!!!!!!!!!!!!!!");
                setSpdA1(0);
                setSpdA2(0);
                setSpdB1(0);
                setSpdB2(0);
                //随时接收指令
                ctrl_flag = true;
                while (true)
                {
                    delay(200);
                    Serial.println("ol Stand BY~~~~~~~~~~~~~~~~~");
                    i = 0;
                    while (Serial.available()>0)
                        {
                            BTData[i] = Serial.read();
                            i++;
                        }
                        delay(100);
                        // i=0;
                        if (i > 0){
                            for (j = 0;j<i+1;j++){
                                //00\n 前
                                if (BTData[j] == 48 && BTData[j+1] == 48){
                                    // Serial.println("go!!!!!!!!!!!!!!!!!!!!!!!!");
                                    setSpdA1(50);
                                    setSpdA2(50);
                                    setSpdB1(50);
                                    setSpdB2(50);
                                    currentStates = goStraight;
                                    directions();
 
                                    Serial.flush();
                                    break;
                                    
                                }//01\n 后
                                if (BTData[j] == 48 && BTData[j+1] == 49){
                                    setSpdA1(50);
                                    setSpdA2(50);
                                    setSpdB1(50);
                                    setSpdB2(50);
                                    currentStates = goBack;
                                    directions();

                                    Serial.flush();
                                    break;
                                    
                                }
                                //10\n 左
                                if (BTData[j] == 49 && BTData[j+1] == 48){
                                    setSpdA1(100);
                                    setSpdA2(100);
                                    setSpdB1(100);
                                    setSpdB2(100);
                                    currentStates = Left;
                                    directions();

                                    Serial.flush();
                                    break;
                                    
                                }
                                //11\n 右
                                if (BTData[j] == 49 && BTData[j+1] == 49){
                                    setSpdA1(100);
                                    setSpdA2(100);
                                    setSpdB1(100);
                                    setSpdB2(100);
                                    currentStates = Right;
                                    directions();
                                    Serial.flush();
                                    break;
                                }
                                //21\n 停
                                if (BTData[j] == 50 && BTData[j+1] == 49){
                                    currentStates = stp;
                                    directions();
                                    delay(100);
                                    Serial.flush();
                                    break;
                                }

                            }
                        }


                }
                
            }

            //ot(outline)
            if (BTData[j]==111 && BTData[j+1]==116)
            {
                // Serial.println("outline!!!!!!!!!!!!!!!!!");
                ctrl_flag =false;
                return;
                
            }
            
        }

    
    

    
}

mission* scanTest(){
    mission demo[1];
    demo[0].A = false;
    demo[0].B = false;
    demo[0].C = true;
    demo[0].D = false;
    demo[0].E = false;
    demo[0].M = true;
    demo[0].M_1 = false;
    demo[0].M_2 = false;
    demo[0].M_3 = false;
    return demo;

}

//LED指示灯,参数为次数
void blbl(int j) {
    pinMode(LED_BUILTIN,OUTPUT);
    for (int i= 0; i<j ; i++){

        digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
        delay(200);                       // wait for a second
        digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
        delay(200);        
    }               // wait for a second
}



/*
* 任务码转类型
* int -> char
*/
void getCode(){
    char* tempCode1 = NULL;
    char* tempCode2 = NULL;

    

    tempCode1 = CODE1 = (char*)malloc(4*sizeof(char));
    Serial.println("local Saving");

    itoa(UART_TARGET[0],CODE1,10);
    Serial.println(*CODE1);
    
    CODE1++;
    
    itoa(UART_TARGET[1],CODE1,10);
    
    Serial.println(*CODE1);
    
    CODE1++;

    itoa(UART_TARGET[2],CODE1,10);
    Serial.println(*CODE1);

    CODE1++;
    *CODE1 = '\0';







    tempCode2 = CODE2 = (char*)malloc(4*sizeof(char));
    

    itoa(UART_TARGET[3],CODE2,10);
    Serial.println(*CODE2);
    CODE2++;
    itoa(UART_TARGET[4],CODE2,10);
    Serial.println(*CODE2);
    CODE2++;
    itoa(UART_TARGET[5],CODE2,10);
    Serial.println(*CODE2);
    CODE2++;
    *CODE2 = '\0';



    CODE1 = tempCode1;
    CODE2 = tempCode2;

    

    LEDCtrl(tempCode1,0,20,8,tempCode2,0,80,8);
    // Serial.println("local Saving"");
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
//   pinMode(CodeA1,INPUT);
//   pinMode(CodeA2,INPUT);
//   pinMode(CodeB1,INPUT);
//   pinMode(CodeB2,INPUT);

/*
* 外部中断口服务初始化
* int.2 int.3 int.4 int.5
*   21   20     19    18
*/
//   attachInterrupt(5, isr0, CHANGE);
//   attachInterrupt(4, isr1, CHANGE);
//   attachInterrupt(3, isr2, CHANGE);
//   attachInterrupt(2, isr3, CHANGE);
  

  
}

//中断服务初始化

void IntServiceInit(){
    MsTimer2::set(5,LEDCtrl);
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
* 电机命令函数 o't
* A1 A2 
* B1 B2
* 
*/

void motorA1(){
  if (spdA1<=0)
  {
    analogWrite(MotorPin1,0);
  }
  else if(spdA1<255)
  {
      if (ctrlFlag){

        //左移
        if ((currentStates == Left || currentStates == micro_line_L || currentStates == mid_line_L) && ctrlFlag){
            analogWrite(MotorPin1,spdA1 + Left_FixA1);
//             Serial.println(spdA1 + Left_FixA1);

        }
        //右移
        if ((currentStates == Right || currentStates == micro_line_R || currentStates == mid_line_R) && ctrlFlag){
            analogWrite(MotorPin1,spdA1 + Right_FixA1);

        }
        //前
        if((currentStates == goStraight || currentStates == micro_line_F || currentStates == mid_line_F) && ctrlFlag){
            analogWrite(MotorPin1,spdA1 + Go_FixA1);
        }
        //后
        if ((currentStates == goBack || currentStates == micro_line_B || currentStates == mid_line_B) && ctrlFlag){
            analogWrite(MotorPin1,spdA1 + Back_FixA1);
        }
        //stp
        if (currentStates == stp && ctrlFlag){
            analogWrite(MotorPin1,spdA1);
        }
      }

      else{
        analogWrite(MotorPin1,spdA1);
      }

    
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
      if (ctrlFlag){

        //左移A2
        if ((currentStates == Left || currentStates == micro_line_L || currentStates == mid_line_L) && ctrlFlag){
            analogWrite(MotorPin2,spdA2 + Left_FixA2);
//             Serial.println(spdA2 + Left_FixA2);

        }
        //右移A2
        if ((currentStates == Right || currentStates == micro_line_R || currentStates == mid_line_R) && ctrlFlag){
            analogWrite(MotorPin2,spdA2 + Right_FixA2);
        }

        if ((currentStates == goStraight || currentStates == micro_line_F || currentStates == mid_line_F) && ctrlFlag){
            analogWrite(MotorPin2,spdA2 + Go_FixA2);
        }

        if ((currentStates == goBack || currentStates == micro_line_B || currentStates == mid_line_B)  && ctrlFlag){
            analogWrite(MotorPin2,spdA2 + Back_FixA2);
        }

        if (currentStates == stp && ctrlFlag){
            analogWrite(MotorPin2,spdA2);
        }

      }

      else{
          analogWrite(MotorPin2,spdA2);
      }

    
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
      if (ctrlFlag){
        //左移B1
        if ((currentStates == Left || currentStates == micro_line_L || currentStates == mid_line_L) && ctrlFlag){
            analogWrite(MotorPin3,spdB1 + Left_FixB1);
//             Serial.println(spdB1 + Left_FixB1);

        }

        //右移B1
        if ((currentStates == Right || currentStates == micro_line_R || currentStates == mid_line_R) && ctrlFlag){
            analogWrite(MotorPin3,spdB1 + Right_FixB1);
        }
        if ((currentStates == goStraight || currentStates == micro_line_F || currentStates == mid_line_F) && ctrlFlag){
            analogWrite(MotorPin3,spdB1 + Go_FixB1);
        }

        if ((currentStates == goBack || currentStates == micro_line_B || currentStates == mid_line_B)  && ctrlFlag){
            analogWrite(MotorPin3,spdB1 + Back_FixB1);
        }

        if (currentStates == stp && ctrlFlag){
            analogWrite(MotorPin3,spdB1);
        }
      }
      else{
          analogWrite(MotorPin3,spdB1);
      }
    
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
      if (ctrlFlag){
        //左移B2
        if ((currentStates == Left || currentStates == micro_line_L || currentStates == mid_line_L) && ctrlFlag){
            analogWrite(MotorPin4,spdB2 + Left_FixB2);
//             Serial.println(spdB2 + Left_FixB2);

        }

        //右移B2
        if ((currentStates == Right || currentStates == micro_line_R || currentStates == mid_line_R) && ctrlFlag){
            analogWrite(MotorPin4,spdB2 + Right_FixB2);
        }
        //前进
        if ((currentStates == goStraight || currentStates == micro_line_F || currentStates == mid_line_F) && ctrlFlag){
            analogWrite(MotorPin4,spdB2 + Go_FixB2);
        }

        if ((currentStates == goBack || currentStates == micro_line_B || currentStates == mid_line_B)  && ctrlFlag){
            analogWrite(MotorPin4,spdB2 + Back_FixB2);
        }

        if (currentStates == stp && ctrlFlag){
            analogWrite(MotorPin4,spdB2);
        }

      }
      else{
          analogWrite(MotorPin4,spdB2);
      }
    
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
*
* 目前只有四个方向
* 读取当前状态-控制电机
* 
*/
void directions(){
    
    short dirct = currentStates;
    int stpCount = 50;



  


    switch (dirct)
    {
    case goStraight:


        //printf("goStraight\n");
        motorA1PNS(P);
        motorA2PNS(P);
        motorB1PNS(P);
        motorB2PNS(P);
        motorA1();

        
        motorA2();

        
        motorB1();

        
        motorB2();

        
        break;
      case micro_line_F:
        //printf("goStraight\n");
        while (true){


            
            Sensor_F_M = digitalRead(SensorPinF_2);


            motorA1PNS(P);
            motorB1PNS(P);
            motorA2PNS(P);
            motorB2PNS(P);

            motorA1();
            motorA2();
            motorB1();
            motorB2();
            //压线
            if (!Sensor_F_M){
                motorA1PNS(S);
                motorA2PNS(S);
                motorB1PNS(S);
                motorB2PNS(S);
                break;
            }

        }
        
        break;
     case mid_line_F:
        //printf("goStraight\n");
        while (true){
            Sensor_L_M = digitalRead(SensorPinF_2);
            Sensor_R_M = digitalRead(SensorPinR_2);

            motorA1PNS(P);
            motorB1PNS(P);
            motorA2PNS(P);
            motorB2PNS(P);

            motorA1();
            motorA2();
            motorB1();
            motorB2();
            //压线
            if (!Sensor_R_M && ! Sensor_L_M){
                motorA1PNS(S);
                motorA2PNS(S);
                motorB1PNS(S);
                motorB2PNS(S);
                

                break;
            }

        }
        
        break;

    case goBack:

        motorA1PNS(N);
        motorA2PNS(N);
        motorB1PNS(N);
        motorB2PNS(N);
        motorA1();

        
        motorA2();

        
        motorB1();

        
        motorB2();
        break;

    case micro_line_B:
        //printf("goStraight\n");
        while (true){
            // Serial.println("!!!!!!!!!!!!!!!1");
            Sensor_B_M = digitalRead(SensorPinB_2);
            


            motorA1PNS(N);
            motorA2PNS(N);
            motorB1PNS(N);
            motorB2PNS(N);
            motorA1();

            
            motorA2();

            
            motorB1();

            
            motorB2();
            //压线
            if (!Sensor_B_M){
                motorA1PNS(S);
                motorA2PNS(S);
                motorB1PNS(S);
                motorB2PNS(S);
                break;
            }

        }
        
        break;

    case mid_line_B:
        //printf("goStraight\n");
        while (true){
            // Serial.println("!!!!!!!!!!!!!!!1");


            Sensor_L_M = digitalRead(SensorPinL_2);
            Sensor_R_M = digitalRead(SensorPinR_2);
            motorA1PNS(N);
            motorA2PNS(N);
            motorB1PNS(N);
            motorB2PNS(N);
            motorA1();

            
            motorA2();

            
            motorB1();

            
            motorB2();
            //压线
            if (!Sensor_L_M && !Sensor_R_M){
                motorA1PNS(S);
                motorA2PNS(S);
                motorB1PNS(S);
                motorB2PNS(S);
                break;
            }

        }
        
        break;
    case Left:


        motorA1PNS(N);
        motorA2PNS(P);
        motorB1PNS(P);
        motorB2PNS(N);
        motorA1();

        
        motorA2();

        
        motorB1();

        
        motorB2();

        break;
     case micro_line_L:
        //printf("goStraight\n");
        while(true){
            Sensor_L_M = digitalRead(SensorPinL_2);

            motorA1PNS(N);
            motorA2PNS(P);
            motorB1PNS(P);
            motorB2PNS(N);

            motorA1();

            
            motorA2();

            
            motorB1();

            
            motorB2();

            if(!Sensor_L_M){
                motorA1PNS(S);
                motorA2PNS(S);
                motorB1PNS(S);
                motorB2PNS(S);
                break;
            }
        }
        
        break;

     case mid_line_L:
        //printf("goStraight\n");
        while(true){
            
        Sensor_F_M = digitalRead(SensorPinF_2);
        Sensor_B_M = digitalRead(SensorPinB_2);

            motorA1PNS(N);
            motorA2PNS(P);
            motorB1PNS(P);
            motorB2PNS(N);

            motorA1();

            
            motorA2();

            
            motorB1();

            
            motorB2();

            if(!Sensor_F_M && !Sensor_B_M){
                motorA1PNS(S);
                motorA2PNS(S);
                motorB1PNS(S);
                motorB2PNS(S);
                break;
            }
        }
        
        break;
        
    case Right:

        motorA1PNS(P);
        motorB1PNS(N);
        motorA2PNS(N);
        motorB2PNS(P);

        motorA1();

        
        motorA2();

        
        motorB1();

        
        motorB2();
        // Serial.println("")
        break;
   case micro_line_R:
        //printf("goStraight\n");

        while (true)
        {
            Sensor_R_M = digitalRead(SensorPinR_2);
        
        
            motorA1PNS(P);
            motorA2PNS(N);
            motorB1PNS(N);
            motorB2PNS(P);
            motorA1();

        
            motorA2();

            
            motorB1();

            
            motorB2();

            if(!Sensor_R_M){
                motorA1PNS(S);
                motorA2PNS(S);
                motorB1PNS(S);
                motorB2PNS(S);
                break;

            }
        }
        
        break;

   case mid_line_R:
        //printf("goStraight\n");

        while (true)
        {

            Sensor_F_M = digitalRead(SensorPinF_2);
            Sensor_B_M = digitalRead(SensorPinB_2);

        
            motorA1PNS(P);
            motorA2PNS(N);
            motorB1PNS(N);
            motorB2PNS(P);
            motorA1();

        
            motorA2();

            
            motorB1();

            
            motorB2();

            if(!Sensor_F_M && !Sensor_B_M){
                motorA1PNS(S);
                motorA2PNS(S);
                motorB1PNS(S);
                motorB2PNS(S);
                break;

            }
        }
        
        break;

    case stp:
        setSpdA1(0);
        setSpdA2(0);
        setSpdB1(0);
        setSpdB2(0);
        motorA1();
        motorA2();
        motorB1();
        motorB2();
        delay(200);

        motorA1PNS(S);
        motorA2PNS(S);
        motorB1PNS(S);
        motorB2PNS(S);

        setSpdA1(targetSpd);
        setSpdA2(targetSpd);
        setSpdB1(targetSpd);
        setSpdB2(targetSpd);

        break;

    }
  
}




//到达位置抓取动作
void ctrl_grab_servo(){
    //处理接收到的x，y
    UART_DELTA_PROC(UART_DELTA[0],UART_DELTA[1]);
    //计算后输出 UART_DELTA_VALUE[0,1,2],对应 3，4，5
    //控制机械臂移动
    LobotServo servos[5];   //舵机ID位置结构数组
    servos[0].ID = 2;       //2号舵机
    servos[0].Position = 896;  //1400位置

    servos[1].ID = 3;       //4号舵机
    servos[1].Position = 1507;  //700位置

    servos[2].ID = 4;       //2号舵机
    servos[2].Position = 1477;  //1400位置

    servos[3].ID = 5;       //4号舵机
    servos[3].Position = 546;  //700位置

    servos[4].ID = 6;       //2号舵机
    servos[4].Position = 2222;  //1400位置

    myse.moveServos(servos,5,9000);
    delay(10000);
    //抓取
    myse.moveServo(2,1863,500);
    delay(1000);
    
} 

/*
* 功能: 路径序列
* 参数: 任务结构体missions的每一项标记位
*/


missions* addmissions(){
    

}





/*
* 功能: 增加一个路径序列项
* 参数: 任务结构体mission的每一项标记位
*/
mission* addMission(bool A,bool B,bool C,bool D,bool E,bool M,bool M_1,bool M_2,bool M_3)
{
  mission* currentMission = NULL;

  currentMission = (mission*)malloc(sizeof(mission));

  currentMission->A = A;
  currentMission->B = B;
  currentMission->C = C;
  currentMission->D = D;
  currentMission->E = E;

  currentMission->M = M;
  currentMission->M_1 = M_1;
  currentMission->M_2 = M_2;
  currentMission->M_3 = M_3;

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
      head = addMission(missArry[i].A,missArry[i].B,missArry[i].C,missArry[i].D,missArry[i].E,missArry[i].M,missArry[i].M_1,missArry[i].M_2,missArry[i].M_3);
      p = head;
      flag = false;
      continue;
    }

    p->next = addMission(missArry[i].A,missArry[i].B,missArry[i].C,missArry[i].D,missArry[i].E,missArry[i].M,missArry[i].M_1,missArry[i].M_2,missArry[i].M_3);
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
missions* missionsInit(){

    missions* p = NULL;
    missions* h = NULL;



  //任务序列

    h = p = (missions*)malloc(sizeof(missions));

  //扫码
    h->head = S_Code();
    h->flag = true;

    //原料区1
    p = (missions*)malloc(sizeof(missions));

    
    p->head = T_Obj1();
    p->flag = true;

    h->next = p;
    p->next = NULL;

    //粗加工1
    p->next = (missions*)malloc(sizeof(missions));

    p = p->next;

    p->head = P_Obj1();
    p->flag = true;

    p->next = NULL;

    // //半成品区1

    // p->next = (missions*)malloc(sizeof(missions));

    // p = p->next;

    // p->head = D_Obj1();
    // p->flag = true;

    // p->next = NULL;

    // //原料区2(第二次抓取)

    // p->next = (missions*)malloc(sizeof(missions));

    // p = p->next;

    // p->head = T_Obj2();
    // p->flag = true;

    // p->next = NULL;

    // //粗加工2

    // p->next = (missions*)malloc(sizeof(missions));

    // p = p->next;

    // p->head = P_Obj2();
    // p->flag = true;

    // p->next = NULL;

    // //半成品2

    // p->next = (missions*)malloc(sizeof(missions));

    // p = p->next;

    // p->head = D_Obj2();
    // p->flag = true;

    // p->next = NULL;

    // //结束










    




    return h;


//   //粗加工区1
//   currentMissions[2].head = createMissionList(12,missionAry);
//   currentMissions[2].flag = true;
//   //粗加工区2
//   currentMissions[3].head = createMissionList(12,missionAry);
//   currentMissions[3].flag = false;


//   //半成品区1
//   currentMissions[4].head = createMissionList(12,missionAry);
//   currentMissions[4].flag = true;
//   //半成品区2
//   currentMissions[5].head = createMissionList(12,missionAry);
//   currentMissions[5].flag = false;


//   //原料区1
//   currentMissions[6].head = createMissionList(12,missionAry);
//   currentMissions[6].flag = true;
//   //原料区2
//   currentMissions[7].head = createMissionList(12,missionAry);
//   currentMissions[7].flag = false;


//   //粗加工区1
//   currentMissions[8].head = createMissionList(12,missionAry);
//   currentMissions[8].flag = true;
//   //粗加工区2
//   currentMissions[9].head = createMissionList(12,missionAry);
//   currentMissions[9].flag = false;



//   //半成品区1
//   currentMissions[10].head = createMissionList(12,missionAry);
//   currentMissions[10].flag = true;
//   //半成品区2
//   currentMissions[11].head = createMissionList(12,missionAry);
//   currentMissions[11].flag = false;

//   //终点区1
//   currentMissions[12].head = createMissionList(12,missionAry);
//   currentMissions[12].flag = true;
//   //终点区2
//   currentMissions[13].head = createMissionList(12,missionAry);
//   currentMissions[13].flag = false;


}


/*
* 路径—任务规划
* 读取传感器-切换路径-确认路径-方格经过确认-任务确认
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
    mp = mp->next;
    flagA = false;
    flagB = false;
    flagC = true;
    flagD = false;
    T1 = T2 = 0;

    //内存溢出保护
    if (mp == NULL){
        Listflag = false;
        currentStates = stp;
        mps = mps->next;
        mp = mps->head;
    }
  }
  

  
  
  
//封装为函数

    //前
    if (!(mp->A) && !(mp->B) && Listflag)
    {
      //更新当前方向
      currentStates = goStraight;
      micro_line_flag = false;
      mid_line_flag = false;
      
    }
    //后
    if (!(mp->A) && mp->B && Listflag)
    {
      //更新当前方向
      currentStates = goBack;
      micro_line_flag = false;
      mid_line_flag = false;
    }
    //左
    if (mp->A && !(mp->B) && Listflag)
    {
      //更新当前方向
      currentStates = Left;
       micro_line_flag = false;
       mid_line_flag = false;
    }
    //右
    if (mp->A && mp->B && Listflag)
    {
      //更新当前方向
      currentStates = Right;
       micro_line_flag = false;
       mid_line_flag = false;
    }
    //停止
    if (mp->C)
    {
        currentStates = stp;
         micro_line_flag = false;
         mid_line_flag = false;
        //停止后可能会一侧越线所以提前置位
         flagA = true;
    }



    //线条模式 前压线

    if (mp->D && Listflag && !(mp->A) && !(mp->B))
    {
        micro_flag_F = true;
        currentStates = micro_line_F;
    }
    
    //线条模式 后压线
    if (mp->D && Listflag && !(mp->A) && mp->B)
    {
        micro_flag_B = true;
        currentStates = micro_line_B;
    }
    //线条模式 左压线
    if (mp->D && Listflag && mp->A && !(mp->B))
    {
        micro_flag_L = true;
        currentStates = micro_line_L;
    }
    //线条模式 右压线
    if (mp->D && Listflag && mp->A && mp->B)
    {
        micro_flag_R = true;
        currentStates = micro_line_R;
    }


    //中线模式 前

    if (mp->E && !(mp->A) && !(mp->B)){
      mid_flag_F = true;
      currentStates = mid_line_F;
    }
    //中线模式 后
    if (mp->E && !(mp->A) && mp->B){
      mid_flag_B = true;
      currentStates = mid_line_B;
    }
    //中线模式 左
    if (mp->E && mp->A && !(mp->B)){
      mid_flag_L = true;
      currentStates = mid_line_L;
    }
    //中线模式 右
    if (mp->E && mp->A && mp->B){
      mid_flag_R = true;
      currentStates = mid_line_R;
    }






   //判断是否经过方格
  switch (currentStates)
  {


  case stp:
    Listflag = false;
    flagA = false;
    flagB = false;
    flagC = true;
    flagD = false;
    break;


  case goStraight:
  
  //上次状态为特殊模式


    if (Sensor_F_M == 0)
    {
      flagA = true;
    }

    if (Sensor_B_M == 0 && flagA)
    
    {
        //tick-mpwm表
        

        flagB = true;

    }

    if (flagB)
    {
        flagD = true;
        // T2 = getTickTime();

        // if (flagC)
        // {
        //     T1 = T2;
        //     flagC = false;
        // }
        
        // if (T2 - T1 > tickThreshold)
        // {
        //     flagD = true;
        // }
    }
    break;
  case goBack:

  //上次状态为特殊模式

    if (Sensor_B_M == 0)
    {
      flagA = true;
    }

    if (Sensor_F_M == 0 && flagA)
    // if (Sensor_F_M == 0)
    {
        // flagA = true;//测试后删掉
        flagB = true;

    }

    if(flagB)
    {
        flagD = true;
        // T2 = getTickTime();

        // if (flagC)
        // {
        //     T1 = T2;
        //     flagC = false;
        // }
        
        // if (T2 - T1 > tickThreshold)
        // {
        //     flagD = true;
        // }
    }
    break;

  case Left:

  //上次状态为特殊模式
    if (Sensor_L_M == 0)
    {
      flagA = true;
    }

    if (Sensor_R_M == 0 && flagA)
    // if (Sensor_R_M == 0)
    {
        // flagA = true;//测试后删掉
        flagB = true;
    }
    if (flagB)
    {
        flagD = true;
        // T2 = getTickTime();

        // if (flagC)
        // {
        //     T1 = T2;
        //     flagC = false;
        // }
        
        // if (T2 - T1 > tickThreshold + 30)
        // {
        //     flagD = true;
        // }
    }
    break;
    
    
  case Right:

  //上次状态为特殊模式
    if (Sensor_R_M == 0)
    {
      flagA = true;
    }
    if (Sensor_L_M == 0 && flagA)
    {
        // flagA = true;//测试后删掉
        flagB =true;
    }

    if (flagB)
    {
        flagD = true;
        // T2 = getTickTime();

        // if (flagC)
        // {
        //     T1 = T2;
        //     flagC = false;
        // }
        
        // if (T2 - T1 > tickThreshold + 30)
        // {
        //     flagD = true;
        // }
    }
    
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
    if (!Sensor_L_M){
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
    if (!Sensor_L_M || !Sensor_R_M){
      flagA = flagD = true;
      
    }
    break;
  case mid_line_B:
  //碰线,路径切换
    if (!Sensor_L_M || !Sensor_R_M){
      flagA = flagD = true;
    }
    break;
    
  case mid_line_L:
  //碰线,切换下一个状态
    if (!Sensor_F_M || !Sensor_B_M){
      flagA = flagD = true;
      
    }
     break;
  case mid_line_R:
  //碰线,切换下一个状态
    if (!Sensor_B_M || !Sensor_F_M){
      flagA = flagD = true;
    }
    break;
    
  default:
    break;
  }






/*===========================任务规划==================================*/

    //任务模式
    if (mp->C && mp->M){
        // if (1){
        //扫码请求
        // Serial.println("mission Mode");
        if ( !(mp->M_1) && !(mp->M_2) && !(mp->M_3) ){
            // // if (1){
            // //机械臂动作
            // Serial.println("ArmAct");
            // // int i = 22500;
            // myse.runActionGroup(1,1);
            // delay(2000);
            // //扫码请求
            // reqs("#A$","#Aa$");
            // //持久化存储，LED显示
            // getCode();
            // //收回动作
            // // i = 500;
            // myse.runActionGroup(2,1);
            // delay(2000);
            // //开始下一个任务序列
            // // flagA = flagD = true;//单个任务序列启用
            // mps = mps->next;
            // mp = mps->head;



            // //防止越线导致检测失败
            // flagA = true;

/*--------------优化后的通信模块----------------*/


            // Serial.println("ArmAct");

            // myse.runActionGroup(1,1);
            // directions();
            // delay(2000);

            //扫码请求
            reqs_1("#A$");

            //收回动作openmv控制

            // myse.runActionGroup(2,1);
            // directions();
            // delay(4000);

            return;

        }   
        
        //第二次动态路径规划
        // if (mp->M_1 && !(mp->M_2) && !(mp->M_3)){
        //     dynamicGrabPlan();
        //     flagA = flagD = true;
        //     return;
        // }

        //抓取请求
        if ( !(mp->M_1) && !(mp->M_2) && mp->M_3){
            // grab_plan_count++;
            // Serial.println("请求抓取");

            // //扫描
            // myse.runActionGroup(3,1);
            // //运行一次防止因延时造成错误
            // directions();

            // delay(10000);

            // //有规划时跳出，反之一直运行
            // reqs("#B$","#Bb$");

            
            // Serial.println("------plan-----");
            // Serial.println(UART_PLAN[0]);
            // Serial.println(UART_PLAN[1]);
            // Serial.println(UART_PLAN[2]);
            // Serial.println("------plan-----");

            

            // //切换路径有两个方法，一个是主动使用指针切换，一个是自动切换任务
            // flagA = flagD = true;//单次任务路径切换启用

/*--------------------优化后的通信模块-----------------------*/
            Serial.println("请求抓取");
            // myse.runActionGroup(3,1);
            // directions();
            // delay(10000);
            reqs_1("#B$");
            

    }
    
/*===========================任务规划==================================*/

    }

}


/*
* 路径序列测试函数
* 将规划好的路径存入该函数
*/

//扫码
mission* S_Code(){
  mission demo[5];
  //L
  demo[0].A = true;
  demo[0].B = false;
  demo[0].C = false;
  demo[0].D = true;
  demo[0].E = false;
  demo[0].M = false;
  demo[0].M_1 = false;
  demo[0].M_2 = false;
  demo[0].M_3 = false;
  //F
  demo[1].A = false;
  demo[1].B = false;
  demo[1].C = false;
  demo[1].D = false;
  demo[1].E = false;
  demo[1].M = false;
  demo[1].M_1 = false;
  demo[1].M_2 = false;
  demo[1].M_3 = false;
  //F
  demo[2].A = false;
  demo[2].B = false;
  demo[2].C = false;
  demo[2].D = false;
  demo[2].E = false;
  demo[2].M = false;
  demo[2].M_1 = false;
  demo[2].M_2 = false;
  demo[2].M_3 = false;
  //R-Line
  demo[3].A = false;
  demo[3].B = true;
  demo[3].C = false;
  demo[3].D = true;
  demo[3].E = false;
  demo[3].M = false;
  demo[3].M_1 = false;
  demo[3].M_2 = false;
  demo[3].M_3 = false;

  //扫码请求
  demo[4].A = false;
  demo[4].B = false;
  demo[4].C = true;
  demo[4].D = false;
  demo[4].E = false;
  demo[4].M = true;
  demo[4].M_1 = false;
  demo[4].M_2 = false;
  demo[4].M_3 = false;


  return createMissionList(5,demo);
  
}



//物料区1
mission* T_Obj1(){
  mission demo[4];
  //Front
  demo[0].A = false;
  demo[0].B = false;
  demo[0].C = false;
  demo[0].D = false;
  demo[0].E = false;
  demo[0].M = false;
  demo[0].M_1 = false;
  demo[0].M_2 = false;
  demo[0].M_3 = false;
  
  //Front
  demo[1].A = false;
  demo[1].B = false;
  demo[1].C = false;
  demo[1].D = false;
  demo[1].E = false;
  demo[1].M = false;
  demo[1].M_1 = false;
  demo[1].M_2 = false;
  demo[1].M_3 = false;



  //Front
  demo[2].A = false;
  demo[2].B = false;
  demo[2].C = false;
  demo[2].D = false;
  demo[2].E = false;
  demo[2].M = false;
  demo[2].M_1 = false;
  demo[2].M_2 = false;
  demo[2].M_3 = false;
  //Stop-REQ-Grab
  demo[3].A = false;
  demo[3].B = false;
  demo[3].C = true;
  demo[3].D = false;
  demo[3].E = false;
  demo[3].M = true;
  demo[3].M_1 = false;
  demo[3].M_2 = false;
  demo[3].M_3 = true;



//   //Front-Mid-Line
//   demo[3].A = false;
//   demo[3].B = false;
//   demo[3].C = false;
//   demo[3].D = false;
//   demo[3].E = true;
//   demo[3].M = false;
//   demo[3].M_1 = false;
//   demo[3].M_2 = false;
//   demo[3].M_3 = false;


//   //Stop-REQ-Grub
//   demo[4].A = false;
//   demo[4].B = false;
//   demo[4].C = true;
//   demo[4].D = false;
//   demo[4].E = false;
//   demo[4].M = true;
//   demo[4].M_1 = false;
//   demo[4].M_2 = false;
//   demo[4].M_3 = true;



  return createMissionList(4,demo);
  
}


//物料区2
mission* T_Obj2(){
  mission demo[4];
  //Front
  demo[0].A = false;
  demo[0].B = false;
  demo[0].C = false;
  demo[0].D = false;
  demo[0].E = false;
  demo[0].M = false;
  demo[0].M_1 = false;
  demo[0].M_2 = false;
  demo[0].M_3 = false;
  
  //Front
  demo[1].A = false;
  demo[1].B = false;
  demo[1].C = false;
  demo[1].D = false;
  demo[1].E = false;
  demo[1].M = false;
  demo[1].M_1 = false;
  demo[1].M_2 = false;
  demo[1].M_3 = false;



  //Stop-REQ-Grab
  demo[2].A = false;
  demo[2].B = false;
  demo[2].C = false;
  demo[2].D = false;
  demo[2].E = false;
  demo[2].M = false;
  demo[2].M_1 = false;
  demo[2].M_2 = false;
  demo[2].M_3 = false;
  //Stop-REQ-Grab
  demo[3].A = false;
  demo[3].B = false;
  demo[3].C = true;
  demo[3].D = false;
  demo[3].E = false;
  demo[3].M = true;
  demo[3].M_1 = false;
  demo[3].M_2 = false;
  demo[3].M_3 = true;



//   //Front-Mid-Line
//   demo[3].A = false;
//   demo[3].B = false;
//   demo[3].C = false;
//   demo[3].D = false;
//   demo[3].E = true;
//   demo[3].M = false;
//   demo[3].M_1 = false;
//   demo[3].M_2 = false;
//   demo[3].M_3 = false;


//   //Stop-REQ-Grub
//   demo[4].A = false;
//   demo[4].B = false;
//   demo[4].C = true;
//   demo[4].D = false;
//   demo[4].E = false;
//   demo[4].M = true;
//   demo[4].M_1 = false;
//   demo[4].M_2 = false;
//   demo[4].M_3 = true;



  return createMissionList(4,demo);
  
}



    //粗加工区1
mission* P_Obj1(){
    
  mission demo[3];
  //R
  demo[0].A = false;
  demo[0].B = true;
  demo[0].C = false;
  demo[0].D = false;
  demo[0].E = false;
  demo[0].M = false;
  demo[0].M_1 = false;
  demo[0].M_2 = false;
  demo[0].M_3 = false;
  //F
  demo[1].A = false;
  demo[1].B = true;
  demo[1].C = false;
  demo[1].D = false;
  demo[1].E = false;
  demo[1].M = false;
  demo[1].M_1 = false;
  demo[1].M_2 = false;
  demo[1].M_3 = false;
  //F
  demo[2].A = false;
  demo[2].B = false;
  demo[2].C = true;
  demo[2].D = false;
  demo[2].E = false;
  demo[2].M = false;
  demo[2].M_1 = false;
  demo[2].M_2 = false;
  demo[2].M_3 = false;

    return createMissionList(3,demo);
}

    //粗加工区2
mission* P_Obj2(){
    return NULL;
}

//半成品区1
mission* D_Obj1(){
    return NULL;
}

//半成品区2
mission* D_Obj2(){
    return NULL;
}
/*
*抓取动态规划
*失败返回false
*/

bool dynamicGrabPlan(){

    //规划有效判断
    if (UART_PLAN[0]){
        
        //123 132 213 231 312 321


        //123 132
        if (UART_PLAN[0] == 1){
            //123
            if (UART_PLAN[1] == 2){

                mp->next = (mission*)malloc(sizeof(mission));
                //Front-mid-line
                mp->next->A = false;
                mp->next->B = false;
                mp->next->C = false;
                mp->next->D = false;
                mp->next->E = true;
                mp->next->M = false;
                mp->next->M_1 = false;
                mp->next->M_2 = false;
                mp->next->M_3 = false;
                mp->next->next = NULL;


                mp->next->next = (mission*)malloc(sizeof(mission));
                //Stp-REQ-grab
                mp->next->next->A = false;
                mp->next->next->B = false;
                mp->next->next->C = true;
                mp->next->next->D = false;
                mp->next->next->E = false;
                mp->next->next->M = true;
                mp->next->next->M_1 = false;
                mp->next->next->M_2 = false;
                mp->next->next->M_3 = true;
                mp->next->next->next = NULL;



                mp->next->next->next = (mission*)malloc(sizeof(mission));
                //Front
                mp->next->next->next->A = false;
                mp->next->next->next->B = false;
                mp->next->next->next->C = false;
                mp->next->next->next->D = false;
                mp->next->next->next->E = false;
                mp->next->next->next->M = false;
                mp->next->next->next->M_1 = false;
                mp->next->next->next->M_2 = false;
                mp->next->next->next->M_3 = false;
                mp->next->next->next->next = NULL;



                mp->next->next->next->next = (mission*)malloc(sizeof(mission));
                //Stp-REQ-grab
                mp->next->next->next->next->A = false;
                mp->next->next->next->next->B = false;
                mp->next->next->next->next->C = true;
                mp->next->next->next->next->D = false;
                mp->next->next->next->next->E = false;
                mp->next->next->next->next->M = true;
                mp->next->next->next->next->M_1 = false;
                mp->next->next->next->next->M_2 = false;
                mp->next->next->next->next->M_3 = true;
                mp->next->next->next->next->next = NULL;  

                return false;



            }

            //132
            else{

                mp->next = (mission*)malloc(sizeof(mission));
                //Front-mid-line
                mp->next->A = false;
                mp->next->B = false;
                mp->next->C = false;
                mp->next->D = false;
                mp->next->E = false;
                mp->next->M = false;
                mp->next->M_1 = false;
                mp->next->M_2 = false;
                mp->next->M_3 = false;
                mp->next->next = NULL;


                mp->next->next = (mission*)malloc(sizeof(mission));
                //Stp-REQ-grab
                mp->next->next->A = false;
                mp->next->next->B = false;
                mp->next->next->C = true;
                mp->next->next->D = false;
                mp->next->next->E = false;
                mp->next->next->M = true;
                mp->next->next->M_1 = false;
                mp->next->next->M_2 = false;
                mp->next->next->M_3 = true;
                mp->next->next->next = NULL;



                mp->next->next->next = (mission*)malloc(sizeof(mission));
                //Back-mid-line
                mp->next->next->next->A = false;
                mp->next->next->next->B = true;
                mp->next->next->next->C = false;
                mp->next->next->next->D = false;
                mp->next->next->next->E = false;
                mp->next->next->next->M = false;
                mp->next->next->next->M_1 = false;
                mp->next->next->next->M_2 = false;
                mp->next->next->next->M_3 = false;
                mp->next->next->next->next = NULL;



                mp->next->next->next->next = (mission*)malloc(sizeof(mission));
                //Stp-REQ-grab
                mp->next->next->next->next->A = false;
                mp->next->next->next->next->B = false;
                mp->next->next->next->next->C = true;
                mp->next->next->next->next->D = false;
                mp->next->next->next->next->E = false;
                mp->next->next->next->next->M = true;
                mp->next->next->next->next->M_1 = false;
                mp->next->next->next->next->M_2 = false;
                mp->next->next->next->next->M_3 = true;
                mp->next->next->next->next->next = NULL;  


                mp->next->next->next->next->next = (mission*)malloc(sizeof(mission));
                //Front
                mp->next->next->next->next->next->A = false;
                mp->next->next->next->next->next->B = false;
                mp->next->next->next->next->next->C = false;
                mp->next->next->next->next->next->D = false;
                mp->next->next->next->next->next->E = false;
                mp->next->next->next->next->next->M = false;
                mp->next->next->next->next->next->M_1 = false;
                mp->next->next->next->next->next->M_2 = false;
                mp->next->next->next->next->next->M_3 = false;
                mp->next->next->next->next->next->next = NULL; 
                return false;
            }

        }

        //213 231
        else if (UART_PLAN[0] == 2){
            //213
            if (UART_PLAN[1] == 1){

                mp->next = (mission*)malloc(sizeof(mission));
                //Front-mid-line
                mp->next->A = false;
                mp->next->B = false;
                mp->next->C = false;
                mp->next->D = false;
                mp->next->E = true;
                mp->next->M = false;
                mp->next->M_1 = false;
                mp->next->M_2 = false;
                mp->next->M_3 = false;
                mp->next->next = NULL;


                mp->next->next = (mission*)malloc(sizeof(mission));
                //Stp-REQ-grab
                mp->next->next->A = false;
                mp->next->next->B = false;
                mp->next->next->C = true;
                mp->next->next->D = false;
                mp->next->next->E = false;
                mp->next->next->M = true;
                mp->next->next->M_1 = false;
                mp->next->next->M_2 = false;
                mp->next->next->M_3 = true;
                mp->next->next->next = NULL;



                mp->next->next->next = (mission*)malloc(sizeof(mission));
                //Back
                mp->next->next->next->A = false;
                mp->next->next->next->B = true;
                mp->next->next->next->C = false;
                mp->next->next->next->D = false;
                mp->next->next->next->E = true;
                mp->next->next->next->M = false;
                mp->next->next->next->M_1 = false;
                mp->next->next->next->M_2 = false;
                mp->next->next->next->M_3 = false;
                mp->next->next->next->next = NULL;



                mp->next->next->next->next = (mission*)malloc(sizeof(mission));
                //Stp-REQ-grab
                mp->next->next->next->next->A = false;
                mp->next->next->next->next->B = false;
                mp->next->next->next->next->C = true;
                mp->next->next->next->next->D = false;
                mp->next->next->next->next->E = false;
                mp->next->next->next->next->M = true;
                mp->next->next->next->next->M_1 = false;
                mp->next->next->next->next->M_2 = false;
                mp->next->next->next->next->M_3 = true;
                mp->next->next->next->next->next = NULL;  


                mp->next->next->next->next->next = (mission*)malloc(sizeof(mission));
                //Front
                mp->next->next->next->next->next->A = false;
                mp->next->next->next->next->next->B = false;
                mp->next->next->next->next->next->C = false;
                mp->next->next->next->next->next->D = false;
                mp->next->next->next->next->next->E = false;
                mp->next->next->next->next->next->M = false;
                mp->next->next->next->next->next->M_1 = false;
                mp->next->next->next->next->next->M_2 = false;
                mp->next->next->next->next->next->M_3 = false;
                mp->next->next->next->next->next->next = NULL;  



                mp->next->next->next->next->next->next = (mission*)malloc(sizeof(mission));
                //Stp-REQ-grab
                mp->next->next->next->next->next->next->A = false;
                mp->next->next->next->next->next->next->B = false;
                mp->next->next->next->next->next->next->C = false;
                mp->next->next->next->next->next->next->D = false;
                mp->next->next->next->next->next->next->E = false;
                mp->next->next->next->next->next->next->M = false;
                mp->next->next->next->next->next->next->M_1 = false;
                mp->next->next->next->next->next->next->M_2 = false;
                mp->next->next->next->next->next->next->M_3 = false;
                mp->next->next->next->next->next->next->next = NULL;
                return false;
            }


            //231
            else{

                mp->next = (mission*)malloc(sizeof(mission));
                //Front-mid-line
                mp->next->A = false;
                mp->next->B = false;
                mp->next->C = false;
                mp->next->D = false;
                mp->next->E = true;
                mp->next->M = false;
                mp->next->M_1 = false;
                mp->next->M_2 = false;
                mp->next->M_3 = false;
                mp->next->next = NULL;


                mp->next->next = (mission*)malloc(sizeof(mission));
                //Stp-REQ-grab
                mp->next->next->A = false;
                mp->next->next->B = false;
                mp->next->next->C = true;
                mp->next->next->D = false;
                mp->next->next->E = false;
                mp->next->next->M = true;
                mp->next->next->M_1 = false;
                mp->next->next->M_2 = false;
                mp->next->next->M_3 = true;
                mp->next->next->next = NULL;



                mp->next->next->next = (mission*)malloc(sizeof(mission));
                //Front
                mp->next->next->next->A = false;
                mp->next->next->next->B = false;
                mp->next->next->next->C = false;
                mp->next->next->next->D = false;
                mp->next->next->next->E = false;
                mp->next->next->next->M = false;
                mp->next->next->next->M_1 = false;
                mp->next->next->next->M_2 = false;
                mp->next->next->next->M_3 = false;
                mp->next->next->next->next = NULL;



                mp->next->next->next->next = (mission*)malloc(sizeof(mission));
                //Stp-REQ-grab
                mp->next->next->next->next->A = false;
                mp->next->next->next->next->B = false;
                mp->next->next->next->next->C = true;
                mp->next->next->next->next->D = false;
                mp->next->next->next->next->E = false;
                mp->next->next->next->next->M = true;
                mp->next->next->next->next->M_1 = false;
                mp->next->next->next->next->M_2 = false;
                mp->next->next->next->next->M_3 = true;
                mp->next->next->next->next->next = NULL;  


                mp->next->next->next->next->next = (mission*)malloc(sizeof(mission));
                //Back
                mp->next->next->next->next->next->A = false;
                mp->next->next->next->next->next->B = true;
                mp->next->next->next->next->next->C = false;
                mp->next->next->next->next->next->D = false;
                mp->next->next->next->next->next->E = false;
                mp->next->next->next->next->next->M = false;
                mp->next->next->next->next->next->M_1 = false;
                mp->next->next->next->next->next->M_2 = false;
                mp->next->next->next->next->next->M_3 = false;
                mp->next->next->next->next->next->next = NULL;  



                mp->next->next->next->next->next->next = (mission*)malloc(sizeof(mission));
                //Stp-REQ-grab
                mp->next->next->next->next->next->next->A = false;
                mp->next->next->next->next->next->next->B = false;
                mp->next->next->next->next->next->next->C = false;
                mp->next->next->next->next->next->next->D = false;
                mp->next->next->next->next->next->next->E = false;
                mp->next->next->next->next->next->next->M = false;
                mp->next->next->next->next->next->next->M_1 = false;
                mp->next->next->next->next->next->next->M_2 = false;
                mp->next->next->next->next->next->next->M_3 = false;
                mp->next->next->next->next->next->next->next = NULL;
                return false;

            }







        }

        //312 321
        else{
            //312
            if(UART_PLAN[1] == 1){

                // Serial.println("plan.....");

                mp->next = (mission*)malloc(sizeof(mission));
                //Front
                mp->next->A = false;
                mp->next->B = false;
                mp->next->C = false;
                mp->next->D = false;
                mp->next->E = false;
                mp->next->M = false;
                mp->next->M_1 = false;
                mp->next->M_2 = false;
                mp->next->M_3 = false;
                mp->next->next = NULL;


                mp->next->next = (mission*)malloc(sizeof(mission));
                //Stp-REQ-grab
                mp->next->next->A = false;
                mp->next->next->B = false;
                mp->next->next->C = true;
                mp->next->next->D = false;
                mp->next->next->E = false;
                mp->next->next->M = true;
                mp->next->next->M_1 = false;
                mp->next->next->M_2 = false;
                mp->next->next->M_3 = true;
                mp->next->next->next = NULL;



                mp->next->next->next = (mission*)malloc(sizeof(mission));
                //Back
                mp->next->next->next->A = false;
                mp->next->next->next->B = true;
                mp->next->next->next->C = false;
                mp->next->next->next->D = false;
                mp->next->next->next->E = false;
                mp->next->next->next->M = false;
                mp->next->next->next->M_1 = false;
                mp->next->next->next->M_2 = false;
                mp->next->next->next->M_3 = false;
                mp->next->next->next->next = NULL;



                mp->next->next->next->next = (mission*)malloc(sizeof(mission));
                //Stp-REQ-grab
                mp->next->next->next->next->A = false;
                mp->next->next->next->next->B = false;
                mp->next->next->next->next->C = true;
                mp->next->next->next->next->D = false;
                mp->next->next->next->next->E = false;
                mp->next->next->next->next->M = true;
                mp->next->next->next->next->M_1 = false;
                mp->next->next->next->next->M_2 = false;
                mp->next->next->next->next->M_3 = true;
                mp->next->next->next->next->next = NULL;  


                mp->next->next->next->next->next = (mission*)malloc(sizeof(mission));
                //Front-mid-line
                mp->next->next->next->next->next->A = false;
                mp->next->next->next->next->next->B = false;
                mp->next->next->next->next->next->C = false;
                mp->next->next->next->next->next->D = false;
                mp->next->next->next->next->next->E = true;
                mp->next->next->next->next->next->M = false;
                mp->next->next->next->next->next->M_1 = false;
                mp->next->next->next->next->next->M_2 = false;
                mp->next->next->next->next->next->M_3 = false;
                mp->next->next->next->next->next->next = NULL;  



                mp->next->next->next->next->next->next = (mission*)malloc(sizeof(mission));
                //Stp-REQ-grab
                mp->next->next->next->next->next->next->A = false;
                mp->next->next->next->next->next->next->B = false;
                mp->next->next->next->next->next->next->C = false;
                mp->next->next->next->next->next->next->D = false;
                mp->next->next->next->next->next->next->E = false;
                mp->next->next->next->next->next->next->M = false;
                mp->next->next->next->next->next->next->M_1 = false;
                mp->next->next->next->next->next->next->M_2 = false;
                mp->next->next->next->next->next->next->M_3 = false;
                mp->next->next->next->next->next->next->next = NULL;


                mp->next->next->next->next->next->next->next = (mission*)malloc(sizeof(mission));
                //Front
                mp->next->next->next->next->next->next->next->A = false;
                mp->next->next->next->next->next->next->next->B = false;
                mp->next->next->next->next->next->next->next->C = false;
                mp->next->next->next->next->next->next->next->D = false;
                mp->next->next->next->next->next->next->next->E = false;
                mp->next->next->next->next->next->next->next->M = false;
                mp->next->next->next->next->next->next->next->M_1 = false;
                mp->next->next->next->next->next->next->next->M_2 = false;
                mp->next->next->next->next->next->next->next->M_3 = false;
                mp->next->next->next->next->next->next->next->next = NULL;
                
                return false;
            }

            //321
            else{

                mp->next = (mission*)malloc(sizeof(mission));
                //Front
                mp->next->A = false;
                mp->next->B = false;
                mp->next->C = false;
                mp->next->D = false;
                mp->next->E = false;
                mp->next->M = false;
                mp->next->M_1 = false;
                mp->next->M_2 = false;
                mp->next->M_3 = false;
                mp->next->next = NULL;


                mp->next->next = (mission*)malloc(sizeof(mission));
                //Stp-REQ-grab
                mp->next->next->A = false;
                mp->next->next->B = false;
                mp->next->next->C = true;
                mp->next->next->D = false;
                mp->next->next->E = false;
                mp->next->next->M = true;
                mp->next->next->M_1 = false;
                mp->next->next->M_2 = false;
                mp->next->next->M_3 = true;
                mp->next->next->next = NULL;



                mp->next->next->next = (mission*)malloc(sizeof(mission));
                //Back-mid-line
                mp->next->next->next->A = false;
                mp->next->next->next->B = false;
                mp->next->next->next->C = false;
                mp->next->next->next->D = false;
                mp->next->next->next->E = true;
                mp->next->next->next->M = false;
                mp->next->next->next->M_1 = false;
                mp->next->next->next->M_2 = false;
                mp->next->next->next->M_3 = false;
                mp->next->next->next->next = NULL;



                mp->next->next->next->next = (mission*)malloc(sizeof(mission));
                //Stp-REQ-grab
                mp->next->next->next->next->A = false;
                mp->next->next->next->next->B = false;
                mp->next->next->next->next->C = true;
                mp->next->next->next->next->D = false;
                mp->next->next->next->next->E = false;
                mp->next->next->next->next->M = true;
                mp->next->next->next->next->M_1 = false;
                mp->next->next->next->next->M_2 = false;
                mp->next->next->next->next->M_3 = true;
                mp->next->next->next->next->next = NULL;  


                mp->next->next->next->next->next = (mission*)malloc(sizeof(mission));
                //Back
                mp->next->next->next->next->next->A = false;
                mp->next->next->next->next->next->B = true;
                mp->next->next->next->next->next->C = false;
                mp->next->next->next->next->next->D = false;
                mp->next->next->next->next->next->E = false;
                mp->next->next->next->next->next->M = false;
                mp->next->next->next->next->next->M_1 = false;
                mp->next->next->next->next->next->M_2 = false;
                mp->next->next->next->next->next->M_3 = false;
                mp->next->next->next->next->next->next = NULL;  



                mp->next->next->next->next->next->next = (mission*)malloc(sizeof(mission));
                //Stp-REQ-grab
                mp->next->next->next->next->next->next->A = false;
                mp->next->next->next->next->next->next->B = false;
                mp->next->next->next->next->next->next->C = false;
                mp->next->next->next->next->next->next->D = false;
                mp->next->next->next->next->next->next->E = false;
                mp->next->next->next->next->next->next->M = false;
                mp->next->next->next->next->next->next->M_1 = false;
                mp->next->next->next->next->next->next->M_2 = false;
                mp->next->next->next->next->next->next->M_3 = false;
                mp->next->next->next->next->next->next->next = NULL;


                mp->next->next->next->next->next->next->next = (mission*)malloc(sizeof(mission));
                //Front
                mp->next->next->next->next->next->next->next->A = false;
                mp->next->next->next->next->next->next->next->B = false;
                mp->next->next->next->next->next->next->next->C = false;
                mp->next->next->next->next->next->next->next->D = false;
                mp->next->next->next->next->next->next->next->E = false;
                mp->next->next->next->next->next->next->next->M = false;
                mp->next->next->next->next->next->next->next->M_1 = false;
                mp->next->next->next->next->next->next->next->M_2 = false;
                mp->next->next->next->next->next->next->next->M_3 = false;
                mp->next->next->next->next->next->next->next->next = NULL;
                return false;
            
            }

        }


    }

    else {
        return true;
    }
}





/*
* 偏移检测
* 读取传感器-读取当前状态-过线修正-方格内保持修正
* 根据时间来确定偏移度
* 对当前状态进行纠正
* 使用定时中断
*/
void stateFix()
{

    //特殊模式不启用
  if (currentStates > 4 && currentStates < 9){
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

        //出格检测
        //ctrlFlag为true时,偏移检测未运行
        if (ctrlFlag){
            //以两个传感器碰线为依据


            //左侧碰线
            if ( (!Sensor_L_M && !Sensor_L_R) || (!Sensor_L_M && !Sensor_L_L) || (!Sensor_L_M && !Sensor_L_R && !Sensor_L_L)){
                
                setSpdA1(0);
                setSpdA2(0);
                setSpdB1(0);
                setSpdB2(0);

                while (true)
                {
                    Sensor_L_R = digitalRead(SensorPinL_3);
                    Sensor_L_L = digitalRead(SensorPinL_1);
                    Sensor_L_M = digitalRead(SensorPinL_2);


                    motorA1PNS(P);
                    motorA2PNS(N);
                    motorB1PNS(N);
                    motorB2PNS(P);

                    setSpdA1(80);
                    setSpdA2(80);
                    setSpdB1(80);
                    setSpdB2(80);

                    motorA1();
                    motorA2();
                    motorB1();
                    motorB2();

                    //跳出条件,俩个及以上没有碰线
                    if ( (Sensor_L_R && Sensor_L_M )  || (Sensor_L_L && Sensor_L_M ) || (Sensor_L_L && Sensor_L_R ) || ( Sensor_L_L && Sensor_L_M && Sensor_L_R) ){

                        // 换向保护
                        motorA1PNS(S);
                        motorA2PNS(S);
                        motorB1PNS(S);
                        motorB2PNS(S);
                        //速度恢复
                        setSpdA1(targetSpd);
                        setSpdA2(targetSpd);
                        setSpdB1(targetSpd);
                        setSpdB2(targetSpd);
                        // Serial.println("左偏跳出");
                        break;

                    }




                }
                

            }


            //右侧碰线

            if ( (!Sensor_R_M && !Sensor_R_R) || (!Sensor_R_M && !Sensor_R_L) || (!Sensor_R_M && !Sensor_R_L && !Sensor_R_R) ){
                setSpdA1(0);
                setSpdA2(0);
                setSpdB1(0);
                setSpdB2(0);

                while (true)
                {
                    Sensor_R_L = digitalRead(SensorPinR_1);
                    Sensor_R_R = digitalRead(SensorPinR_3);
                    Sensor_R_M = digitalRead(SensorPinR_2);

                    motorA1PNS(N);
                    motorA2PNS(P);
                    motorB1PNS(P);
                    motorB2PNS(N);


                    setSpdA1(80);
                    setSpdA2(80);
                    setSpdB1(80);
                    setSpdB2(80);

                    motorA1();
                    motorA2();
                    motorB1();
                    motorB2();


                    //跳出条件,俩个及以上没有碰线
                    if ( (Sensor_R_R && Sensor_R_M )  || (Sensor_R_L && Sensor_R_M ) || (Sensor_R_L && Sensor_R_R) || (Sensor_R_L && Sensor_R_M && Sensor_R_R)){

                        // 换向保护
                        motorA1PNS(S);
                        motorA2PNS(S);
                        motorB1PNS(S);
                        motorB2PNS(S);
                        //速度恢复
                        setSpdA1(targetSpd);
                        setSpdA2(targetSpd);
                        setSpdB1(targetSpd);
                        setSpdB2(targetSpd);
                        break;


                    }




                }
            }



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
            setSpdA2(targetSpd);
            setSpdB1(targetSpd);
            //加速修正
            setSpdA1(targetSpd);
            setSpdB2(targetSpd);

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
            setSpdB2(targetSpd);
            //加速恢复
            setSpdA2(targetSpd);
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

        //出格检测
        //ctrlFlag为true时,偏移检测未运行
        if (ctrlFlag){
            //以两个传感器碰线为依据


            //左侧碰线
            if ( (!Sensor_L_M && !Sensor_L_R) || (!Sensor_L_M && !Sensor_L_L) || (!Sensor_L_M && !Sensor_L_R && !Sensor_L_L)){
                
                setSpdA1(0);
                setSpdA2(0);
                setSpdB1(0);
                setSpdB2(0);

                while (true)
                {
                    Sensor_L_R = digitalRead(SensorPinL_3);
                    Sensor_L_L = digitalRead(SensorPinL_1);
                    Sensor_L_M = digitalRead(SensorPinL_2);


                    motorA1PNS(P);
                    motorA2PNS(N);
                    motorB1PNS(N);
                    motorB2PNS(P);

                    setSpdA1(80);
                    setSpdA2(80);
                    setSpdB1(80);
                    setSpdB2(80);

                    motorA1();
                    motorA2();
                    motorB1();
                    motorB2();

                    //跳出条件,俩个及以上没有碰线
                    if ( (Sensor_L_R && Sensor_L_M )  || (Sensor_L_L && Sensor_L_M ) || (Sensor_L_L && Sensor_L_R)  || (Sensor_L_L && Sensor_L_M && Sensor_L_R)){

                        // 换向保护
                        motorA1PNS(S);
                        motorA2PNS(S);
                        motorB1PNS(S);
                        motorB2PNS(S);
                        //速度恢复
                        setSpdA1(targetSpd);
                        setSpdA2(targetSpd);
                        setSpdB1(targetSpd);
                        setSpdB2(targetSpd);
                        // Serial.println("左偏跳出");
                        break;

                    }




                }
                

            }


            //右侧碰线

            if ( (!Sensor_R_M && !Sensor_R_R) || (!Sensor_R_M && !Sensor_R_L) || (!Sensor_R_M && !Sensor_R_L && !Sensor_R_R) ){
                setSpdA1(0);
                setSpdA2(0);
                setSpdB1(0);
                setSpdB2(0);

                while (true)
                {
                    Sensor_R_L = digitalRead(SensorPinR_1);
                    Sensor_R_R = digitalRead(SensorPinR_3);
                    Sensor_R_M = digitalRead(SensorPinR_2);

                    motorA1PNS(N);
                    motorA2PNS(P);
                    motorB1PNS(P);
                    motorB2PNS(N);


                    setSpdA1(80);
                    setSpdA2(80);
                    setSpdB1(80);
                    setSpdB2(80);

                    motorA1();
                    motorA2();
                    motorB1();
                    motorB2();


                    //跳出条件,俩个及以上没有碰线
                    if ( (Sensor_R_R && Sensor_R_M )  || (Sensor_R_L && Sensor_R_M ) || (Sensor_R_L && Sensor_R_R) || (Sensor_R_L && Sensor_R_M && Sensor_R_R)){

                        // 换向保护
                        motorA1PNS(S);
                        motorA2PNS(S);
                        motorB1PNS(S);
                        motorB2PNS(S);
                        //速度恢复
                        setSpdA1(targetSpd);
                        setSpdA2(targetSpd);
                        setSpdB1(targetSpd);
                        setSpdB2(targetSpd);
                        break;


                    }




                }
            }



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
            setSpdA2(targetSpd);
            setSpdB1(targetSpd);
            //加速修正
            setSpdA1(targetSpd);
            setSpdB2(targetSpd);

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
            setSpdB2(targetSpd);
            //加速恢复
            setSpdA2(targetSpd);
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


        //出格检测
        //ctrlFlag为true时,偏移检测未运行
        if (ctrlFlag){
            //以两个传感器碰线为依据


            //左侧碰线(当前朝向)
            if ( (!Sensor_B_M && !Sensor_B_R) || (!Sensor_B_M && !Sensor_B_L) || (!Sensor_B_M && !Sensor_B_R && !Sensor_B_L)){
                
                // setSpdA1(0);
                // setSpdA2(0);
                // setSpdB1(0);
                // setSpdB2(0);

                while (true)
                {
                    Sensor_B_L = digitalRead(SensorPinB_1);
                    Sensor_B_R = digitalRead(SensorPinB_3);
                    Sensor_B_M = digitalRead(SensorPinB_2);


                    // motorA1PNS(N);
                    // motorA2PNS(P);
                    // motorB1PNS(P);
                    // motorB2PNS(N);

                    setSpdA1(spdA1 - 0.4);
                    setSpdA2(spdA2);
                    setSpdB1(spdB1);
                    setSpdB2(spdB2  -0.4);

                    motorA1();
                    motorA2();
                    motorB1();
                    motorB2();

                    //跳出条件,俩个及以上没有碰线
                    if ( (Sensor_B_R && Sensor_B_M )  || (Sensor_B_L && Sensor_B_M ) || (Sensor_B_L && Sensor_B_R)  || (Sensor_B_L && Sensor_B_M && Sensor_B_R)){

                        // 换向保护
                        motorA1PNS(S);
                        motorA2PNS(S);
                        motorB1PNS(S);
                        motorB2PNS(S);
                        //速度恢复
                        setSpdA1(targetSpd);
                        setSpdA2(targetSpd);
                        setSpdB1(targetSpd);
                        setSpdB2(targetSpd);
                        // Serial.println("左偏跳出");
                        break;

                    }




                }
                

            }


            //右侧碰线(当前朝向)

            if ( (!Sensor_F_M && !Sensor_F_R) || (!Sensor_F_M && !Sensor_F_L) || (!Sensor_F_M && !Sensor_F_L && !Sensor_F_R) ){
                // setSpdA1(0);
                // setSpdA2(0);
                // setSpdB1(0);
                // setSpdB2(0);

                while (true)
                {
                    Sensor_F_L = digitalRead(SensorPinF_1);
                    Sensor_F_R = digitalRead(SensorPinF_3);
                    Sensor_F_M = digitalRead(SensorPinF_2);


                    // motorA1PNS(N);
                    // motorA2PNS(P);
                    // motorB1PNS(P);
                    // motorB2PNS(N);


                    setSpdA1(spdA1);
                    setSpdA2(spdA2 - 0.4);
                    setSpdB1(spdB1 - 0.4);
                    setSpdB2(spdB2);

                    motorA1();
                    motorA2();
                    motorB1();
                    motorB2();


                    //跳出条件,俩个及以上没有碰线
                    if ( (Sensor_F_R && Sensor_F_M )  || (Sensor_F_L && Sensor_F_M ) || (Sensor_F_L && Sensor_F_R) || (Sensor_F_L && Sensor_F_M && Sensor_F_R)){

                        // 换向保护
                        motorA1PNS(S);
                        motorA2PNS(S);
                        motorB1PNS(S);
                        motorB2PNS(S);
                        //速度恢复
                        setSpdA1(targetSpd);
                        setSpdA2(targetSpd);
                        setSpdB1(targetSpd);
                        setSpdB2(targetSpd);
                        break;


                    }




                }
            }



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
            setSpdA1(targetSpd);
            setSpdB2(targetSpd);
            //加速修正
            setSpdA2(targetSpd);
            setSpdB1(targetSpd);
            

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
            setSpdA2(targetSpd);
            setSpdB1(targetSpd);
            //加速恢复
            setSpdA1(targetSpd);
            setSpdB2(targetSpd);

        }
        break;

    case Right:
        //偏右
        if (!Sensor_R_L && Sensor_R_R && ctrlFlag)
        {
        //  Serial.println("=============================================偏右===");

            SensorFlagL = true;//标记偏向
            ctrlFlag = false;//计时控制因子
            Fix_T1 = getTickTime();
        }

        //偏左
        if (!Sensor_R_R && Sensor_R_L && ctrlFlag) 
        {
        //  Serial.println("=============================================偏左===");
            SensorFlagR = true;//标记偏向哪一侧
            ctrlFlag = false;//计时控制因子
            Fix_T1 = getTickTime();
        }


        //出格检测
        //ctrlFlag为true时,偏移检测服务未运行
        if (ctrlFlag){
            //以两个传感器碰线为依据


            //左侧碰线(当前朝向)
            if ((!Sensor_B_M && !Sensor_B_R) || (!Sensor_B_M && !Sensor_B_L) || (!Sensor_B_R && !Sensor_B_L) || (!Sensor_B_M && !Sensor_B_R && !Sensor_B_L)){
                
                // setSpdA1(0);
                // setSpdA2(0);
                // setSpdB1(0);
                // setSpdB2(0);

                while (true)
                {
                    // Serial.println("================================================方格前侧碰线修正中===");
                    Sensor_B_L = digitalRead(SensorPinB_1);
                    Sensor_B_R = digitalRead(SensorPinB_3);
                    Sensor_B_M = digitalRead(SensorPinB_2);


                    // motorA1PNS(P);
                    // motorA2PNS(P);
                    // motorB1PNS(P);
                    // motorB2PNS(P);

                    setSpdA1(spdA1);
                    setSpdA2(spdA2 - 0.4);
                    setSpdB1(spdB1 - 0.4);
                    setSpdB2(spdB2);

                    motorA1();
                    motorA2();
                    motorB1();
                    motorB2();

                    //跳出条件,俩个及以上没有碰线
                    if ( (Sensor_B_R && Sensor_B_M )  || (Sensor_B_L && Sensor_B_M ) || (Sensor_B_L && Sensor_B_R)  || (Sensor_B_L && Sensor_B_M && Sensor_B_R)){

                        // 换向保护
                        motorA1PNS(S);
                        motorA2PNS(S);
                        motorB1PNS(S);
                        motorB2PNS(S);
                        //速度恢复
                        setSpdA1(targetSpd);
                        setSpdA2(targetSpd);
                        setSpdB1(targetSpd);
                        setSpdB2(targetSpd);
                        // Serial.println("=============================================方格前侧跳出");
                        break;

                    }




                }
                

            }


            //右侧碰线(当前朝向)

            if ( (!Sensor_F_M && !Sensor_F_R) || (!Sensor_F_M && !Sensor_F_L) || (!Sensor_F_R && !Sensor_F_L) || (!Sensor_F_M && !Sensor_F_L && !Sensor_F_R) ){
                // setSpdA1(0);
                // setSpdA2(0);
                // setSpdB1(0);
                // setSpdB2(0);

                while (true)
                {
                    // Serial.println("================================================方格后侧碰线修正中===");
                    Sensor_F_L = digitalRead(SensorPinF_1);
                    Sensor_F_R = digitalRead(SensorPinF_3);
                    Sensor_F_M = digitalRead(SensorPinF_2);


                    // motorA1PNS(N);
                    // motorA2PNS(N);
                    // motorB1PNS(N);
                    // motorB2PNS(N);


                    setSpdA1(spdA1 - 0.4);
                    setSpdA2(spdA2);
                    setSpdB1(spdB1);
                    setSpdB2(spdB2 - 0.4);

                    motorA1();
                    motorA2();
                    motorB1();
                    motorB2();


                    //跳出条件,俩个及以上没有碰线
                    if ( (Sensor_F_R && Sensor_F_M )  || (Sensor_F_L && Sensor_F_M ) || (Sensor_F_L && Sensor_F_R) || (Sensor_F_L && Sensor_F_M && Sensor_F_R)){

                        // 换向保护
                        motorA1PNS(S);
                        motorA2PNS(S);
                        motorB1PNS(S);
                        motorB2PNS(S);
                        delay(2);
                        //速度恢复
                        setSpdA1(targetSpd);
                        setSpdA2(targetSpd);
                        setSpdB1(targetSpd);
                        setSpdB2(targetSpd);
                        // Serial.println("================================================方格后侧碰线跳出===");
                        break;


                    }




                }
            }



        }


        //偏左未修正

        if (SensorFlagR)
        {
        //  Serial.println("================================================偏左修正中===");
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
        //  Serial.println("================================================偏左已修正===");
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
        //  Serial.println("================================================偏右修正中==");
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
        //  Serial.println("================================================偏右已修正===");
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
//缓冲区重置
void UART_DATA_FLUSH(){
    
    int j = 50;
    while(j--){
        UART_DATABUF[j] = 0x00;
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

    if (Serial2.available() < 0){
        Serial.println("Not Data");
        return false;
    }
    

    while (Serial2.available() >= 0 && i != 50){
        // 每次只读取一个字节    
        temp = Serial2.read();

        UART_DATABUF[i] = temp;
       
        i++;
     }
     Serial.println("getdata");
     return true;
 }
/*
*获取通信数据
*阻塞
*传入开始时间,超时返回true
*/
bool get_data_1(unsigned long startTime){
    int i = 0;
    while(true){
        
        //如果缓冲区有数据
        if (Serial2.available() > 0){

            //读取数据直到读完
            while(Serial2.available() > 0){
                UART_DATABUF[i] = Serial2.read();
                i++;
            }
            //清空缓冲区
            Serial2.flush();

            return false;
        }
        if (millis() - startTime > 10000){
            return true;
        }

    }

}


/*
*数据解码
*通信协议1
*/
/*
bool decodes(){
    short i = 0;

     
    while (UART_DATABUF[i] != 36 && i < 51){
        
        
        //解码# a123321 $
        if (UART_DATABUF[i] == 0x23 && UART_DATABUF[i+8] == 0x24){
            //confmCode:a

            decode_a(i);//数据解码存储
            

            // Serial.println("!!!!!!!!!!!!!!!!!!!");
            // Serial.println(UART_DATABUF[i]);
            // Serial.println(UART_DATABUF[i+1]);
            // Serial.println(UART_DATABUF[i+2]);
            // Serial.println(UART_DATABUF[i+3]);
            // Serial.println(UART_DATABUF[i+4]);
            // Serial.println(UART_DATABUF[i+5]);
            // Serial.println(UART_DATABUF[i+6]);
            // Serial.println(UART_DATABUF[i+7]);
            // Serial.println("EEEEEEEEEEEEEEEEEEE");
            //缓冲区重置
            UART_DATA_FLUSH();
            return false;

        }
        //          #b$(扫描下一个)             #b@&(+/-)xx&(+/-)yy$(抓取当前)            #b&(+/-)xx&(+/-)yy&xxx$(抓取当前并规划)
        
        //#b.................
        else if (UART_DATABUF[i] == 0x23 && UART_DATABUF[i+1] == 0x62){
            
            //扫描下一个 0x24 '$'
            if (UART_DATABUF[i+2] == 0x24){
                //切换状态,车辆移动至下一格子
                //不可抓取
                UART_DELTA[2] = 0;
                return false;
            }

            //抓取当前的 0x40 '@'~$
            if (UART_DATABUF[i+2] == 0x40 && UART_DATABUF[i+11] == 0x24){
                //解码,将数据存放至UART_DELTA中
                //b@& - 1 2 & + 2 2$
                decode_b(i);
                //可抓取
                UART_DELTA[2] = 1;
                return false;

            }
            //抓取并存放路径规划
            if (UART_DATABUF[i+2] == 0x26 && UART_DATABUF[i+14] == 0x24){
                //解码存放
                decode_b(i);
                UART_DELTA[2] = 1;
                //更新路径
                return false;
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
*/

/*
*数据解码
*openmv控制机械臂抓取(通信协议2)
*/
bool decodes(){
    int i = 0;
    

     Serial.println("enter decode");
    while (UART_DATABUF[i] != 36 && i < 51){
        
        
        
        //解码# a123321 $
        if (UART_DATABUF[i] == 35 && UART_DATABUF[i+1] == 97 &&  UART_DATABUF[i+8] == 36 && UART_SERVICE_A_FLAG){
            //confmCode:a

            decode_a(i);//数据解码存储
            

            // Serial.println("!!!!!!!!!!!!!!!!!!!");
            // Serial.println(UART_DATABUF[i]);
            // Serial.println(UART_DATABUF[i+1]);
            // Serial.println(UART_DATABUF[i+2]);
            // Serial.println(UART_DATABUF[i+3]);
            // Serial.println(UART_DATABUF[i+4]);
            // Serial.println(UART_DATABUF[i+5]);
            // Serial.println(UART_DATABUF[i+6]);
            // Serial.println(UART_DATABUF[i+7]);
            // Serial.println("EEEEEEEEEEEEEEEEEEE");
            //缓冲区重置
            UART_DATA_FLUSH();
            Serial2.flush();
            UART_SERVICE_A_FLAG = false;
            return false;

        }
        //          #b$(扫描下一个)           #bxxx$(规划)
        
        //#b.................
        if (UART_DATABUF[i] == 35 && UART_DATABUF[i+1] == 98 && (UART_DATABUF[i+2] == 36 || UART_DATABUF[i+5] == 36)){
            
            //扫描下一个 36 '$'
            if (UART_DATABUF[i+2] == 36){

                //缓存清空
                UART_DATA_FLUSH();
                Serial2.flush();
                return false;
            }


            //存放路径规划
            if (UART_DATABUF[i+1] == 98 && UART_DATABUF[i+5] == 36){
                //解码存放
                decode_b(i);

                //更新路径
                // dynamicGrabPlan();

                //缓冲区清空
                UART_DATA_FLUSH();
                Serial2.flush();
                //有规划返回false，开始下一个，反之继续等待
                return dynamicGrabPlan();
            }

            //confmCode:b

        }

        i++;

        
    }
    Serial.println("decoding error");
    UART_DATA_FLUSH();
    Serial2.flush();
    
    return true;
}


/*
*数据解码
*openmv控制机械臂抓取(通信协议2)
*解码失败返回true
*/
bool decodes_1(){
    int i = 0;
    

     Serial.println("enter decode");
    while (UART_DATABUF[i] != 36 && i < 51){
        
        
        
        //解码# a123321 $
        if (UART_DATABUF[i] == 35 && UART_DATABUF[i+1] == 97 &&  UART_DATABUF[i+8] == 36){
            //confmCode:a

            decode_a(i);//数据解码存储
            

            Serial.println("!!!!!!!!!!!!!!!!!!!");
            Serial.println(UART_DATABUF[i]);
            Serial.println(UART_DATABUF[i+1]);
            Serial.println(UART_DATABUF[i+2]);
            Serial.println(UART_DATABUF[i+3]);
            Serial.println(UART_DATABUF[i+4]);
            Serial.println(UART_DATABUF[i+5]);
            Serial.println(UART_DATABUF[i+6]);
            Serial.println(UART_DATABUF[i+7]);
            Serial.println("EEEEEEEEEEEEEEEEEEE");
            //缓冲区重置

            UART_DATA_FLUSH();
            //路径切换
            // mps = mps -> next;
            // mp = mps->head;
            flagA = flagD = true;
            return false;

        }
        //          #b$(扫描下一个)           #bxxx$(规划)
        
        //#b.................
        if (UART_DATABUF[i] == 35 && UART_DATABUF[i+1] == 98 && (UART_DATABUF[i+2] == 36 || UART_DATABUF[i+5] == 36)){
            
            //扫描下一个 36 '$'
            if (UART_DATABUF[i+2] == 36){

                //缓存清空
                UART_DATA_FLUSH();
                // Serial2.flush();
                flagA = flagD = true;
                return false;
            }


            //存放路径规划
            if (UART_DATABUF[i+1] == 98 && UART_DATABUF[i+5] == 36){
                //解码存放
                decode_b(i);
                dynamicGrabPlan();

                //缓冲区清空
                UART_DATA_FLUSH();
                //有规划返回false，开始下一个，反之继续等待
                flagA = flagD = true;
                return false;
            }


        }

        i++;

        
    }
    Serial.println("decoding error");
    return true;
}



//请求服务,参数为请求码,反馈码
void reqs(char* req_code,char* resp_code){
    int i = 0;
    bool uart_flag = true;

    while (uart_flag)
    {
        // Serial2.flush();

        Serial2.write(req_code,3);
        Serial.println("REQ_SCAN");


        
        //没有收到数据进行下一次循环
        if (!get_data()){
            blbl(5);
            continue;
        }
        //blbl(100);
        //请求成功,跳出循环
        bool tempFlag = decodes();
        uart_flag = respServices(tempFlag,resp_code);
        
    }
    Serial.println("REQ_SACN_DONE");

}


//请求服务1,参数为请求码
void reqs_1(char* req_code){ 
    //循环标记位
    bool flag = true;
    unsigned long startTime = 0;
    //获取开始时间
    startTime = millis();

    while(flag){

        //发送请求
        Serial2.write(req_code,3);

        //阻塞等待数据
        //如果超时，重新请求
        if(get_data_1(startTime)){continue;}


        //数据解码
        //如果解码失败返回True，重新请求
        flag = decodes_1();

    }

    
}
//任务码本地存储
void decode_a(short i){
    //#a123321$
    UART_TARGET[0] = byte_decode(UART_DATABUF[i+2]);
    UART_TARGET[1] = byte_decode(UART_DATABUF[i+3]);
    UART_TARGET[2] = byte_decode(UART_DATABUF[i+4]);
    UART_TARGET[3] = byte_decode(UART_DATABUF[i+5]);
    UART_TARGET[4] = byte_decode(UART_DATABUF[i+6]);
    UART_TARGET[5] = byte_decode(UART_DATABUF[i+7]);



    // Serial.println("---------");
    // Serial.println(UART_TARGET[0]);
    // Serial.println(UART_TARGET[1]);
    // Serial.println(UART_TARGET[2]);

    // Serial.println(UART_TARGET[3]);
    // Serial.println(UART_TARGET[4]);
    // Serial.println(UART_TARGET[5]);
    // Serial.println("---------");
    

    // delay(100000);
    // delay(2);
}

//反馈服务
bool respServices(bool flag,char* resp_code){
    int i = 0;
    //flag为false，则表明解码成功
    if(!flag){
        while (i < 2000)
        {      
            Serial2.write(resp_code,4);
            delay(2);
            i++;
            //中断服务加入特殊情况,如果收到openmv特殊请求,重新运行请求服务
        }

        Serial.println("respSer1");
        
        //返回的false跳出循环
        return false;
    }
    Serial.println("respSer2");
    
    return flag;
}


    
/*
//数据解码服务
void decode_b(int i){

    //  b @ &(+/-)xx &(+/-)yy $    #b@ &+01 &-19 $     b &(+/-)xx &(+/-)yy %xxx $     b %xxx $

    int code_count = 0;
    while (true){
        i++;

        //数据结尾退出 $==36
        if (UART_DATABUF[i] == 36){
            return;
        }

        // &==38
        if (UART_DATABUF[i] == 38){
            code_count++;
            continue;
        }   
        //"+/-" 43/45
        
        if (UART_DATABUF[i] == 43 || UART_DATABUF[i] == 45){
            //0~9
            if (UART_DATABUF[i+1] >= 47 && UART_DATABUF[i+1] <= 58){

                if (code_count == 1){
                    
                    UART_DELTA[0] = UART_DATABUF[i] == 43?byte_decode(UART_DATABUF[i+1]) * 10 + byte_decode(UART_DATABUF[i+2]):byte_decode(UART_DATABUF[i+1]) * -10 + -1 * byte_decode(UART_DATABUF[i+2]);
                    continue;
                }
                if(code_count == 2){
                    UART_DELTA[1] = UART_DATABUF[i] == 43?byte_decode(UART_DATABUF[i+1]) * 10 + byte_decode(UART_DATABUF[i+2]):byte_decode(UART_DATABUF[i+1]) * (-10) + -1 * byte_decode(UART_DATABUF[i+2]);
                    UART_DELTA[2] = 1;
                    continue;
                }

                
                
            }
            
        }


        //规划 % == 37
        if (UART_DATABUF[i] == 37){
            //0~9
            if (UART_DATABUF[i+1] > 48 && UART_DATABUF[i+1] < 52){
                //存放规划
                //规划为0,则无效
                UART_PLAN[0] = UART_DATABUF[i+1];
                UART_PLAN[1] = UART_DATABUF[i+2];
                UART_PLAN[2] = UART_DATABUF[i+3];
            }
            
        }



    }
}
*/

//数据解码，openmv控制机械臂
void decode_b(int i){

    // #bxxx$
    UART_PLAN[0] = byte_decode(UART_DATABUF[i+2]);
    UART_PLAN[1] = byte_decode(UART_DATABUF[i+3]);
    UART_PLAN[2] = byte_decode(UART_DATABUF[i+4]);
    Serial.println("local Saving plan.......");

}

//数字解码,返回数字
short byte_decode(byte UART_BYTE_DATA){
    switch (UART_BYTE_DATA)
    {
    case 0x30:
        return 0;
        break;
    case 0x31:
        return 1;
        break;
    case 0x32:
        return 2;
        break;
    case 0x33:
        return 3;
        break;
    case 0x34:
        return 4;
        break;
    case 0x35:
        return 5;
        break;
    case 0x36:
        return 6;
        break;
    case 0x37:
        return 7;
        break;
    case 0x38:
        return 8;
        break;

    case 0x39:
        return 9;
        break;
    }
}

// char* int_char(int x){
//     switch
// }


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
    if (spdA1 < targetSpd){
        spdA1+=1;
        spdA2+=1;
        spdB1+=1;
        spdB2+=1;
    }
    else{
        spdA1=spdA2=spdB1=spdB2 = targetSpd;
        MsTimer2::stop();
    }


}


void stpInt(){
    if(currentStates == stp){

    
    setSpdA1(0);
    setSpdA2(0);
    setSpdB1(0);
    setSpdB2(0);
    motorA1();
    motorA2();
    motorB1();
    motorB2();
    delay(2);

    motorA1PNS(S);
    motorA2PNS(S);
    motorB1PNS(S);
    motorB2PNS(S);

    setSpdA1(targetSpd);
    setSpdA2(targetSpd);
    setSpdB1(targetSpd);
    setSpdB2(targetSpd);
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

char* int_char(int A){
    char* p = (char*)malloc(sizeof(1));
    switch (A)
    {
    case 1:
        *p = "1";
        break;

    case 2:
        *p = "2";
        
        break;

    case 3:
        *p = "3";
        
        break;
    
    }

    return p;

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