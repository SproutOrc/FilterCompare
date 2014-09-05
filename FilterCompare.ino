#include "I2Cdev.h"
#include <PID_v1.h>
#include <SelfHardwareMake.h>
#include "Timer.h"//时间操作系统头文件  本程序用作timeChange时间采集并处理一次数据

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

#define ACCEL 0x3B
#define GYRO  0x43
<<<<<<< HEAD
#define SAMPLE_TIME 10
#define SPEED 250.0
=======
double KP = 40;
double KI = 38;
double KD = 1;
double SET_POINT = 9;
#define SAMPLE_TIME 20
#define SPEED 110.0

>>>>>>> parent of 2b183cb... change PID

uint8_t ADR = 0x68;

uint8_t MAG_ADR = 0x0c;

uint8_t buffer[6]; 

SelfHardwareMake motion;

//时间类
Timer t;
//滤波法采样时间间隔毫秒
float timeChange=SAMPLE_TIME;
//注意：dt的取值为滤波器采样时间
float dt=timeChange*0.001;
// 陀螺仪
// 计算后的角度（与x轴夹角）和角速度
float angleAx,gyroGy;
// 陀螺仪原始数据 3个加速度+3个角速度
int16_t ax, ay, az, gx, gy, gz;
/////////////////////////////////////////////////
///////////////PID///////////////////////////////
unsigned long lastTime;
double Input, Output, Setpoint;
double errSum, lastErr;
int SampleTime = 1000; //1 sec

double kp = 40;
double ki = 38;
double kd = 0.9;

double KP = 40;
double KI = 38;
double KD = 0.8;

/////////////////////////////////////////////////

/////////////////////////////////////////////////
////////////////卡尔曼滤波参数与函数/////////////
float angle, angle_dot;//角度和角速度
float angle_0, angle_dot_0;//采集来的角度和角速度
// 一下为运算中间变量
float P[2][2] = {{ 1, 0 },
              { 0, 1 }};
float Pdot[4] ={ 0,0,0,0};
 //角度数据置信度,角速度数据置信度
float Q_angle=0.001, Q_gyro=0.003;
float R_angle=0.5 ,C_0 = 1; 
float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
/////////////////////////////////////////////////
///
void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    // 24: 400kHz I2C clock (200kHz if CPU is 8MHz)
    TWBR = 2;
    //TWBR = 12; 
    // 12;400kHz I2C clock (400kHz if CPU is 16MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
    #endif
    Serial1.begin(57600);
    uint8_t who_am_i;
    I2Cdev::readByte(ADR, 0x75, &who_am_i);
    I2Cdev::writeByte(ADR, 0x1b, 0x00);
    delay(1);
    I2Cdev::writeByte(ADR, 0x1c, 0x00);
    delay(1);
    I2Cdev::writeByte(ADR, 0x6b, 0x00);
    delay(1);
    I2Cdev::writeByte(ADR, 0x37, 0x02);
    I2Cdev::readByte(MAG_ADR,0x00,&who_am_i);
    delay(5);
    I2Cdev::writeByte(MAG_ADR, 0x0a, 0x00);
    delay(10);
    I2Cdev::writeByte(MAG_ADR, 0x0a, 0x16);
    delay(10); 
    //本语句执行以后timeChange毫秒执行回调函数getangle
    int tickEvent1=t.every(timeChange, getangle);
    //本语句执行以后50毫秒执行回调函数printout，串口输出
    int tickEvent2=t.every(50, printout);

    SetSampleTime(SAMPLE_TIME);
    SetTunings(KP, KI, KD);
    Setpoint = 8;
}   

void loop() {

    t.update();//时间操作系统运行
}
void printout()
{
    if (Serial1.available() >= 4) {
        KP = (double)Serial1.read();
        KI = (double)Serial1.read();
        double a = (double)Serial1.read();
        Setpoint = (double)Serial1.read();

        SetTunings(KP, KI, KD);
        Serial1.print("KP = ");
        Serial1.print(KP);
        Serial1.print("\tKI = ");
        Serial1.print(KI);
        Serial1.print("\tKD = ");
        Serial1.print(KD);
        Serial1.print("\tSetpoint  = ");
        Serial1.println(Setpoint);
    }
    // Serial1.print(angleAx);Serial1.print(',');
    // Serial1.print(angle);Serial1.print(',');
    // Serial1.print(angle2);Serial1.print(',');
    // // Serial.print(gx/131.00);Serial.print(',');
    // Serial1.println(angle1);//Serial.print(',');

//   Serial.println(Output);
}


void getangle() 
{
    I2Cdev::readBytes(ADR,ACCEL,6,buffer);
    ax = (((int16_t)buffer[0]) << 8) | buffer[1];
    ay = (((int16_t)buffer[2]) << 8) | buffer[3];
    az = (((int16_t)buffer[4]) << 8) | buffer[5];

    I2Cdev::readBytes(ADR,GYRO,6,buffer);
    gx = (((int16_t)buffer[0]) << 8) | buffer[1];
    gy = (((int16_t)buffer[2]) << 8) | buffer[2];
    gz = (((int16_t)buffer[4]) << 8) | buffer[5]; 

    angleAx=atan2(ay,az)*180/PI;//计算与x轴夹角
    gyroGy=gx/131.00;//计算角速度
    Kalman_Filter(angleAx,gyroGy);   //卡尔曼滤波

    if (abs(angle) > 40) {
        motion.stop();
        return;
    }
    Input = angle;
    Compute();

    if (Output > SPEED) {
        Output = SPEED;
    } else if (Output < -SPEED) {
        Output = -SPEED;
    }

    if (Output > 0) {
        motion.back((int)Output, (int)Output);
    } else if (Output < 0){
        motion.front((int)-Output, (int)-Output);
    } else {
        motion.stop();
    }
}

void Kalman_Filter(double angle_m,double gyro_m)
{
    angle+=(gyro_m-q_bias) * dt;
    angle_err = angle_m - angle;
    Pdot[0]=Q_angle - P[0][1] - P[1][0];
    Pdot[1]=- P[1][1];
    Pdot[2]=- P[1][1];
    Pdot[3]=Q_gyro;
    P[0][0] += Pdot[0] * dt;
    P[0][1] += Pdot[1] * dt;
    P[1][0] += Pdot[2] * dt;
    P[1][1] += Pdot[3] * dt;
    PCt_0 = C_0 * P[0][0];
    PCt_1 = C_0 * P[1][0];
    E = R_angle + C_0 * PCt_0;
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;
    t_0 = PCt_0;
    t_1 = C_0 * P[0][1];
    P[0][0] -= K_0 * t_0;
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;
    angle += K_0 * angle_err; //最优角度
    q_bias += K_1 * angle_err;
    angle_dot = gyro_m-q_bias;//最优角速度
}
