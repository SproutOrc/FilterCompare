#include "I2Cdev.h"
#include "PID_v1.h"
#include "KalmanFilter.h"
#include <SelfHardwareMake.h>
//时间操作系统头文件  
//本程序用作timeChange时间采集并处理一次数据
#include "Timer.h"


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

#define ACCEL 0x3B
#define GYRO  0x43
double KP = 50.0;
double KI = 10;
double KD = 0.0;
#define SET_POINT 2.4
#define SAMPLE_TIME 10
#define SPEED 150.0
#define OFFSET_LEFT 10
#define OFFSET_RIGHT 20
#define SET_ERROR 0.5

uint8_t ADR = 0x68;

uint8_t buffer[6]; 

// Define Variables we'll be connecting to
double Setpoint, Input, Output;

// Specify the links and initial tuning parameters
PID MotorPID(&Input, &Output, &Setpoint,KP,KI,KD, DIRECT);

SelfHardwareMake motion;

KalmanFilter kf(SAMPLE_TIME * 0.001);

Timer t;//时间类



//滤波法采样时间间隔毫秒
float timeChange=SAMPLE_TIME;
//注意：dt的取值为滤波器采样时间
float dt=timeChange*0.001;
// 陀螺仪
// //计算后的角度（与x轴夹角）和角速度
float angleAx,gyroGy;
//陀螺仪原始数据 3个加速度+3个角速度
int16_t ax, ay, az, gx, gy, gz;


//卡尔曼滤波参数与函数
//角度和角速度
float angle, angle_dot;
//采集来的角度和角速度
float angle_0, angle_dot_0;
//float dt=20*0.001;//注意：dt的取值为kalman滤波器采样时间
//一下为运算中间变量
float P[2][2] = {{ 1, 0 },
              { 0, 1 }};
float Pdot[4] ={ 0,0,0,0};
//角度数据置信度,角速度数据置信度
float Q_angle=0.001, Q_gyro=0.005; 
float R_angle=0.5 ,C_0 = 1; 
float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 2;    // 24: 400kHz I2C clock (200kHz if CPU is 8MHz) //2014.01.10変えてみた．
    //TWBR = 12; // 12; 400kHz I2C clock (400kHz if CPU is 16MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
    #endif

    Serial1.begin(57600);

    I2Cdev::writeByte(ADR, 0x1b, 0x00);
    delay(1);

    I2Cdev::writeByte(ADR, 0x1c, 0x00);
    delay(1);

    I2Cdev::writeByte(ADR, 0x6b, 0x00);
    delay(1);

    I2Cdev::writeByte(ADR, 0x37, 0x02);

    //本语句执行以后timeChange毫秒执行回调函数getangle
    int tickEvent1=t.every(timeChange, getangle);
    //本语句执行以后50毫秒执行回调函数printout，串口输出
    int tickEvent2=t.every(50, printout);

    MotorPID.SetOutputLimits(-SPEED, SPEED);
    MotorPID.SetMode(AUTOMATIC);
    MotorPID.SetSampleTime(SAMPLE_TIME);
    MotorPID.SetTunings(KP, KI, KD);
    Setpoint = SET_POINT;
}   

void loop() {
    //时间操作系统运行
    t.update();
}

void printout()
{
    Serial1.print(Setpoint);Serial1.print(',');
    Serial1.print(angle);Serial1.print(',');
    Serial1.print(kf.getAngle());Serial1.print(',');
    Serial1.println(Setpoint);
}

int flag = 0;

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

    angleAx=atan2(ax,-az)*180/PI;//计算与x轴夹角
    gyroGy=gy/131.00;//计算角速度

    //Kalman_Filter(angleAx,gyroGy);
    kf.state_update(gyroGy);
    kf.kalman_update(angleAx);
    

    angle = kf.getAngle();
    if (abs(angle) > 40) {
        flag = 1;
        MotorPID.SetMode(MANUAL);
        Output = 0;
        motion.stop();
        
        return;
    }

    if (angle <= Setpoint + SET_ERROR 
     && angle >= Setpoint - SET_ERROR) {
        flag = 1;
        MotorPID.SetMode(MANUAL);
        Output = 0;
        motion.stop();
        return;
    } else if (flag == 1) {
        flag = 0;
        MotorPID.SetMode(AUTOMATIC);
    }

    Input = angle;

    MotorPID.Compute();

    if (Output > 0) {
        motion.front(OFFSET_LEFT + (int)Output, OFFSET_RIGHT + (int)Output);
    } else{
        motion.back(OFFSET_LEFT - (int)Output, OFFSET_RIGHT - (int)Output);
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
