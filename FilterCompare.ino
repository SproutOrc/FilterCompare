#include "I2Cdev.h"
#include "KalmanFilter.h"
//时间操作系统头文件  
//本程序用作timeChange时间采集并处理一次数据
#include "Timer.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

#define ACCEL 0x3B
#define GYRO  0x43

#define SAMPLE_TIME 10
#define SPEED 240.0
#define OFFSET_LEFT 0
#define OFFSET_RIGHT 0

uint8_t ADR = 0x68;

uint8_t buffer[6]; 

KalmanFilter kf(SAMPLE_TIME * 0.001);

//时间类
Timer t;


//******角度参数************

float Gyro_y;        //Y轴陀螺仪数据暂存
float Angle;         //小车最终倾斜角度

//******PWM参数*************

int   rightSpeed = 0;      //右电机转速
int   leftSpeed = 0;      //左电机转速

int   sendRSpeed = 0;
int   sendLSpeed = 0;
int   PWM_R;         //右轮PWM值计算
int   PWM_L;         //左轮PWM值计算
float PWM;           //综合PWM计算

//******电机参数*************

float speed_r_l;    //电机转速
float speed = 0;        //电机转速滤波
float position = 0;     //位移

char  turn_need = 0;
char  speed_need = 0;


static float Kp  = 40.0;       //PID参数
static float Kd  = 0.5;        //PID参数
static float Kpn = 0.07;      //PID参数
static float Ksp = 1.5;        //PID参数


//滤波法采样时间间隔毫秒
float timeChange=SAMPLE_TIME;
// //计算后的角度（与x轴夹角）和角速度
float angleAx,gyroGy;
//陀螺仪原始数据 3个加速度+3个角速度
int16_t ax, ay, az, gx, gy, gz;


void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 12;    // 24: 400kHz I2C clock (200kHz if CPU is 8MHz)
    //TWBR = 12; // 12; 400kHz I2C clock (400kHz if CPU is 16MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
    #endif

    Serial2.begin(57600);

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

    portInit();
}   

void loop() {
    //时间操作系统运行
    t.update();
}

int remote_char = 0;

void printout()
{
    if (Serial2.available() > 0) {
        remote_char = Serial2.read();

        if(remote_char ==0x02) speed_need = -20;       //前进
        else if(remote_char ==0x01) speed_need = 20;   //后退
        else speed_need = 0;                      //不动

        if(remote_char ==0x03) turn_need = 15;         //左转
        else if(remote_char ==0x04) turn_need = -15;   //右转
        else turn_need = 0;    
    }
    Serial2.print("angleAx = ");
    Serial2.print(angleAx);
    Serial2.print(',');

    Serial2.print("kfAngle = ");
    Serial2.print(kf.getAngle());
    Serial2.print(',');

    Serial2.print("rightSpeed = ");
    Serial2.print(float(sendRSpeed));
    Serial2.print(',');

    Serial2.print("leftSpeed = ");
    Serial2.print(float(sendLSpeed));
    Serial2.print(',');

    Serial2.print('\r');

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
    // 计算与x轴夹角
    angleAx = atan2(ax, -az) * 180.0 / PI;
    // 计算角速度
    gyroGy = gy / 131.00;

    kf.state_update(gyroGy);
    kf.kalman_update(angleAx);
    Angle = kf.getAngle();
    //Angle = Angle * 0.01745;
    Gyro_y = kf.getRate();
    Psn_Calcu();
    PWM_Calcu();
}

#define MRA 8
#define MRB 9
#define MREN 6

#define MLA 10
#define MLB 11
#define MLEN 7

#define SPEED_INT_R 3
#define SPEED_INT_L 2

#define SPEED_DIR_R 5

#define SPEED_DIR_L 4

#define MPU_INT 18

void portInit() {
    // left motor
    pinMode(MLA, OUTPUT);
    pinMode(MLB, OUTPUT);
    pinMode(MLEN, OUTPUT);

    // right motor
    pinMode(MRA, OUTPUT);
    pinMode(MRB, OUTPUT);
    pinMode(MREN, OUTPUT);

    attachInterrupt(1, setRightSpeed, RISING);
    attachInterrupt(0, setLeftSpeed, RISING);
}


void setRightSpeed() {
    if (digitalRead(SPEED_DIR_R)) {
        rightSpeed += 1;
    } else {
        rightSpeed -= 1;
    }
}


void setLeftSpeed() {
    if (digitalRead(SPEED_DIR_L)) {
        leftSpeed += 1;
    } else {
        leftSpeed -= 1;
    }
}


void motion(int rightPWM, int leftPWM) {
    if (rightPWM > 0) {
        digitalWrite(MRA, HIGH);
        digitalWrite(MRB, LOW);
        
    } else {
        digitalWrite(MRA, LOW);
        digitalWrite(MRB, HIGH);
        rightPWM = -rightPWM;
    }

    if (leftPWM > 0) {
        digitalWrite(MLA, HIGH);
        digitalWrite(MLB, LOW);
    } else {
        digitalWrite(MLA, LOW);
        digitalWrite(MLB, HIGH);
        leftPWM = -leftPWM;
    }

    rightPWM += OFFSET_RIGHT;
    leftPWM  += OFFSET_LEFT;

    if (rightPWM > SPEED) {
        rightPWM = SPEED;
    } 

    if (leftPWM > SPEED) {
        leftPWM = SPEED;
    }
    analogWrite(MREN, rightPWM);
    analogWrite(MLEN, leftPWM);
}

void stop() {
    analogWrite(MREN, 0);
    analogWrite(MLEN, 0);
}


void Psn_Calcu(void)     
{
    speed_r_l =(rightSpeed + leftSpeed)*0.5;
    sendRSpeed = rightSpeed;
    sendLSpeed = leftSpeed;

    leftSpeed = 0;
    rightSpeed = 0;

    speed *= 0.7;                         //车轮速度滤波
    speed += speed_r_l*0.3; 
    position += speed;                    //积分得到位移
    position += speed_need;
    if(position<-30000) position = -30000; 
    if(position> 30000) position =  30000; 
}



//*********************************************************
//电机PWM值计算
//*********************************************************

void PWM_Calcu(void)     
{
    
    if(Angle<-40||Angle>40)               //角度过大，关闭电机 
    {  
        stop();
        return;
    }
    PWM  = Kp*Angle + Kd*Gyro_y;          //PID：角速度和角度
    PWM += Kpn*position + Ksp*speed;      //PID：速度和位置
    PWM_R = PWM + turn_need;
    PWM_L = PWM - turn_need;
    motion(PWM_R,PWM_L); 
     
}