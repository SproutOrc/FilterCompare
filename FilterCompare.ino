#include "I2Cdev.h"
#include "KalmanFilter.h"
#include "Timer.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif
#include "Bluetooth.h"

#define ACCEL 0x3B
#define GYRO  0x43

#define SAMPLE_TIME 10

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


//滤波法采样时间间隔毫秒
float timeChange=SAMPLE_TIME;
// //计算后的角度（与x轴夹角）和角速度
float angleAx,gyroGy;
//陀螺仪原始数据 3个加速度+3个角速度
int16_t ax, ay, az, gx, gy, gz;

void getangle();
void printout();
void portInit();
void stop();
void AngleSabilityControl(
            float &angleControl, 
      const float &angle, 
      const float &gyro
);
void SpeedSabilityControl(
        float &nowSpeedControl, 
        float &lastSpeedControl, 
  const int &setSpeed,
  const int &leftSpeed,
  const int &rightSpeed,
  const bool &clearAll
);
void turnSabilityControl(
        float &turnSpeedControl,
  const int &setTurnSpeed,
  const int &leftSpeed,
  const int &rightSpeed,
  const bool &clearAll
);
void setRightSpeed();
void setLeftSpeed();
void motion(int leftPWM, int rightPWM);


void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 12;    // 24: 400kHz I2C clock (200kHz if CPU is 8MHz)
    //TWBR = 12; // 12; 400kHz I2C clock (400kHz if CPU is 16MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
    #endif
    //Bluetooth::setBluetoothBaud(Serial2, 115200);
    Serial2.begin(115200);

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


int walkSpeed = 0;
int turnSpeed = 0;
int setZero  = 0;
int setSpeed = 0;
int setTurnSpeed = 0;

void printout()
{
    if (Serial2.available() >= 3) {

        setZero  = Serial2.read();
        walkSpeed = Serial2.read();
        turnSpeed = Serial2.read();

        setSpeed = walkSpeed - setZero;
        setTurnSpeed  = turnSpeed - setZero;
    }
    Serial2.print("setPoint = ");
    Serial2.print(0.0);
    Serial2.print(',');

    Serial2.print("angleAx = ");
    Serial2.print(angleAx);
    Serial2.print(',');

    Serial2.print("kfAngle = ");
    Serial2.print(kf.getAngle());
    Serial2.print(',');

    Serial2.print("GYRO = ");
    Serial2.print(kf.getRate());
    Serial2.print(',');

    Serial2.print("rightSpeed = ");
    Serial2.print(float(sendRSpeed));
    Serial2.print(',');

    Serial2.print("leftSpeed = ");
    Serial2.println(float(sendLSpeed));

}

float angleControl;


float nowSpeedControl;
float lastSpeedControl;
float speedControl;
float turnSpeedControl;

float control;

bool clearAll = false;

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
    angleAx = -(atan2(ax, -az) * 180.0 / PI - 1.5);
    // 计算角速度
    gyroGy = -gy / 131.00;

    kf.state_update(gyroGy);
    kf.kalman_update(angleAx);
    Angle = kf.getAngle();
    //Angle = Angle * 0.01745;
    Gyro_y = kf.getRate();
    if(Angle<-50||Angle>50)
    {  
        clearAll = true;
        stop();
        return;
    }

    AngleSabilityControl(angleControl, Angle, Gyro_y);
    SpeedSabilityControl(
        nowSpeedControl, 
        lastSpeedControl, 
        setSpeed, 
        leftSpeed, 
        rightSpeed,
        clearAll
    );
    sendLSpeed = leftSpeed;
    sendRSpeed = rightSpeed;

    leftSpeed = 0;
    rightSpeed = 0;
    turnSabilityControl(
        turnSpeedControl,
        setTurnSpeed,
        sendLSpeed,
        sendRSpeed,
        clearAll
    );
    

    control = angleControl;
    control -= nowSpeedControl;

    PWM_L = control + turnSpeedControl;
    PWM_R = control - turnSpeedControl;

    motion(PWM_L, PWM_R);
    clearAll = false;
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

#define SPEED 230.0
#define OFFSET_RIGHT 0
#define OFFSET_LEFT 0
void motion(int leftPWM, int rightPWM) {
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

/**
 * 角度环控制
 */

#define ANGLE_OFFSET 0
#define GYRO_OFFSET 0
#define ANGLE_P 55.60
#define ANGLE_D 0.52
void AngleSabilityControl(
        float &angleControl, 
  const float &angle, 
  const float &gyro
) {
    float value;

    value = (ANGLE_OFFSET - angle) * ANGLE_P +
            (GYRO_OFFSET - gyro) *  ANGLE_D;

    angleControl = value;
}

/**
 * 速度环控制
 */

#define SPEED_CONSTANT 1
#define SPEED_P 0.11
#define SPEED_D 2.05

void SpeedSabilityControl(
        float &nowSpeedControl, 
        float &lastSpeedControl, 
  const int &setSpeed,
  const int &leftSpeed,
  const int &rightSpeed,
  const bool &clearAll
) {
    static float position = 0;
    static float sabilitySpeed = 0;
    static int sabilitySetSpeed = 0;
    float error;
    float pValue, dValue;

    if (clearAll) {
        position = 0;
        sabilitySpeed = 0;
        sabilitySetSpeed = 0;
    }
   
    float realSpeed = (leftSpeed + rightSpeed) / 2.0;
    realSpeed *= SPEED_CONSTANT;

    sabilitySpeed *= 0.85;
    sabilitySpeed += realSpeed * 0.15;

    sabilitySetSpeed = sabilitySetSpeed * 0.95 + setSpeed * 0.05;

    error = sabilitySetSpeed - sabilitySpeed;
    pValue = error * SPEED_P;
    dValue = error * SPEED_D;

    position += pValue;

    lastSpeedControl = nowSpeedControl;
    if (nowSpeedControl > 100) nowSpeedControl = 100;
    else if (nowSpeedControl < -100) nowSpeedControl = -100;
    nowSpeedControl  = dValue + position;
}

/**
 * 速度平滑控制
 */
#define SPEED_SCALE_MAX 10

void SpeedSmoothControl(
        float &speedControl,
  const float &nowSpeedControl,
  const float &lastSpeedControl
) {
    static int count = 1;
    float value;

    value = nowSpeedControl - lastSpeedControl;

    value = value * count / SPEED_SCALE_MAX + lastSpeedControl;
    count ++;
    if (count == SPEED_SCALE_MAX + 1) count = 1;
    speedControl = value;
}

#define TURN_P 0.01
#define TURN_D 0.02
void turnSabilityControl(
        float &turnSpeedControl,
  const int &setTurnSpeed,
  const int &leftSpeed,
  const int &rightSpeed,
  const bool &clearAll 
) {
    static float sabilitySetTurnSpeed = 0;
    static float sabilityTurnSpeed = 0;
    static float offsetPosition = 0;

    float errorSpeed;
    float error;
    float pValue, dValue;

    if (clearAll) {
        sabilitySetTurnSpeed = 0;
        sabilityTurnSpeed = 0;
        offsetPosition = 0;
    }

    errorSpeed = leftSpeed - rightSpeed;

    sabilityTurnSpeed = sabilityTurnSpeed * 0.85 + errorSpeed * 0.15;
    sabilitySetTurnSpeed = sabilitySetTurnSpeed * 0.85 + setTurnSpeed * 0.15;
    
    error = sabilitySetTurnSpeed - sabilityTurnSpeed;

    pValue = error * TURN_P;
    dValue = error * TURN_D;

    offsetPosition += pValue;

    turnSpeedControl = offsetPosition + dValue;
}