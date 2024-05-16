#include <cmath>
#include <Wire.h> //I2C communication library
#include "Adafruit_BNO08x_RVC.h" //Gyroscope library

#define A_PIN_L 36 //Left motor encoder A pin
#define B_PIN_L 39 //Left motor encoder B pin
#define PWM_PIN_L 21 //Left motor PWM pin (speed control)
#define BACKWARD_PIN 12 //Backward pin (set to HIGH to drive motors backward)
#define FORWARD_PIN 14 //Forward pin (set to HIGH to drive motors forward)
#define A_PIN_R 35 //Right motor encoder A pin
#define B_PIN_R 34 //Right motor encoder B pin
#define PWM_PIN_R 16 //Right motor PWM pin (speed control)
#define LED_PIN 2 //Built-in Led on the ESP
#define S_PWM_PIN 27 //Servo PWM pin (turning)
#define I2C_SDA_PIN 25 //I2C communication SDA pin (with raspberry)
#define I2C_SCL_PIN 26 //I2C communication SCL pin (with raspberry)
#define IMU_RX_PIN 17 //Serial communication receive pin (gyroscope)
#define IMU_TX_PIN 32 //Serial communication transmit pin (gyroscope)

#define SERVO_MIN 250 //maximum safe left state servo PWM
#define SERVO_MAX 415 //maximum safe right state servo PWM
#define SERVO_CENT 325 //calibrated central state servo PWM

#define spwm_freq 50 //servo PWM frequency
#define spwm_res 12 //servo PWM resolution (2^12)
#define spwm_channel 0 //servo PWM channel
#define motor_pwm_freq 50 //motor PWM frequency
#define motor_pwm_res 12 //motor PWM resolution (2^12)
#define leftm_pwm_channel 2 //left motor pwm channel !temporary
#define rightm_pwm_channel 3 //right motor pwm channel !temporary

#define I2C_ADDRESS 0x8 //I2C address of this ESP device

//Commands: These are used in the communication between the ESP and the raspberry
#define CMD_SYNC 17 //Synchronization command (raspberry)
#define CMD_STEER  6 //Steering command (not used)
#define CMD_HEARTBEAT 7 //Heartbeat command signal
#define CMD_LOG 8 //(retrieve)Log command
#define CMD_SET_VMODE 9 //set vmode (velocity mode) command
#define CMD_SET_TARGETSPEED 10 //set target speed command
#define CMD_SET_SMODE 18 //set smode (steer mode) command
#define CMD_SET_BREAKPERCENT 19 //set breaking percentage command
#define CMD_SET_TARGET_YAW 20 //set target yaw (heading) command

#define CMD_DATA_POSL 11 //retrieve left motor position command
#define CMD_DATA_POSR 12 // retieve right motor position command
#define CMD_DATA_POSAVG 13 // retieve average motor position command
#define CMD_DATA_VMODE 14 // retrieve vmode command
#define CMD_DATA_SPEED 15 // retieve current speed command (real speed, not target)
#define CMD_DATA_ABS_GYRO 16 //retieve "absolute" gyro (does not loop back after 180 degrees to -180)


#define LED_C 15 //led PWM channel, not used

#define SYNC_CODE 18 //synchronization command response

#define STATUS_SYNC 1 //ESP is trying to sync with raspberry pi
#define STATUS_CONNECTED 2 //ESP is successfully connected to raspberry pi

//VModes (velocity modes)
#define V_FORWARD 1 //robot is moving forward
#define V_BACKWARD -1 //robot is moving backward
#define V_STOP 0 //robot is stationary
#define V_BRAKE -2 //robot is brakeing (counter drive)

//SModes (steer modes)
#define S_NONE 0 //no steering
#define S_GYRO 1 //gyro controlled steering (keeping the robot straight)

//PID constants for dribing the motors
#define kP 10
#define kI 0
#define kD 0

//PID constants for steering
#define kPY 5
#define kIY 0
#define kDY 0

int conn_state=STATUS_SYNC;
int heartBeatT0=0; //last heartbeat time

int posL=0;
int posR=0;
int posAvg=0;
int lastAvg=0;
int speed=0;
int targetSpeed=0;
int brakePowerPercent=10;

//driving PID variables
int integral=0;
int lastError=0;

int vMode=V_STOP;
int sMode=S_NONE;

//steering PID variables
volatile int integralY=0;
volatile int lastYError=0;

//gyro variables
int lastYaw=0;
int yawOffset=0;
int newYaw=0;
volatile int absYaw=0;
volatile int targetYaw=0;
//retievable log variable
int log_var=0;
//stored response to i2c command
int i2cResponse=-1;

volatile int turnRatioL=1000;
volatile int turnRatioR=1000;

Adafruit_BNO08x_RVC rvc = Adafruit_BNO08x_RVC(); //gyro communication class in RVC mode

//timer interrupt, run exactly 100 times a second, handles real speed calulation, PID driving motor control and PID steering control
void IRAM_ATTR onTick(){
  posAvg=(posL+posR)/2;
  speed=(speed*7+(((posAvg-lastAvg) * 100)*3))/10;
  lastAvg=posAvg;
  switch(sMode){
    case S_NONE:
      break;
    case S_GYRO:
      int yError=absYaw-targetYaw;
      integralY+=yError;
      int duty=SERVO_CENT-((kPY*yError+integralY*kIY+(yError-lastYError)*kDY)/10)*((vMode==V_BACKWARD)?-1:1);
      lastYError=yError;
      if(duty>SERVO_MAX) {duty=SERVO_MAX;}
      if(duty<SERVO_MIN) {duty=SERVO_MIN;}
      ledcWrite(spwm_channel,duty);
  }
  switch(vMode){
    case V_STOP:
    {
      ledcWrite(leftm_pwm_channel, 0);
      ledcWrite(rightm_pwm_channel, 0);
      break;
    }
    case V_FORWARD:
    {
      digitalWrite(BACKWARD_PIN, LOW);
      digitalWrite(FORWARD_PIN, HIGH);
      int error=targetSpeed-speed;
      integral+=error;
      int PWM=kP*error+integral*kI+(error-lastError)*kD;
      lastError=error;
      PWM=std::max(0,PWM);
      PWM=std::min(4095,PWM);
      ledcWrite(leftm_pwm_channel, PWM);
      ledcWrite(rightm_pwm_channel, PWM);
      break;
    }
    case V_BACKWARD:
    {
      digitalWrite(BACKWARD_PIN, HIGH);
      digitalWrite(FORWARD_PIN, LOW);
      int error=targetSpeed-speed;
      integral+=error;
      int PWM=(kP*error+integral*kI+(error-lastError)*kD);
      PWM*=-1;

      lastError=error;
      PWM=std::max(-4095*10/100,PWM);
      PWM=std::min(4095,PWM);
      ledcWrite(leftm_pwm_channel, PWM);
      ledcWrite(rightm_pwm_channel, PWM);
      break;
    }
    case V_BRAKE:
    {
      if(abs(speed)<100){
        vMode=V_STOP;
        break;
      }
      if(speed<0){
        digitalWrite(BACKWARD_PIN, LOW);
        digitalWrite(FORWARD_PIN, HIGH);
        
      } else {
        digitalWrite(BACKWARD_PIN, HIGH);
        digitalWrite(FORWARD_PIN, LOW);
      }
      int brake=brakePowerPercent*pow(2,motor_pwm_res)/100;
      ledcWrite(leftm_pwm_channel, brake);
      ledcWrite(rightm_pwm_channel, brake);
      break;
    }
  }
}

void setup() {
  // begin
  Serial.begin(115200);
  Serial.println("START");
  //EN A and EN B
  pinMode(FORWARD_PIN,OUTPUT);
  pinMode(BACKWARD_PIN,OUTPUT);

  digitalWrite(FORWARD_PIN, 0);
  digitalWrite(BACKWARD_PIN, 0);

  //driving PWM setup
  ledcSetup(leftm_pwm_channel, motor_pwm_freq, motor_pwm_res);
  ledcAttachPin(PWM_PIN_L, leftm_pwm_channel);
  ledcSetup(rightm_pwm_channel, motor_pwm_freq, motor_pwm_res);
  ledcAttachPin(PWM_PIN_R, rightm_pwm_channel);

  //timer interrupts
  hw_timer_t *timer=timerBegin(0,80,true);
  timerAttachInterrupt(timer,&onTick,true);
  timerAlarmWrite(timer,10000, true);
  timerAlarmEnable(timer);

  //servo pwm
  ledcSetup(spwm_channel, spwm_freq, spwm_res);
  ledcAttachPin(S_PWM_PIN, spwm_channel);
  ledcWrite(spwm_channel, SERVO_CENT);

  //encoder
  pinMode(A_PIN_L, INPUT);
  pinMode(B_PIN_L, INPUT);
  pinMode(A_PIN_R, INPUT);
  pinMode(B_PIN_R, INPUT);
  attachInterrupt(digitalPinToInterrupt(A_PIN_L), readEncoder<'B','L'>, RISING);
  attachInterrupt(digitalPinToInterrupt(B_PIN_L), readEncoder<'A','L'>, RISING);
  attachInterrupt(digitalPinToInterrupt(A_PIN_R), readEncoder<'B','R'>, RISING);
  attachInterrupt(digitalPinToInterrupt(B_PIN_R), readEncoder<'A','R'>, RISING);

  //led
  pinMode(LED_PIN,OUTPUT);
  // ledcAttachPin(LED_PIN, LED_C);
  // ledcSetup(LED_C, 4, 8);
  // setLedStatus(STATUS_SYNC);

  //IMU (gyro)
  Serial2.setPins(IMU_RX_PIN, IMU_TX_PIN);  
  Serial2.begin(115200);
  rvc.begin(&Serial2);

  //I2C COMM (raspberry)
  Wire.setPins(I2C_SDA_PIN,I2C_SCL_PIN);
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
  
}
//C++ template in order to avoid having to write 4 slightly different encoder pin detector methods. Keeps track of motors position.
template<char AorBToCheck,char side>
void readEncoder(){
  int pinToCheck=(AorBToCheck=='A' ? side=='L'? A_PIN_L : A_PIN_R  : side=='L' ? B_PIN_L : B_PIN_R);
  if(digitalRead(pinToCheck)==(AorBToCheck=='B')){
    side=='L' ? posL++ : posR--;
    
  } else {
    side=='L' ? posL-- : posR++;
    
  }
}
void setLedStatus(int status){
  switch(status){
    case STATUS_SYNC:
      digitalWrite(LED_PIN,LOW);
      break;
    case STATUS_CONNECTED:
      digitalWrite(LED_PIN,HIGH);
      break;
  }
}
//called if sync between ESP and raspberry is broken, immediately stops all motors and resets the servo
void disconnect(){
  vMode=V_STOP;
  sMode=S_NONE;
  conn_state=STATUS_SYNC;
  ledcWrite(spwm_channel,SERVO_CENT);
  ledcWrite(leftm_pwm_channel,0);
  ledcWrite(rightm_pwm_channel,0);
  setLedStatus(STATUS_SYNC);
}
//sends an into to the raspberry pi via i2c
void sendInt(int data, int len=4){
  Wire.write(len);
  for(int i=0;i<len;i++){
    // Serial.println("d");
    // Serial.println(((data>>(8*i)) & 255));
    Wire.write((data>>(8*i)) & 255);
  }

}
//reads an incoming int from the raspberry via i2c
int readInt(){
  int num=0;
  int len=Wire.read();
  Serial.println("len:");
  Serial.println(len);
  for(int i=0;i<len;i++){
    int b=Wire.read();
    Serial.print("b ");
    Serial.println(b);
    num=((num>>8) | (b<<((len-1)*8)));
    Serial.println(num);
  }
  Serial.println(num);
  if (num>=(std::pow(2,(len*8-1)))){
    num-=std::pow(2,(len*8));
  }
  return num;
}
//main loop
void loop() {
  BNO08x_RVC_Data heading;
  log_var=conn_state;
  //checks if there is new gyro data available, if yes, updates internal absyaw variable. Also prevents the gyro from looping around
  if (rvc.read(&heading)) {
    newYaw=heading.yaw*10; //data is in .1 degrees
    if(newYaw!=lastYaw){
      if(newYaw-lastYaw>1800) {yawOffset-=3600;}
      if(newYaw-lastYaw<-1800){yawOffset+=3600;}
      absYaw=newYaw+yawOffset;
      lastYaw=newYaw;
    }
  }
  int t=millis();
  int hb=heartBeatT0;
  //Heartbeat detetction, if the raspberry hasn't sent a hearrtbeat command in .2 seconds it assumes the program on the pi may have crashed or stopped
  if((conn_state==STATUS_CONNECTED) && (t-hb)>200) {
    Serial.println("dc");
    Serial.println(t);
    Serial.println(hb);
    Serial.println((t-hb));
    disconnect();
  }
}
//On command received from raspberry
void onI2CReceive(int byteCount){
  int command=Wire.read();
  // Serial.println("comm:");
  // Serial.println(command);
  switch(command){
      case CMD_STEER:
        break;
      case CMD_SYNC:
        // digitalWrite(LED_PIN,HIGH);
        if(conn_state!=STATUS_CONNECTED) {
          i2cResponse=SYNC_CODE;
          setLedStatus(STATUS_CONNECTED);
          conn_state=STATUS_CONNECTED;
          heartBeatT0=millis();
        }
          
        break;
      case CMD_HEARTBEAT:
        //resets last heartbeat time
        if(conn_state==STATUS_CONNECTED){
          heartBeatT0=millis();
        }
        break;
      case CMD_LOG:
        i2cResponse=readInt();
        // sendInt(i2cResponse);
        // Serial.print("rec: ");
        // Serial.println(i2cResponse);
        break;
      case CMD_SET_VMODE:
      {
        int data=readInt();
        //checks if vmode is valid
        if(data==V_FORWARD || data==V_BACKWARD || data==V_BRAKE || data==V_STOP){
          vMode=data;
        }
      }
        break;
      case CMD_SET_TARGETSPEED:
        targetSpeed=readInt();
        break;
      case CMD_DATA_ABS_GYRO:
        i2cResponse=absYaw;
        break;
      case CMD_DATA_POSAVG:
        i2cResponse=posAvg;
        // Serial.println("set rp");
        break;
      case CMD_DATA_POSL:
        i2cResponse=posL;
        break;
      case CMD_DATA_POSR:
        i2cResponse=posR;
        break;
      case CMD_DATA_SPEED:
        i2cResponse=speed;
        break;
      case CMD_DATA_VMODE:
        i2cResponse=vMode;
        break;
      case CMD_SET_SMODE:
      {
        int data=readInt();
        if(data==S_NONE || data==S_GYRO){
          sMode=data;
        }
      }
        break;
      case CMD_SET_BREAKPERCENT:
        brakePowerPercent=readInt();
        break;
      case CMD_SET_TARGET_YAW:
        targetYaw=readInt();
        break;
  }
}
//if the raspberry pi also expects a response via i2c it sends the previously set i2cResponse variable
void onI2CRequest(){
  sendInt(i2cResponse);
  // i2cResponse=-1;
}