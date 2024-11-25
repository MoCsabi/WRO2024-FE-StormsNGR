#include <cmath>
#include <Wire.h> //I2C communication library
#include "Adafruit_BNO08x_RVC.h" //Gyroscope library

#define B_PIN_L 36 //Left motor encoder A pin
#define A_PIN_L 39 //Left motor encoder B pin
#define PWM_PIN_L 21 //Left motor PWM pin (speed control)
#define BACKWARD_PIN 14 //Backward pin (set to HIGH to drive motors backward)
#define FORWARD_PIN 12 //Forward pin (set to HIGH to drive motors forward)
#define A_PIN_R 35 //Right motor encoder A pin
#define B_PIN_R 34 //Right motor encoder B pin
#define PWM_PIN_R 16 //Right motor PWM pin (speed control)
#define LED_PIN 2 //Built-in Led on the ESP
#define S_PWM_PIN 27 //Servo PWM pin (turning)
#define SRL_TX_PIN 25 //Serial communication TX pin (with raspberry)
#define SRL_RX_PIN 26 //Serial communication RX pin (with raspberry)
#define IMU_RX_PIN 17 //Serial communication receive pin (gyroscope)
#define IMU_TX_PIN 32 //Serial communication transmit pin (gyroscope)
#define US_TRIGGER_PIN 18 //Ultrasonic sensor trigger pin (send soundwave)
#define US_ECHO_PIN 19 //Ultrasonic sensor echo pin (on measurement done)


#define spwm_freq 50 //servo PWM frequency
#define spwm_res 12 //servo PWM resolution (2^12)
// #define spwm_channel 0 //servo PWM channel
#define motor_pwm_freq 50 //motor PWM frequency

#define motor_pwm_res 12 //motor PWM resolution (2^12)
// #define leftm_pwm_channel 2 //left motor pwm channel !temporary
// #define rightm_pwm_channel 3 //right motor pwm channel !temporary

#define I2C_ADDRESS 0x8 //I2C address of this ESP device

//Commands: These are used in the communication between the ESP and the raspberry
#define CMD_SYNC 17 //Synchronization command (raspberry)
#define CMD_STEER  6 //Steering command
#define CMD_HEARTBEAT 7 //Heartbeat command signal
#define CMD_LOG 8 //(retrieve)Log command
#define CMD_SET_VMODE 9 //set vmode (velocity mode) command
#define CMD_SET_TARGETSPEED 10 //set target speed command
#define CMD_SET_SMODE 18 //set smode (steer mode) command
#define CMD_SET_BREAKPERCENT 19 //set breaking percentage command
#define CMD_SET_TARGET_YAW 20 //set target yaw (heading) command
#define CMD_SET_SERVO_MAX 21 //set the maximum servo state
#define CMD_SET_SERVO_MIN 22 //set the minimum servo state
#define CMD_SET_SERVO_CENT 23 //set the central servo state
#define CMD_SET_UNREG 25 //set unregulated motor power

#define CMD_DATA_POSL 11 //retrieve left motor position command
#define CMD_DATA_POSR 12 // retieve right motor position command
#define CMD_DATA_POSAVG 13 // retieve average motor position command
#define CMD_DATA_VMODE 14 // retrieve vmode command
#define CMD_DATA_SPEED 15 // retieve current speed command (real speed, not target)
#define CMD_DATA_ABS_GYRO 16 //retieve "absolute" gyro (does not loop back after 180 degrees to -180)
#define CMD_DATA_US 24 //retieve ultrasonic sensor measaured distance

#define US_DELAY 50 //Ultrasonic sensor delay (how much delay should be between sensor readings in milliseconds)

#define LED_C 15 //led PWM channel, not used

#define SYNC_CODE 18 //synchronization command response

#define STATUS_SYNCING 1 //ESP is trying to sync with raspberry pi
#define STATUS_CONNECTED 2 //ESP is successfully connected to raspberry pi

#define PiSerial Serial1

//VModes (velocity modes)
#define V_FORWARD 1 //robot is moving forward
#define V_BACKWARD -1 //robot is moving backward
#define V_STOP 0 //robot is stationary
#define V_BRAKE -2 //robot is braking (counter drive)
#define V_UNREGULATED 2 //the motors are powered with a constatnt force without speed control

//SModes (steer modes)
#define S_NONE 0 //no steering
#define S_GYRO 1 //gyro controlled steering (keeping the robot straight)

//PID constants for dribing the motors
#define kP 2
#define kI 0
#define kD 0

//PID constants for steering
#define kPY 5
#define kIY 0
#define kDY 0

int SERVO_MIN=250; //maximum safe left state servo PWM
int SERVO_MAX=415; //maximum safe right state servo PWM
int SERVO_CENT=325; //calibrated central state servo PWM

int conn_state=STATUS_SYNCING;
int heartBeatT0=0; //last heartbeat time

int posL=0;
int posR=0;
// int posAvg=0;
int lastTicks=0;
int speed=0;
int targetSpeed=0;
int brakePowerPercent=10;
int unregPower=0;

volatile int distance, duration, usStart, lastUSread; //ultrasonic sensor variables
volatile bool usSent=false; //is expecting an ultrasonic echo

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

int steerPercentage=0;

Adafruit_BNO08x_RVC rvc = Adafruit_BNO08x_RVC(); //gyro communication class in RVC mode
void setDrivingDirection(int dir){
  if(dir==1){
    digitalWrite(BACKWARD_PIN, LOW);
    digitalWrite(FORWARD_PIN, HIGH);
  } else{
    digitalWrite(BACKWARD_PIN, HIGH);
    digitalWrite(FORWARD_PIN, LOW);
  }
  
}
//Ultrasonic sensor distance calculating (on echo interrupt)
void echo(){
  if(usSent){
    duration = micros()-usStart;
    int newDistance = (duration*172);
    distance=(distance*8+newDistance*2)/10;
    usSent=false;
  }
}
//Ultrasonic sensor start measure
void sendUSPulse(){
	digitalWrite(US_TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIGGER_PIN, HIGH);
	delayMicroseconds(10);
	digitalWrite(US_TRIGGER_PIN, LOW);
  usStart=micros();
  usSent=true;
}
//timer interrupt, run exactly 100 times a second, handles real speed calulation, PID driving motor control and PID steering control
void IRAM_ATTR onTick(){
  // posAvg=(posL+posR)/2;
  speed=(speed*80+(((posR-lastTicks) * 100)*20))/100;
  lastTicks=posR;
  
  switch(sMode){
    case S_NONE:
    {
      int duty=-1;
      if(steerPercentage>0){
        duty=(SERVO_MAX-SERVO_CENT)*abs(steerPercentage)/100+SERVO_CENT;
      } else {
        duty=SERVO_CENT-(SERVO_CENT-SERVO_MIN)*abs(steerPercentage)/100;
      }
      if(duty>SERVO_MAX) {duty=SERVO_MAX;}
      if(duty<SERVO_MIN) {duty=SERVO_MIN;}
      ledcWrite(S_PWM_PIN,duty);
      break;
    }
    case S_GYRO:
      int yError=absYaw-targetYaw;
      integralY+=yError;
      int duty=SERVO_CENT-((kPY*yError+integralY*kIY+(yError-lastYError)*kDY)/10)*((vMode==V_BACKWARD)?-1:1);
      lastYError=yError;
      if(duty>SERVO_MAX) {duty=SERVO_MAX;}
      if(duty<SERVO_MIN) {duty=SERVO_MIN;}
      ledcWrite(S_PWM_PIN,duty);
  }
  switch(vMode){
    case V_UNREGULATED:
    {
      if(unregPower>0){
        setDrivingDirection(1);
      } else{
        setDrivingDirection(-1);
      }
      // ledcWrite(PWM_PIN_R, 1000);
      // ledcWrite(PWM_PIN_L, 1000);
      ledcWrite(PWM_PIN_R, int(4095/100*abs(unregPower)));
      break;
    }
    case V_STOP:
    {
      ledcWrite(PWM_PIN_L, 0);
      ledcWrite(PWM_PIN_R, 0);
      break;
    }
    case V_FORWARD:
    {
      // Serial.println("s, l ,r");
      // Serial.println(speed);
      // Serial.println(posL);
      // Serial.println(posR);
      int error=targetSpeed-speed;
      integral+=error;
      int PWM=kP*error+integral*kI+(error-lastError)*kD;
      lastError=error;
      PWM=std::max(-4095/10,PWM);
      PWM=std::min(4095,PWM);
      
      if(PWM<0){
        setDrivingDirection(-1);
      } else{
        setDrivingDirection(1);
      }
      ledcWrite(PWM_PIN_L, abs(PWM));
      ledcWrite(PWM_PIN_R, abs(PWM));
      // ledcWrite(PWM_PIN_L, 1000);
      // ledcWrite(PWM_PIN_R, 1000);
      break;
    }
    case V_BACKWARD:
    {
      int error=targetSpeed-speed;
      integral+=error;
      int PWM=(kP*error+integral*kI+(error-lastError)*kD);
      PWM*=-1;

      lastError=error;
      PWM=std::max(-4095/10,PWM);
      PWM=std::min(4095,PWM);
      if(PWM<0){
        setDrivingDirection(1);
      } else{
        setDrivingDirection(-1);
      }
      ledcWrite(PWM_PIN_L, abs(PWM));
      ledcWrite(PWM_PIN_R, abs(PWM));
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
      ledcWrite(PWM_PIN_L, brake);
      ledcWrite(PWM_PIN_R, brake);
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
  ledcAttach(PWM_PIN_L, motor_pwm_freq, motor_pwm_res);
  ledcAttach(PWM_PIN_R, motor_pwm_freq, motor_pwm_res);

  //timer interrupts
  hw_timer_t *timer=NULL;
  timer=timerBegin(10000);
  timerAttachInterrupt(timer, &onTick);
  timerAlarm(timer,100,true,0);

  //servo pwm
  ledcAttach(S_PWM_PIN, spwm_freq, spwm_res);
  ledcWrite(S_PWM_PIN, SERVO_CENT);

  //encoder
  pinMode(A_PIN_L, INPUT);
  pinMode(B_PIN_L, INPUT);
  pinMode(A_PIN_R, INPUT);
  pinMode(B_PIN_R, INPUT);
  attachInterrupt(digitalPinToInterrupt(A_PIN_L), readEncoder<'B','L'>, RISING);
  attachInterrupt(digitalPinToInterrupt(B_PIN_L), readEncoder<'A','L'>, RISING);
  attachInterrupt(digitalPinToInterrupt(A_PIN_R), readEncoder<'B','R'>, RISING);
  attachInterrupt(digitalPinToInterrupt(B_PIN_R), readEncoder<'A','R'>, RISING);
  //Ultrasonic sensor pins setup
  pinMode(US_TRIGGER_PIN, OUTPUT);
  pinMode(US_ECHO_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(US_ECHO_PIN), echo, FALLING); //function 'echo' gets called when measurement is done
  
  //led
  pinMode(LED_PIN,OUTPUT);
  // ledcAttachPin(LED_PIN, LED_C);
  // ledcSetup(LED_C, 4, 8);
  // setLedStatus(STATUS_SYNCING);
  
  //Serial communication (raspberry)
  PiSerial.setPins(SRL_RX_PIN,SRL_TX_PIN);
  PiSerial.begin(115200);

  //IMU (gyro)
  Serial2.setPins(IMU_RX_PIN, IMU_TX_PIN);  
  Serial2.begin(115200);
  rvc.begin(&Serial2);

  //I2C COMM (raspberry)
  // Wire.setPins(I2C_SDA_PIN,I2C_SCL_PIN);
  // Wire.begin(I2C_ADDRESS);
  // Wire.onReceive(onI2CReceive);
  // Wire.onRequest(onI2CRequest);
  
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
    case STATUS_SYNCING:
      digitalWrite(LED_PIN,LOW);
      break;
    case STATUS_CONNECTED:
      digitalWrite(LED_PIN,HIGH);
      break;
  }
}
//called if sync between ESP and raspberry is broken, immediately stops all motors and resets the servo
void disconnect(){
  Serial.println("in disconnect");
  vMode=V_STOP;
  steerPercentage=0;
  sMode=S_NONE;
  conn_state=STATUS_SYNCING;
  ledcWrite(PWM_PIN_L,0);
  ledcWrite(PWM_PIN_R,0);
  setLedStatus(STATUS_SYNCING);
}
//sends an int to the raspberry pi via serial
void sendInt(int data){
  for(int i=0;i<4;i++){
    PiSerial.write((data>>(8*i)) & 255);
  }
}
//reads an incoming int from the raspberry via serial
int readInt(){
  int num=0;
  Serial.println("in readInt");
  for(int i=0;i<3;i++){
    while (PiSerial.available() == 0);
    int b=PiSerial.read();
    num=((num>>8) | (b<<((3-1)*8)));
  }
  if (num>=(std::pow(2,(3*8-1)))){
    num-=std::pow(2,(3*8));
  }
  Serial.println(num);
  return num;
}

void steer(int percentage){
  steerPercentage=percentage;
}
//Send packet containing all data collected by th ESP to the Raspberry Pi via Serial protocol
void sendPacket() {
  PiSerial.write('E'); //1
  PiSerial.write('S'); //2
  PiSerial.write('P'); //3
  PiSerial.write(conn_state); //4
  sendInt(absYaw); //8
  sendInt(posL); //12
  sendInt(posR); //16
  PiSerial.write(vMode); //17
  PiSerial.write(sMode); //18
  sendInt(speed); //22
  sendInt(log_var); //26 bytes per packet
}
//main loop
void loop() {
  int t=millis();
  BNO08x_RVC_Data heading;
  log_var=t-heartBeatT0;
  //checks if there is new gyro data available, if yes, updates internal absyaw variable. Also prevents the gyro from looping around
  if (rvc.read(&heading)) {
    newYaw=heading.yaw*10; //data is in .1 degrees
    if(newYaw==0){
      newYaw=1;
    }
    if(newYaw==-1){
      newYaw=-2;
    }
    if(newYaw!=lastYaw){
      if(newYaw-lastYaw>1800) {yawOffset-=3600;}
      if(newYaw-lastYaw<-1800){yawOffset+=3600;}
      absYaw=newYaw+yawOffset;
      lastYaw=newYaw;
    }
    sendPacket();
  }
  if (PiSerial.available()){
    int command=PiSerial.read();  
    if (conn_state==STATUS_CONNECTED){
      switch(command){
        case CMD_STEER:
          steer(readInt());
          break;
        case CMD_SET_SERVO_CENT:
          SERVO_CENT=readInt();
          break;
        case CMD_SET_SERVO_MAX:
          SERVO_MAX=readInt();
          break;
        case CMD_SET_SERVO_MIN:
          SERVO_MIN=readInt();
          break;
        case CMD_HEARTBEAT:
          //resets last heartbeat 
          conn_state=STATUS_CONNECTED;
          setLedStatus(STATUS_CONNECTED);
          heartBeatT0=millis();
          break;
        case CMD_SET_VMODE:
        {
          Serial.println("set vmode");
          int data=readInt();
          //checks if vmode is valid
          if(data==V_FORWARD || data==V_BACKWARD || data==V_BRAKE || data==V_STOP || data==V_UNREGULATED){
            vMode=data;
          }
        }
          break;
        case CMD_SET_TARGETSPEED:
          targetSpeed=readInt();
          break;
        case CMD_SET_SMODE:
        {
          int data=readInt();
          Serial.println("smode");
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
        case CMD_SET_UNREG:
          Serial.println("set unreg");
          unregPower=readInt();
          break;
      }
    } else {
      if (command==CMD_HEARTBEAT){
        //resets last heartbeat 
          conn_state=STATUS_CONNECTED;
          setLedStatus(STATUS_CONNECTED);
          heartBeatT0=millis();
      }
    }
    
  }
  int hb=heartBeatT0;
  //Heartbeat detetction, if the raspberry hasn't sent a hearrtbeat command in .5 seconds it assumes the program on the pi may have crashed or stopped
  if((conn_state==STATUS_CONNECTED) && (t-hb)>500) {
    Serial.println("disconnect!");
    disconnect();
  }
  if(t-lastUSread>US_DELAY){
    sendUSPulse();
    lastUSread=t;
  }

}