#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <FreqCount.h>

//============================
//***********MOTORS********
//============================
#define Motor4IN1 4
#define Motor4IN2 11
#define Motor4EN 30

#define Motor3IN1 2
#define Motor3IN2 5
#define Motor3EN 32

#define Motor1IN1 7
#define Motor1IN2 6
#define Motor1EN 34

#define Motor2IN1 12
#define Motor2IN2 8
#define Motor2EN 36

int duration;
boolean moved = false;
unsigned long motionStarted;
//============================
//********LIne Detector******
//============================
int r = 48;
int c = 50;
int l = 52;

char state='n'; 
char laststate='n';
int actionRequest;


//============================
//*******METAL DETECTROR*****
//============================
#define buzz 28
#define COIL_THRES 70

unsigned long lastCount;

//============================
//***********IMU**************
//============================
MPU6050 mpu;

#define X_THRES 20
#define Y_THRES 20
#define YAW_THRES 1
#define NO_STEPS 1

#define CAL_WAIT 1000

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
//VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// uint8_t dataPacket[6] = { '$', 0x02, 0,0, 0,'#' };
//uint8_t dataPacket[5] = { '$',0, 0, 0,'#' };
uint8_t dataPacket[12] = {'$', 0x02, 0,0, 0,0,0,0,0,0,'\r','\n' };

bool calibrated = false;
bool firstCal = true;
unsigned long interval = 0;
float yaw = 0;
float roll = 0;
float pitch = 0;
uint16_t aX = 0;
uint16_t aY = 0;
uint16_t aZ = 0;

float deltaYaw;
uint16_t deltaX;
uint16_t deltaY;

unsigned int prevX = 0;
unsigned int prevY = 0;
unsigned int prevYaw = 0;

//============================
//*******ENCODER**************
//============================
#define Motor1Encoder_A 18
#define Motor1Encoder_B 26

#define PPR 420

volatile long M1BackwardCount=0;
volatile long M1ForwardCount=0;
unsigned long lastCountF = 0;
unsigned long lastCountB = 0;
unsigned long fSteps = 0;
unsigned long bSteps = 0;



volatile boolean mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


void setup() 
{
    pinMode(buzz, OUTPUT);
    pinMode(Motor1Encoder_A, INPUT);
    pinMode(Motor1Encoder_B, INPUT);

    attachInterrupt(5, Motor1Encoder, RISING);

    FreqCount.begin(200);
    
    
    pinMode(Motor1IN1, OUTPUT);
  pinMode(Motor1IN2, OUTPUT);
  pinMode(Motor1EN, OUTPUT);
  
  pinMode(Motor2IN1, OUTPUT);
  pinMode(Motor2IN2, OUTPUT);
  pinMode(Motor2EN, OUTPUT);
  
  pinMode(Motor3IN1, OUTPUT);
  pinMode(Motor3IN2, OUTPUT);
  pinMode(Motor3EN, OUTPUT);

  pinMode(Motor4IN1, OUTPUT);
  pinMode(Motor4IN2, OUTPUT);
  pinMode(Motor4EN, OUTPUT);

  digitalWrite(Motor1EN, HIGH);
  digitalWrite(Motor2EN, HIGH);
  digitalWrite(Motor3EN, HIGH);
  digitalWrite(Motor4EN, HIGH);
  
  
  pinMode(r, INPUT);
  pinMode(c, INPUT);
  pinMode(l, INPUT);
  
  
 
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

    Serial.begin(112500);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    
    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(4, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    duration = 3000;
}


void loop() 
{
    if (!calibrated)
    {
        if (firstCal)
        {
            interval = millis();
            firstCal=false;
        }
        if ((millis() - interval) > (CAL_WAIT))
        {
            calibrateSens();
            calibrated=true;
        }
        readValues();
    }
    else
    {
      readValues();
      boolean temp = processData();
      if (temp)
      {
        Serial.write(dataPacket, 12);
      }
      delay(50);

      recieveOrder();
    }
}


void readValues(){
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
    }
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        mpu.resetFIFO();
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
//        Serial.print("areal\t");
//        Serial.print(aaReal.x);
//        Serial.print("\t");
//        Serial.print(aaReal.y);
//        Serial.print("\t");
//        Serial.println(aaReal.z);
    }
}


//get order from the main program
void recieveOrder(){
char order = serial.read();
moveRobot(order,200);

}
boolean processData(){
  Serial.println("dd");
    boolean flag = false;
    deltaYaw = yaw - (ypr[0] * 180/M_PI); 
    deltaX = aX - aaReal.x; 
    deltaY = aY - aaReal.y;
    
    //adjust angle breb2

    if (true)
    {
      if(deltaYaw < 0){
        dataPacket[3] = 1;
        dataPacket[2] = (uint8_t) (-1*(deltaYaw));
        prevYaw = deltaYaw;
        flag = true;
      }
      else{
        dataPacket[3] = 0;
        dataPacket[2] = (uint8_t) deltaYaw;
        prevYaw = deltaYaw;
        flag = true;
      }
    }

    if ((deltaX - prevX) > X_THRES)
    {
        dataPacket[4] = (uint8_t)deltaX;
        prevX = deltaX;
        flag = true;
    }

    if ((deltaY - prevY) > Y_THRES)
    {
        dataPacket[5] = (uint8_t)deltaY;
        prevY = deltaY;
        flag = true;
    }
    
    if ((fSteps - lastCountF) >= NO_STEPS)
    {
        dataPacket[6] = (uint8_t)((fSteps - lastCountF));
        lastCountF = fSteps;
        flag = true;
    }
    else
    {
      dataPacket[6] = 0;
    }
    
    if ((bSteps - lastCountB) >= NO_STEPS)
    {
        dataPacket[7] = (uint8_t)((bSteps - lastCountB));
        lastCountB = bSteps;
        flag = true;
    }
    else{
      dataPacket[7] = 0;
    }
    //Serial.print("ssssss");
    //int mState =  isMine();
    dataPacket[8] = isMine();
//    Serial.print("mine");
    //Serial.println(dataPacket[8]);
    getAction();
    dataPacket[9] = actionRequest;
    
    Serial.println(dataPacket[9]);
    Serial.println("444");
    return flag;
}

void calibrateSens(){
    Serial.println("CALIBRATING.");
    for (int i = 0; i < 10; ++i)
    {
        for (int j = 0; j < 100; ++j)
        {
            readValues();
        
            yaw+=(ypr[0] * 180/M_PI);
            pitch+=(ypr[1] * 180/M_PI);
            roll+=(ypr[2] * 180/M_PI);
            aX+=aaReal.x;
            aY+=aaReal.y;
            aZ+=aaReal.z;
        }
        yaw/=100.0;
        pitch/=100.0;
        roll/=100.0;
        aX/=100;
        aY/=100;
        aZ/=100;
    }
    Serial.println("CALIBRATION COMPLETED.");
}

void Motor1Encoder() {
  if (digitalRead(Motor1Encoder_A) == HIGH) 
  {
    if (digitalRead(Motor1Encoder_B) == LOW) 
    {
      if (M1ForwardCount >= 420){
        M1ForwardCount=0;
        fSteps+=1;
      }
      else 
      {
        M1ForwardCount++;
      }
      M1BackwardCount=0;
    } 
    else 
    {
      if (M1BackwardCount >= 420){
        M1BackwardCount=0;
        bSteps+=1;
      }
      else 
      {
        M1BackwardCount++;
      }
      M1ForwardCount=0;
    }
  } 
  else 
  {
    if (digitalRead(Motor1Encoder_B) == LOW) 
    {
      if (M1BackwardCount >= 420){
        M1BackwardCount=0;
        bSteps+=1;
      }
      else 
      {
        M1BackwardCount++;
      }
      M1ForwardCount=0;
    } 
    else 
    {
      if (M1ForwardCount >= 420){
        M1ForwardCount=0;
        fSteps+=1;
      }
      else 
      {
        M1ForwardCount++;
      }
      M1BackwardCount=0;
    }
  }
}

boolean isMine(){
  int diff = 0;
  if (FreqCount.available()) 
  {
    unsigned long count = FreqCount.read();
    //Serial.println(count);
    diff = count - lastCount;
    diff = abs(diff);
    //Serial.println(diff);
    if(lastCount == 0){
      lastCount = count;
    }
    else if(diff > COIL_THRES)
    {
      return true;
    }
    else
    {
      return false;
    }
    //lastCount = count;
  }
}

void moveRobot(char dir, int pwm){
  switch (dir){
    case 'f':
      forward(pwm);
      break;
    case 'b':
      backward(pwm);
      break;
    case 'r':
      right(pwm);
      break;
    case 'l':
      left(pwm);
      break;
    case 's':
      Stop;
      break;
     
    default :
      Stop();
  }
  
  moved = true;
  motionStarted = millis();
}


void Stop(){
 
  analogWrite(Motor1IN1 , 0);
  analogWrite(Motor1IN2 , 0);
  analogWrite(Motor2IN1 , 0);
  analogWrite(Motor2IN2 , 0);
  analogWrite(Motor3IN1 , 0);
  analogWrite(Motor3IN2 , 0);
  analogWrite(Motor4IN1 , 0);
  analogWrite(Motor4IN2 , 0);
  }

  
void forward(int speed){

  
  analogWrite(Motor1IN1 , 0);
  analogWrite(Motor1IN2 , speed);
  analogWrite(Motor2IN1 , speed);
  analogWrite(Motor2IN2 , 0);
  analogWrite(Motor3IN1 , speed);
  analogWrite(Motor3IN2 , 0);
  analogWrite(Motor4IN1 , speed);
  analogWrite(Motor4IN2 , 0);
  }

void backward(int speed){
  
  analogWrite(Motor1IN1 , speed);
  analogWrite(Motor1IN2 , 0);
  analogWrite(Motor2IN1 , 0);
  analogWrite(Motor2IN2 , speed);
  analogWrite(Motor3IN1 , 0);
  analogWrite(Motor3IN2 , speed);
  analogWrite(Motor4IN1 , 0);
  analogWrite(Motor4IN2 , speed);
  
  }

void left(int speed){

  analogWrite(Motor1IN1 , 0);
  analogWrite(Motor1IN2 , speed);
  analogWrite(Motor2IN1 , 0);
  analogWrite(Motor2IN2 , speed);
  analogWrite(Motor3IN1 , 0);
  analogWrite(Motor3IN2 , speed);
  analogWrite(Motor4IN1 , speed);
  analogWrite(Motor4IN2 , 0);
  
  }

void right(int speed){

  analogWrite(Motor1IN1 , speed);
  analogWrite(Motor1IN2 , 0);
  analogWrite(Motor2IN1 , speed);
  analogWrite(Motor2IN2 , 0);
  analogWrite(Motor3IN1 , speed);
  analogWrite(Motor3IN2 , 0);
  analogWrite(Motor4IN1 , 0);
  analogWrite(Motor4IN2 , speed);
  
}

char getStatus(){
  
  int right = digitalRead(r);
  int center = digitalRead(c);
  int left = digitalRead(l);
  
  
  if((left==0) && (center==0) && (right==0)){
    state='u';
  }
  else if((left==0) && (center==0) && (right==1)){
    state='r';
  }
  else if((left==0) && (center==1) && (right==0)){
    state='f';
  }
  else if((left==0) && (center==1) && (right==1)){
    state='w';
  }
  else if((left==1) && (center==0) && (right==0)){
    state='l';
  }
  else if((left==1) && (center==1) && (right==0)){
    state='e';
  }
  
  else if((left==1) && (center==1) && (right==1)){
    state='i';
  }
  return state;
 }

void getAction(){
  
  if((state=='w') && (laststate=='u')){
    Serial.println("Starting");
    actionRequest = 0;
  }
  else if((state=='f') && (laststate=='w')){
    Serial.println("leaving left edge");
    actionRequest = 1;
  }
  
  else if((state=='w') && (laststate=='f')){
    Serial.println("arriving at left edge");
    actionRequest = 2;
    Stop();
  }
  
  else if((state=='f') && (laststate=='e')){
    Serial.println("leaving right edge");
    actionRequest = 3;
  }
  
  else if((state=='e') && (laststate=='f')){
    Serial.println("arriving at right edge");
    actionRequest = 4;
    Stop();
  }
  
  else if((state=='f') && (laststate=='i')){
    Serial.println("leaving intersection");
    actionRequest = 5;
  }
  
  else if((state=='i') && (laststate=='f')){
    Serial.println("arriving at intersection");
    actionRequest = 6;
    Stop();
  }
    else if((state=='r') && (laststate=='f')){
    fix_right();
  }
     else if((state=='r') && (laststate=='r')){
    fix_right();
  }
    else if((state=='l') && (laststate=='f')){
    fix_left();
  }
    else if((state=='l') && (laststate=='l')){
    fix_left();
  }
  
  else if((state=='u')){
    Serial.println("undefined");
    actionRequest = 7;
    //Stop();
  }
  else if((state==laststate)){
    Serial.println("no change");
    actionRequest = 8;
  }
  laststate=state;
}



 void fix_left(){

  analogWrite(Motor1IN1 , 0);
  analogWrite(Motor1IN2 , 100);
  analogWrite(Motor2IN1 , 0);
  analogWrite(Motor2IN2 , 0);
  analogWrite(Motor3IN1 , 0);
  analogWrite(Motor3IN2 , 0);
  analogWrite(Motor4IN1 , 100);
  analogWrite(Motor4IN2 , 0);
  delay(500);
  Stop();
  }


 void fix_right(){

  analogWrite(Motor1IN1 , 0);
  analogWrite(Motor1IN2 , 0);
  analogWrite(Motor2IN1 , 100);
  analogWrite(Motor2IN2 , 0);
  analogWrite(Motor3IN1 , 100);
  analogWrite(Motor3IN2 , 0);
  analogWrite(Motor4IN1 , 0);
  analogWrite(Motor4IN2 , 0);
  delay(500);
  Stop();
  }





