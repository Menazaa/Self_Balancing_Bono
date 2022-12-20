/********** PID **********/
#include <PID_v1.h>

double kP = 150;
double kI = 300;
double kD = 10;

double setpoint, input, output;   // PID variables
PID pid(&input, &output, &setpoint, kP, kI, kD, DIRECT); // PID setup


/********** Line Follower **********/

char lineFollowerMode =0;

/********** Communication **********/ 

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>


const char *SSID = "Bono";
const char *PWD = "12345678";
 
// Web server running on port 80
WebServer server(80);
 
// JSON data buffer
StaticJsonDocument<250> jsonDocument;
char buffer[250];
 
// variables
int base_angle = 0;
int link_angle = 0;
int grabber_angle = 0;
float distance_M =0;
float distance_L =0;
float distance_R =0;
float x_pos =0; 
float y_pos =0; 

/********** Remote control **********/

float angleV = 0, turnV = 0; // values from remote

/********** Line Follower **********/

// const char IR1_Pin = 36;
// const char IR2_Pin = 39;


/********** L298N **********/
#include <L298N.h>

// Pin definition
const unsigned int EN_B = 32;
const unsigned int IN1_B = 25;
const unsigned int IN2_B = 33;

const unsigned int IN1_A = 26;
const unsigned int IN2_A = 27;
const unsigned int EN_A = 14;

// Create motor instances
L298N rightMotor(EN_A, IN1_A, IN2_A);
L298N leftMotor(EN_B, IN1_B, IN2_B);

// motor speeds
int speedLeft = 0;
int speedRight = 0;

/********** MPU **********/
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high


// INTERRUP PIN
const unsigned int MPU_INTERRUPT_PIN = 17;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[100]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float pitch;
long velocity;


int IMUdataReady = 0;
volatile bool mpuInterrupt = false;

/********** SETUP **********/
void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);

    // initialize device
    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(172);
    mpu.setYGyroOffset(28);
    mpu.setZGyroOffset(-23);
    mpu.setZAccelOffset(1342); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(MPU_INTERRUPT_PIN, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

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
    
    // PID Setup
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(-255,255);
    pid.SetSampleTime(10);
    
    // Communication Setup
   connectToWiFi();
   setup_routing();  

}

void loop() {

// Communication loop
 server.handleClient();


  if (IMUdataReady == 1) {
    readAngles();
  }

  pitch = -(ypr[1] * 180/M_PI); // adjust to degrees

  if (abs(turnV) < 15) { // turnV threshold
    turnV = 0;
  }

  if (abs(angleV) < .17) { // angleV threshold
    angleV = 0;
  }

  // PID vars
  setpoint =  angleV -0.5+y_pos ; 
  input = pitch;

  pid.Compute();

  // set motor speed with adjusted turn values
  speedLeft = output -  turnV;
  speedRight = output + turnV;
  
  if (pitch > 50 || pitch < -50) { // angle threshold
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(0); 
  } else {
    leftMotor.setSpeed(abs(speedLeft));
    rightMotor.setSpeed(abs(speedRight));
  }

  // move motors
  if (speedLeft < 0) { 
    leftMotor.forward();
  } else {
    leftMotor.backward();
  }
  
  if (speedRight < 0) { 
    rightMotor.forward();
  } else {
    rightMotor.backward();
  }

  // print some control info
  // Serial.print("pitch: ");
  // Serial.print(pitch);
  // Serial.print(" , angleV: ");
  // Serial.print(angleV);
  // Serial.print(" , turnV: ");
  // Serial.println(turnV);
  Serial.print(" , x_pos: ");
  Serial.println(x_pos);
  Serial.print(" , y_pos: ");
  Serial.println(y_pos);

}