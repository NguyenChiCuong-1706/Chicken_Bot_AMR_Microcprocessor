#include "Arduino.h"
#include "Comms_driver.h"

#define USE_TENSSY
// #define USE_NANO

#define BAUDRATE 115200

#define Analog_Resolution 16 // Analog_Resolution config 8/16 bit

#if defined(Analog_Resolution) && ARDUINO == 16
  #define PWM_Resolution 65536
#else
  #define PWM_Resolution 65536
#endif

#ifdef USE_TENSSY // define library names

  #define MOTOR_184P
  #define QUAD_ENC
  #define PID_CTRL
  #define RADIO_FLYSKY_I6
  #define ODOM_CAL
  #define COMMANDS_H

#endif

// #include "commands.h" /* Include definition of serial commands */
 
#ifdef USE_TENSSY /* Include driver functions */

  #include "Motor_driver.h"
  #include "QuadENC_driver.h"
  #include "Controller.h"
  #include "Radio_driver.h"

  /* Declare necessary global varibales for drivers*/
  int LEFT_desireSPD=0;
  int RIGHT_desireSPD=0;

#endif


/* Global Variable initialization */

volatile int8_t OldL=0, NewL=0, OldR=0, NewR=0;
#define LEFT_RPM_CMD 100
#define RIGHT_RPM_CMD 100

struct PID{
  inline static unsigned long prevT=0;
  inline static float output;
  inline static float pre_er;
  inline static float preIterm;
  inline static float Kp = 10;
  inline static float Ki = 10;
  inline static float Kd = 10;
};

struct PID LeftPID;
struct PID RightPID;
long Lpulse, Rpulse;
float Lspeed, Rspeed;
unsigned char moving = 0; // is the base in motion?
#define AUTO_STOP_INTERVAL 2000
long lastMotorCommand = AUTO_STOP_INTERVAL;
int ctrl_mode = 0;

/* Global Variable initialization */



void setup() 
{
  Serial.begin(BAUDRATE);

  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);

  pinMode(EN_M1, OUTPUT);
  pinMode(EN_M2, OUTPUT);  

  pinMode(PWML1_M1, OUTPUT);
  pinMode(PWML2_M1, OUTPUT);

  pinMode(PWML1_M2, OUTPUT);
  pinMode(PWML2_M2, OUTPUT);

  analogWriteResolution(Analog_Resolution);
  
  resetEnc();
  // Initial state of encoder
  NewL = digitalReadFast(ENC_LEFT_A) * 2 + digitalReadFast(ENC_LEFT_B);
  NewR = digitalReadFast(ENC_RIGHT_A) * 2 + digitalReadFast(ENC_RIGHT_B);
  OldL = NewL;
  OldR = NewR;

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_B), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), rightEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_B), rightEncoderISR, CHANGE);
}



void loop()
{

  Lpulse = readPulse_LEFT();
  Rpulse = readPulse_RIGHT();
  Lspeed = readSpeed_LEFT(Lpulse);
  Rspeed = readSpeed_RIGHT(Rpulse);
  if(ctrl_mode == 0 || ctrl_mode ==2){
    readCommand();
    runCommand();
  }
  else{
    receive();
    RF_read();

    PIDcal (&LeftPID, LEFT_desireSPD, Lspeed);
    PIDcal (&RightPID, RIGHT_desireSPD, Rspeed);
    
    motor1_ctrl(LeftPID.output);
    motor2_ctrl(RightPID.output);

  }

}
