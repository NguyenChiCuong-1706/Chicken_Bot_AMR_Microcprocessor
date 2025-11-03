#ifdef  QUAD_ENC
  #define ENC_LEFT_A 2
  #define ENC_LEFT_B 3
  #define ENC_RIGHT_A 4
  #define ENC_RIGHT_B 5
  #define Ts 10000 //micros
  #define PPR 9337.12 
  #define MAX_SPD 100 //rpm
#endif


void leftEncoderISR();
void rightEncoderISR();


long readPulse_LEFT(); // micros
long readPulse_RIGHT(); // micros

float readSpeed_LEFT(float deltaPUL) ;
float readSpeed_RIGHT(float deltaPUL) ;


void resetEncoder(int i);
void resetEnc();
