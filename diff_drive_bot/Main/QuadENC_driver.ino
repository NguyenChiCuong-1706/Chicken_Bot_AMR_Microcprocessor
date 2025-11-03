long Pre_LEFT_PUL;
long Pre_RIGHT_PUL;
volatile long ENC_LEFT_PUL=0;
volatile long ENC_RIGHT_PUL=0;

unsigned long lastTime=0;
volatile int8_t OutL,OutR;

const int8_t QEM [16] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0};



void leftEncoderISR() 
{
  OldL = NewL;
  NewL = digitalReadFast (ENC_LEFT_A) * 2 + digitalReadFast (ENC_LEFT_B); 
  OutL = QEM [OldL * 4 + NewL];
  if (OutL==1) ENC_LEFT_PUL ++;
  if (OutL==-1) ENC_LEFT_PUL --;
}
void rightEncoderISR() 
{
  OldR = NewR;
  NewR = digitalReadFast (ENC_LEFT_A) * 2 + digitalReadFast (ENC_LEFT_B); 
  OutR = QEM [OldR * 4 + NewR];
  if (OutR==1) ENC_RIGHT_PUL ++;
  if (OutR==-1) ENC_RIGHT_PUL --;
}

long readPulse_LEFT() // micros
{
  long deltaPUL=0;
  unsigned long currentTime = micros();

  if (currentTime - lastTime >= Ts) {
    noInterrupts();
    deltaPUL = ENC_LEFT_PUL - Pre_LEFT_PUL;
    Pre_LEFT_PUL = deltaPUL;
    interrupts();
    lastTime = currentTime;
  }
  return deltaPUL;
}

long readPulse_RIGHT() // micros
{
  long deltaPUL=0;
  unsigned long currentTime = micros();

  if (currentTime - lastTime >= Ts) {
    noInterrupts();
    deltaPUL = ENC_RIGHT_PUL - Pre_RIGHT_PUL;
    Pre_RIGHT_PUL = deltaPUL;
    interrupts();
    lastTime = currentTime;
  }

  return deltaPUL;
}

float readSpeed_LEFT(float deltaPUL) 
{
  float RPTs = deltaPUL / PPR; // Round per frame
  float speed = (RPTs * 2 * PI) /1000; // rad/s
  return speed;
}

float readSpeed_RIGHT(float deltaPUL) 
{
  float RPTs = deltaPUL / PPR; // Round per frame
  float speed = (RPTs * 2 * PI) /1000; // rad/s
  return speed;
}

void resetEncoder(int i) 
{
  if (i == 1){
    ENC_LEFT_PUL=0L;
    return;} 
  else { 
    ENC_RIGHT_PUL=0L;
    return;}
}

void resetEnc() {
  resetEncoder(1);
  resetEncoder(2);
}
