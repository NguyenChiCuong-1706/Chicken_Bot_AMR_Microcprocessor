#ifdef MOTOR_184P
  #define EN_M1 10
  #define PWML1_M1 11
  #define PWML2_M1 12
  #define EN_M2 13
  #define PWML1_M2 14
  #define PWML2_M2 15
#endif

void motor1_ctrl(int pwm);
void motor2_ctrl(int pwm);
