
#ifdef PID_CTRL
  // #define Kp 1
  // #define Ki 1
  // #define Kd 1
  #define PIDrate 10 //ms
#endif

void PIDcal (struct PID *pid, float sp, float pv);
void resetPID(struct PID *Left, struct PID *Right, float Lspeed, float Rspeed);
