

void PIDcal (struct PID *pid, float sp, float pv)
{
  float er     = sp-pv; 
  float p_er   = pid->pre_er;
  float preIterm = pid->preIterm;
  unsigned long preT = pid->prevT;

  unsigned long currT = micros();
  float dt = ((float) (currT - preT)) / 1.0e6; //seconds
  if(dt >= PIDrate)
  {  
    float Pterm  = (pid->Kp)*er;
    float Dterm  = (pid->Kd)*(er-p_er)/dt;
    float Iterm  = ((pid->Ki)*(er + p_er)*dt/2) + preIterm;

    int PIDOutput = (int) (Pterm + Iterm + Dterm);

    if(PIDOutput < PWM_Resolution && PIDOutput > -PWM_Resolution) 
    pid->preIterm = Iterm;
    if (PIDOutput>PWM_Resolution) PIDOutput=PWM_Resolution;
    else if (PIDOutput <-PWM_Resolution) PIDOutput=-PWM_Resolution;

    pid->pre_er = er;
    pid->output = PIDOutput;
  }
  pid->prevT = currT;
}


void resetPID(struct PID *Left, struct PID *Right, float Lspeed, float Rspeed)  
{
  if (Lspeed !=0 || Rspeed !=0) moving=1;
  else moving=0;

  float left = Left->pre_er;
  float right = Right->pre_er;
  if (!moving)     // Reset PIDs once robot not moving, to prevent startup spikes,
  {
    Serial.println("robot's not moving");
    if ( left != 0 || right != 0) 
    {
      Left->prevT = 0.0;
      Left->output = 0;
      Left->pre_er = 0;
      Left->preIterm = 0;

      Right->prevT = 0.0;
      Right->output = 0;
      Right->pre_er = 0;
      Right->preIterm = 0;
    }
    return;
  }
}
