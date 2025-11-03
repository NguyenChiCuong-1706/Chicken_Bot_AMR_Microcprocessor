int channel_count = 0;
int pwm_range= PWM_Resolution;
boolean prt = false;
int res_min = 950, res_max = 2020;
int receiver_values[] = {0, 0, 0, 0, 0, 0};
int receiver_pins[] = {A0, A1, A2, A3, A4, A5};

void receive() 
{
  receiver_values[channel_count] = map(pulseIn (receiver_pins[channel_count], HIGH), res_min, res_max, -pwm_range, pwm_range);
  channel_count++;
  if (channel_count == 6){
    channel_count = 0;
  }
  boolean activevalues = true;
  for (int i = 0; i < 6; i++) {
    if (prt) {
      Serial.print("CH"); Serial.print(i); Serial.print(" : ");
      Serial.print(receiver_values[i]); Serial.print(",\t");
    }
    if (receiver_values[i] < -500) {
      activevalues = false;
    }
  }
  ctrl_mode = 0;
  if (!activevalues) {
    ctrl_mode = -1;
  } 
  else if (receiver_values[4] > -100) {
    ctrl_mode = 1;
  } else {
    ctrl_mode = 2;
  }
  if (prt) {
    Serial.println("");
  }
}

void RF_read()
{
  int m1 = 0;
  int m2 = 0;
  int rot = receiver_values[0];

  if (ctrl_mode == 1) {
    m1 = receiver_values[1] / 2 + (rot)/1.5;
    m2 = receiver_values[1] / 2 - (rot)/1.5;

  } else if (ctrl_mode == 2) {
    m1 = 0;
    m2 = 0;
  }
  LEFT_desireSPD=m1;
  RIGHT_desireSPD=m2;

  if (abs(receiver_values[1])<20)
  resetPID(&LeftPID, &RightPID, Lspeed, Rspeed);
}