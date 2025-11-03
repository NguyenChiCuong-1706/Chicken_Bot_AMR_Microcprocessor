
// float x=0, y=0, theta=0;   //Robot frame pose

// void Odometry(int16_t delta_left_pul, int16_t delta_right_pul)
// {

//   float d_left = 2*PI*R_wheel * (delta_left_pul/PPR);
//   float d_right = 2*PI*R_wheel * (delta_right_pul/PPR);

//   float d_center = (d_left + d_right) /2;
//   float phi = (d_right - d_left) / d_baseline;

//   // Small time steps, phi approximately zero
//   float theta_n = phi + theta;
//   float x_n = x + d_center*cos(theta);
//   float y_n = y + d_center*sin(theta);

//   x = x_n; y = y_n; theta = theta_n;

// }