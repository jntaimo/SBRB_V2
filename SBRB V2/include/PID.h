void motorSetup();
void updateController();
void updatePID();
void getSetPoint();
void motorPWM(int pwm_L, int pwm_R);
void runPID(float yaw, float pitch, float gyroPitchRate);
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);