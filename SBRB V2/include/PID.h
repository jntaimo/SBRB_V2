void motorSetup();
void updateController();
void updatePID();
void getSetPoint();
void motorPWM(int pwm_L, int pwm_R);
void runPID(float yaw, float pitch, float gyroPitchRate);