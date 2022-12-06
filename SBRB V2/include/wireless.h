void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
void wirelessSetup(void);
void drawJoyXYCircle(uint16_t joyX, uint16_t joyY, uint8_t radius);
void printJoyXYText();
bool readJoystick();
float getY();
float getX();