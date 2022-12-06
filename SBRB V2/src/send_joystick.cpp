#include <Arduino.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
//TFT Display setup
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
#define TFT_WIDTH 240
#define TFT_HEIGHT 135
uint16_t BG_COLOR  = ST77XX_BLACK;

//Joystick setup
#include "Adafruit_seesaw.h"

Adafruit_seesaw ss;

#define BUTTON_RIGHT 6
#define BUTTON_DOWN  7
#define BUTTON_LEFT  9
#define BUTTON_UP    10
#define BUTTON_SEL   14
uint32_t button_mask = (1 << BUTTON_RIGHT) | (1 << BUTTON_DOWN) | 
                (1 << BUTTON_LEFT) | (1 << BUTTON_UP) | (1 << BUTTON_SEL);
//ESP-NOW SETUP
#include <esp_now.h>
#include <WiFi.h>
uint8_t broadcastAddress[] = {0xF4, 0x12, 0xFA, 0x5A, 0x1F, 0xC8};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  uint16_t joyX;
  uint16_t joyY;
  bool rightPressed;
  bool downPressed;
  bool leftPressed;
  bool upPressed;
  bool selPressed;
} struct_message;

struct_message joyData;

esp_now_peer_info_t peerInfo;
// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {

    bool success = status == ESP_NOW_SEND_SUCCESS ;
    
    if (success){
        BG_COLOR = ST77XX_GREEN;
    } else {
        BG_COLOR = ST77XX_RED;
    }     

}

bool readJoystick();
void printJoyXYText();
void drawJoyXYCircle(uint16_t joyX, uint16_t joyY, uint8_t radius);
void sendJoystick();
//Joystick tracking variables
int last_x = 0, last_y = 0;
bool rightPressed = false;
bool downPressed = false;
bool leftPressed = false;
bool upPressed = false;
bool selPressed = false;

void setup(void){
    //ESP_NOW Setup
    // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // ESP-NOW Setup Complete

    //////Initialize TFT Display
    // turn on backlight
    pinMode(TFT_BACKLITE, OUTPUT);
    digitalWrite(TFT_BACKLITE, HIGH);
    // turn on the TFT / I2C power supply
    pinMode(TFT_I2C_POWER, OUTPUT);
    digitalWrite(TFT_I2C_POWER, HIGH);
    delay(10);
    // initialize TFT
    tft.init(135, 240); // Init ST7789 240x135
    tft.setRotation(3);
    tft.fillScreen(ST77XX_BLACK);
    /////TFT Initialized

    //Initialize Joystick
    ss.begin(0x49);
    ss.pinModeBulk(button_mask, INPUT_PULLUP);
    ss.setGPIOInterrupts(button_mask, 1);    


}

long sendDataDelay = 100; //Millis
long lastSendData = 0;

void loop(){
    //print the new reading to display if we get it 
    if (readJoystick()){
        drawJoyXYCircle(last_x, last_y, 6);
        sendJoystick();
    }
    // //send data every set amount of time
    // if (millis()-lastSendData > sendDataDelay){       
    //     lastSendData = millis();
    // }
}

void sendJoystick(){
    // Set values to send
  joyData.joyX = last_x;
  joyData.joyY = last_y;
  joyData.rightPressed = rightPressed;
  joyData.downPressed = downPressed;
  joyData.leftPressed = leftPressed;
  joyData.upPressed = upPressed;
  joyData.selPressed = selPressed;
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &joyData, sizeof(joyData));
}
uint16_t tft_x = 0;
uint16_t tft_y = 0;
uint16_t last_tft_x = 0;
uint16_t last_tft_y = 0;
//draws a circle representing the position of the joystick
void drawJoyXYCircle(uint16_t joyX, uint16_t joyY, uint8_t radius){
        tft_x = map(joyY, 0, 1023, 0, TFT_WIDTH/3);
        tft_y = map(joyX, 0, 1023, 0, TFT_HEIGHT);
        //remove the previous circle
        tft.fillCircle(last_tft_x, last_tft_y, radius , BG_COLOR);
        //plot the latest
        tft.fillCircle(tft_x, tft_y, radius, ST77XX_BLUE);
        //store the last circle
        last_tft_x = tft_x;
        last_tft_y = tft_y;
}
//prints the text of the X Y position on the screen
void printJoyXYText(){
        //empty section of screen
        tft.fillRect(0,0,120,60, ST77XX_BLACK);
        //Configure text       
        tft.setTextColor(ST77XX_CYAN);
        tft.setTextSize(3);
        //print joystick readings
        tft.setCursor(0,0);
        //print joystick x value
        tft.printf("X:%d", last_x);
        tft.println();

        //print joystick y value
        tft.setTextColor(ST77XX_GREEN);
        tft.printf("Y:%d", last_y);
        tft.println();
        //print button readings
}

//Reads the current joystick values and updates the tracking variables
//Returns true if the values have changed
bool readJoystick(){
    //Assume no reading has occured
    bool newReading = false;
    //get newest reading from the joystick
    int x = ss.analogRead(2);
    int y = ss.analogRead(3);
    //If it moved, update the position of the joystick
    if ( (abs(x - last_x) > 3)  ||  (abs(y - last_y) > 3)) {
        newReading = true;
        last_x = x;
        last_y = y;
    }    
    uint32_t buttons = ss.digitalReadBulk(button_mask);
    rightPressed = !(buttons & (1 << BUTTON_RIGHT));
    downPressed = !(buttons & (1 << BUTTON_DOWN));
    leftPressed = !(buttons & (1 << BUTTON_LEFT));
    upPressed = !(buttons & (1 << BUTTON_UP));
    selPressed = ! (buttons & (1 << BUTTON_SEL));
    //If a button was pressed, show that there was a new reading
    if (rightPressed || downPressed || leftPressed || upPressed || selPressed){
        newReading = true;
    }
    return newReading;
}