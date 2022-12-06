#include <Arduino.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
//TFT Display setup
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

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

bool readJoystick();
//Joystick tracking variables
int last_x = 0, last_y = 0;
bool rightPressed = false;
bool downPressed = false;
bool leftPressed = false;
bool upPressed = false;
bool selPressed = false;

void setup(void){
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


void loop(){
    //print the new reading to display if we get it
    if (readJoystick()){
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