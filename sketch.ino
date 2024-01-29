#include <LiquidCrystal_I2C.h>
#include "wifiControl.h"
LiquidCrystal_I2C lcd(0x27, 16, 2);
#define ENC_A 13
#define ENC_B 12
int pump_relay = 10;
unsigned long _lastIncReadTime = micros();
unsigned long _lastDecReadTime = micros();
int _pauseLength = 25000;
int _fastIncrement = 10;
int blocked = 9999;
int RotaryButtonPin = 5;
// longpress variables:
int lastState = LOW;  // the previous state from the input pin
int currentState;     // the current reading from the input pin
unsigned long pressedTime  = 0;
unsigned long releasedTime = 0;
const int button_longpress_time  = 1000; // 1000 milliseconds longpress
unsigned long time_now = 0;
volatile int counter = 0;
int manualMenu = false;
int autoMenu = false;
int wifiMenu = false;
//////////////////////////////////////////////////////
byte directionPin = 33;
byte stepPin = 32;
int numberOfSteps = 40;
int numberOfSteps2 = 100;
int pulseWidthMicros = 10;  // microsecondo
int millisbetweenSteps = 10; // milliseconds - or try 100 for slower steps
void setup() {
  pinMode(directionPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  // Set encoder pins and attach interrupts
  pinMode(RotaryButtonPin, INPUT_PULLUP);
  pinMode(pump_relay, OUTPUT);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), read_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), read_encoder, CHANGE);
  // Start the serial monitor to show output
  Serial.begin(115200);
  Serial.println("start");
  //boot
  lcd.init(); //100 kHz I2C speed - Standard Mode
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Pet Feeder v1.0");
  delay(2000);
  lcd.setCursor(0, 1);
  lcd.print(".");
  delay(500);
  lcd.print(".");
  delay(500);
  lcd.print(".");
  delay(500);
  lcd.print(".");
  delay(850);
  lcd.clear();
  lcd.print("Welkom!");
  delay(1500);
}
void loop() {
  static int Menu1 = 0;
  static int lastCounter = 0;
  ///////////////////////////////////////////////////////////////////////////////////////////////
  lcd.setCursor(0, 0);
  lcd.print("Select mode:    ");
  if (counter <= 1) {
    counter = 1;
  }
  if (counter >= 4) {
    counter = 4;
  }
  /////////////////////////////////////////////////////////////////////////////////////

  if (counter == 1) {
    manualMenu = true;
    autoMenu = false;
    wifiMenu = false;
    lcd.setCursor(0, 1);
    lcd.print("Manual ");
  }
  else if (counter == 2) {
    manualMenu = false;
    autoMenu = true;
    wifiMenu = false;
    lcd.setCursor(0, 1);
    lcd.print("Auto   ");
  }
  else if (counter == 3) {
    manualMenu = false;
    autoMenu = false;
    wifiMenu = true;
    lcd.setCursor(0, 1);
    lcd.print("WiFi   ");
  }
  else if (counter == 4) {
    manualMenu = false;
    autoMenu = false;
    wifiMenu = false;
    counter = 3;
  }
  ////////
  int buttonState = digitalRead(RotaryButtonPin);
  if (buttonState == LOW) {
    counter = 1;
    lcd.clear();
    /////////////////////////////////////////////////////////////////
    while (manualMenu == true) {
      lcd.setCursor(0, 0);
      lcd.print("Manual Control:");
      Serial.println(counter);
      if (counter <= 1) {
        counter = 1;
      }
      if (counter >= 3) {
        counter = 3;
      }
      if (counter == 1) {
        lcd.setCursor(0, 1);
        lcd.print("Food    ");
      }
      if (counter == 2) {
        lcd.setCursor(0, 1);
        lcd.print("Water   ");
      }
      if (counter == 3) {
        lcd.setCursor(0, 1);
        lcd.print("Return");
      }
      int buttonState = digitalRead(RotaryButtonPin);
      if (counter == 3 && buttonState == LOW) {
        manualMenu = false;
        autoMenu = false;
        wifiMenu = false;
        counter = 1;
        lcd.clear();
        delay(100);
      }
    }
    ////////////////////////////////////////////////////////////
    while (autoMenu == true) {
      lcd.setCursor(0, 0);
      lcd.print("Auto Control: ");
      Serial.println(counter);
      if (counter <= 1) {
        counter = 1;
      }
      if (counter >= 3) {
        counter = 3;
      }
      if (counter == 1) {
        lcd.setCursor(0, 1);
        lcd.print("Food    ");
      }
      if (counter == 2) {
        lcd.setCursor(0, 1);
        lcd.print("Water   ");
      }
      if (counter == 3) {
        lcd.setCursor(0, 1);
        lcd.print("Return");
      }
      int buttonState = digitalRead(RotaryButtonPin);
      if (counter == 3 && buttonState == LOW) {
        manualMenu = false;
        autoMenu = false;
        wifiMenu = false;
        counter = 1;
        lcd.clear();
        delay(100);
      }
    }
    //////////////////////////////////
    while (wifiMenu == true) {
      lcd.setCursor(0, 0);
      lcd.print("WiFi Control: ");
      Serial.println(counter);
      if (counter <= 1) {
        counter = 1;
      }
      if (counter >= 3) {
        counter = 3;
      }
      if (counter == 1) {
        lcd.setCursor(0, 1);
        lcd.print("status server");
      }
      if (counter == 2) {
        lcd.setCursor(0, 1);
        lcd.print("launch server");
      }
      if (counter == 3) {
        lcd.setCursor(0, 1);
        lcd.print("Return       ");
      }
      int buttonState = digitalRead(RotaryButtonPin);
      if (counter == 3 && buttonState == LOW) {
        manualMenu = false;
        autoMenu = false;
        wifiMenu = false;
        counter = 1;
        lcd.clear();
        delay(100);
      }
    }
  }
  if (counter != lastCounter) {
    Serial.println(counter);
    lastCounter = counter;
  }

}


////////////////////////////////////////////////////////////////COUNT////COUNT///COUNT/////////
// If count has changed print the new value to serial


void read_encoder() {
  // Encoder interrupt routine for both pins. Updates counter
  // if they are valid and have rotated a full indent

  static uint8_t old_AB = 3;  // Lookup table index
  static int8_t encval = 0;   // Encoder value
  static const int8_t enc_states[]  = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0}; // Lookup table

  old_AB <<= 2; // Remember previous state

  if (digitalRead(ENC_A)) old_AB |= 0x02; // Add current state of pin A
  if (digitalRead(ENC_B)) old_AB |= 0x01; // Add current state of pin B

  encval += enc_states[( old_AB & 0x0f )];

  // Update counter if encoder has rotated a full indent, that is at least 4 steps
  if ( encval > 3 ) {       // Four steps forward
    int changevalue = 1;
    if ((micros() - _lastIncReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue;
    }
    _lastIncReadTime = micros();
    counter = counter + changevalue;              // Update counter
    encval = 0;
  }
  else if ( encval < -3 ) {       // Four steps backward
    int changevalue = -1;
    if ((micros() - _lastDecReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue;
    }
    _lastDecReadTime = micros();
    counter = counter + changevalue;              // Update counter
    encval = 0;
  }
}
