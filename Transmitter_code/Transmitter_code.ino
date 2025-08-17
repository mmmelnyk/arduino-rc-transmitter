#include <SPI.h>
#include <nRF24L01.h>             //Download it here: https://www.electronoobs.com/eng_arduino_NRF24.php
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_GFX.h>         //Download it here: https://www.electronoobs.com/eng_arduino_Adafruit_GFX.php
#include <Adafruit_SSD1306.h>     //Download it here: https://www.electronoobs.com/eng_arduino_Adafruit_SSD1306.php
#include <EEPROM.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//OLED setup
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin (D4) (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define NUMFLAKES 5
#define XPOS 0
#define YPOS 0
#define DELTAY 2
#if (SSD1306_LCDHEIGHT != 32)
#error("Height incorrect, please fix this in the Adafruit_SSD1306.h!");
#endif

//IMPORTANT:
//Create a unique pipe out. The receiver has to wear the same unique code as in the receiver
const uint64_t pipeOut = 0xE8E8F0F0E1LL;

RF24 radio(9, 10); // select  CSN  pin

// The size of this struct should not exceed 32 bytes
// This gives us up to 32 8 bits channels
struct MyData {
  byte throttle;
  byte steer;
  byte pot1;
  byte pot2;
  byte AUX1;
  byte AUX2;
  byte AUX3;
  byte AUX4;
};
MyData data;

//Inputs outputs
int battery_in = A7;                  //pin for analog in from the battery divider
int buttons_analog_in = A6;   //Analog in from all the push buttons
int toggle_1 = 2;
int toggle_2 = 3;
int toggle_3 = 4;
int toggle_4 = 5;
int throttle_in = A0;
int steer_in = A1;
int pot1_in = A2;
int pot2_in = A3;
int mode_in = 6;
int buzzer = 7;

//Variables
float battery_level = 0;
int throttle_fine = 0;
int steer_fine = 0;
int pot1_fine = 0;
int pot2_fine = 0;
int button_read = 0;

int throttle_to_send = 0;
int steer_to_send = 0;
int pot1_to_send = 0;
int pot2_to_send = 0;

bool throttle_inverted = false;
bool steer_inverted = true;
bool pot1_inverted = true;
bool pot2_inverted = false;

bool steer_decrease = false;
bool throttle_decrease = false;
bool pot1_decrease = false;
bool pot2_decrease = false;

bool steer_increase = false;
bool throttle_increase = false;
bool pot1_increase = false;
bool pot2_increase = false;

bool mode = true;
bool mode_button_pressed = false;
bool sound = true;
int counter = 0;
int invert_counter = 0;
bool sound_changed = false;

void resetData()
{
  //This are the start values of each channal
  // Throttle is 0 in order to stop the motors
  //127 is the middle value of the 10ADC.

  data.throttle = 127;
  data.steer = 127;
  data.pot1 = 127;
  data.pot2 = 127;
  data.AUX1 = 0;
  data.AUX2 = 0;
  data.AUX3 = 0;
  data.AUX4 = 0;
}

void setup()
{
  Serial.begin(9600);
  
  if ( EEPROM.read(1) != 55)
  {
    EEPROM.write(2, 127);
    EEPROM.write(3, 127);
    EEPROM.write(4, 127);
    EEPROM.write(5, 127);
    EEPROM.write(6, 1);
    EEPROM.write(7, 0);
    EEPROM.write(8, 0);
    EEPROM.write(9, 1);
    EEPROM.write(1, 55);
  }

  throttle_fine = EEPROM.read(2);
  steer_fine = EEPROM.read(3);
  pot1_fine = EEPROM.read(4);
  pot2_fine = EEPROM.read(5);
  throttle_inverted = EEPROM.read(6);
  steer_inverted = EEPROM.read(7);
  pot1_inverted = EEPROM.read(8);
  pot2_inverted = EEPROM.read(9);

  pinMode(buttons_analog_in, INPUT);
  pinMode(mode_in, INPUT_PULLUP);
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW);

  pinMode(toggle_1, INPUT);
  pinMode(toggle_2, INPUT);
  pinMode(toggle_3, INPUT);
  pinMode(toggle_4, INPUT);


  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  delay(100);
  display.clearDisplay();
  display.setTextSize(1);             //Set text size
  display.setTextColor(WHITE);        //Choose color
  display.setCursor(0, 15);
  display.print("       Arduino");
  display.setCursor(0, 25);
  display.print("     Pistol-grip");
  display.setCursor(0, 35);
  display.print("     Transmitter");


  digitalWrite(buzzer, HIGH);
  delay(40);
  digitalWrite(buzzer, LOW);
  delay(40);
  digitalWrite(buzzer, HIGH);
  delay(40);
  digitalWrite(buzzer, LOW);

  display.display();
  delay(2000);

  //Start everything up
  radio.begin();
  radio.setAutoAck(false);
  
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(pipeOut);
  resetData();
}

/**************************************************/

int map_normal(int val, int lower, int middle, int upper, bool reverse)
{
  val = constrain(val, lower, upper);
  if ( val < middle )
    val = map(val, lower, middle, 0, 128);
  else
    val = map(val, middle, upper, 128, 255);
  return ( reverse ? 255 - val : val );
}

// Returns a corrected value for a joystick position that takes into account
// the values of the outer extents and the middle of the joystick range.
int map_exponential(int val, bool reverse)
{
  val = constrain(val, 0, 1023);
  float cube = ((pow((val - 512), 3) / 520200) + 258.012) / 2;
  return ( reverse ? 255 - cube : cube );
}

void loop()
{
  //battery read
  battery_level = analogRead(battery_in) / 201.993; //////Voltage divider is 10k and 20K so 1/3

  //Buttons read
  button_read = analogRead(buttons_analog_in);
  //Reset buttons
  if (button_read > 820)
  {
    steer_decrease = false;
    throttle_decrease = false;
    pot1_decrease = false;
    pot2_decrease = false;
    steer_increase = false;
    throttle_increase = false;
    pot1_increase = false;
    pot2_increase = false;
  }

  //THROTTLE buttons
  if (button_read < 260 && button_read > 200 && !throttle_decrease)
  {
    throttle_fine = throttle_fine + 1;
    throttle_decrease = true;
    EEPROM.write(3, throttle_fine);
    if (sound)
    {
      digitalWrite(buzzer, HIGH);
      delay(50);
      digitalWrite(buzzer, LOW);
    }
  }
  if (button_read < 120 && button_read > 50 && !throttle_increase)
  {
    throttle_fine = throttle_fine - 1;
    throttle_increase = true;
    EEPROM.write(3, throttle_fine);
    if (sound)
    {
      digitalWrite(buzzer, HIGH);
      delay(50);
      digitalWrite(buzzer, LOW);
    }
  }

  //STEERING buttons
  if (button_read < 500 && button_read > 430 && !steer_decrease)
  {
    steer_fine = steer_fine + 1;
    steer_decrease = true;
    EEPROM.write(2, steer_fine);
    if (sound)
    {
      digitalWrite(buzzer, HIGH);
      delay(50);
      digitalWrite(buzzer, LOW);
    }
  }
  if (button_read < 380 && button_read > 320 && !steer_increase)
  {
    steer_fine = steer_fine - 1;
    steer_increase = true;
    EEPROM.write(2, steer_fine);
    if (sound)
    {
      digitalWrite(buzzer, HIGH);
      delay(50);
      digitalWrite(buzzer, LOW);
    }
  }

  //POT1 buttons (Disabled)
  if (button_read < 610 && button_read > 550 && !pot1_decrease)
  {
    pot1_fine = pot1_fine + 1;
    pot1_decrease = true;
    EEPROM.write(4, pot1_fine);
    if (sound)
    {
      digitalWrite(buzzer, HIGH);
      delay(50);
      digitalWrite(buzzer, LOW);
    }
  }
  if (button_read < 690 && button_read > 630 && !pot1_increase)
  {
    pot1_fine = pot1_fine - 1;
    pot1_increase = true;
    EEPROM.write(4, pot1_fine);
    if (sound)
    {
      digitalWrite(buzzer, HIGH);
      delay(50);
      digitalWrite(buzzer, LOW);
    }
  }
  //////////////////////////////////////////////////////////////////////////////////////////

  //POT2 buttons (Disabled)
  if (button_read < 820 && button_read > 760 && !pot2_decrease)
  {
    pot2_fine = pot2_fine + 1;
    pot2_decrease = true;
    EEPROM.write(5, pot2_fine);
    if (sound)
    {
      digitalWrite(buzzer, HIGH);
      delay(50);
      digitalWrite(buzzer, LOW);
    }
  }
  if (button_read < 760 && button_read > 700 && !pot2_increase)
  {
    pot2_fine = pot2_fine - 1;
    pot2_increase = true;
    EEPROM.write(5, pot2_fine);
    if (sound)
    {
      digitalWrite(buzzer, HIGH);
      delay(50);
      digitalWrite(buzzer, LOW);
    }
  }

  //Mode select button
  if (!digitalRead(mode_in) && !mode_button_pressed)
  {
    mode = !mode;
    mode_button_pressed = true;
    if (sound)
    {
      digitalWrite(buzzer, HIGH);
      delay(50);
      digitalWrite(buzzer, LOW);
    }
  }

  if (!digitalRead(mode_in) && !sound_changed)
  {
    if (counter > 20)
    {
      sound = !sound;
      counter = 0;
      sound_changed = true;
      if (sound)
      {
        digitalWrite(buzzer, HIGH);
        delay(50);
        digitalWrite(buzzer, LOW);
      }
    }
    counter = counter + 1;
  }

  //Invert channels
  //STEER INVERT
  if (button_read < 500 && button_read > 430)
  {
    if (invert_counter > 30)
    {
      steer_inverted = !steer_inverted;
      invert_counter = 0;
      EEPROM.write(6, steer_inverted);
      display.clearDisplay();            //Clear the display
      display.setCursor(13, 30);           //Select where to print 124 x 64
      display.print("Steering Inverted");
      display.display();
      if (sound)
      {
        digitalWrite(buzzer, HIGH);
        delay(50);
        digitalWrite(buzzer, LOW);
      }
      delay(1500);
    }
    invert_counter = invert_counter + 1;
  }

  //THROTTLE INVERT
  if (button_read < 260 && button_read > 200)
  {
    if (invert_counter > 30)
    {
      throttle_inverted = !throttle_inverted;
      invert_counter = 0;
      EEPROM.write(7, throttle_inverted);
      display.clearDisplay();            //Clear the display
      display.setCursor(15, 30);           //Select where to print 124 x 64
      display.print("Throttle Inverted");
      display.display();
      if (sound)
      {
        digitalWrite(buzzer, HIGH);
        delay(50);
        digitalWrite(buzzer, LOW);
      }
      delay(1500);
    }
    invert_counter = invert_counter + 1;
  }

  //POT1 INVERT
  if (button_read < 610 && button_read > 550)
  {
    if (invert_counter > 30)
    {
      pot1_inverted = !pot1_inverted;
      invert_counter = 0;
      EEPROM.write(8, pot1_inverted);
      display.clearDisplay();            //Clear the display
      display.setCursor(13, 30);           //Select where to print 124 x 64
      display.print("  POT1 inverted");
      display.display();
      if (sound)
      {
        digitalWrite(buzzer, HIGH);
        delay(50);
        digitalWrite(buzzer, LOW);
      }
      delay(1500);
    }
    invert_counter = invert_counter + 1;
  }

  //POT2 INVERT
  if (button_read < 820 && button_read > 760)
  {
    if (invert_counter > 30)
    {
      pot2_inverted = !pot2_inverted;
      invert_counter = 0;
      EEPROM.write(9, pot2_inverted);
      display.clearDisplay();            //Clear the display
      display.setCursor(15, 30);           //Select where to print 124 x 64
      display.print("  POT2 inverted");
      display.display();
      if (sound)
      {
        digitalWrite(buzzer, HIGH);
        delay(50);
        digitalWrite(buzzer, LOW);
      }
      delay(1500);
    }
    invert_counter = invert_counter + 1;
  }

  if (digitalRead(mode_in) && mode_button_pressed)
  {
    mode_button_pressed = false;
    sound_changed = false;
    counter = 0;
    invert_counter = 0;
  }

  //Mode select
  if (!mode)
  {
    throttle_to_send = map_normal(analogRead(throttle_in), 0, 512, 1023, throttle_inverted);
    steer_to_send = map_normal(analogRead(steer_in), 0, 512, 1023,           steer_inverted);
    pot1_to_send = map_normal(analogRead(pot1_in), 0, 512, 1023,       pot1_inverted);
    pot2_to_send = map_normal(analogRead(pot2_in), 0, 512, 1023,         pot2_inverted);
  }

  if (mode)
  {
    throttle_to_send = map_exponential(analogRead(throttle_in), throttle_inverted);
    steer_to_send = map_exponential(analogRead(steer_in),           steer_inverted);
    pot1_to_send = map_exponential(analogRead(pot1_in),       pot1_inverted);
    pot2_to_send = map_exponential(analogRead(pot2_in),         pot2_inverted);
  }

  throttle_to_send = throttle_to_send  + throttle_fine - 111;
  steer_to_send = steer_to_send  + steer_fine - 127;
  pot1_to_send = pot1_to_send  + pot1_fine - 111;
  pot2_to_send = pot2_to_send  + pot2_fine - 166;


  data.throttle = constrain(throttle_to_send, 0, 255);
  data.steer      = constrain(steer_to_send, 0, 255);
  data.pot1    = constrain(pot1_to_send, 0, 255);
  data.pot2     = constrain(pot2_to_send, 0, 255);
  data.AUX1     = digitalRead(toggle_1);
  data.AUX2     = digitalRead(toggle_2);
  data.AUX3     = digitalRead(toggle_3);
  data.AUX4     = digitalRead(toggle_4);

  radio.write(&data, sizeof(MyData));


  display.clearDisplay();            //Clear the display
  if (sound)
  {
    display.setCursor(0, 0);           //Select where to print 124 x 64
    display.print("Sound ON");
  }
  if (!sound)
  {
    display.setCursor(0, 0);           //Select where to print 124 x 64
    display.print("Sound OFF");
  }
  display.setCursor(90, 0);           //Select where to print 124 x 64
  display.print(battery_level, 1);
  display.print("V");
  
  display.setCursor(0, 16);           //Select where to print 124 x 64
  display.print("POT1:");
  display.print(pot1_to_send);
  display.print("      POT2:");
  display.print(pot2_to_send);
  
  display.setCursor(0, 30);
  display.print("TH: ");
  display.print(throttle_to_send);
  display.print("      T1: ");
  display.print(digitalRead(toggle_2));
  display.print("|");
  display.print(digitalRead(toggle_1));
  //
  display.setCursor(0, 41);
  display.print("ST: ");
  display.print(steer_to_send);
  display.print("      T2: ");
  display.print(digitalRead(toggle_3));
  display.print("|");
  display.print(digitalRead(toggle_4));
  if (mode)
  {
    display.setCursor(0, 56);
    display.print("Mode: ");
    display.print("Exponential");
  }
  if (!mode)
  {
    display.setCursor(0, 56);
    display.print("Mode: ");
    display.print("Linear");
  }
  
  display.display();
}
