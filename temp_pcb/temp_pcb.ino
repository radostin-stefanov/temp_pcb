// Adafruit SSD1306 - Version: 2.4.0
#include <Adafruit_SSD1306.h>
#include <splash.h>

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Wire.h>

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define MIN_SET_TEMP 5
#define MAX_SET_TEMP 50


float T_Peltier;
float T_PCB;
int T_avg;
int avg_size = 10; // averaging size
int ThermistorPin = A0;
int ThermistorPinOut = A1;
int Vo;
float R1 = 10000;
float logR2, R2, T, Tc, Tf;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

// Variables will change:
//int buttonPushLow = 0;   // counter for the number of button presses
//int buttonPushHigh = 0;   // counter for the number of button presses
//int buttonState = 0;         // current state of the button
//int lastButtonState = 0;     // previous state of the button
bool started = 0;
bool SW_Pressed = 0;
const long Button_Debounce_ms = 300;
unsigned long NextButtonPress = 0;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int Interrupt = 2;
const int PinA = 8;
const int PinB = 9;
const int SW = 10;

const int P_Cool = 3;
const int P_EN_Cool = 4;
const int P_PWM = 5;
const int P_EN_Heat = 6;
const int P_Heat = 7;

const int NTC_Peltier = A0;
const int NTC_PCB = A1;
const int Peltier_Current = A2;

static long rotaryCount = MIN_SET_TEMP;

void Rotary_Check_ISR () {
  cli ();
  int Rotary = PINB & 0x03;

  if (Rotary == 2) {
    rotaryCount++;
    if (rotaryCount > MAX_SET_TEMP) {
      rotaryCount = MAX_SET_TEMP;
    }
  }
  else if (Rotary == 1) {
    rotaryCount--;
    if (rotaryCount < MIN_SET_TEMP) {
      rotaryCount = MIN_SET_TEMP;
    }
  }
  sei ();
}

void Display_Routine (float Display_Temp) {
  // routine for displaying text for temp/hum readout
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  if ((0 < Display_Temp) && (Display_Temp < 60)) {
    display.print("T: ");
    display.print(Display_Temp, 2); //T_avg
    display.print("C");
  }
  else {
    display.print("Insert NTC");
  }

  display.println();
  display.print("Set: ");
  display.print(rotaryCount);
  display.println("C");
  display.println();

  if (started == 1) {
    display.println("  Started  ");
  }
  else {
    display.println("  Stopped  ");
  }

  display.display();
}

void Read_Switch () {

  if ((digitalRead(SW) == LOW) && (SW_Pressed == 0)) {
    started = !started;
    SW_Pressed = 1;
    NextButtonPress = (millis() + Button_Debounce_ms);
  }
  else if ((digitalRead(SW) == HIGH) && (millis() > NextButtonPress)) {
    SW_Pressed = 0;
  }
}

float Read_NTC (int Analog_IN) {
  //head temp
  int NTC_Averaged;
  float T_sum = 0.0;
  for (int i = 0; i < avg_size; i++) {
    Vo = analogRead(Analog_IN);
    R2 = R1 * (1023.0 / (float)Vo - 1.0);
    logR2 = log(R2);
    T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
    Tc = T - 273.15;
    T_sum += Tc;
  }
  //   averaging values from loop
  T_sum = T_sum / float(avg_size);
  NTC_Averaged = round(T_sum);
  return T_sum;
}

void Peltier_Control () {

  if (started) {
    if (rotaryCount > 23) {
      digitalWrite (P_EN_Heat, HIGH);
      digitalWrite (P_Heat, HIGH);
      digitalWrite (P_Cool, LOW);
      digitalWrite (P_EN_Cool, LOW);
    }
    else {
      digitalWrite (P_EN_Cool, HIGH);
      digitalWrite (P_Cool, HIGH);
      digitalWrite (P_EN_Heat, LOW);
      digitalWrite (P_Heat, LOW);
    }
    analogWrite (P_PWM, 255);
  }
  else {
    digitalWrite (P_Cool, LOW);
    digitalWrite (P_EN_Cool, LOW);
    digitalWrite (P_EN_Heat, LOW);
    digitalWrite (P_Heat, LOW);
    analogWrite (P_PWM, 0);
  }
}

void setup() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    for (;;); // Don't proceed, loop forever
  }
  display.display();

  display.ssd1306_command(SSD1306_SETCONTRAST);  // 0x81 max contrast
  display.ssd1306_command(255);
  // Clear the buffer
  display.clearDisplay();

  pinMode(SW, INPUT_PULLUP);
  pinMode(PinA, INPUT_PULLUP);
  pinMode(PinB, INPUT_PULLUP);
  pinMode(Interrupt, INPUT_PULLUP);
  attachInterrupt (0, Rotary_Check_ISR, CHANGE);   // Interrupt 0 is pin 2, Interrupt 1 is pin 3

  pinMode (P_Cool, OUTPUT);
  pinMode (P_EN_Cool, OUTPUT);
  pinMode (P_PWM, OUTPUT);
  pinMode (P_EN_Heat, OUTPUT);
  pinMode (P_Heat, OUTPUT);

  digitalWrite (P_Cool, LOW);
  digitalWrite (P_EN_Cool, LOW);
  digitalWrite (P_PWM, LOW);
  digitalWrite (P_EN_Heat, LOW);
  digitalWrite (P_Heat, LOW);
  Display_Routine (T_Peltier);

}

void loop() {

  Read_Switch ();
  T_Peltier = Read_NTC (NTC_Peltier);
  T_PCB = Read_NTC (NTC_PCB);

  Peltier_Control ();

  Display_Routine (T_PCB);
}
