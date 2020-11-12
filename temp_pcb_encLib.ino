// Adafruit SSD1306 - Version: 2.4.0
#include <Adafruit_SSD1306.h>
#include <splash.h>

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Wire.h>

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

float T_Peltier;
float T_Peltier_out;
int T_avg;
int T_avg_out;
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
const long Button_Debounce_ms = 900;
unsigned long NextButtonPress = 0;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int Interrupt = 2;
const int PinA = 8;
const int PinB = 9;
const int SW = 10;

static long rotaryCount = 0;



int start = 0;
int stop_temp = 0;

void Rotary_Check_ISR () {
  cli ();
  int Rotary = PINB & 0x03;

  if (Rotary == 2) {
    rotaryCount++;
  }
  else if (Rotary == 1) {
    rotaryCount--;
  }
  sei ();
}

void Display_Routine () {
  // routine for displaying text for temp/hum readout
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  if ((0 < T_Peltier) && (T_Peltier < 60)) {
    display.print("T: ");
    display.print(T_Peltier, 2); //T_avg
    display.println("C");
  }
  else {
    display.print("Insert NTC");
  }

  display.println();
  display.print("Set: ");
  display.print(rotaryCount);
  display.println("C");
  if (started == 1) {
    display.println("  Started  ");
  }
  else {
    display.println("  Stopped  ");
  }
  display.display();
}

void Read_Switch () {

  if ((digitalRead(SW) == HIGH) && SW_Pressed) {
    NextButtonPress = NextButtonPress + Button_Debounce_ms;
    SW_Pressed = 0;
  }
  //If we detect LOW signal, button is pressed
  else if ((digitalRead(SW) == LOW) && (millis() >= NextButtonPress)) {
    started = !started;
    SW_Pressed = 1;
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

  Display_Routine ();

}

void loop() {


  Read_Switch ();
  T_Peltier = Read_NTC (A0);

  Display_Routine ();






  //  if (start == 1) {
  //    display.println("  Started  ");
  //    if (rotaryCount > T_Peltier) { // hot T_Peltier 25gradusa hardcore
  //      digitalWrite(6, HIGH); // polarity
  //    }
  //
  //    if (rotaryCount < T_Peltier) { // cold T_Peltier 25gradusa hardcore
  //      digitalWrite(6, LOW); // polarity
  //    }
  //
  //  } else {
  //    display.println("  Stopped  ");
  //  }
  //
  //  // save the current state as the last state, for next time through the loop
  //  lastButtonState = buttonState;

}
