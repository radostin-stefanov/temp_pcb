/*
    This sketch demonstrates how we can output a value in both channels of MCP4822 or MCP4812 or MCP4802.
*/


// A6 H-Bridge temp
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET     8 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

float T_approx;
float T_approx_out;
int T_avg;
int T_avg_out;
int avg_size = 10; // averaging size
int ThermistorPin = A0;
int ThermistorPinOut = A1;
int Vo;
float R1 = 10000;
float logR2, R2, T, Tc, Tf;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;


#define DAC_B_Disable 0x8000
#define MAX_Voltage_mV 4096
#define DAC_Voltage_Step_mV 16

// set pin 10 as the slave select for the digital pot:
const int slaveSelectPin = 10;
const int LDACPin = 9;

// Variables will change:
int buttonPushLow = 0;   // counter for the number of button presses
int buttonPushHigh = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

static int pinA = 2; // Our first hardware interrupt pin is digital pin 2
static int pinB = 3; // Our second hardware interrupt pin is digital pin 3
static int SW = 4;   //button
volatile byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile byte encoderPos = 0; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
volatile byte oldEncPos = 0; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
volatile byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent

float enc_tol = 0.1;
float enc_max = 0;
float enc_min = 0;

unsigned long lastButtonPress = 0;
int start = 0;
int stop_temp = 0;


void setup() {
  //  Serial.begin(9600);
  pinMode(slaveSelectPin, OUTPUT);
  pinMode(LDACPin, OUTPUT);
  //button init
  pinMode(pinA, INPUT_PULLUP); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pinB, INPUT_PULLUP); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  attachInterrupt(0, PinA, CHANGE); // set an interrupt on PinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
  attachInterrupt(1, PinB, CHANGE); // set an interrupt on PinB, looking for a rising edge signal and executing the "PinB" Interrupt Service Routine (below)
  pinMode(SW, INPUT_PULLUP);

  SPI.begin();
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    for (;;); // Don't proceed, loop forever
  }

  display.display();
  delay(1000); // Pause for 1 seconds

  display.ssd1306_command(SSD1306_SETCONTRAST);  // 0x81 max contrast
  display.ssd1306_command(255);
  // Clear the buffer
  display.clearDisplay();
  delay(1000); // Pause for 1 seconds

  DAC_Write (DAC_B_Disable);
}

void PinA() {
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; // read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos ++; //decrement the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00000100) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

void PinB() {
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; //read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos --; //increment the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00001000) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

void DAC_Write(int data) {
  // take the SS pin low to select the chip:
  digitalWrite(slaveSelectPin, LOW);
  delay(10);
  //  send in the address and value via SPI:
  //  SPI.transfer(highByte(data));
  //  SPI.transfer(lowByte(data));
  SPI.transfer16(data);
  Serial.println(data, BIN);
  delay(10);
  // take the SS pin high to de-select the chip:
  digitalWrite(slaveSelectPin, HIGH);

  digitalWrite(LDACPin, LOW);
  delay(10);
  digitalWrite(LDACPin, HIGH);

  return;
}

void WriteDAC_A (int Voltage) {
  if (Voltage > MAX_Voltage_mV) {
    Voltage = MAX_Voltage_mV;
  }
  Serial.print("Voltage Before modification: ");
  Serial.println(Voltage, DEC);
  Serial.println(Voltage, BIN);
  Voltage = Voltage / DAC_Voltage_Step_mV;
  Serial.print("Voltage After division: ");
  Serial.println(Voltage, DEC);
  Serial.println(Voltage, BIN);
  Voltage = Voltage << 4;
  Serial.print("Voltage After shift: ");
  Serial.println(Voltage, DEC);
  Serial.println(Voltage, BIN);
  bitClear(Voltage, 15); //Select channel A - write "0" to bit 15
  bitClear(Voltage, 14); //Bit not used, cleared just because - write "0" to bit 14
  bitClear(Voltage, 13); //Choose High gain - write "0" to bit 13
  bitSet(Voltage, 12); //Activate selected channel (A) - write "1" to bit 12

  Serial.print("Voltage After modification: ");
  Serial.println(Voltage, DEC);
  Serial.println(Voltage, BIN);
  DAC_Write(Voltage);

  return;
}

void loop() {
  // routine for displaying text for temp/hum readout
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("T: ");
  display.print(T_approx, 1); //T_avg
  display.println("C");
  display.println();
  display.print("Set: ");
  display.print(encoderPos);
  display.println("C");

  enc_min = encoderPos - enc_tol;
  enc_max = encoderPos + enc_tol;
  // Read the button state
  int btnState = digitalRead(SW);
  //If we detect LOW signal, button is pressed
  if (btnState == LOW) {
    //if 500 ms have passed since last LOW pulse, it means that the
    //button has been pressed, released and pressed again
    if (millis() - lastButtonPress > 500) {
      if (start != 1) {
        start = 1;
      } else {
        start = 0;
      }
    }
    // Remember last button press event
    lastButtonPress = millis();
  }

  //head temp
  float T_sum = 0.0;
  for (int i = 0; i < avg_size; i++) {
    Vo = analogRead(A0);
    R2 = R1 * (1023.0 / (float)Vo - 1.0);
    logR2 = log(R2);
    T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
    Tc = T - 273.15;
    T_sum += Tc;
  }
  // averaging values from loop
  T_approx = T_sum / float(avg_size);
  //  T_avg = round(T_approx);


  //  Serial.print(enc_min);
  //  Serial.print("   ");
  //  Serial.print(enc_max);
  //  Serial.print(" >  ");
  //  Serial.println(T_approx);

  if (start == 1) {
    display.println("  Started  ");
    if (enc_min > T_approx) { // hot T_approx 25gradusa hardcore
      digitalWrite(6, HIGH); // polarity
      WriteDAC_A (4090);
    }

    if (enc_max < T_approx) { // cold T_approx 25gradusa hardcore
      digitalWrite(6, LOW); // polarity
      WriteDAC_A (1000);
    }

  } else {
      WriteDAC_A (0);
    display.println("  Stopped  ");
  }

  // save the current state as the last state, for next time through the loop
  lastButtonState = buttonState;
  display.display();

  if (oldEncPos != encoderPos) {
    Serial.println(encoderPos);
    oldEncPos = encoderPos;
  }

  delay(100);
}
