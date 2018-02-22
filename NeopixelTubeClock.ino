/* 
 *  NeopixelTubeClock
 *  
 *  Hardware
 *  - Arduino Nano Microcontroller
 *  - Real Time Clock DS1307 - TinyRTC
 *  - Illuminated Rotary Encoder RGB
 *  - Strip of 60 RGBW Neopixels (ADA2837)
 *  
 *  Software
 *  - RTClib    https://github.com/adafruit/RTClib
 *  - Neopixel  https://github.com/adafruit/Adafruit_NeoPixel
 *  - Standard Ardiono Wire library for I2C
 *  
 *  Connections
 *  
 *  Arduino Nano  | TinyRTC DS1307 
 *  --------------+------------------
 *  A4            | SDA
 *  A5            | SCL
 *  GND           | GND
 *  +5V           | VCC
 *  D2 (INT0)     | SQ
 */
#define RTC_SQ  (2)
 
 /*  
 *  Arduino Nano  | RGBW Neopixel Strip
 *  --------------+--------------------
 *  Din           | D12  
 */
#define NEO_DIN (12)

/* Used to initialise - how many Neopixels in the strip. */
#define NEO_MAX (60)
 
 /*  
 *  Used external power supply for +5V and GND
 *  
 *  Arduino Nano  | Illuminated Rotary Encoder RGB
 *  --------------+-------------------------------
 *  A0 (14)       | A
 *  A1 (15)       | B
 *  GND           | C
 *  D11           | R - Red
 *  D10           | G - Green
 *  D3 (INT1)     | SW - requires pull down resistor.
 *  D09           | B - Blue
 *  +5V           | +
 */
#define ENC_A  (14)
#define ENC_B  (15)

#define ENC_RED (11)
#define ENC_GRE (10)
#define ENC_SW  (3)
#define ENC_BLU (9)

 /*
  * Defines for indicating the Encoder's RGB LED 
  */
#define LED_OFF (0)
#define LED_ON  (1)

/*
 * Serial Monitor Debug instrumentation
 * 
 * comment out #define DEBUG_PRINT to disable.
 */
#define DEBUG_PRINT
#ifdef DEBUG_PRINT
#define PRINT_INIT(X) Serial.begin((X))
#define PRINT(X) Serial.print(X)
#define PRINTLN(X) Serial.println(X)
#else 
#define PRINT_INIT(X)
#define PRINT(X)
#define PRINTLN(X)
#endif

/* 
 *  Library Headers
 */
 
#include <Wire.h>
#include "RTClib.h"
#include "Adafruit_NeoPixel.h"

/* 
 *  Global Variables
 */

RTC_DS1307 rtc;
DateTime dt_now;   
DateTime dt_then;

Adafruit_NeoPixel np60 = Adafruit_NeoPixel(NEO_MAX, NEO_DIN, NEO_GRBW + NEO_KHZ800);

volatile byte rtc_sq_interrupt = HIGH; 
volatile byte enc_sw_interrupt = LOW;

/*
 * In case of fault, print to serial monitor, turn on the Nano LED and hang.
 */
void fault(const String& description) {
  PRINTLN(description);
  digitalWrite(LED_BUILTIN, HIGH);
  while(1);
}

/*
 * Interrupt Service Routines
 */
void rtc_sq_isr() {
  rtc_sq_interrupt = !rtc_sq_interrupt;
}

void enc_sw_isr() {
  enc_sw_interrupt = !enc_sw_interrupt;
}

/*
 * Set the Rotary Encoder RGB Led state
 * 
 * Invert the inputs as OFF means setting the output HIGH and ON means setting the output LOW.
 */
void encoder_rgb_led(byte red, byte green, byte blue) {
  digitalWrite(ENC_RED, (red) ? LOW : HIGH);
  digitalWrite(ENC_GRE, (green) ? LOW : HIGH);
  digitalWrite(ENC_BLU, (blue) ? LOW : HIGH);
}

byte previous_second = 0;
/*
 * Setup and intialisation
 */
void setup() {
  PRINT_INIT( 9600 );
  PRINTLN("Setup started.");

  // Arduino LED used to indicate a fault.
  pinMode(LED_BUILTIN, OUTPUT);

  // Setup and initialise the RTC
  PRINTLN(".. Initialising RTC.");
  
  if (!rtc.begin()) {
    fault("Couldn't initialise the RTC.");
  }

  // Configure an initial date time from the build time of this project.
  if (!rtc.isrunning()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Use the RTC Square Wave output as an interrupt for time display.
  rtc.writeSqwPinMode(SquareWave1HZ);  
  pinMode(RTC_SQ, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RTC_SQ), rtc_sq_isr, RISING);

  // Initial read of the DateTime from the RTC
  dt_now = rtc.now();
  dt_then = dt_now;

  // Setup inputs and outputs for the rotary encoder
  PRINTLN(".. Setting up Rotary Encoder");

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT);         // Requires a pulldown resistor
  pinMode(ENC_RED, OUTPUT);
  pinMode(ENC_BLU, OUTPUT);
  pinMode(ENC_GRE, OUTPUT);

  // Initially the Encoder LED are all off
  encoder_rgb_led(LED_OFF, LED_OFF, LED_OFF);

  // Initialise the Neopixel Strip
  PRINTLN(".. Initialising Neopixel Strip");

  np60.begin();
  for (int idx=0; idx<NEO_MAX; idx++) np60.setPixelColor(idx, 0, 0, 0, 0);
  np60.show();


  PRINTLN("Setup completed.");
}

void loop() {
  if (rtc_sq_interrupt) {
    rtc_sq_interrupt = LOW;

    dt_now = rtc.now();

    PRINT(dt_now.hour());
    PRINT(":"); 
    PRINT(dt_now.minute());
    PRINT(":"); 
    PRINTLN(dt_now.second());

    np60.setPixelColor(dt_now.second(), 0, 0, 0, 10);
    np60.setPixelColor(previous_second, 0, 0, 0);
    np60.show();

    previous_second = dt_now.second();
  }

}
