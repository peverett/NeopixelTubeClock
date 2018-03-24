
/* 
 *  NeopixelTubeClock
 *  
 *  Hardware
 *  - Arduino Nano Microcontroller
 *  - Real Time Clock DS1607 - TinyRTC
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
 *  Arduino Nano  | TinyRTC DS1607 
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
#define ENC_A  (15)
#define ENC_B  (14)

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
#include <Arduino.h>
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

volatile byte rtc_sq_interrupt = LOW; 
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
  rtc_sq_interrupt = HIGH;
}

void enc_sw_isr() {
  enc_sw_interrupt = HIGH;
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

void debounce_enc_switch(void) {
  while(digitalRead(ENC_SW));
  enc_sw_interrupt = LOW;
}

/*!
 * @brief Base class for changing a setting.
 */
class set_base {
public:
  virtual void set_encoder_led(void) { PRINTLN("set_base->set_encoder_led"); }
  virtual void enc_left(void) { PRINTLN("set_base->enc_left"); }
  virtual void enc_right(void) { PRINTLN("set_base->enc_right"); }
  virtual void init_display(void) { PRINTLN("set_base->init_display"); }
  virtual void update_display(void) { PRINTLN("set_base->update_display"); }
  virtual void final(void) { PRINTLN("set_base->final"); }

  void action(void) {
    byte enc_a;
    byte enc_b;
    byte prev_enc_a=0;

    debounce_enc_switch();

    this->set_encoder_led();
    this->init_display();

    // Loop until the Encoder Switch is activated again.
    while(enc_sw_interrupt == LOW) {
      enc_a = digitalRead(ENC_A);
      enc_b = digitalRead(ENC_B);

      if (!enc_a && prev_enc_a) {
        if (enc_b) {
          this->enc_left();
        }
        else { 
          this->enc_right();
        }
      }
      prev_enc_a = enc_a;
      delay(1);
    }
    this->final();
  };
};

class set_hour : public set_base {
public:
  int hour;
  int hour_r;
  int hour_b;
  int prev;

  void set_encoder_led(void) {
    encoder_rgb_led(LED_ON, LED_OFF, LED_OFF); // Red
  };

  virtual void enc_left(void) {
    this->hour = (this->hour == 23) ? 0 : this->hour + 1;
    this->update_display();
  };

  virtual void enc_right(void) {
    this->hour = (this->hour == 0) ? 23 : this->hour - 1;
    this->update_display();
  };

  void set_hour_pixel_colour(void) {
    if (this->hour < 12) {    // In AM - Hour indicator is red
      this->hour_r = 60;
      this->hour_b = 0;
     }
     else {                   // In PM - Hour indicator is Blue
      this->hour_b = 60;
      this->hour_r = 0;
     }    
  };

  virtual void init_display(void) {
    DateTime time_now = rtc.now();

    this->hour = time_now.hour();
    this->set_hour_pixel_colour();
    
    for(int idx=0; idx < NEO_MAX; idx++) {
      if ((idx % 5) == 0) 
        np60.setPixelColor(idx, 0, 0, 0, 30);
      else
        np60.setPixelColor(idx, 0, 0, 0, 0);

    int hour_pixel = (this->hour < 12) ? this->hour * 5 : (this->hour-12) * 5;
    np60.setPixelColor(hour_pixel, this->hour_r,  0, this->hour_b, 1);
    np60.show();
    this->prev = hour_pixel;
    }
  };

  virtual void update_display(void) {
     int hour_pixel = (this->hour < 12) ? this->hour * 5 : (this->hour-12) * 5;
     
     this->set_hour_pixel_colour();
     
     np60.setPixelColor(this->prev, 0, 0, 0, 1);
     np60.setPixelColor(hour_pixel, this->hour_r, 0, this->hour_b, 1);
     np60.show();
     this->prev = hour_pixel;    
  };

  virtual void final(void) {
    DateTime time_now = rtc.now();
    DateTime new_time = DateTime(
      time_now.year(),
      time_now.month(),
      time_now.day(),
      this->hour,
      time_now.minute(),
      0
      );
    rtc.adjust(new_time);
  };
};

class set_minute : public set_base {
public:
  int minute;
  int prev;

  void set_encoder_led(void) {
    encoder_rgb_led(LED_OFF, LED_ON, LED_OFF); // green
  }

  void enc_left(void) {
    this->minute = (this->minute == 59) ? 0 : this->minute + 1;
    this->update_display();
  }

  void enc_right(void) {
    this->minute = (this->minute == 0) ? 59 : this->minute - 1;
    this->update_display();
  }

  void init_display(void) {
    DateTime time_now = rtc.now();
    int white;

    this->minute = time_now.minute();
    
    for(int idx=0; idx < NEO_MAX; idx++) {
      white =  ((idx % 5) == 0) ? 1 : 0;
      np60.setPixelColor(idx, 0, 0, 0, white);
    }

    white = ((this->minute % 5) == 0) ? 30 : 0;

    np60.setPixelColor(this->minute, 0,  60, 0, white);
    np60.show();
    this->prev = this->minute;
  }

  void update_display(void) {
     int white = ((this->prev % 5) == 0) ? 30 : 0;
     
     np60.setPixelColor(this->prev, 0, 0, 0, white);
     
     white = ((this->minute % 5) == 0) ? 30 : 0;
     np60.setPixelColor(this->minute, 0, 60, 0, white);
     
     np60.show();
     this->prev = this->minute;    
  };

  void final(void) {
    DateTime time_now = rtc.now();
    DateTime new_time = DateTime(
      time_now.year(),
      time_now.month(),
      time_now.day(),
      time_now.hour(),
      this->minute,
      0
      );
    rtc.adjust(new_time);
  };
};


class BaseTimeDisplay {
public:
  BaseTimeDisplay(Adafruit_NeoPixel &r60) : ring60(r60) {};

  virtual void Display(const DateTime &tn) {};
  virtual void Update(const DateTime &tn, const DateTime &tt) {};

protected:
  Adafruit_NeoPixel ring60;
}; 

void DecodeRGBW(const uint32_t color, uint8_t *r, uint8_t *g, uint8_t *b, uint8_t *w) {
  *w = (uint8_t)(color>>24 & 0xFF);
  *r = (uint8_t)(color>>16 & 0xFF);
  *g = (uint8_t)(color>>8 & 0xFF);
  *b = (uint8_t)(color & 0xFF);
}

inline uint8_t TwentyFourToTwelve(uint8_t twenty_four) {
  return (twenty_four >= 12) ? twenty_four - 12 : twenty_four;
}

class AnalogTimeDisplay: public BaseTimeDisplay {
public:
  AnalogTimeDisplay(Adafruit_NeoPixel &r60) : BaseTimeDisplay(r60) {};

  void Display(const DateTime &tn) {
    uint8_t hr = TwentyFourToTwelve(tn.hour()) * 5;
    uint8_t w, r, g, b;

    for(int idx=0; idx < NEO_MAX; idx++) {
      w =  ((idx % 5) == 0) ? 10 : 0;
      np60.setPixelColor(idx, 0, 0, 0, w);
    }

    DecodeRGBW(ring60.getPixelColor(hr), &r, &g, &b, &w);
    ring60.setPixelColor(hr, 60, 0, 0, w);
    
    DecodeRGBW(ring60.getPixelColor(tn.minute()), &r, &g, &b, &w);
    ring60.setPixelColor(tn.minute(), 0, 60, 0, w);   

    DecodeRGBW(ring60.getPixelColor(tn.second()), &r, &g, &b, &w);
    ring60.setPixelColor(tn.second(), 0, 0, 60, w);  

    ring60.show();
  };

  void Update(const DateTime &tn, const DateTime &tt) {
    uint8_t hn = TwentyFourToTwelve(tn.hour()) * 5;
    uint8_t ht = TwentyFourToTwelve(tt.hour()) * 5;
    uint8_t w, r, g, b;

    if (hn != ht) {
      DecodeRGBW(ring60.getPixelColor(ht), &r, &g, &b, &w);
      ring60.setPixelColor(ht, 0, 0, 0, w);
      DecodeRGBW(ring60.getPixelColor(hn), &r, &g, &b, &w);
      ring60.setPixelColor(hn, 60, 0, 0, w);
    }

    if (tn.minute() != tt.minute()) {
      DecodeRGBW(ring60.getPixelColor(tt.minute()), &r, &g, &b, &w);
      if (tt.minute() == hn) r = 60;
      ring60.setPixelColor(tt.minute(), r, 0, 0, w);
      DecodeRGBW(ring60.getPixelColor(tn.minute()), &r, &g, &b, &w);
      ring60.setPixelColor(tn.minute(), 0, 60, 0, w);
    }

    if (tn.second() != tt.second()) {
      DecodeRGBW(ring60.getPixelColor(tt.second()), &r, &g, &b, &w);
      PRINT(r);
      PRINT(":");
      PRINT(g);
      PRINT(":");
      PRINT(b);
      PRINT(":");
      PRINTLN(w);
      
      if (tt.second() == hn) {
        r = 60;
        g = 0;
      }
      if (tt.second() == tn.minute()) {
        r = 0;
        g = 60;
      }
      ring60.setPixelColor(tt.second(), r, g, 0, w);
      DecodeRGBW(ring60.getPixelColor(tn.second()), &r, &g, &b, &w);
      ring60.setPixelColor(tn.second(), 0, 0, 60, w);
    }  

    ring60.show();
  };

};

AnalogTimeDisplay atd = AnalogTimeDisplay(np60);

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

  attachInterrupt(digitalPinToInterrupt(ENC_SW), enc_sw_isr, FALLING);

  // Initially the Encoder LED are all off
  encoder_rgb_led(LED_OFF, LED_OFF, LED_OFF);

  // Initialise the Neopixel Strip
  PRINTLN(".. Initialising Neopixel Strip");
  np60.begin();

  // Display Time - default display mode
  PRINTLN(".. Displaying time - default display mode");
  atd.Display(dt_now);

  PRINTLN("Setup completed.");
}

void loop() {
  if (enc_sw_interrupt) {
    PRINTLN("ENC_SW");
    
    set_hour().action();
    set_minute().action();
    
    encoder_rgb_led(OFF, OFF, OFF);
    debounce_enc_switch();

    dt_now = rtc.now();
    atd.Display(dt_now);
  }
  if (rtc_sq_interrupt) {
    rtc_sq_interrupt = LOW;

    dt_now = rtc.now();

    //PRINT(dt_now.hour());
    //PRINT(":"); 
    //PRINT(dt_now.minute());
    //PRINT(":"); 
    //PRINTLN(dt_now.second());

    atd.Update(dt_now, dt_then);

    dt_then = dt_now;
  }

}
