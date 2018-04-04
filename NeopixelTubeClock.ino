
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
 * defined indexes for Red, Green, Blue, White
 */
#define RED   0
#define GREEN 1
#define BLUE  2
#define WHITE 3

/*
 * Serial Monitor Debug instrumentation
 * 
 * comment out #define DEBUG_PRINT to disable.
 */
//#define DEBUG_PRINT
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

/* 
 *  A thin shim class to Adafruit_NeoPixel, which allows the following:
 *  - offset into the pixel range e.g. 0 = pixel 30 and pixel 30 wraps.
 *  - Decode colours into passed uint8_t values.
 */
class NeoPixelOffset {
public: 
  NeoPixelOffset(uint16_t Offset) {
    np = Adafruit_NeoPixel(NEO_MAX, NEO_DIN, NEO_GRBW + NEO_KHZ800);
    offset = Offset;
  };

  void begin(void) { np.begin(); };

  void show(void) { np.show(); };

  void setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
    np.setPixelColor((n + offset) % NEO_MAX, r, g, b, w);
  };

  void getPixelColor(uint16_t n, uint8_t *r, uint8_t *g, uint8_t *b, uint8_t *w) {      
    uint32_t color = np.getPixelColor((n + offset) % NEO_MAX);
    
    *w = (uint8_t)(color>>24 & 0xFF);
    *r = (uint8_t)(color>>16 & 0xFF);
    *g = (uint8_t)(color>>8 & 0xFF);
    *b = (uint8_t)(color & 0xFF);
}
  
private:
  Adafruit_NeoPixel np;
  uint16_t offset;
};

// For my clock, the top of the Neopixel circle (pin 29 is the 0 position, so use an offset
// of 30.
NeoPixelOffset np60 = NeoPixelOffset(30);

volatile byte rtc_sq_interrupt = LOW; 
volatile byte enc_sw_interrupt = LOW;


uint8_t wi = 30;    /* White LED Intensity - white is generally twice as bright */
uint8_t ri = 60;    /* Red LED Intensity */
uint8_t gi = 60;    /* Green LED Intensity */
uint8_t bi = 60;    /* Blue LED Intensity */

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

/* Convert between 24 hour and 12 hour format. */
inline uint8_t HourToPixel(uint8_t twenty_four) {
  return (twenty_four % 12) * 5;
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

bool debounce_enc_switch(int time_ms) {
  int lo_count = 0;
  long unsigned start = millis();
  long unsigned finish;
  
  while(digitalRead(ENC_SW) == HIGH);

  // debounce the release
  while(lo_count < 300)
    {
    if (digitalRead(ENC_SW) == LOW)
      lo_count++;
    else
      lo_count = 0;
    }   

  // How long is the button held down
  finish = millis();

  enc_sw_interrupt = LOW;
 
  return (time_ms > (finish - start)) ? false : true;
}

class BaseTimeDisplay {
public:
  BaseTimeDisplay(NeoPixelOffset &r60) : ring60(r60) {};

  virtual void Display(const DateTime &tn) {};
  virtual void Update(const DateTime &tn, const DateTime &tt) {};

protected:
  NeoPixelOffset ring60;
}; 


class AnalogTimeDisplay: public BaseTimeDisplay {
public:
  AnalogTimeDisplay(NeoPixelOffset &r60) : BaseTimeDisplay(r60) {};

  void Display(const DateTime &tn) {
    for(int idx=0; idx < NEO_MAX; idx++) {
      ring60.setPixelColor(idx, 0, 0, 0, ((idx % 5) == 0) ? wi : 0);
    }

    ring60.setPixelColor(HourToPixel(tn.hour()), ri, 0, 0, 0);
    ring60.setPixelColor(tn.minute(), 0, gi, 0, 0);   
    ring60.setPixelColor(tn.second(), 0, 0, bi, 0);  
    ring60.show();
  };

  void Update(const DateTime &tn, const DateTime &tt) {
    uint8_t hn = HourToPixel(tn.hour());
    uint8_t ht = HourToPixel(tt.hour()) * 5;

    if (tn.hour() != tt.hour()) {
      ring60.setPixelColor(HourToPixel(tt.hour()), 0, 0, 0, wi);
      ring60.setPixelColor(HourToPixel(tn.hour()), ri, 0, 0, 0);
    }

    if (tn.minute() != tt.minute()) {
      if (tt.minute() == HourToPixel(tn.hour())) 
        ring60.setPixelColor(tt.minute(), ri, 0, 0, 0);
      else
        ring60.setPixelColor(tt.minute(), 0, 0, 0, ((tt.minute() % 5) == 0) ? wi : 0);
      ring60.setPixelColor(tn.minute(), 0, gi, 0, 0);
    }

    if (tn.second() != tt.second()) {
      if (tt.second() == tn.minute())
        ring60.setPixelColor(tt.second(), 0, gi, 0, 0);
      else if (tt.second() == HourToPixel(tn.hour()))
        ring60.setPixelColor(tt.second(), ri, 0, 0, 0);
      else
        ring60.setPixelColor(tt.second(), 0, 0, 0, ((tt.second() % 5) == 0) ? wi : 0);
      ring60.setPixelColor(tn.second(), 0, 0, bi, 0);
    }

    ring60.show();
  };

};

/* 
 *  Hours is 5 pixels
 *  Minutes is 3 pixels 
 *  Seconds is 1 Pixel. 
 *  Seconds overlays minutes, which overlays hours.
 */
class PulseTimeDisplay: public BaseTimeDisplay {
public:
  PulseTimeDisplay(NeoPixelOffset &r60) : 
    BaseTimeDisplay(r60),
    fi(0)
    {};

  void Display(const DateTime &tn) {
    this->fi = ri/60;
    
    for(int idx=0; idx < NEO_MAX; idx++) {
      ring60.setPixelColor(idx, 0, 0, 0, 0);
    }

    ring60.setPixelColor(HourToPixel(tn.hour()), ri, 0, 0, 0);
    ring60.setPixelColor(tn.minute(), 0, gi, 0, 0);   
    ring60.setPixelColor(tn.second(), 0, 0, bi, 0);  
    ring60.show();
  };

  void Update(const DateTime &tn, const DateTime &tt) {
    uint8_t h = HourToPixel(tn.hour());
    uint8_t m = tn.minute();
    uint8_t s = tn.second();
    uint8_t r=ri, g=gi, b=bi;

    ring60.setPixelColor(h, r, 0, 0, 0);
    ring60.setPixelColor(m, 0, g, 0, 0);   
    ring60.setPixelColor(s, 0, 0, b, 0); 
    ring60.show();
    
    for (uint8_t idx=0; idx<NEO_MAX; idx++)
    {
      delay(10);

      r = (r>0) ? r - this->fi: 0;
      g = (g>0) ? g - this->fi: 0;
      b = (b>0) ? b - this->fi: 0;
      
      ring60.setPixelColor(h, r, 0, 0, 0);
      ring60.setPixelColor(m, 0, g, 0, 0);   
      ring60.setPixelColor(s, 0, 0, b, 0); 

      if (enc_sw_interrupt) return;

      ring60.show();
    }
  } 

private:
  uint8_t fi;
};

/* 
 *  This display mode is not affected by the brightness setting and doesn't really show the
 *  time. It's just a pastel ring that rotates anticlockwise once per minute.
 */
class PastelRing: public BaseTimeDisplay {
public:
  PastelRing(NeoPixelOffset &r60) : BaseTimeDisplay(r60) {};

  void pastel(int pixel, uint8_t cidx) {
    uint8_t index;
    int brg = 0;
    int stp = 10;
   
    for (int offset=0; offset<40; offset++) {
      index = (uint8_t)((pixel + offset) % 60);
      ring60.getPixelColor(index, &rgbw[RED], &rgbw[GREEN], &rgbw[BLUE], &rgbw[WHITE]);
      brg += stp;
      rgbw[cidx] = (uint8_t)(brg);
      ring60.setPixelColor(index, rgbw[RED], rgbw[GREEN], rgbw[BLUE], 0);
      if (brg == 200) stp = -10;
    }
  };
  
  void Display(const DateTime &tn) {
    for (int i=0; i<NEO_MAX; i++)
      ring60.setPixelColor(i, 0, 0, 0, 0);
    this->pastel(0, RED);
    this->pastel(20, GREEN);
    this->pastel(40, BLUE);
    ring60.show();
  };

  void Update(const DateTime &tn, const DateTime &tt) { 
    uint8_t r, g, b, w;

    ring60.getPixelColor(0, &r, &g, &b, &w);
    for (int i=1; i<NEO_MAX; i++) {
      ring60.getPixelColor(i, &rgbw[RED], &rgbw[GREEN], &rgbw[BLUE], &rgbw[WHITE]);
      ring60.setPixelColor(i-1, rgbw[RED], rgbw[GREEN], rgbw[BLUE], 0);
    }
    ring60.setPixelColor(NEO_MAX-1, r, g, b, 0);
    ring60.show();
  };
  
private:
  uint8_t rgbw[4];
};

/* 
 *  This display mode is not affected by the brightness setting.
 */
class PastelTimeDisplay: public BaseTimeDisplay {
public:
  PastelTimeDisplay(NeoPixelOffset &r60) : BaseTimeDisplay(r60) {};

  void pastel(int pixel, uint8_t cidx) {
    uint8_t index;
    int brg = ri/2;
    int stp = (ri/2) / 30;

    // The first pixel is double brightness
    ring60.getPixelColor(pixel, &rgbw[RED], &rgbw[GREEN], &rgbw[BLUE], &rgbw[WHITE]);
    rgbw[cidx] = (uint8_t)(ri);
    ring60.setPixelColor(pixel, rgbw[RED], rgbw[GREEN], rgbw[BLUE], 0);

    // Trailing pixels start half as bright and fade out.
    for (int offset=1; offset<30; offset++) {
      brg -= stp;

      index = (uint8_t)(((pixel-offset) >= 0) ? pixel-offset : (pixel+60)-offset);
      ring60.getPixelColor(index, &rgbw[RED], &rgbw[GREEN], &rgbw[BLUE], &rgbw[WHITE]);
      rgbw[cidx] = (uint8_t)(brg);
      ring60.setPixelColor(index, rgbw[RED], rgbw[GREEN], rgbw[BLUE], 0);
    }
  };
  
  void Display(const DateTime &tn) {
    for (int i=0; i<NEO_MAX; i++)
      ring60.setPixelColor(i, 0, 0, 0, 0);

    this->pastel(HourToPixel(tn.hour()), RED);
    this->pastel(tn.minute(), GREEN);
    this->pastel(tn.second(), BLUE);
    
    ring60.show();
  };

  void Update(const DateTime &tn, const DateTime &tt) { 
     this->Display(tn);
  };
  
private:
  uint8_t rgbw[4];
};

/*
 * The different display modes.
 */
AnalogTimeDisplay analog  = AnalogTimeDisplay(np60);
PulseTimeDisplay pulse    = PulseTimeDisplay(np60);
PastelTimeDisplay pastel  = PastelTimeDisplay(np60);

BaseTimeDisplay *dm[] = { &analog, &pulse, &pastel };
const int disp_mode_max = sizeof(dm) / sizeof(BaseTimeDisplay*);
int mode;

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
  virtual void action_loop(void) { PRINTLN("set_base->action_loop"); }
  virtual void final(void) { PRINTLN("set_base->final"); }

  void action(void) {
    byte enc_a;
    byte enc_b;
    byte prev_enc_a=0;

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

      /* What to do while looping - usually a small delay */
      this->action_loop();
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

  virtual void init_display(void) {
    DateTime time_now = rtc.now();

    this->hour = time_now.hour();

    for(int idx=0; idx < NEO_MAX; idx++) {
      np60.setPixelColor(idx, 0 , 0, 0, ((idx % 5) == 0) ? wi : 0);
    }
    
    np60.setPixelColor(HourToPixel(this->hour), ri, 0, 0, 0);
    np60.show();
    this->prev = this->hour;
  };

  virtual void update_display(void) {
     int hour_pixel = (this->hour < 12) ? this->hour * 5 : (this->hour-12) * 5;
          
     np60.setPixelColor(HourToPixel(this->prev), 0, 0, 0, wi);
     np60.setPixelColor(HourToPixel(this->hour), ri, 0, 0, 0);
     np60.show();
     this->prev = this->hour;    
  };

  virtual void action_loop(void) { delay(1); };

  virtual void final(void) {
    DateTime time_now = rtc.now();
    DateTime new_time = DateTime(
      time_now.year(),
      time_now.month(),
      time_now.day(),
      this->hour,
      time_now.minute(),
      time_now.second()
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
      np60.setPixelColor(idx, 0, 0, 0, ((idx % 5) == 0) ? wi : 0);
    }

    np60.setPixelColor(this->minute, 0,  gi, 0, 0);
    np60.show();
    this->prev = this->minute;
  }

  void update_display(void) {
     int white = ((this->prev % 5) == 0) ? 30 : 0;
     
     np60.setPixelColor(this->prev, 0, 0, 0, ((this->prev % 5) == 0) ? wi : 0);
     np60.setPixelColor(this->minute, 0, gi, 0, 0);
     
     np60.show();
     this->prev = this->minute;    
  };

  virtual void action_loop(void) { delay(1); };

  void final(void) {
    DateTime time_now = rtc.now();
    DateTime new_time = DateTime(
      time_now.year(),
      time_now.month(),
      time_now.day(),
      time_now.hour(),
      this->minute,
      time_now.second()
      );
    rtc.adjust(new_time);
  };
};

class set_second : public set_base {
public:
  int second;
  int prev;

  void set_encoder_led(void) {
    encoder_rgb_led(LED_OFF, LED_OFF, LED_ON); // Blue
  }

  void enc_left(void) {
    this->second = (this->second == 59) ? 0 : this->second + 1;
    this->update_display();
  }

  void enc_right(void) {
    this->second = (this->second == 0) ? 59 : this->second - 1;
    this->update_display();
  }

  void init_display(void) {
    DateTime time_now = rtc.now();
    int white;

    this->second = time_now.second();
    
    for(int idx=0; idx < NEO_MAX; idx++) {
      np60.setPixelColor(idx, 0, 0, 0, ((idx % 5) == 0) ? wi : 0);
    }

    np60.setPixelColor(this->second, 0,  0, bi, 0);
    np60.show();
    this->prev = this->second;
  }

  void update_display(void) {
     int white = ((this->prev % 5) == 0) ? 30 : 0;
     
     np60.setPixelColor(this->prev, 0, 0, 0, ((this->prev % 5) == 0) ? wi : 0);
     np60.setPixelColor(this->second, 0, 0, bi, 0);
     
     np60.show();
     this->prev = this->second;    
  };

  virtual void action_loop(void) { delay(1); };

  void final(void) {
    DateTime time_now = rtc.now();
    DateTime new_time = DateTime(
      time_now.year(),
      time_now.month(),
      time_now.day(),
      time_now.hour(),
      time_now.minute(),
      this->second
      );
    rtc.adjust(new_time);
  };
};

class set_brightness : public set_base {
public:
  void set_encoder_led(void) {
    encoder_rgb_led(LED_OFF, LED_ON, LED_ON); // Cyan
  }

  void enc_left(void) {
    wi = (wi == 120) ? 30 : wi + 30;
    ri = wi * 2;
    gi = wi * 2;
    bi = wi * 2;
    this->init_display();
  }

  void enc_right(void) {
    wi = (wi == 30) ? 120 : wi - 30;
    ri = wi * 2;
    gi = wi * 2;
    bi = wi * 2;
    this->init_display();
  }

  void init_display(void) {
    for(int idx=0; idx < NEO_MAX; idx++) {

      if ((idx % 15) == 0)
        np60.setPixelColor(idx, ri, 0, 0, 0);
      else if ((idx % 10) == 0)
        np60.setPixelColor(idx, 0, gi, 0, 0);
      else if ((idx % 5) == 0)
        np60.setPixelColor(idx, 0, 0, bi, 0);
      else 
        np60.setPixelColor(idx, 0, 0, bi, wi);
    }

    np60.show();
  }

  void update_display(void) {  
  };

  virtual void action_loop(void) { delay(1); };

  void final(void) {
  };
};

class set_display : public set_base {
public:
  void set_encoder_led(void) {
    encoder_rgb_led(LED_ON, LED_OFF, LED_ON); // purple
  }

  void enc_left(void) {
    mode = (mode + 1) % disp_mode_max;

    this->init_display();
  }

  void enc_right(void) {
    mode = (mode == 0) ? disp_mode_max-1 : mode - 1;
    this->init_display();
  }

  void init_display(void) {
    PRINT("Display Mode = ");
    PRINTLN(mode);
    delay(300);
    dt_now = rtc.now();
    dm[mode]->Display(dt_now);
  }

  void update_display(void) {  
  };

  virtual void action_loop(void) {
    //if (rtc_sq_interrupt) {
    //  rtc_sq_interrupt = LOW;
    //  dt_now = rtc.now();
    //  dm[mode]->Update(dt_now, dt_then);
    //  dt_then = dt_now;
    //}
    delay(1);
  };

  void final(void) {
  };
  
};

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

  attachInterrupt(digitalPinToInterrupt(ENC_SW), enc_sw_isr, RISING);

  // Initially the Encoder LED are all off
  encoder_rgb_led(LED_OFF, LED_OFF, LED_OFF);

  // Initialise the Neopixel Strip
  PRINTLN(".. Initialising Neopixel Strip");
  np60.begin();

  // Display Time - default display mode
  PRINT(".. Displaying time - default mode (");
  mode = 0;
  dm[mode]->Display(dt_now);
  PRINT(disp_mode_max);
  PRINTLN(" Display modes available)");

  PRINTLN("Setup completed.");
}

void loop() {

  
  if (enc_sw_interrupt) {
    PRINTLN("ENC_SW");

    if (debounce_enc_switch(500)) {   // Long Press
      set_hour().action();
      debounce_enc_switch(0);
      set_minute().action();
      debounce_enc_switch(0);
      set_second().action();
    } 
    else {// Short press
      set_brightness().action();
      debounce_enc_switch(0);
      set_display().action();
    }
    debounce_enc_switch(0);
   
    encoder_rgb_led(OFF, OFF, OFF);

    dt_now = rtc.now();
    dm[mode]->Display(dt_now);
  }
  if (rtc_sq_interrupt) {
    rtc_sq_interrupt = LOW;

    dt_now = rtc.now();
    dm[mode]->Update(dt_now, dt_then);
    dt_then = dt_now;
  }
}
