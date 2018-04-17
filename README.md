# Neopixel Tube Clock

## Introduction

I think maybe I am obsessed with making Neopixel clocks. It's definitely 
becoming a problem.

Anyhoo, I bought a Neopixel RGBW 60-LED strip only to discover it was too big 
the perspex circular 'chocolate' box that I thought would make a neat 
transparent clock. I kind of left it for a while, then discovered 
[hula-hoop-shop.eu](https://www.hula-hoop-shop.eu/) who sell the all the 
components to [make your own hula-hoop](https://www.hula-hoop-shop.eu/Hula-Hoop-create-it-yourself).

Obviously, a hula-hoop Neopixel clock was going to be the mutt's generative 
organs.

**Side note**: Really important - don't forget the '-shop' part to the above 
website address. Boy, you don't want to forget that, at work, and have to 
explain to your IT department. Trust me.

## Hardware

Components                   | Src | Cost   | Notes
-----------------------------|-----|--------|-------------------------------
Arduino Nano                 | 1   | £2.41  | V3 Compatible ATMEGA328P CH340
Rotary Encoder RGB           | 1   | £4.99  | Didn't come with the knob :(
Clear Plastic Knob           | 2   | £0.66  | Plus £2.40 postage!!!
PCB Protoboard               | 1   | £0.69  | Out of a pack of 10
Adafruit Neipixel RGBW Strip | 1   | £29.43 | Black PCB 60 LED/m (ADA2837)
Tiny RTC I2C DS1307          | 1   | £0.99  | From a batch of 5 for £4.99
A blue project box           | ?   | £??    | Don't remember where I got it!
16mm Transparent tube x 3M   | 3   | £13.28 | And £10.65 DPD Shipping cost!
19mm Transparent tube x 1M   | 3   | £5.17  | And the shipping cost... again!
5v 2A PSU (UK Plug)          | 1   | £7.99  | 5.5mm input jack

### Sources

1. [Amazon UK](https://www.amazon.co.uk)
2. [Hobbytronics](https://www.hobbytronics.co.uk)
3. [Hula-Hoop-Shop.eu](https://www.hula-hoop-shop.eu)

### Notes

The prices of the tubing were in Euros, so the cost in GBP is likely to 
fluctuate with the exchange rate. 

I am a fool when it comes to checking the shipping costs. Like a real idiot I
never check. In retrospect, I could have got the RGB Encoder cheaper from
Hobbytronics along with the knob for the same postage. And I never checked 
the shipping costs from Hula-Hoop-Shop.eu until after I made the order - doh! 

And, if I'd known at the start that the trick to reliably making a hoop was
to use a slightly bigger bit of hoop to fit the ends in, I wouldn't have to 
have made a second order. On the plus side, I made a decent sized hula-hoop 
out of the tubing I had left over.

## Construction

So, this is the bastard child of my
[RGB Rotary Encode](https://github.com/peverett/RGBRotaryEncoderTest) 
and OTHER [Neopixel Clock](https://github.com/peverett/NeopixelClock) projects.

Honestly, there was a lot of hot glue used too. It's a bit of a blur. However,
the Arduino Nano pin connections are documented in the source code. 

Software-wise - this project relies on the following Adafruit libraries:
* https://github.com/adafruit/RTClib
* https://github.com/adafruit/Adafruit_NeoPixel

## User Instructions

I wrote some fancy user instructions. The [PDF file is here in the repo](https://github.com/peverett/NeopixelTubeClock/blob/master/NeopixelDisplayRingInstuctions.pdf).

## Interesting Issues

### RGB Encoder is not robust!

The RGB Encoder is designed to sit on a circuit board with through hole soldered
connections. In this project, I soldered the legs to wires and then connectors.

The problem was I dropped the Controller box and it landed on the Encoder Knob.
Since the Encoder itself is a push-fit construction of about 3-layers, the metal
legs that hold them together are easily bent apart by enough force and the whole
thing comes apart. So, one broken RGB Encoder and I had to buy a new one.

### You need bigger tube to connect smaller tube

I wanted to make a perfect circle of plastic tubing to hold the Adafruit 
Neopixel strip. Hot glue didn't cut it. I found out that the normal approach
is to use a connection bit that goes inside the tube - not an option in this
project - or a bigger bit of the same tube that the ends go into. That is why
I had to by 1-metre of slightly bigger (19mm) tubing and pay the stupid,
expensive shipping cost again, to use about 10cm of that to make the 
connection.

However, I really like the finished product, so it was worth it.

### More power

Just as [Jeremy](https://www.youtube.com/watch?v=ygBP7MtT3Ac) says... 

I was having reset problems when trying the more LED intensive display modes. 
Follow the 
[Adafruit advice for powering Neopixel products](https://learn.adafruit.com/adafruit-neopixel-uberguide/powering-neopixels), 
and get a 5v 2-amp power supply and have a big 100uf Cap in the circuit.
