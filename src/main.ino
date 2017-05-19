#include <Adafruit_NeoPixel.h>
#include <math.h>
#include <SoftwareSerial.h>
#define N_PIXELS  84  // Number of pixels in strand
#define N_PIXELS_HALF (N_PIXELS/2)
#define MIC_PIN   A5  // Microphone is attached to this analog pin
#define LED_PIN    6  // NeoPixel LED strand is connected to this pin
#define SAMPLE_WINDOW   10  // Sample window for average level
#define PEAK_HANG 24 //Time of pause before peak dot falls
#define PEAK_FALL 4 //Rate of falling peak dot
#define INPUT_FLOOR 10 //Lower range of analogRead input
#define INPUT_CEILING 100 //Max range of analogRead input, the lower the value the more sensitive (1023 = max)300 (150)

#include "FastLED.h"
#if FASTLED_VERSION < 3001000
#error "Requires FastLED 3.1 or later; check github for latest code."
#endif
#define BRIGHTNESS  255
#define LED_TYPE    WS2812B     // Only use the LED_PIN for WS2812's
#define COLOR_ORDER GRB

struct CRGB leds[N_PIXELS];

Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

static uint16_t dist;         // A random number for noise generator.
uint16_t scale = 30;          // Wouldn't recommend changing this on the fly, or the animation will be really blocky.
uint8_t maxChanges = 48;      // Value for blending between palettes.

CRGBPalette16 currentPalette(CRGB::Black);
CRGBPalette16 targetPalette(OceanColors_p);

//Bluetooth
#define BLUETOOTH_SPEED 9600
SoftwareSerial bluetooth(10, 11);
int choice = 0; // bluetooth choice
// button test
//const int buttonPin = 2;     // the number of the pushbutton pin

// Variables will change:
//int buttonPushCounter = 0;   // counter for the number of button presses
//int buttonState = 0;         // current state of the button
//int lastButtonState = 0;

byte peak = 16;      // Peak level of column; used for falling dots
unsigned int sample;

byte dotCount = 0;  //Frame counter for peak dot
byte dotHangCount = 0; //Frame counter for holding peak dot

void setup() {
 analogReference(EXTERNAL);
 bluetooth.begin(BLUETOOTH_SPEED);
  // Serial.begin(9600);
  strip.begin();
  strip.show(); // all pixels to 'off'
  Serial.begin(57600);
  delay(3000);
  LEDS.addLeds<LED_TYPE,LED_PIN,COLOR_ORDER>(leds,N_PIXELS);
  LEDS.setBrightness(BRIGHTNESS);
  dist = random16(12345);          // A semi-random number for our noise generator
}

float fscale( float originalMin, float originalMax, float newBegin, float newEnd, float inputValue, float curve){
  float OriginalRange = 0;
  float NewRange = 0;
  float zeroRefCurVal = 0;
  float normalizedCurVal = 0;
  float rangedValue = 0;
  boolean invFlag = 0;
  // condition curve parameter
  // limit range
  if (curve > 10) curve = 10;
  if (curve < -10) curve = -10;

  curve = (curve * -.1) ; // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output
  curve = pow(10, curve); // convert linear scale into lograthimic exponent for other pow function
  //Serial.println(curve * 100, DEC);   // multply by 100 to preserve resolution
  //Serial.println();

  // Check for out of range inputValues
  if (inputValue < originalMin) {
  inputValue = originalMin;
  }
  if (inputValue > originalMax) {
  inputValue = originalMax;
  }
  // Zero Refference the values
  OriginalRange = originalMax - originalMin;
  if (newEnd > newBegin){
    NewRange = newEnd - newBegin;
  }else{
    NewRange = newBegin - newEnd;
    invFlag = 1;
  }

  zeroRefCurVal = inputValue - originalMin;
  normalizedCurVal  =  zeroRefCurVal / OriginalRange;   // normalize to 0 - 1 float

  /*
  Serial.print(OriginalRange, DEC);
  Serial.print("   ");
  Serial.print(NewRange, DEC);
  Serial.print("   ");
  Serial.println(zeroRefCurVal, DEC);
  Serial.println();
  */

  // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine
  if (originalMin > originalMax ) {
    return 0;
  }

  if (invFlag == 0){
    rangedValue =  (pow(normalizedCurVal, curve) * NewRange) + newBegin;
  }else{     // invert the ranges
    rangedValue =  newBegin - (pow(normalizedCurVal, curve) * NewRange);
  }
  return rangedValue;
}
// uint8_t curControl = BTN_NONE;
void loop(){
  if (bluetooth.available() > 0) {
    // read the incoming byte:
    uint8_t incomingByte = bluetooth.read();
    Serial.println(incomingByte);
    if (incomingByte == 11){
    choice = 11;}
    else if (incomingByte == 12){
    choice = 12;}
    else if (incomingByte == 13){
    choice = 13;}
    else if (incomingByte == 14){
    choice = 14;}
    else if (incomingByte == 15){
    choice = 15;}
    else if (incomingByte == 16){
    choice = 16;}
    else if (incomingByte == 17){
    choice = 17;}
    else if (incomingByte == 18){
    choice = 18;}
    else if (incomingByte == 19){
    choice = 19;}
  }
  switch(choice){
  case 11 :
    choice == 11;
    vu1();
    break;
  case 12:
    choice == 12;
    vu2();
    break;
  case 13:
    choice == 13;
    vu3();
    break;
  case 14:
    choice == 14;
    Fire(55,120,15);
    break;
  case 15:
    choice == 15;
    rainbow(20);
    break;
  //case 16:
  //   choice == 16; {
  //     RGBLoop();
  //  break;}
  case 17:
    choice == 17;
    TwinkleRandom(40, 100, false);
    break;
  case 18:
    choice == 18;
    SnowSparkle(0x10, 0x10, 0x10, 20, random(100,1000));
    break;
  case 19:
    choice == 19;
    Colorful();
    break;
  }
}

void vu1() {
  unsigned long startMillis= millis();  // Start of sample window
  float peakToPeak = 0;   // peak-to-peak level
  unsigned int signalMax = 0;
  unsigned int signalMin = 1023;
  unsigned int c, y;

  while (millis() - startMillis < SAMPLE_WINDOW){
    sample = analogRead(MIC_PIN);
    if (sample < 1024){
      if (sample > signalMax){
        signalMax = sample;
      }else{
        if (sample < signalMin){
          signalMin = sample;
        }
      }
    }
  }
  peakToPeak = signalMax - signalMin;
  // Serial.println(peakToPeak);
  for (int i=0;i<=strip.numPixels()-1;i++){
    strip.setPixelColor(i,Wheel(map(i,0,strip.numPixels()-1,30,150)));
  }

  c = fscale(INPUT_FLOOR, INPUT_CEILING, strip.numPixels(), 0, peakToPeak, 2);
  if(c < peak) {
    peak = c;        // Keep dot on top
    dotHangCount = 0;    // make the dot hang before falling
  }
  if (c <= strip.numPixels()) { // Fill partial column with off pixels
    drawLine(strip.numPixels(), strip.numPixels()-c, strip.Color(0, 0, 0));
  }

  y = strip.numPixels() - peak;
  strip.setPixelColor(y-1,Wheel(map(y,0,strip.numPixels()-1,30,150)));
  strip.show();
  // Frame based peak dot animation
  if(dotHangCount > PEAK_HANG) { //Peak pause length
    if(++dotCount >= PEAK_FALL) { //Fall rate
      peak++;
      dotCount = 0;
    }
  }else {
    dotHangCount++;
  }
}

void vu2(){
  unsigned long startMillis= millis();  // Start of sample window
  float peakToPeak = 0;   // peak-to-peak level

  unsigned int signalMax = 0;
  unsigned int signalMin = 1023;
  unsigned int c, y;

  while (millis() - startMillis < SAMPLE_WINDOW){
    sample = analogRead(MIC_PIN);
    if (sample < 1024){
      if (sample > signalMax){
        signalMax = sample;
      }else{
        if (sample < signalMin){
          signalMin = sample;
        }
      }
    }
  }
  peakToPeak = signalMax - signalMin;
  // Serial.println(peakToPeak);
  for (int i=0;i<=N_PIXELS_HALF-1;i++){
    uint32_t color = Wheel(map(i,0,N_PIXELS_HALF-1,30,150));
    strip.setPixelColor(N_PIXELS-i,color);
    strip.setPixelColor(0+i,color);
  }

  c = fscale(INPUT_FLOOR, INPUT_CEILING, N_PIXELS_HALF, 0, peakToPeak, 2);
  if(c < peak) {
    peak = c;        // Keep dot on top
    dotHangCount = 0;    // make the dot hang before falling
  }
  if (c <= strip.numPixels()) { // Fill partial column with off pixels
    drawLine(N_PIXELS_HALF, N_PIXELS_HALF-c, strip.Color(0, 0, 0));
    drawLine(N_PIXELS_HALF, N_PIXELS_HALF+c, strip.Color(0, 0, 0));
  }

  y = N_PIXELS_HALF - peak;
  uint32_t color1 = Wheel(map(y,0,N_PIXELS_HALF-1,30,150));
  strip.setPixelColor(y-1,color1);
  //strip.setPixelColor(y-1,Wheel(map(y,0,N_PIXELS_HALF-1,30,150)));
  y = N_PIXELS_HALF + peak;
  strip.setPixelColor(y,color1);
  //strip.setPixelColor(y+1,Wheel(map(y,0,N_PIXELS_HALF+1,30,150)));
  strip.show();
  // Frame based peak dot animation
  if(dotHangCount > PEAK_HANG) { //Peak pause length
    if(++dotCount >= PEAK_FALL) { //Fall rate
      peak++;
      dotCount = 0;
    }
  }else {
    dotHangCount++;
  }
}

void vu3(){
  unsigned long startMillis= millis();  // Start of sample window
  float peakToPeak = 0;   // peak-to-peak level

  unsigned int signalMax = 0;
  unsigned int signalMin = 1023;
  unsigned int c, y;

  while (millis() - startMillis < SAMPLE_WINDOW){
    sample = analogRead(MIC_PIN);
    if (sample < 1024){
      if (sample > signalMax) {
        signalMax = sample;
      }else{
        if (sample < signalMin){
          signalMin = sample;
        }
      }
    }
  }
  peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
  // Serial.println(peakToPeak);
  for (int i=0;i<=N_PIXELS_HALF-1;i++){
    uint32_t color = Wheel(map(i,0,N_PIXELS_HALF-1,30,150));
    strip.setPixelColor(N_PIXELS_HALF-i-1,color);
    strip.setPixelColor(N_PIXELS_HALF+i,color);
  }

  c = fscale(INPUT_FLOOR, INPUT_CEILING, N_PIXELS_HALF, 0, peakToPeak, 2);
  if(c < peak) {
    peak = c;
    dotHangCount = 0;
  }
  if (c <= N_PIXELS) {
    drawLine(N_PIXELS, N_PIXELS-c, strip.Color(0, 0, 0));
    drawLine(0, 0+c, strip.Color(0, 0, 0));
  }
  y = N_PIXELS - peak;

  // uint32_t color = Wheel(map(i,0,N_PIXELS_HALF-1,30,150));
  //strip.setPixelColor(y,color);
  strip.setPixelColor(y-1,Wheel(map(N_PIXELS_HALF+y,0,N_PIXELS_HALF-1,30,150)));
  y = 0 + peak;
  //strip.setPixelColor(y,color);
  strip.setPixelColor(y,Wheel(map(N_PIXELS_HALF-y,0,N_PIXELS_HALF+1,30,150)));
  strip.show();
  // Frame based peak dot animation
  if(dotHangCount > PEAK_HANG) { //Peak pause length
    if(++dotCount >= PEAK_FALL) { //Fall rate
      peak++;
      dotCount = 0;
    }
  }else {
    dotHangCount++;
  }
}

void Fire(int Cooling, int Sparking, int SpeedDelay) {
  static byte heat[N_PIXELS];
  int cooldown;

  for( int i = 0; i < N_PIXELS; i++) {
    cooldown = random(0, ((Cooling * 10) / N_PIXELS) + 2);
    if(cooldown>heat[i]) {
      heat[i]=0;
    } else {
      heat[i]=heat[i]-cooldown;
    }
  }
  for( int k= N_PIXELS - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
  }
  if( random(255) < Sparking ) {
    int y = random(7);
    heat[y] = heat[y] + random(160,255);
    //heat[y] = random(160,255);
  }
  for( int j = 0; j < N_PIXELS; j++) {
    setPixelHeatColor(j, heat[j] );
  }
  strip.show();
  delay(SpeedDelay);
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

void TwinkleRandom(int Count, int SpeedDelay, boolean OnlyOne) {
  setAll(0,0,0);

  for (int i=0; i<Count; i++) {
    setPixel(random(N_PIXELS),random(0,255),random(0,255),random(0,255));
    strip.show();
    delay(SpeedDelay);
    if(OnlyOne) {
      setAll(0,0,0);
    }
  }
  delay(SpeedDelay);
}

void SnowSparkle(byte red, byte green, byte blue, int SparkleDelay, int SpeedDelay) {
  setAll(red,green,blue);
  int Pixel = random(N_PIXELS);
  setPixel(Pixel,0xff,0xff,0xff);
  strip.show();
  delay(SparkleDelay);
  setPixel(Pixel,red,green,blue);
  strip.show();
  delay(SpeedDelay);
}

void Colorful() {
  EVERY_N_MILLISECONDS(10) {
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);
    fillnoise8();
  }
  EVERY_N_SECONDS(5) {
    targetPalette = CRGBPalette16(CHSV(random8(), 255, random8(128,255)), CHSV(random8(), 255, random8(128,255)), CHSV(random8(), 192, random8(128,255)), CHSV(random8(), 255, random8(128,255)));
  }
  LEDS.show();
} // loop()

//help functions
void setPixelHeatColor (int Pixel, byte temperature) {
  byte t192 = round((temperature/255.0)*191);
  byte heatramp = t192 & 0x3F; // 0..63
  heatramp <<= 2; // scale up to 0..252

  if( t192 > 0x80) {                     // hottest
    setPixel(Pixel, 255, 255, heatramp);
  } else{
    if( t192 > 0x40 ) {             // middle
      setPixel(Pixel, 255, heatramp, 0);
    } else {                               // coolest
      setPixel(Pixel, heatramp, 0, 0);
    }
  }
}
//Used to draw a line between two points of a given color
void drawLine(uint8_t from, uint8_t to, uint32_t c) {
  uint8_t fromTemp;
  if (from > to) {
    fromTemp = from;
    from = to;
    to = fromTemp;
  }
  for(int i=from; i<=to; i++){
    strip.setPixelColor(i, c);
  }
}
// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
void fillnoise8() {
  for(int i = 0; i < N_PIXELS; i++) {
    uint8_t index = inoise8(i*scale, dist+i*scale) % 255;
    leds[i] = ColorFromPalette(currentPalette, index, 255, LINEARBLEND);
  }
  dist += beatsin8(10,1, 4);
} // fillnoise8()

uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }else{
    if(WheelPos < 170) {
      WheelPos -= 85;
      return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
    } else {
      WheelPos -= 170;
      return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
    }
  }
}

void setPixel(int Pixel, byte red, byte green, byte blue) {
  strip.setPixelColor(Pixel, strip.Color(red, green, blue));
}

void setAll(byte red, byte green, byte blue) {
  for(int i = 0; i < N_PIXELS; i++ ) {
    setPixel(i, red, green, blue);
  }
  strip.show();
}
