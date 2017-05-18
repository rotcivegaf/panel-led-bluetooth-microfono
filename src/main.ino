/* VUmeter by BGprojectz */
#include <Adafruit_NeoPixel.h>
#include <math.h>
#define N_PIXELS  84
#define MIC_PIN   A5
#define LED_PIN    6
#define SAMPLE_WINDOW   10
#define PEAK_HANG 24
#define PEAK_FALL 4
#define INPUT_FLOOR 10
#define INPUT_CEILING 100
byte peak = 16;
unsigned int sample;

byte dotCount = 0;
byte dotHangCount = 0;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup(){
  analogReference(EXTERNAL);
  Serial.begin(9600);
  strip.begin();
  strip.show();
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
  //Serial.print(OriginalRange, DEC);
  //Serial.print("   ");
  //Serial.print(NewRange, DEC);
  //Serial.print("   ");
  //Serial.println(zeroRefCurVal, DEC);
  //Serial.println();
  // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine
  if (originalMin > originalMax ) {
    return 0;
  }
  if (invFlag == 0){
    rangedValue =  (pow(normalizedCurVal, curve) * NewRange) + newBegin;
  }else{ // invert the ranges
    rangedValue =  newBegin - (pow(normalizedCurVal, curve) * NewRange);
  }
  return rangedValue;
}



void loop() {
  unsigned long startMillis= millis();
  float peakToPeak = 0;
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
  peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude

  // Serial.println(peakToPeak);
  for (int i=0;i<=strip.numPixels()-1;i++){
    strip.setPixelColor(i,Wheel(map(i,0,strip.numPixels()-1,30,150)));
  }

  c = fscale(INPUT_FLOOR, INPUT_CEILING, strip.numPixels(), 0, peakToPeak, 2);

  if(c < peak) {
    peak = c;
    dotHangCount = 0;
  }
  if (c <= strip.numPixels()) {
    drawLine(strip.numPixels(), strip.numPixels()-c, strip.Color(0, 0, 0));
  }

  y = strip.numPixels() - peak;
  strip.setPixelColor(y-1,Wheel(map(y,0,strip.numPixels()-1,30,150)));
  strip.show();

  if(dotHangCount > PEAK_HANG) {
    if(++dotCount >= PEAK_FALL) {
      peak++;
      dotCount = 0;
    }
  }else {
    dotHangCount++;
  }
}

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

uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }else{
    if(WheelPos < 170) {
      WheelPos -= 85;
      return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
    }else {
      WheelPos -= 170;
      return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
    }
  }
}
