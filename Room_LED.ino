#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

/**************************************************************************************/

/* DEFINE VALUES */ 
#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024

#define FACTORYRESET_ENABLE         0
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

#define OPEN 1
#define CLOSED 0

#define TRUE 1
#define FALSE 0 

/**************************************************************************************/

/* FUNCTION PROTOTYPES */ 
void start_timer(int frequencyHz);
void set_timer_frequency(int frequencyHz);
void TC3_Handler();
void program_timer(); 
char* BLE_Update(); 
void BLE_Setup(); 
void error(); 
void reset_strip(); 
uint32_t Wheel(byte WheelPos, uint32_t b);

/**************************************************************************************/

/* TYPE DEFINITONS */ 
typedef struct{
  boolean door : 1; 
  boolean resetStrip : 1; 
  boolean ledOn : 1; 
  boolean interrupt : 1; 
  boolean toggle : 1; 
  boolean maxBrightness : 1; 
  boolean timeout : 1; 
  
  boolean basic : 1; 
  boolean party : 1; 
  boolean rainbow : 1; 
  
}flag_t; 

/**************************************************************************************/

/* TYPE DECLARATIONS */ 
  static flag_t flag; 

/**************************************************************************************/

/* PRIVATE VARIABLES */ 
  // I/O
  static const int sampleRate = 1; 
  static const int REED_PIN = 10; 
  static const int LED_PIN = 13;
  static const int BUZZER_PIN = 12; 
  static const int STRING_PIN = 20; 
  
  // LED
  static const int NUM_LEDS = 150; // Total number of LEDs on the strip
  static const int SIDE_LEDS = 60; // Number of LEDs active per side
  static const int MIDDLE_LEDS = 14; // Number of LEDs unused between sides
  static const int END_LEDS = 16; // Number of LEDs unused at end of strip
  
  // Polling and timing
  static const int ISR_PER_SEC = 100; // 10ms interrupt
  static const int POLL_REED = 5;
  
  // General
  static uint32_t interruptCount = 0; // incremented in milliseconds
  static char* rxString = NULL; 

  boolean res = 1; 
  uint32_t brightness = 30; 
  uint32_t lightTimeout = 0; 
  uint32_t userTimeout = 12000; // About 1 minute until timeout
  uint8_t partyFade = 5;
  uint8_t partyWave = 20;
  uint8_t basicFade = 2;

/**************************************************************************************/

/* PUBLIC VARIABLES */ 
/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, STRING_PIN, NEO_RGB);
const uint32_t WHITE = strip.Color(255,255,255);
const uint32_t OFF = strip.Color(0,0,0); 

unsigned long programTime = 0; 
uint32_t i, j; 
uint32_t wheelReverse; 

/**************************************************************************************/
// Setup function; runs once at startup
//void(* resetFunc) (void) = 0;
void setup() {
  flag.basic = 1; 
  flag.party = 0; 
  
  strip.setBrightness(brightness); 
  strip.begin();
  strip.show(); 
  
  pinMode(REED_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT); 
  digitalWrite(LED_PIN, LOW); 
  digitalWrite(BUZZER_PIN, LOW);
  Serial.begin(9600); 
  strip_reset(); 
  BLE_Setup(); 
  start_timer(ISR_PER_SEC); 
}

/**************************************************************************************/
// Main loop; executes indefflag.resely 
void loop() {
  static uint8_t pixel = 0;
  static uint32_t fade = 0; 
  programTime = millis(); 

  parseRxString(BLE_Update());  

  /***********************/
  // Interrupt driven polling
  if(flag.interrupt == 1){
    flag.interrupt = 0; 

    /* Poll reed switch */ 
    if(interruptCount % POLL_REED == 0){
      flag.door = digitalRead(REED_PIN); 
    }

    if(flag.door == OPEN){
      if(flag.timeout == FALSE){
        lightTimeout++; 
        if(lightTimeout >= userTimeout){
          flag.timeout = TRUE; 
        }
      }
    }
    else{
      flag.timeout = FALSE; 
      lightTimeout = 0;
    }
  }
  /***********************/

  /* If door has been opened and timeout has not triggered */ 
  if(flag.door == OPEN && flag.timeout == FALSE){

    /* Reset local variables */ 
    if(res == 1){ 
      flag.resetStrip = 1; 
      flag.maxBrightness = 0;
      pixel = 0; 
      j = random(0, 255); 
      i = 0;
      fade = 0; 
      res = 0; 
    }


    /* Party logic */ 
    if(flag.party == TRUE){
      /* Fade in */ 
      uint32_t colorReverse = 0; 
       
      if(fade < brightness){
        if(j < 256){
          for(i=0; i<SIDE_LEDS; i++){
            strip.setPixelColor(i, (Wheel(((i+j) & 255), fade))); 
          }
          
          colorReverse = (NUM_LEDS - END_LEDS);
          wheelReverse = 0; 
          for(i=NUM_LEDS - END_LEDS; i>SIDE_LEDS+MIDDLE_LEDS; i--){
            strip.setPixelColor(colorReverse, (Wheel((((wheelReverse)+j) & 255), fade)));
            colorReverse--;
            wheelReverse++; 
          }

          j++;
          if(j >= 256){
            j = 0;
          }
          fade += partyFade; // Controls speed of fade in 
          strip.show();
          delay(partyWave); 
        } 
      }

      /* Persistent color change after fade in */ 
      else{
        if(j < 256){
          for(i=0; i<SIDE_LEDS; i++){
            strip.setPixelColor(i, (Wheel(((i+j) & 255), brightness))); 
          }

          colorReverse = (NUM_LEDS - END_LEDS);
          wheelReverse = 0; 
          for(i=NUM_LEDS - END_LEDS; i>SIDE_LEDS+MIDDLE_LEDS; i--){
            strip.setPixelColor(colorReverse, (Wheel((((wheelReverse)+j) & 255), brightness)));
            colorReverse--; 
            wheelReverse++; 
          }
          
          j++;
          if(j>=256){
            j=0;
          }
          strip.show(); 
          delay(partyWave); 
        }
      }
    }


    /* Default show */
    else{
      /* Fade in */ 
      if(fade < brightness){
        for(i=0; i<SIDE_LEDS; i++){
          strip.setPixelColor(i, fade, fade, fade); 
        }
        for(i=NUM_LEDS - END_LEDS; i>SIDE_LEDS+MIDDLE_LEDS; i--){
          strip.setPixelColor(i, fade, fade, fade);
        }
        
        fade += basicFade; // controls speed of fade in 
        strip.show(); 
      }

      /* Called once after fade in */ 
      else if(!flag.maxBrightness){ 
        for(i=0; i<SIDE_LEDS; i++){
          strip.setPixelColor(i, brightness, brightness, brightness); 
        }
        for(i=NUM_LEDS - END_LEDS; i>SIDE_LEDS+MIDDLE_LEDS; i--){
          strip.setPixelColor(i, brightness, brightness, brightness);
        }
        
        flag.maxBrightness = 1; 
        strip.show(); 
      }
    }
  }

  /* Reset strip values upon door close */ 
  else if(flag.resetStrip == 1){
    res = 1;
    strip_reset(); 
  }
}


/**************************************************************************************/
// Zeros strip data and pushes erased strip
void strip_reset(){
  flag.resetStrip = 0;  
  digitalWrite(LED_PIN, LOW);
  for(i=0; i<NUM_LEDS; i++){
    strip.setPixelColor(i, OFF);
  }
  strip.show(); 
}


/**************************************************************************************/
// Parses incoming string via bluetooth 
void parseRxString(char* s){
  
    digitalWrite(LED_PIN, HIGH); 
    if (strcmp(s, "ledon") == 0){
      digitalWrite(LED_PIN, HIGH); 
      Serial.println("test");
    }
    
    else if (strcmp(s, "ledoff") == 0){
      digitalWrite(LED_PIN, LOW); 
      Serial.println("test");
    }
  
    /* Reset all settings */ 
    else if(s[0] == 'r'){
      res = 1; 
      flag.basic = 1;
      flag.party = 0; 
      basicFade = 2; 
      partyFade = 2;
      partyWave = 20; 
      brightness = 200; 
      userTimeout = 6000; 

      Serial.println("test"); 
      digitalWrite(BUZZER_PIN, HIGH);
      delay(50);
      digitalWrite(BUZZER_PIN, LOW); 
    }
  
    /* Adjust basic settings */ 
    else if(s[0] == 'b'){
      if(s[1] == 'f'){
        basicFade = atoi(&s[2]);
      }
  
      else{
        flag.basic = 1; 
        flag.party = 0;  
      }
      res = 1;
    }
  
    /* Adjust party settings */ 
    else if(s[0] == 'p'){ 
      if(s[1] == 'w'){
        partyWave = atoi(&s[2]); 
      }
  
      else if(s[1] == 'f'){
        partyFade = atoi(&s[2]);
      }
      
      else{
        flag.party = 1; 
        flag.basic = 0; 
      }
      res = 1; 
    }
  
    /* Adjust timeout setting */ 
    else if(s[0] == 't'){
      userTimeout = atoi(&s[1]);
    }
  
    /* Adjust brightness */ 
    else if(isdigit(*s)){
      int16_t tempNum = atoi(s);
      if(tempNum > 255){
        tempNum = 255; 
      }
      
      if(tempNum < 0){
        tempNum = 0; 
      }
      
      brightness = tempNum; 
      flag.maxBrightness = 0; 
      ble.println("brightness reassigned to:");
      ble.println(brightness);
    }
}


/**************************************************************************************/
// Checks for new data in bluetooth buffer
char* BLE_Update(){

  // Check for user input
  char inputs[BUFSIZE+1];

  // Check for incoming characters from Bluefruit
  ble.println("AT+BLEUARTRX");
  ble.readline();
  if (strcmp(ble.buffer, "OK") == 0) {
    // no data
    return NULL;
  }
  // Some data was found, its in the buffer
  Serial.print(F("[Receive] ")); Serial.println(ble.buffer);
  ble.waitForOK();
  return ble.buffer; 
}

// Bluetooth setup function; runs once at startup
void BLE_Setup(){
  //while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Command Mode Example"));
  Serial.println(F("---------------------------------------"));

  /* resetialise the module */
  Serial.print(F("resetialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  //ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!
  //#ifdef DELAY 
  /* Wait for connection *//*
  while (! ble.isConnected()) {
      delay(500);
  }

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("******************************"));
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    
    Serial.println(F("******************************"));
  }
  //#endif*/
}

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  digitalWrite(LED_PIN, HIGH); 
  while (1);
}

/**************************************************************************************/
// Input a value 0 to 255 to get a color value.
// Also accepts brightness value (0 - 255)
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos, uint32_t intensity) {
  uint8_t red, green, blue;
  float intensityPercent = 0; 

  /* Compute wheel position and intensity percentage */
  WheelPos = 255 - WheelPos;
  intensityPercent = intensity * 100; 
  intensityPercent = intensityPercent / 255; 
  intensityPercent = intensityPercent / 100; 
  
  if(WheelPos < 85){
    red = round((255 - WheelPos * 3) * intensityPercent);
    green = 0;
    blue = round((WheelPos * 3) * intensityPercent); 
    return strip.Color(red, green, blue);
  }
  
  if(WheelPos < 170){
    WheelPos -= 85;
    red = 0;
    green = round((WheelPos * 3) * intensityPercent);
    blue = round((255 - WheelPos * 3) * intensityPercent); 
    return strip.Color(red, green, blue);
  }
  
  WheelPos -= 170;
  red = round((WheelPos * 3) * intensityPercent);
  green = round((255 - WheelPos * 3) * intensityPercent);
  blue = 0; 
  return strip.Color(red, green, blue);
}


/**************************************************************************************/
// resetializes timed interrupt
void set_timer_frequency(int frequencyHz) {
  int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;
  TcCount16* TC = (TcCount16*) TC3;
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  //Serial.println(TC->COUNT.reg);
  //Serial.println(TC->CC[0].reg);
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

// resetializes timed interrupt
void start_timer(int frequencyHz) {
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

  TcCount16* TC = (TcCount16*) TC3;

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use match mode so that the timer counter resets when the count matches the compare register
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Set prescaler to 1024
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  set_timer_frequency(frequencyHz);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  NVIC_EnableIRQ(TC3_IRQn);

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

/**************************************************************************************/
// Timed interrupt handler
void TC3_Handler() {
  TcCount16* TC = (TcCount16*) TC3;
  // If this interrupt is due to the compare register matching the timer count
  // we toggle the LED.
  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;
    // Write callback here!!!
    
    flag.interrupt = 1; 
    interruptCount++; 
    if(interruptCount >= 99999){interruptCount = 0;} 
  }
}
