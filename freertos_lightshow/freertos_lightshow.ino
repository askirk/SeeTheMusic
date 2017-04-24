#include <Arduino_FreeRTOS.h>
#include <Adafruit_NeoPixel.h>
#include <semphr.h>  // add the FreeRTOS functions for Semaphores (or Flags).
#include <avr/pgmspace.h>
#include <ffft.h>
#include <math.h>
#include <Wire.h>

#define RING_PIN 6               // PIN for NeoPixel Ring
#define ADC_SOUND_CHANNEL 0 // PIN for Microphone AMP
#define ADC_LIGHT_CHANNEL 1 // PIN for Photoresistor
#define BUTTON_PIN 8        // PIN for Pushbutton

const int sampleWindow = 5; 
enum MODE {EQ, COUNT, QUAD};
MODE LightMode = EQ; // Global light mode

//Queue
QueueHandle_t xQueue1;

// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
// Learned from https://learn.adafruit.com/adafruit-neopixel-uberguide/arduino-library
Adafruit_NeoPixel strip = Adafruit_NeoPixel(12, RING_PIN, NEO_GRB + NEO_KHZ800);

// the setup function runs once when you press reset or power the board
void setup() {

  pinMode(BUTTON_PIN, INPUT_PULLUP);  

  strip.begin();
  strip.setBrightness(32);
  strip.show(); // Initialize all pixels to 'off'

  xQueue1 = xQueueCreate( FFT_N, sizeof( int16_t ) );

  
  // Now set up three tasks
 
  xTaskCreate(
  TaskAnalogRead
  ,  (const portCHAR *) "Analog"   
  ,  128  // Stack size
  ,  NULL
  ,  2 // priority
  ,  NULL );
  
  xTaskCreate(
  TaskLightShow
  ,  (const portCHAR *) "Display"   
  ,  128  // Stack size
  ,  NULL
  ,  2  // priority
  ,  NULL );

  
  xTaskCreate(
  TaskDigitalRead
  ,  (const portCHAR *) "Digital"
  ,  128  // Stack size
  ,  NULL
  ,  2  // priority
  ,  NULL );

  // Now the task scheduler, which takes over control of scheduling individual tasks, is started.
  vTaskStartScheduler();
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskLightShow(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  int scaled;
  int i = 0;
  int pointer = 0;

  for (;;) // A Task shall never return or exit.
  {
   
  
    if( xQueue1 != 0 )
    {
      // Receive a message on the created queue.  Block for 30 ticks if a
      // message is not immediately available.
      if( xQueueReceive( xQueue1, &( scaled ), ( TickType_t ) 30 ) )
      {
          // blank
      }
    }
    if(scaled >= 0 && scaled <= 240){

      if(LightMode == EQ){
        int EQ_scaled = map(scaled, 0, 240, 0, 11);
        for(i = 0; i < 12; i++){
          if(i <= EQ_scaled){
            if(i < 4){
              strip.setPixelColor(i, 0, 255, 0); // turn on GREEN
            }else if(i < 8){
              strip.setPixelColor(i, 255, 255, 0); // turn on YELLOW
            }else{
              strip.setPixelColor(i, 255, 0, 0); // turn on RED
            }
          }else{
            strip.setPixelColor(i, 0, 0, 0);
          }
        }
      }else if(LightMode == COUNT){
        int COUNT_scaled = map(scaled, 0, 240, 0, 11);
        int counter = pointer + COUNT_scaled;
        for(i = 0; i < 12; i++){
          if(i >= pointer && i <= counter && counter <= 11){
            strip.setPixelColor(i, 255/i, 20*i, i*12); // turn on variable color
            pointer = i;
          }else if(i <= counter % 11 && counter > 11){
            strip.setPixelColor(i, 255/i, 20*i, i*12); // turn on variable color
            pointer = i;
          }else{
            strip.setPixelColor(i, 0, 0, 0); // turn off
          }
        }
      }else{ // LightMode == QUAD
        int QUAD_scaled = map(scaled, 0, 130, 0, 11);
        
        for(int z = 0; z < 4; z++){
          strip.setPixelColor(3*z, 255, 0, 0); // red
          if(QUAD_scaled % 3 >= 1){
            strip.setPixelColor(3*z + 1, 25, 25, 112); // blue
          }else{
            strip.setPixelColor(3*z + 1, 0, 0, 0); // off
          }
          if(QUAD_scaled % 3 == 2){
            strip.setPixelColor(3*z + 2, 255, 0, 255); // purple
          }else{
            strip.setPixelColor(3*z + 2, 0, 0, 0); // off
          }
        }
      }    
      
      strip.show();
    }
 
    vTaskDelay( 70 / portTICK_PERIOD_MS ); // wait .07 s
  }
}

void TaskAnalogRead(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  int prev_brightness = 0;
 
  for (;;)
  {
    
    if ( true )
    {
      unsigned long startMillis= millis(); // Start of sample window
      unsigned int peakToPeak = 0; // peak-to-peak level
      unsigned int signalMax = 0;
      unsigned int signalMin = 1023;
      unsigned int sample;
      // collect data for 5 mS
      // code adapted from https://learn.adafruit.com/adafruit-agc-electret-microphone-amplifier-max9814/overview 
      while (millis() - startMillis < sampleWindow)
      {
        sample = analogRead(ADC_SOUND_CHANNEL);
        if (sample < 1024) // toss out spurious readings
        {
          if (sample > signalMax){
            signalMax = sample; // save just the max levels
          }else if (sample < signalMin){
            signalMin = sample; // save just the min levels
          }
        }
      }

      peakToPeak = signalMax - signalMin; // max - min = peak-peak amplitude

      xQueueSend( xQueue1, ( void * ) &peakToPeak, ( TickType_t ) 0 );

      int brightness = analogRead(ADC_LIGHT_CHANNEL);
      map(brightness, 0, 1023, 0, 255);
      
      if(abs(brightness - prev_brightness) > 2){
        strip.setBrightness(brightness);
      }

      prev_brightness = brightness;
    
    }
    
    vTaskDelay(70 / portTICK_PERIOD_MS);  // .07 sec in between reads for quick response
  }
}

void TaskDigitalRead(void *pvParameters)
{
  (void) pvParameters;
  bool press = false;
  unsigned long debounce_start; // have to debounce the button
 
  for (;;)
  {
    unsigned long time = millis();
    Serial.println(time);
    
    int val = digitalRead(BUTTON_PIN);

    if(val > 0 && press == false){
      press = true;
      debounce_start = millis();
      changeMode();
    }else if (val == 0 && press == true){
      if(millis() - debounce_start >= 500){ // debounce for 500 ms
        press = false;
      }
    }
    
    vTaskDelay(205 / portTICK_PERIOD_MS); // check again in 205 ms
  }
}

void changeMode(){ // subroutine to change mode
  if(LightMode == EQ){
    LightMode = COUNT;
  }else if(LightMode == COUNT){
    LightMode = QUAD;
  }else{
    LightMode = EQ;
  }
}




