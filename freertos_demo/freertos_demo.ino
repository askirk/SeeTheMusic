#include <Arduino_FreeRTOS.h>
#include <Adafruit_NeoPixel.h>
#include <semphr.h>  // add the FreeRTOS functions for Semaphores (or Flags).

// IMPORTANT: FFT_N should be #defined as 128 in ffft.h.

// Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
// It will be used to ensure only only one Task is accessing this resource at any time.
SemaphoreHandle_t xSerialSemaphore;

#include <avr/pgmspace.h>
#include <ffft.h>
#include <math.h>
#include <Wire.h>

// PIN for NeoPixel Ring
#define PIN 6

// Microphone connects to Analog Pin 0.  Corresponding ADC channel number
// varies among boards...it's ADC0 on Uno and Mega, ADC7 on Leonardo.
// Other boards may require different settings; refer to datasheet.
#define ADC_CHANNEL 0

const int sampleWindow = 5; 

/*
These tables were arrived at through testing, modeling and trial and error,
exposing the unit to assorted music and sounds.  But there's no One Perfect
EQ Setting to Rule Them All, and the graph may respond better to some
inputs than others.  The software works at making the graph interesting,
but some columns will always be less lively than others, especially
comparing live speech against ambient music of varying genres.
*/

// define two tasks for Blink & AnalogRead
void TaskBlink( void *pvParameters );
void AudioSample(void *pvParameters);
void TaskLightShow(void *pvParameters);

//Queue
QueueHandle_t xQueue1;

// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(12, PIN, NEO_GRB + NEO_KHZ800);

// the setup function runs once when you press reset or power the board
void setup() {

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  strip.begin();
  strip.setBrightness(32);
  strip.show(); // Initialize all pixels to 'off'

  xQueue1 = xQueueCreate( FFT_N, sizeof( int16_t ) );

  if ( xSerialSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerialSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xSerialSemaphore ) != NULL )
      xSemaphoreGive( ( xSerialSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }

  // Now set up two tasks to run independently.
  xTaskCreate(
    TaskAnalogRead
    ,  (const portCHAR *) "Sample"   // A name just for humans
    ,  128  // Stack size
    ,  NULL
    ,  2  // priority
    ,  NULL );

    xTaskCreate(
    TaskLightShow
    ,  (const portCHAR *) "Blink"   // A name just for humans
    ,  128  // Stack size
    ,  NULL
    ,  2  // priority
    ,  NULL );

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
  vTaskStartScheduler();
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/
void TaskBlink(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  

  int i = 0;
  for(;;){
    if(i > 11)
      i = 0;
    strip.setPixelColor(i, 255, 0, 255);
    if(i != 0)
      strip.setPixelColor(i-1, 0, 0, 0);
    else
      strip.setPixelColor(11, 0, 0, 0);
    strip.show();
    i++;

    vTaskDelay( 50 / portTICK_PERIOD_MS );
  }
}


void TaskLightShow(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  int scaled;
  int i = 0;

  for (;;) // A Task shall never return or exit.
  {
    if ( true )
    {
      if( xQueue1 != 0 )
      {
        // Receive a message on the created queue.  Block for 10 ticks if a
        // message is not immediately available.
        if( xQueueReceive( xQueue1, &( scaled ), ( TickType_t ) 0 ) )
        {
            // pcRxedMessage now points to the struct AMessage variable posted
            // by vATask.
        }
      }
      if(scaled >= 0 && scaled <= 240){
        //scaled = scaled % 12;
        scaled = map(scaled, 0, 130, 0, 11);
        //Serial.println("scaled"+scaled);
        //scaled = (11.0/(80.0 - 60.0)) * (scaled - (80.0 - 60.0)) + 11;

        for(i = 0; i < 12; i++){
          if(i <= scaled){
            if(i < 4){
              strip.setPixelColor(i, 0, 255, 0); // turn on
            }else if(i < 8){
              strip.setPixelColor(i, 255, 255, 0); // turn on
            }else{
              strip.setPixelColor(i, 255, 0, 0); // turn on
            }
          }else{
            strip.setPixelColor(i, 0, 0, 0);
          }
        }

        strip.show();
      }
    }
   
    vTaskDelay( 50 / portTICK_PERIOD_MS ); // wait .5 s
  }
}

void TaskAnalogRead(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
 
  for (;;)
  {
    if ( true )
    {
      unsigned long startMillis= millis(); // Start of sample window
      unsigned int peakToPeak = 0; // peak-to-peak level
      unsigned int signalMax = 0;
      unsigned int signalMin = 1023;
      unsigned int sample;
      // collect data for 50 mS
      while (millis() - startMillis < sampleWindow)
      {
        sample = analogRead(ADC_CHANNEL);
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
     
      // print out the value you read:
      Serial.println(peakToPeak);
      xQueueSend( xQueue1, ( void * ) &peakToPeak, ( TickType_t ) 0 );
    }
    
    vTaskDelay(50 / portTICK_PERIOD_MS);  // one tick delay (1ms) in between reads for stability
  }
}




