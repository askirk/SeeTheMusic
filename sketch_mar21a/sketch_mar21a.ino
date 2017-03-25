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
#ifdef __AVR_ATmega32U4__
 #define ADC_CHANNEL 7
#else
 #define ADC_CHANNEL 0
#endif

int32_t sensorValue;
int16_t       capture[FFT_N];    // Audio capture buffer
complex_t     bfly_buff[FFT_N];  // FFT "butterfly" buffer
uint16_t      spectrum[FFT_N/2]; // Spectrum output buffer
volatile byte samplePos = 0;     // Buffer position counter

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

  // initialize serial communication at 9600 bits per second
  Serial.begin(9600);

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  xQueue1 = xQueueCreate( 5, sizeof( unsigned int ) );

  if ( xSerialSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerialSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xSerialSemaphore ) != NULL )
      xSemaphoreGive( ( xSerialSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }

  // Now set up two tasks to run independently.
  xTaskCreate(
    TaskBlink
    ,  (const portCHAR *) "Blink"   // A name just for humans
    ,  128  // Stack size
    ,  NULL
    ,  1  // priority
    ,  NULL );

  
    xTaskCreate(
    TaskSampleAudio
    ,  (const portCHAR *) "Sample"   // A name just for humans
    ,  128  // Stack size
    ,  NULL
    ,  1  // priority
    ,  NULL );
    
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.

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

    vTaskDelay( 50 / portTICK_PERIOD_MS); /// portTICK_PERIOD_MS
  }
}


void TaskLightShow(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  int scaled;
  int last_val = 0;
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
      if(scaled >= 0){
        //scaled = scaled % 12;
        scaled = (11.0/1023.0) * (scaled - 1023.0) + 11;

        for(i = 0; i < 12; i++){
          if(i <= scaled)
            strip.setPixelColor(i, 255, 0, 255);
          else
            strip.setPixelColor(i, 0, 0, 0);
        }

        strip.show();
        last_val = scaled;
      }
    }
   
    vTaskDelay( 50 ); // wait .5 s
  }
}

void TaskSampleAudio(void *pvParameters)  // This is a task.
{
  for(;;){
    
    static const int16_t noiseThreshold = 4;
    int16_t              sample         = analogRead(ADC_CHANNEL); // 0-1023
  
    capture[samplePos] =
      ((sample > (512-noiseThreshold)) &&
       (sample < (512+noiseThreshold))) ? 0 :
      sample - 512; // Sign-convert for FFT; -512 to +511

    Serial.println(capture[samplePos]);
  
    if(++samplePos >= FFT_N){
      xSemaphoreGive(xSerialSemaphore);
    }

    vTaskDelay( 1 / portTICK_PERIOD_MS);
  }
}





