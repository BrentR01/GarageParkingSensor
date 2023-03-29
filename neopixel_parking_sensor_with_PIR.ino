// Libraries
#include <EEPROM.h>
#include <NewPing.h>
#include <FastLED.h>
#include <platforms.h>
#include <SoftwareSerial.h>

// Pin definitions
const int PIR_PIN = 2;
const int BUTTON_PIN = 5;
const int BUTTON_SINK_PIN = 6;
const int LED_PIN = 8;
const int SERIAL_TX_PIN = 12;
const int SERIAL_RX_PIN = 13;
const int TRIGGER_PIN = 9;
const int ECHO_PIN = 10;

// Sensor distance settings
const int MIN_DISTANCE = 25;
const int MAX_DISTANCE = 400;
const int YELLOW_DISTANCE_LENGTH = 150;

// Neopixel settings
const int NUM_PIXELS = 52;
const int BRIGHTNESS = 128;

// EEPROM address for storing red zone distance
const int RED_ZONE_STORE = 0;

// Sonar ping delay in ms
const int SONAR_TRIGGER_DELAY_MS = 60;

// Whether to use serial sonar or not
const bool USE_SERIAL_SONAR = false;


// LIBRARY CALLS
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);


// GLOBAL VARIABLES
unsigned int distance = 0;
unsigned int yellowDistanceStart;
unsigned int redDistanceStart;

CRGB neoPixels[NUM_PIXELS];
SoftwareSerial us100(SERIAL_RX_PIN, SERIAL_TX_PIN);


// SET UP
void setup() {


  Serial.begin(9600);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for Native USB only
  }

  if (USE_SERIAL_SONAR) {
    pinMode(SERIAL_TX_PIN, OUTPUT);
    pinMode(SERIAL_RX_PIN, INPUT);
    us100.begin(9600);
  }

  //pixels.begin(); // This initializes the NeoPixel library.
  FastLED.addLeds<NEOPIXEL, LED_PIN>(neoPixels, NUM_PIXELS);

  //Initial setup of the distance start values
  setupDistance();

  // Define pin button as input and activate the internal pull-up resistor so you don't need a resistor on the button
  // Connect other button pin low to be the GROUND
  pinMode(BUTTON_SINK_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(BUTTON_SINK_PIN, LOW);
  digitalWrite(BUTTON_PIN, HIGH);
  pinMode(PIR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
}



// MAIN LOOP
void loop()

{
  //Check to see if a button is being pushed
  doConfig();

  //Start by setting the distance with a sonar ping
  distance = pingSonar();

  //Read out the Actual Distance and Setpoints for Red and Yellow
  Serial.println(" Act Dist: " + String(distance) + " Red Dist: " + String(redDistanceStart) + " Yellow Dist: " + String(yellowDistanceStart));

  // Green Zone (pull the car in)
  if (digitalRead(PIR_PIN) == LOW)
    for (int pix = 0; pix < NUM_PIXELS; pix++) {
      neoPixels[pix] = CRGB::Black;
    }
  if (distance > yellowDistanceStart) {
    for (int pix = 0; pix < NUM_PIXELS; pix++) {
      neoPixels[pix] = CHSV(HUE_GREEN, 255, BRIGHTNESS);
    }
  }
  // Yellow Zone (slow the car down) with blue leds increasing to the start of the red zone
  if (digitalRead(PIR_PIN) == LOW)
    for (int pix = 0; pix < NUM_PIXELS; pix++) {
      neoPixels[pix] = CRGB::Black;
    }
  else if (distance <= yellowDistanceStart && distance > redDistanceStart) {
    // ratio between the current distance and it's position between the yellow and red starting distances
    double distanceRatio = double(distance - redDistanceStart) / double(yellowDistanceStart - redDistanceStart);

    // gets the number of pixels to light from the distance ratio
    int pixelsToLight = double(NUM_PIXELS) * (1 - distanceRatio);

    // set the pixels
    for (int pix = 0; pix <= NUM_PIXELS; pix++) {
      if (pix < pixelsToLight) neoPixels[pix] = CHSV(HUE_BLUE, 255, BRIGHTNESS);
      else neoPixels[pix] = CHSV(HUE_YELLOW, 255, BRIGHTNESS / 2);
    }
  }
  // Red Zone (stop the car)
  if (digitalRead(PIR_PIN) == LOW)
    for (int pix = 0; pix < NUM_PIXELS; pix++) {
      neoPixels[pix] = CRGB::Black;
    }
  else if (distance <= redDistanceStart && distance > 0) {
    for (int pix = 0; pix < NUM_PIXELS; pix++) {
      neoPixels[pix] = CHSV(HUE_RED, 255, BRIGHTNESS);
    }
  }

  //Show the updated LEDs
  FastLED.show();
}

// Get back a centimeter measurement from the sonar
unsigned int pingSonar() {
  unsigned int cmOut = MAX_DISTANCE;

  // Try for a good value 10 times, to weed out erroneous values.
  // I'm not sure why the sensor sometimes returns bad data.
  for (int i = 0; i < 10; i++) {
    if (USE_SERIAL_SONAR) {

      us100.flush();                 // clear receive buffer of serial port
      us100.write(0X55);             // trig US-100 begin to measure the distance
      while (us100.available() < 2)  // Wait for the first two bytes to be in the "buffer"
      {
        delay(1);
      }
      // Equation is High Byte (first one) * 256 + Low Byte (second one) to mm / 10 to get cm
      // The whole app could be converted to inches or whatever units you want with a couple small changes here
      // but really the value is irrelevant because it's just lighting LEDS.  Just make sure you check the
      // MIN_DISTANCE, MAX_DISTANCE, YELLOW_DISTANCE_LENGTH, if you adjust the units.
      cmOut = (us100.read() * 256 + us100.read()) / 10.0;
    } else  // using a non-serial sonar. This is untested.
    {
      delay(SONAR_TRIGGER_DELAY_MS);
      cmOut = sonar.ping_cm();
    }

    if (cmOut < MAX_DISTANCE) return cmOut;  //probably a good value, return it

    Serial.println("Oops! Got " + String(cmOut) + "cm, which is out of bounds.");
  }
  // Couldn't get a good value? Return it anyways.
  return cmOut;
}

// Button press checker, procedure saves new parking distance to the EEPROM
void doConfig() {
  if (digitalRead(BUTTON_PIN) == LOW)  // button pressed
  {

    //clear the led
    for (int pix = 0; pix < NUM_PIXELS; pix++) {
      neoPixels[pix] = CRGB::Black;
    }
    FastLED.show();

    int litPixel = 0;
    //scroll through the pixels while the button is pushed, when the pixels are all lit, start setting the distance
    while (digitalRead(BUTTON_PIN) == LOW && litPixel < NUM_PIXELS) {
      Serial.println("Pixel = " + String(litPixel) + " NUM_PIXELS = " + String(NUM_PIXELS));

      neoPixels[litPixel] = CRGB::Green;
      FastLED.show();

      //if all the pixels are lit, start saving the settings
      if (litPixel++ == NUM_PIXELS - 1) {

        // flash the led 3 times to allow human escape from sensor reading
        flashLed(HUE_BLUE, 1000, 3);

        // Take 5 readings to get an average distance
        int sum = 0;
        int smoothing = 5;
        for (int i = 0; i < smoothing; i++) {
          int current = pingSonar();
          Serial.println("Reading " + String(i + 1) + ": " + current + "cm");
          sum = sum + pingSonar();
          delay(SONAR_TRIGGER_DELAY_MS);
        }
        int average = sum / smoothing;

        // Save the new distance to eeprom

        Serial.print("Saving new average redDistanceStart = " + average);
        //write the new value to the EEPROM
        EEPROM.write(RED_ZONE_STORE, average);
        //set the loop to use the new values
        setupDistance();
        //flash the led to confirm saved
        flashLed(HUE_PURPLE, 100, 3);
        //exit the loop now that the settings are saved
        break;
      }
      //delay before grabbing the next pixel.  Total button hold time will be NUM_PIXELS times this delay in ms
      delay(100);
    }
  }
}

// Get the configured distances from EEPROM and set the global vars
void setupDistance() {
  Serial.println("Current redDistanceStart = " + redDistanceStart);
  //read the stored value for the red distance (where the car stops)
  redDistanceStart = EEPROM.read(RED_ZONE_STORE);
  Serial.println("Read redDistanceStart = " + redDistanceStart);

  //check to make sure the red distance isn't ever less than the min
  if (redDistanceStart < MIN_DISTANCE) {
    Serial.print("Red Distance set to min");
    redDistanceStart = MIN_DISTANCE;
    EEPROM.write(RED_ZONE_STORE, redDistanceStart);
  }

  // set the distances the loop uses
  // might want to add some logic to make sure the distances don't go greater than the max distance.
  redDistanceStart = EEPROM.read(RED_ZONE_STORE);
  yellowDistanceStart = redDistanceStart + YELLOW_DISTANCE_LENGTH;

  Serial.println("Red: " + String(redDistanceStart) + " Yellow: " + String(yellowDistanceStart));
}

// Helper method to flash the led.  Does not set the LEDs back to the previous color settings.
void flashLed(uint8_t color, int duration, int flashes) {
  for (int f = 1; f <= flashes; f++) {
    for (int pix = 0; pix < NUM_PIXELS; pix++) {
      neoPixels[pix] = CHSV(color, 255, BRIGHTNESS);
    }
    FastLED.show();
    FastLED.delay(duration);
    for (int pix = 0; pix < NUM_PIXELS; pix++) {
      neoPixels[pix] = CHSV(color, 255, BRIGHTNESS / 3);
    }
    FastLED.show();
    FastLED.delay(duration);
  }
}
