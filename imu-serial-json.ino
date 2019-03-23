#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <ArduinoJson.h>

uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;
uint16_t PRINT_DELAY_MS = 20;
uint16_t printCount = 0;

Adafruit_BNO055 bno = Adafruit_BNO055(55);

/**
 * Setup
 */
void setup() {
  Serial.begin(115200);

  if (!bno.begin()) {
    Serial.print("No BNO055 detected");
    while (1);
  }

  delay(1000);
}

/**
 * Reset
 */
void(* reset) (void) = 0;

/**
 * Loop
 */
void loop() {
  if (Serial.available() > 0) {
    if (Serial.peek() == 's') {
      Serial.read();

      if (Serial.parseInt() == 0) {
        reset();
      }
    }

    while (Serial.available() > 0) {
      Serial.read();
    }
  }

  unsigned long tStart = micros();
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  //enough iterations have passed that we can print the latest data
  if (printCount * BNO055_SAMPLERATE_DELAY_MS >= PRINT_DELAY_MS) {
    serializeHeading(orientationData.orientation.x);
    printCount = 0;
  } else {
    printCount = printCount + 1;
  }

  //poll until the next sample is ready
  while ((micros() - tStart) < (BNO055_SAMPLERATE_DELAY_MS * 1000)) {}
}

/**
 * Serialize heading response
 * @param {float} heading
 */
void serializeHeading(float heading) {
  String output;

  const size_t capacity = JSON_OBJECT_SIZE(1);
  DynamicJsonBuffer jsonBuffer(capacity);

  JsonObject& root = jsonBuffer.createObject();
  root["heading"] = heading;

  root.printTo(output);
  Serial.println(output);
}
