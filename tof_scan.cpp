#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// Define XSHUT pins for each sensor
#define XSHUT_FRONT 35
#define XSHUT_LEFT  36
#define XSHUT_RIGHT 37
#define XSHUT_BACK  38

// Create sensor objects
Adafruit_VL53L0X sensor_front = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor_left  = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor_right = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor_back  = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);

  // Initialize I2C on GPIO 39 (SDA) and 40 (SCL)
  Wire.begin(39, 40);  // SDA = GPIO 39, SCL = GPIO 40

  // Set XSHUT pins as OUTPUT
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  pinMode(XSHUT_BACK, OUTPUT);

  // Disable all sensors first
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  digitalWrite(XSHUT_BACK, LOW);
  delay(10);

  // Initialize each sensor one by one and assign unique I2C addresses

  // Front Sensor - 0x30
  digitalWrite(XSHUT_FRONT, HIGH);
  delay(10);
  if (!sensor_front.begin(0x30, &Wire)) {
    Serial.println("Failed to initialize front sensor!");
    while (1);
  }

  // Left Sensor - 0x31
  digitalWrite(XSHUT_LEFT, HIGH);
  delay(10);
  if (!sensor_left.begin(0x31, &Wire)) {
    Serial.println("Failed to initialize left sensor!");
    while (1);
  }

  // Right Sensor - 0x32
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(10);
  if (!sensor_right.begin(0x32, &Wire)) {
    Serial.println("Failed to initialize right sensor!");
    while (1);
  }

  // Back Sensor - 0x33
  digitalWrite(XSHUT_BACK, HIGH);
  delay(10);
  if (!sensor_back.begin(0x33, &Wire)) {
    Serial.println("Failed to initialize back sensor!");
    while (1);
  }

  Serial.println("All sensors initialized successfully.");
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;

  // Front
  sensor_front.rangingTest(&measure, false);
  Serial.print("Front: ");
  Serial.print(measure.RangeStatus != 4 ? measure.RangeMilliMeter : -1);
  Serial.print(" mm, ");

  // Left
  sensor_left.rangingTest(&measure, false);
  Serial.print("Left: ");
  Serial.print(measure.RangeStatus != 4 ? measure.RangeMilliMeter : -1);
  Serial.print(" mm, ");

  // Right
  sensor_right.rangingTest(&measure, false);
  Serial.print("Right: ");
  Serial.print(measure.RangeStatus != 4 ? measure.RangeMilliMeter : -1);
  Serial.print(" mm, ");

  // Back
  sensor_back.rangingTest(&measure, false);
  Serial.print("Back: ");
  Serial.println(measure.RangeStatus != 4 ? measure.RangeMilliMeter : -1);

  delay(300);
}
