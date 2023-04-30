//https://github.com/adafruit/Adafruit_NeoPixel
#include <Adafruit_NeoPixel.h>

// esp32 board must be installed
#include "esp_camera.h"
#include <WiFi.h>

// not needed as no internet is currently being used
// Replace with your own Wi-Fi credentials
const char* ssid = "your_SSID";
const char* password = "your_PASSWORD";

// Camera and motor control pins
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

// Motor control pins
#define MOTOR_PIN1 14
#define MOTOR_PIN2 15

// Ultrasonic sensor pins
#define TRIGGER_PIN 16
#define ECHO_PIN 17

// Initialize motor control
// main code for movingtowards garbage
void moveTowardsGarbage() {
  digitalWrite(MOTOR_PIN1, HIGH);
  digitalWrite(MOTOR_PIN2, LOW);
}

// controls of motor need work according to configuration of motors
void moveForward() {
  digitalWrite(MOTOR_PIN1, HIGH);
  digitalWrite(MOTOR_PIN2, LOW);
}

void moveBackward() {
  // to be implemented
}

void turnLeft() {
  // to be implemented
}

void turnRight() {
  // to be implemented
}

void stopMotor() {
  digitalWrite(MOTOR_PIN1, LOW);
  digitalWrite(MOTOR_PIN2, LOW);
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // Set up motor control pins
  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);

  // Set up ultrasonic sensor pins
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  //initialize camera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = 5;
  config.pin_d1 = 18;
  config.pin_d2 = 19;
  config.pin_d3 = 21;
  config.pin_d4 = 36;
  config.pin_d5 = 39;
  config.pin_d6 = 34;
  config.pin_d7 = 35;
  config.pin_xclk = 0;
  config.pin_pclk = 22;
  config.pin_vsync = 25;
  config.pin_href = 23;
  config.pin_sscb_sda = 26;
  config.pin_sscb_scl = 27;
  config.pin_pwdn = 32;
  config.pin_reset = -1;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_UXGA;
  config.jpeg_quality = 10;
  config.fb_count = 2;

   // Initialize the camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
}

void loop() {
  
  // Capture an image
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  // Measure the distance to the nearest object using the ultrasonic sensor
  float distance = measureDistance();

  // Process the image to detect floating garbage
  bool garbageDetected = detectGarbage(fb->buf, fb->len);

  // Free the image buffer
  esp_camera_fb_return(fb);

  // Control the motor or actuator to move towards the detected garbage

  if (garbageDetected && distance > 0 && distance < 100) {
    // moves continously need to calculate time as distance/speed of motor
    moveTowardsGarbage();
    delay(5000);
    stopMotor();

  } else {
    stopMotor();
  }

  delay(1000);
}

float measureDistance() {
  // Send a 10us pulse to the trigger pin
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  // Measure the duration of the echo pulse
  float duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate the distance in centimeters
  float distance = duration * 0.034 / 2;

  return distance;
}

bool detectGarbage(uint8_t *imageData, size_t imageSize) {
  // Create a bitmap from the image data
  Adafruit_NeoPixel pixels = Adafruit_NeoPixel(64, 0, NEO_GRB + NEO_KHZ800);
  pixels.begin();
  pixels.show();

  // Count the number of dark pixels in the bitmap
  uint32_t darkPixels = 0;
  uint8_t threshold = 80; // Adjust this value based on the color of the garbage and water

  for (uint16_t i = 0; i < pixels.numPixels(); i++) {
    uint32_t color = pixels.getPixelColor(i);
    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;

    // Check if the pixel is darker than the threshold
    if (r < threshold && g < threshold && b < threshold) {
      darkPixels++;
    }
  }

  // Calculate the percentage of dark pixels in the bitmap
  float darkPixelPercentage = (float)darkPixels / (float)pixels.numPixels() * 100.0;

  // Determine if the image contains garbage based on the percentage of dark pixels
  return darkPixelPercentage > 5.0; // Adjust this value based on your requirements
}
