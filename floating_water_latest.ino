#include "esp_camera.h"
#include "img_converters.h"
#include "Arduino.h"

// the pins for the ESP32-CAM
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

// Define the threshold values for garbage detection
#define RED_THRESHOLD     100
#define GREEN_THRESHOLD   100
#define BLUE_THRESHOLD    100

void setup() {
  Serial.begin(115200);

  // Initialize the ESP32-CAM
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
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = 22;
  config.pin_vsync = 25;
  config.pin_href = 23;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // Initialize the camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
}

void loop() {
  // Capture an image from the camera
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  // Convert the image to RGB format
  size_t out_len;
  uint8_t out_buf[fb->len * 3];
  fmt2rgb888(fb->buf, fb->len, fb->format, out_buf);

  // Count the number of dark pixels in the image
  int dark_pixels = 0;
  for (int i = 0; i < out_len; i += 3) {
    // Check if the pixel is dark
    if (out_buf[i] < RED_THRESHOLD && out_buf[i+1] < GREEN_THRESHOLD && out_buf[i+2] < BLUE_THRESHOLD) {
      dark_pixels++;
    }
  }

  // Calculate the percentage of dark pixels in the image
  float dark_pixel_percentage = (float)dark_pixels / (float)(out_len / 3) * 100.0;

  // Print the percentage of dark pixels to the serial monitor
  Serial.print("Percentage of dark pixels: ");
  Serial.print(dark_pixel_percentage);
  Serial.println("%");

  // Free the memory used by the image buffers
  free(out_buf);
  esp_camera_fb_return(fb);
}
