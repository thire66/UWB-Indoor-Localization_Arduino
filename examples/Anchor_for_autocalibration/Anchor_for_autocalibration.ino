// This program calibrates an ESP32_UWB module intended for use as a fixed anchor point
// uses binary search to find anchor antenna delay to calibrate against a known distance
//
// modified version of Thomas Trojer's DW1000 library is required!

// Remote tag (at origin) must be set up with default antenna delay (library default = 16384)

// user input required, possibly unique to each tag:
// 1) accurately measured distance from anchor to tag
// 2) address of anchor
//
// output: antenna delay parameter for use in final anchor setup.
// S. James Remington 2/20/2022

#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// ESP32_UWB pin definitions

enum UWB_SPI_Pins {
  SPI_SCK = 18,
  SPI_MISO = 19,
  SPI_MOSI = 23,
  PIN_RST = 27,
  PIN_IRQ = 4,
  SPI_CS = 5
};

enum PollDelay {
  POLL_DELAY_102MS = 80, // 80 defines Poll Delay - Duration 102ms
  POLL_DELAY_62MS  = 40, // 40 defines Poll Delay - Duration 62ms
  POLL_DELAY_47MS  = 25  // 25 defines Poll Delay - Duration 47ms
};

char this_anchor_addr[] = "81:00:22:EA:82:60:3B:9C";

// Globale Variablen
uint16_t this_anchor_Adelay = 16384;  //  initial value 16384
float Adelay_delta = 100.0;           //  initial step size
const float this_anchor_target_distance = 2.45;  //  Target distance in metres

bool calibration_done = false;

TaskHandle_t uwbLoopTaskHandle = nullptr;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  //init the configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  uint32_t Default_Replay_Delay_Time = 1500; 
  uint32_t Default_Timer_Delay = Default_Replay_Delay_Time/1000*3+10; 
 
  DW1000Ranging.initCommunication(PIN_RST, SPI_CS, PIN_IRQ, Default_Timer_Delay, Default_Replay_Delay_Time); //Reset, CS, IRQ pin


  Serial.print("Starting Adelay "); Serial.println(this_anchor_Adelay);
  Serial.print("Measured distance "); Serial.println(this_anchor_target_distance);
  
  DW1000.setAntennaDelay(this_anchor_Adelay);

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  //start the module as anchor, don't assign random short address
  DW1000Ranging.startAsAnchor(this_anchor_addr, DW1000.MODE_SHORTDATA_FAST_ACCURACY, DW1000Class::CHANNEL_7);
  xTaskCreate(uwbLoopTask, "UWB_PROC", 4*1024, nullptr, 2, &uwbLoopTaskHandle);
}



void loop() {
  vTaskDelay(portMAX_DELAY);
}

void uwbLoopTask(void *pvParameters) {
  (void)pvParameters;
  for (;;) {
    DW1000Ranging.loop(false);
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void newRange() {
  static float last_delta = 0.0;
  if (calibration_done) return;
  
  float dist = DW1000Ranging.getDistantDevice()->getRange();
  float this_delta = dist - this_anchor_target_distance;

  Serial.print("\nDevice ");
  Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  Serial.print(", dist = ");
  Serial.print(dist, 4);
  Serial.print(" m, delta = ");
  Serial.print(this_delta, 4);
  /*
  // Calibration complete?
  if (Adelay_delta < 1.0f && fabs(this_delta) < 0.05f){
    Serial.print(", final Adelay = ");
    Serial.println(this_anchor_Adelay);
    //while(1);
    delay(1000);
    esp_restart();
  }
  
  // Check sign change → Direction changed → Halve step size
  if (this_delta * last_delta < 0.0f) {
    Adelay_delta /= 2.0f;
    last_delta = 0.0f;  // Reset
  } else {
    last_delta = this_delta;
  }

  // Customise Adelay
  if (this_delta > 0.0f) {
    this_anchor_Adelay += (uint16_t)(Adelay_delta + 0.5f);
  } else {
    this_anchor_Adelay -= (uint16_t)(Adelay_delta + 0.5f);
  }

  // Limitation to valid value range
  this_anchor_Adelay = constrain(this_anchor_Adelay, 0x1000, 0xFFFF);
  DW1000.setAntennaDelay(this_anchor_Adelay);
  Serial.print(", Adelay set to ");
  Serial.println(this_anchor_Adelay);*/
}

void newDevice(DW1000Device *device) {
  Serial.print("Device added: ");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device) {
  Serial.print("delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}