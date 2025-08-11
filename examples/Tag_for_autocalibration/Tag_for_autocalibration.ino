#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

enum UWB_SPI_Pins {
  SPI_SCK = 18,
  SPI_MISO = 19,
  SPI_MOSI = 23,
  PIN_RST = 27,
  PIN_IRQ = 4,
  SPI_CS = 5
};

// TAG antenna delay defaults to 16384
char tag_addr[] = "7A:00:22:EA:82:60:3B:9C";
TaskHandle_t uwbLoopTaskHandle = nullptr;

void setup() {
  Serial.begin(115200);
  delay(1000);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  uint32_t Default_Replay_Delay_Time = 1500; 
  uint32_t Default_Timer_Delay = Default_Replay_Delay_Time/1000*3+8; 
  DW1000Ranging.initCommunication(PIN_RST, SPI_CS, PIN_IRQ, Default_Timer_Delay, Default_Replay_Delay_Time); //Reset, CS, IRQ pin
  DW1000.setAntennaDelay(16384);
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_SHORTDATA_FAST_ACCURACY, DW1000Class::CHANNEL_7); 
  xTaskCreate(uwbLoopTask, "UWB_PROC", 4*1024, nullptr, 2, &uwbLoopTaskHandle);
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}

void uwbLoopTask(void *pvParameters) {
  (void)pvParameters;
  for (;;) {
    DW1000Ranging.loop(true);
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void newRange() {
  Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  Serial.print(",");
  Serial.println(DW1000Ranging.getDistantDevice()->getRange());
}

void newDevice(DW1000Device *device) {
  Serial.print("Device added: ");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device) {
  Serial.print("delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}