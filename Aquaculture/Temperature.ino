#include <OneWire.h> 
#include <DallasTemperature.h>
#include <SoftwareSerial.h>
#include "esp_timer.h"
esp_timer_handle_t periodic_timer;
int periodic_count = 0;

//============================================================== Sensor Pin
#define ONE_WIRE_BUS 27

//============================================================== Temperature Sensor
OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature sensors(&oneWire);
float tmpValue = 0;

void setup() {
  delay(3000);
  Serial.begin(115200);
  Serial.println("Temperature sensor test begin");
  esp_timer_init();
  periodic_init();
  sensors.begin();

  // List all detected sensor addresses
  int numSensors = sensors.getDeviceCount();
  Serial.print("Number of sensors found: ");
  Serial.println(numSensors);
  for (int i = 0; i < numSensors; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" Address: ");
  }
}

void loop() {
}

//===================== Periodic Timer =====================
void periodic_init(){
  esp_timer_create_args_t timer_args = {
    .callback = &periodic_callback,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "periodic_timer"
  };
  esp_timer_create(&timer_args, &periodic_timer); //Ubah
  esp_timer_start_periodic(periodic_timer, 1000 * 2000); 
}

void periodic_callback(void* arg) {
  //============================================================== Temperature Sensor
  sensors.requestTemperatures(); // Send the command to get temperature readings
  // the index 0 refers to the first device
  if (sensors.getTempCByIndex(0) > 0) {
    tmpValue = sensors.getTempCByIndex(0);
    Serial.print("tmpValue: "); Serial.println(tmpValue);
  }
}