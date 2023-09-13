#include "esp_timer.h"
#include <WiFi.h>
#include <MQTT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SoftwareSerial.h>
//============================================================== Sensor Pin

float NTU = 0;
#define ONE_WIRE_BUS 27
int pHPin = 35;
#define TdsSensorPin 32
#define turbidity 34
int pump = 18;
//============================================================== pH Sensor
unsigned long int avgValue;  //Store the average value of the sensor feedback
float b;
int buf[10], temp;
int pHValue = 0;
//============================================================== Temperature Sensor
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float tmpValue = 0;

//============================================================== Salinity Sensor
#define VREF 3.3      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, temperature = 25;

//============================================================== MQTT BY Joel Gaehwiller
MQTTClient client;
const char clientId[] = "KVESP111"; //MQTT ID
const char server[]   = "broker.emqx.io"; //Broker
const char publishmqtt1[] = "KV/18/temp"; //Publish
const char publishmqtt2[] = "KV/18/ph"; //Publish
const char publishmqtt3[] = "KV/18/turbidity"; //Publish
const char publishmqtt4[] = "KV/18/salinity"; //Publish
const char subscribemqtt[] = "KV/18/esp32"; //Subscribe own
int stopsubscribe = 0;
int publishnow = 0;
//============================================================== WIFI SETTING
WiFiClient net;
const char ssid[]     = "SEA-IC";
const char password[] = "seaic2022";
//const char ssid[]     = "KVAqua";
//const char password[] = "kv12345678";
//============================================================== TIMER SETTING
esp_timer_handle_t periodic_timer;
//============================================================== Connect WIFI MQTT
void connect_wifi_mqtt() {
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("NO WIFI");
  }
  while (!client.connect(clientId)) {
    delay(500);
    Serial.println("NO MQTT");
  }
  Serial.println("YES Connected");
  client.subscribe(subscribemqtt);
}

void setup() {
  delay(3000);
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  client.begin(server, 1883, net);
  connect_wifi_mqtt();
  client.onMessage(messageReceived);
  esp_timer_init();
  periodic_init();
  sensors.begin();
  pinMode(turbidity, INPUT);
  pinMode(pHPin, INPUT);
  pinMode(TdsSensorPin, INPUT);
  Serial.println("ESP test begin");
  pinMode(pump, OUTPUT);
  digitalWrite(pump, HIGH);
}

void loop() {
  client.loop();
  if (!client.connected()) {
    connect_wifi_mqtt();
  }
}

//==============================================================  Periodic Timer
void periodic_init() {
  esp_timer_create_args_t timer_args = {
    .callback = &periodic_callback,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "periodic_timer"
  };
  esp_timer_create(&timer_args, &periodic_timer);
  esp_timer_start_periodic(periodic_timer, 1000 * 5000);
}

void periodic_callback(void* arg) {



  //============================================================== Temperature Sensor
  sensors.requestTemperatures(); // Send the command to get temperature readings
  // the index 0 refers to the first device
  if (sensors.getTempCByIndex(0) > 0) {
    tmpValue = sensors.getTempCByIndex(0);
    Serial.print("tmpValue: "); Serial.println(tmpValue);
  }

  client.publish(publishmqtt1, String(tmpValue));

  //============================================================== pH Sensor
  //Get 10 sample value from the sensor for smooth the value
  for (int i = 0; i < 10; i++) {
    buf[i] = analogRead(pHPin);
    delay(10);
  }
  //sort the analog from small to large
  for (int i = 0; i < 9; i++) {
    for (int j = i + 1; j < 10; j++) {
      if (buf[i] > buf[j]) {
        temp = buf[i];
        buf[i] = buf[j];
        buf[j] = temp;
      }
    }
  }
  avgValue = 0;

  //take the average value of 6 center sample
  for (int i = 2; i < 8; i++) {
    avgValue += buf[i];
  }
  float phValue = (float)avgValue * 3.3 / 4096 / 6; //convert the analog into millivolt
  phValue = 2.09 * phValue;                    //convert the millivolt into pH value
  Serial.print("pH: ");
  Serial.print(phValue, 2);
  Serial.println(" ");
  client.publish(publishmqtt2, String(phValue));

  //============================================================== Turbidity Sensor
  float NTU = analogRead(turbidity);
  Serial.print("AN: "); Serial.println(NTU);
  float NTUvoltage = map(NTU, 0, 1023, 0, 100); // Adjust the mapping as needed
  Serial.print("NTU: "); Serial.println(NTUvoltage);
  client.publish(publishmqtt3, String(NTUvoltage));

  //============================================================== Salinity Sensor
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U)  //every 40 milliseconds,read the analog value from the ADC
  {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
  }
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U) {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = averageVoltage / compensationCoefficient; //temperature compensation
    tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5; //convert voltage value to tds value
    //Serial.print("voltage:");
    //Serial.print(averageVoltage,2);
    //Serial.print("V   ");
    Serial.print("TDS Value:");
    Serial.print(tdsValue, 0);
    Serial.println("ppm");
    client.publish(publishmqtt4, String(tdsValue));
  }

}

int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}

//============================================================== Subscribe MQTT
void messageReceived(String &topic, String &payload) {
  String statusButton = payload;
  Serial.println(payload);
  if (statusButton == "1") {
    digitalWrite(pump, LOW);
    Serial.println("Pump ON");
  }
  else {
    digitalWrite(pump, HIGH);
    Serial.println("Pump OFF");
  }
}