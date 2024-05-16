//============================================================== Sensor Pin
#include <WiFi.h>
#include <MQTT.h>
#include <SoftwareSerial.h>

MQTTClient client;
const char clientId[] = "KVESP111"; //MQTT ID
const char server[]   = "txio.uitm.edu.my"; //Broker
const char publishmqtt1[] = "agro/turbidity/tadir"; //Publish
const char subscribemqtt[] = "KV/18/esp32"; //Subscribe own

WiFiClient net;
const char ssid[]     = "SEA-IC";
const char password[] = "seaic2022";
#include "DHT.h"
#define DHTPIN 4

#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

void setup (){
  delay(3000);
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  client.begin(server, 1883, net);
  Serial.println("DHT11 Humidity & Temperature Sensor\n\n");
  dht.begin();
}

void loop (){
   checkConnect();
   DataProcessing();
}

void checkConnect(){
  client.loop();

  if (!client.connected()) {
    connect();
  }
}

void connect() {
  
  Serial.print("Connecting to WiFi ...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println(" connected!");

  Serial.print("Connecting to MQTT broker ...");
  while (!client.connect(clientId)) {
    Serial.print(".");
    delay(500);
  }
  Serial.println(" connected!");

 // client.subscribe(topic);
 client.subscribe(publishmqtt1);
}

void DataProcessing(){
  if (runEvery(3000)) {
   float h=dht.readHumidity();
  float t=dht.readTemperature();

  if (isnan(h) || isnan(t)){
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  Serial.print("Humidity:");
  Serial.print(h);
  Serial.print("%\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print("*C\n");
  client.publish(publishmqtt1, String(h));
  }
}

boolean runEvery(unsigned long interval)
{
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    return true;
  }
  return false;
}

