//============================================================== Sensor Pin
#include <WiFi.h>
#include <MQTT.h>
#include <SoftwareSerial.h>

MQTTClient client;
const char clientId[] = "KVESP111"; //MQTT ID
const char server[]   = "txio.uitm.edu.my"; //Broker
const char publishmqtt1[] = "KV/18/temp"; //Publish
const char subscribemqtt[] = "KV/18/esp32"; //Subscribe own

WiFiClient net;
const char ssid[]     = "SEA-IC";
const char password[] = "seaic2022";
#define turbidity 27
float NTU = 0;

void setup (){
  delay(3000);
  Serial.begin(115200);
   WiFi.begin(ssid, password);
  client.begin(server, 1883, net);
  Serial.println("NTU sensor test begin");
  pinMode(turbidity, INPUT);

 
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

void DataProcessing(){
  if (runEvery(3000)) {
  int NTU = analogRead(turbidity); 
  float NTUvoltage = map(NTU, 0, 1023, 0, 100); // Adjust the mapping as needed
  Serial.println(NTUvoltage);
  delay(3000);
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