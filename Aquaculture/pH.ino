#include "esp_timer.h"
esp_timer_handle_t periodic_timer;
int periodic_count = 0;

//============================================================== Sensor Pin
int pHPin = 27;
//============================================================== pH Sensor
unsigned long int avgValue;  //Store the average value of the sensor feedback
float b;
int buf[10],temp;
int pHValue = 0;

void setup() {
  delay(3000);
  Serial.begin(115200);
  Serial.println("PH sensor test begin");
  esp_timer_init();
  periodic_init();
  pinMode(pHPin, INPUT);
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
  esp_timer_start_periodic(periodic_timer, 1000 * 5000); 
}

void periodic_callback(void* arg) {
    //============================================================== pH Sensor
    //Get 10 sample value from the sensor for smooth the value
    for(int i = 0; i < 10; i++){ 
      buf[i] = analogRead(pHPin);
      delay(10);
    }
    //sort the analog from small to large
    for(int i = 0; i < 9; i++){
      for(int j = i + 1; j < 10; j++){
        if(buf[i]>buf[j]){
          temp=buf[i];
          buf[i]=buf[j];
          buf[j]=temp;
        }
      }
    }
    avgValue=0;
    
    //take the average value of 6 center sample
    for(int i = 2; i < 8; i++){
      avgValue += buf[i];
    }
    float phValue = (float)avgValue*3.3/4096/6; //convert the analog into millivolt
    phValue = 2.09 * phValue;                    //convert the millivolt into pH value
    Serial.print("pH: ");  
    Serial.print(phValue,2);
    Serial.println(" ");
}

/*
PH
0 - 6.9     acidic
7 -       neutral
7.1 - 14    alkaline
*/