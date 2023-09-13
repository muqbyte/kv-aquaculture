//============================================================== Sensor Pin
#define turbidity 27
float NTU = 0;

void setup (){
  delay(3000);
  Serial.begin(115200);
  Serial.println("NTU sensor test begin");
  pinMode(turbidity, INPUT);
}

void loop (){
  int NTU = analogRead(turbidity); 
  float NTUvoltage = map(NTU, 0, 1023, 0, 100); // Adjust the mapping as needed
  Serial.println(NTUvoltage);
  delay(3000);
}

// NTU = 0 , V = 4.1     PURE WATER
// NTU = 3000 , V = 2.5  CLOUDED WATER