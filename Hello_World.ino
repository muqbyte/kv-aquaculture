/*****
This is an example of programming to display "Hello, World" in serial monitor
******/
void setup() {
  Serial.begin(115200);
//  delay(1000);
}

void loop() {
  Serial.println("Hello, World!");
  delay(9000);
}