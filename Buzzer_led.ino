int LED = 23;
int buzzer = 22;

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(buzzer, OUTPUT);
}

void loop() {
  digitalWrite(LED, HIGH);      //turns the Red LED on
  delay(1);                     //Wait for 1 milliseconds = 0.001 seconds
  digitalWrite(buzzer, HIGH);   //turns the Buzzer on
  delay(1000);                  //Wait for 1 seconds

  digitalWrite(LED, LOW);       //turns the Red LED off
  delay(1);                     //Wait for 0.001 seconds
  digitalWrite(buzzer, LOW);    //turns the Buzzer off
  delay(1000);                  //Wait for 1 seconds
}