const int RED = 22;
const int YELLOW = 21;
const int GREEN = 23;

void setup() {
  pinMode(RED, OUTPUT);
  pinMode(YELLOW, OUTPUT);
  pinMode(GREEN, OUTPUT);
}

void loop() {
  digitalWrite(RED, HIGH);       //turns the Red LED on
  digitalWrite(YELLOW, LOW);     //turns the Yellow LED off
  digitalWrite(GREEN, LOW);      //turns the Green LED off

  delay(2000);                   // Wait for 2000 milliseconds = 2 seconds

  digitalWrite(RED, LOW);        //turns the Red LED on
  digitalWrite(YELLOW, LOW);     //turns the Yellow LED off
  digitalWrite(GREEN, HIGH);     //turns the Green LED off

  delay(2000);                   // Wait for 2 seconds

  digitalWrite(RED, LOW);        //turns the Red LED on
  digitalWrite(YELLOW, HIGH);    //turns the Yellow LED off
  digitalWrite(GREEN, LOW);      //turns the Green LED off

  delay(1000);                   // Wait for 1 seconds
}