int ledPin=23;
int ldrPin=4;

void setup() {
  Serial.begin(9600);
  pinMode(ledPin,OUTPUT);
  pinMode(ldrPin, INPUT);
}

void loop() {
  int ldrStatus=analogRead(ldrPin);
  Serial.println(ldrStatus);
  delay(200);
  if(ldrStatus >=1000){
    digitalWrite(ledPin, HIGH);
  }

  else{
    digitalWrite(ledPin, LOW);
    delay(100);
  }
}