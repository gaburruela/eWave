void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

}

void loop() {
  for(int i=0; i<10; i++){
    Serial.println(i);
    delay(4000);
  }

}
