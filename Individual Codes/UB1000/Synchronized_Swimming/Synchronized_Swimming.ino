const int sync = 7;
const int noBond = A3;
const int Bond = A2;
int i = 0;

void setup() {
  Serial.begin(115200);
  pinMode(sync, OUTPUT);
  pinMode(noBond, INPUT);
  pinMode(Bond, INPUT);

  
  //digitalWrite(sync, HIGH);
  //delayMicroseconds(100);
  //digitalWrite(sync, LOW);
  //delayMicroseconds(100);
}

void loop() {
  // Start synchronization - Square signal
  Serial.print("0: ");
  Serial.println(analogRead(Bond));
  digitalWrite(sync, HIGH);
  Serial.print("1: ");
  Serial.println(analogRead(Bond));
  delay(1.5);
  Serial.print("2: ");
  Serial.println(analogRead(Bond));
  digitalWrite(sync, LOW);
  Serial.print("3: ");
  Serial.println(analogRead(Bond));
  delay(1.5);
  Serial.print("4: ");
  Serial.println(analogRead(Bond));
  /*
  if (i < 10) {
    // Start printing results
    Serial.print("Millis: ");
    Serial.println(millis());
    Serial.print("No Bond: ");
    Serial.println(analogRead(noBond));
    Serial.print("Bond: ");
    Serial.println(analogRead(Bond));
    delay(3000);
    i += 1;
  }
  */
}
