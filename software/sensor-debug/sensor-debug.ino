void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("INIT");
  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("A0 ");
  Serial.println(analogRead(0));

    Serial.print("A1 ");
  Serial.println(analogRead(1));

    Serial.print("A2 ");
  Serial.println(analogRead(2));

    Serial.print("A3 ");
  Serial.println(analogRead(3));

    Serial.print("A4 ");
  Serial.println(analogRead(4));

    Serial.print("A5 ");
  Serial.println(analogRead(5));

  Serial.print("D0 ");
  Serial.println(digitalRead(0));

  Serial.print("D1 ");
  Serial.println(digitalRead(1));

  Serial.print("D2 ");
  Serial.println(digitalRead(2));

  Serial.print("D3 ");
  Serial.println(digitalRead(3));
  
  Serial.print("D9 ");
  Serial.println(digitalRead(9));

  Serial.print("D10 ");
  Serial.println(digitalRead(10));

  
  delay(100);
}
