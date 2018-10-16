void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
int sensorValue=analogRead(1);
  float volts = sensorValue*0.0048828125;     //convert to volts
  float range = 65*pow(volts, -1.10);     // approximate exp data graph function
Serial.println(range);
// delay(550);

}
