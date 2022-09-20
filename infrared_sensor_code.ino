/*
 * Note: LED is used for debugging--will be removed once bugs are fixed :)
 * 
 */

const int _SENSOR_PIN = A0; // infrared sensor input pin
int sensorValue = 0; // variable to store value coming from sensor
float distance = 0;
float voltage = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  sensorValue = analogRead(_SENSOR_PIN); // read infrared sensor value
  voltage = sensorValue * (5.0 / 1023.0);
  distance = log((voltage - 0.5)/4)/(-3.5);

  // print to serial monitor for matlab
  Serial.print(voltage);
  Serial.print("\n");
  Serial.print(distance);
  Serial.print("\n");
  delay(1000); // wait 1 second
  
}
