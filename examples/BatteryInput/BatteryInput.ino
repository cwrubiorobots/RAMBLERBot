/*
  Zumo Battery Analog Input
  Reads the Battery voltage, outputs to the Serial port, 
  and times the length of the analog reading.
  
  Battery is attached to pin A1. 
  Note the battery switch much be ON for the reading to be accurate. 
  
 */

int sensorPin = A1;    // select the input pin for the potentiometer
int ledPin = 13;      // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor
unsigned long startTime = 0;
unsigned long endTime = 0;

void setup() {
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);  
  Serial.begin(9600);
}

void loop() {
  // read the value from the sensor:
  startTime = micros();
  sensorValue = analogRead(sensorPin);  
  endTime = micros(); 
  Serial.print("sensor: ");
  Serial.print(sensorValue);
  Serial.print(" Micros: ");
  Serial.println(endTime-startTime); 
  delay(100);                  
}
