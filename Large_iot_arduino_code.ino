#include <Wire.h>
#include <SoftwareSerial.h>

const int Output_Pin = 2;
volatile int Pulse_Count;
unsigned int Liter_per_hour;
unsigned long Current_Time, Loop_Time;

#define IR_SENSOR_PIN 10  // GPIO pin connected to the IR sensor
#define LDR_SENSOR_PIN 11
#define Soil_SENSOR_PIN A0
#define Rain_SENSOR_PIN A1
const int relayPin = 9;
SoftwareSerial espSerial(0, 1);

String str;

void Detect_Rising_Edge() {
  Pulse_Count++;
}

void setup() {
  Serial.begin(115200);
  espSerial.begin(115200);
  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(LDR_SENSOR_PIN, INPUT);
  pinMode(relayPin, OUTPUT);

  pinMode(Output_Pin, INPUT);

  attachInterrupt(digitalPinToInterrupt(Output_Pin), Detect_Rising_Edge, RISING);

  Current_Time = millis();
  Loop_Time = Current_Time;
}

void loop() {
  int irSensorValue = digitalRead(IR_SENSOR_PIN);
  int ldrValue = digitalRead(LDR_SENSOR_PIN);
  int soilValue = analogRead(Soil_SENSOR_PIN);
  int rainValue = analogRead(Rain_SENSOR_PIN);

  digitalWrite(relayPin, HIGH);
  delay(3000); // Wait for 3 seconds

  // Turn off the relay
  digitalWrite(relayPin, LOW);
  delay(3000); // Wait for 3 seconds

  Current_Time = millis();
  if (Current_Time >= (Loop_Time + 1000)) {
    Loop_Time = Current_Time;
    Liter_per_hour = (Pulse_Count * 60 / 7.5);
    Pulse_Count = 0;
  }

  Serial.print("IR:");
  Serial.print(irSensorValue);
  Serial.print(",");
  Serial.print("LDR:");
  Serial.print(ldrValue);
  Serial.print(",");
  Serial.print("SOIL:");
  Serial.print(soilValue);
  Serial.print(",");
  Serial.print("RAIN:");
  Serial.print(rainValue);
  Serial.print(",");
  Serial.print("FLOWRATE:");
  Serial.print(Liter_per_hour);
  Serial.println(" Liter/hour");

  str = "IR=" + String(irSensorValue) + ",LDR=" + String(ldrValue) + ",SOIL=" + String(soilValue) + ",RAIN=" + String(rainValue) + ",FLOWRATE=" + String(Liter_per_hour) + " Liter/hour";
  espSerial.println("Coming from Arduino: " + str);

  delay(2000);
}
