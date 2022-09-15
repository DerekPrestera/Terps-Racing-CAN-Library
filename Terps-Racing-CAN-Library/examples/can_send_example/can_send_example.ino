#include <TR_CAN_Shield.h>

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);
  Serial.println("Serial started");
}

void loop() {
  // put your main code here, to run repeatedly:
  static TR_CAN_Shield s(8, true);
  byte data[8];
  int sensor_value = 0;

  // Read a value from the shield's ADC
  sensor_value = s.analogRead(0);


  // Place the integer value into the data port
  data[1] = (byte)(sensor_value >> 8);
  data[0] = (byte)(sensor_value & 0b0000000011111111);

  Serial.print("Sending the value ");
  Serial.print(sensor_value);
  Serial.println(" over CAN");

  s.can_send(0, data);

  delay(10);
}
