#include <TR_CAN_Shield.h>

void setup() {
  Serial.begin(115200);
  //Serial.println("Serial started");
}

void loop() {

  // The shield must be a static variable in the loop function.
  static TR_CAN_Shield shield(9, false);
  byte input[8];
  int can_sensor_value;

  Serial.end();
  Serial.begin(115200);

  // Create a sawtooth function and print the values to the
  //    serial port every 0.1 seconds.
  static int data_point = 0;  // simulated data point
  static int modifier = 25;

  for (int i = 0; i < 8; i++) {
    input[i] = 0;
  }

  //Serial.println("Reading from the CAN bus...");
  shield.can_receive(0, input);

  // Convert the byte array values to an integer
  can_sensor_value = (int)((int)input[1] << 8);
  can_sensor_value = (can_sensor_value | (int)input[0]);

  Serial.print("Sensor value received over CAN: ");
  Serial.println(can_sensor_value);
}
