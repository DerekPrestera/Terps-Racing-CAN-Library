# Terps-Racing-CAN-Library
Function library to facilitate use of the Terps Racing CAN Arduino Shield

---
## Usage

To use this library, import the zip file into your project through the Sketch > Include Library > Add .ZIP Library menu option. This will allow you to use the `TR_CAN_Shield` class which includes a few methods that allow you to send and receive data over a CAN bus.

## Methods

    TR_CAN_Shield(byte can_id, bool debug)

Constructor that creates a new `TR_CAN_Shield` object. The method takes a byte that represents the CAN ID that the device will use, and a boolean that selects whether diagnostic information about the device will be printed to the serial output.

    void can_send(byte tx_buffer, byte send_buf[8])

Method to send data to the CAN bus. The device will wait until it has priority on the bus before sending this message. The `tx_buffer` parameter is the transmit message buffer on the shield's MCP2515 chip that this message will be stored in before sending. This can take the value of 0 or 1. For additional information on this functionality, see the MCP2515 datasheet. The second parameter `send_buf` is the array of bytes that will be sent. It is suggested that you use an eight-byte `union` structure to pack data into these bytes.

    short can_receive(byte rx_buffer, byte receive_buf[8])

Method to receive data on the CAN bus. The parameter `rx_buffer` is the received data input buffer on the shield's MCP2515 chip. This buffer should be read from to clear the flag that it has been filled. The parameter can take the value of 0 to 2. For more information, see the MCP2515 datasheet. The second parameter `receive_buf` is an eight-byte array that is populated by the data that was received. The return value of this method is the CAN ID of the device that sent the data that was received.

    int analogRead(byte channel)

Method to read from one of the shield's four analog inputs. The parameter `channel` is the id of the channel that is being read from. Due to the design of the shield, this value can be 0, 2, 4, or 6. The return value is the analog value that was read which can range from 0 to 4095 since the shield uses a 12-bit Analog-to-Digital converter chip. For more information on this chip, see the MCP3208 datasheet.

---

For information on the shield that this library should be used in conjuntion with, see the following repository: https://github.com/DerekPrestera/Terps-Racing-CAN-PCB
