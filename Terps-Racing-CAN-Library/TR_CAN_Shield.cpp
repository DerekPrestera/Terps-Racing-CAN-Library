/*
 * TR_CAN_Shield.cpp
 */

#include "TR_CAN_Shield.h"

// Constructor
TR_CAN_Shield::TR_CAN_Shield(byte can_id, bool debug) {
    // Set pinmode for mcp2515 pins
  pinMode(MCP2515_RESET_PIN,  OUTPUT);
  pinMode(MCP2515_CS_PIN, OUTPUT);
  pinMode(MCP2515_INT_PIN, INPUT);
  pinMode(MCP2515_RX0BF_PIN, INPUT);
  pinMode(MCP2515_RX1BF_PIN, INPUT);
  pinMode(MCP2515_TX0RTS_PIN, OUTPUT);
  pinMode(MCP2515_TX1RTS_PIN, OUTPUT);
  pinMode(MCP2515_TX2RTS_PIN, OUTPUT);

  // Set pinmode for mcp3208 pins
  pinMode(MCP3208_CS_PIN, OUTPUT);

  // Deselect both chips for now
  digitalWrite(MCP2515_CS_PIN, HIGH);
  digitalWrite(MCP3208_CS_PIN, HIGH);

  // Start Serial for debugging
  //Serial.being(9600)

  // Start the SPI bus. Do I need the SPI.beginTransaction() function?
  SPI.begin();

  if (debug) Serial.println("Resetting...");
  // First, write the reset pin to low (it's active low)
  digitalWrite(MCP2515_RESET_PIN, LOW);
  delay(1); // wait. RESET must be low for 2 microseconds but this works
  digitalWrite(MCP2515_RESET_PIN, HIGH);

  /*
  * Do any other configuration here:
  * Message reception:
  * - enable the receive interrupt
  * - configure the RXB0F pin to be a buffer flag
  * - configure the chip to accept all messages
  * - change the chip from configuration mode to normal mode
  * Message transmission:
  * - configure TXBnCTRL with the CAN ID (short) and DLC
  */
  // -------------------- Configure Receive Buffer -------------------- //
  if (debug) Serial.println("---------- initializing RXB0 -----------");
  // enable the interrupt for receive buffer 0
  // RX1IE is bit 0 of CANINTE
  if (debug) Serial.println("\nEnabling RX0BF interrupt...");
  mcp2515_register_write(MCP2515_CANINTE_REG, mcp2515_register_read(MCP2515_CANINTE_REG) | 0b00000001);
  if (debug) {
    Serial.print("CAN Interrupt Enable Register:\t\t");
    print_byte(mcp2515_register_read(MCP2515_CANINTE_REG));
    Serial.print("CAN Interrupt Flag Register:\t\t");
    print_byte(mcp2515_register_read(MCP2515_CANINTF_REG));
  }

  // configure the RX0BF pin to be a receive buffer interrupt
  // set the B0BFE bit in the BFPCTRL register
  if (debug) Serial.println("\nConfiguring RX0BF pin as interrupt pin...");
  mcp2515_register_write(MCP2515_BFPCTRL_REG, mcp2515_register_read(MCP2515_BFPCTRL_REG) | 0b00000100);
  if (debug) {
    Serial.print("Buffer Flag Pin Control Register:\t");
    print_byte(mcp2515_register_read(MCP2515_BFPCTRL_REG));
  }

  // Configure RX0 to receive all messages
  // set RXM[0:1] bits to '11'
  // RXM0 and RXM1 are RXB0CTRL bits 5 and 6
  if (debug) Serial.println("\nConfiguring RXB0 to receive all messages...");
  mcp2515_register_write(MCP2515_RXB0CTRL_REG, mcp2515_register_read(MCP2515_RXB0CTRL_REG) | 0b01100000);
  if (debug) {
    Serial.print("RXB0 Control Register:\t\t\t");
    print_byte(mcp2515_register_read(MCP2515_RXB0CTRL_REG));
  }

  // Disable rollover for RXB0
  if (debug) Serial.println("\nDisabling rollover for RXB0...");
  mcp2515_register_write(MCP2515_RXB0CTRL_REG, mcp2515_register_read(MCP2515_RXB0CTRL_REG) & 0b11111011);

  if (debug) Serial.println("-------- Completed initilizing RXB0 --------");

  // ----------------- Configure Transmission Buffer ----------------- //
  if (debug) Serial.println("---------- initializing TXB0 ----------");
  // Set the CAN ID to 15 (temporary arbitrary value for now)
  // TXB0SIDH - Short ID High = 00000000
  // TXB0SIDL - Short ID Low =  00001111
  if (debug) {
    Serial.print("Configuring TXB0 with CAN ID ");
    Serial.print(can_id);
    Serial.println("...");
  }
  
  mcp2515_register_write(MCP2515_TXB0SIDH_REG, 0b00000000);
  mcp2515_register_write(MCP2515_TXB0SIDL_REG, can_id);
  if (debug) {
    Serial.print("TXB0 Short ID High: ");
    print_byte(mcp2515_register_read(MCP2515_TXB0SIDH_REG));
    Serial.print("TXB0 Short ID Low:  ");
    print_byte(mcp2515_register_read(MCP2515_TXB0SIDL_REG));
  }

  // Set the Data Lenght Code (DLC) to 8 (2 bytes for each of 4 sensor inputs)
  // Bit 6 of DLC reg is "Remote Transmission Request" (RTR) bit -> set to 0 for data frame
  if (debug) Serial.println("Set the Data Length Code (DLC) to 8...");
  mcp2515_register_write(MCP2515_TXB0DLC_REG, 0b00001000);
  if (debug) {
    Serial.print("TXB0 Data Length Code: ");
    print_byte(mcp2515_register_read(MCP2515_TXB0DLC_REG));
  }

  // Set TXB0 to have the highest buffer priority (11)
  if (debug) Serial.println("Configuring TXB0 to have highest buffer priority...");
  mcp2515_register_write(MCP2515_TXB0CTRL_REG, mcp2515_register_read(MCP2515_TXB0CTRL_REG) | 0b00000011);
  if (debug) {
    Serial.print("TXB0 Control Register: ");
    print_byte(mcp2515_register_read(MCP2515_TXB0CTRL_REG));
    Serial.println("-------- Completed initializing TXB0 --------");
  }

  // -------------------- Configure Bit Timing -------------------- //
  if (debug) Serial.println("---------- initializing bit timings ----------");
  // testing is conducted at 500kbps

  // 125 kbps -> t_bit = 1/125000  =  8 us  =  8000 ns
  // 500 kbps -> t_bit = 1/500000  =  2 us  =  2000 ns  <----
  // 1 Mbps   -> t_bit = 1/1000000 =  1 us  =  1000 ns

  // Time Quantum = 125 ns  -->  need 16 of them for 500 kbps
  
  // | ----- | --------- | ------------------ | ------------- |
  //  SyncSeg   PropSeg      PhaseSeg1 (PS1)   PhaseSeg2 (PS2)
  //
  //   1 T_Q     2 T_Q            7 T_Q             6 T_Q

  // set the Baud Rate Prescaler (BRP) bits to determin Time Quantum (T_Q)
  // T_OSC = 1/F_OSC = 1/16MHZ = 62.5 ns
  // 2 * T_OSC = 125 ns -> min Time Quantum
  // T_Q = 2 * BRP * T_OSC
  // set BRP in the CNF1 register -> set to 0 for T_Q = 2 * T_OSC
  // Bits CNF1[5:0]
  // ALSO Set the Synchronization Jump Width Length bits
  // make it 2 Time Quanta -> CNF1[7:6] = 01 
  if (debug) {
    Serial.println("\nConfiguring Time Quantum to 2*T_OSC...");
    Serial.println("Configuring Synchronization Jump Width...");
  }

  mcp2515_register_write(MCP2515_CNF1_REG, mcp2515_register_read(MCP2515_CNF1_REG) & 0b00000000);
  mcp2515_register_write(MCP2515_CNF1_REG, mcp2515_register_read(MCP2515_CNF1_REG) | 0b01000000);
  if (debug) {
    Serial.print("Configuration Register 1:\t\t");
    print_byte(mcp2515_register_read(MCP2515_CNF1_REG));
  }

  // Set the sample point to sample once
  if (debug) Serial.println("\nConfiguring sample point...");
  mcp2515_register_write(MCP2515_CNF2_REG, mcp2515_register_read(MCP2515_CNF2_REG) & 0b10111111);
  if (debug) {
    Serial.print("Configuration Register 2:\t\t");
    print_byte(mcp2515_register_read(MCP2515_CNF2_REG));
  }

  // Set the PS2 length with CNF3 bits
  // CNF2[7] = 1
  if (debug) Serial.println("\nConfiguring PS2 Bit Time Selection Method...");
  mcp2515_register_write(MCP2515_CNF2_REG, mcp2515_register_read(MCP2515_CNF2_REG) | 0b10000000);
  if (debug) {
    Serial.print("Configuration Register 2:\t\t");
    print_byte(mcp2515_register_read(MCP2515_CNF2_REG));
  }

  // ----- Propagation Segment (PropSeg) length ----- //
  // set to 2 -> CNF2[2:0] = 001
  if (debug) Serial.println("\nConfiguring Propagation Segment Length...");
  mcp2515_register_write(MCP2515_CNF2_REG, mcp2515_register_read(MCP2515_CNF2_REG) | 0b00000001);
  if (debug) {
    Serial.print("Configuration Register 2:\t\t");
    print_byte(mcp2515_register_read(MCP2515_CNF2_REG));
  }

  // ----- Phase Segment 1 (PS1) length ----- //
  // set to 7 -> CNF2[5:3] = 7 - 1 = 6 = 110
  if (debug) Serial.println("\nConfiguring Phase Segment 1 Length...");
  mcp2515_register_write(MCP2515_CNF2_REG, mcp2515_register_read(MCP2515_CNF2_REG) & 0b11000111);
  mcp2515_register_write(MCP2515_CNF2_REG, mcp2515_register_read(MCP2515_CNF2_REG) | 0b00110000);
  if (debug) {
    Serial.print("Configuration Register 2:\t\t");
    print_byte(mcp2515_register_read(MCP2515_CNF2_REG));
  }

  // ----- Phase Segment 2 (PS2) length ----- //
  // set to 6 -> CNF3[2:0] = 6 - 1 = 5 = 101
  if (debug) Serial.println("\nConfiguring Phase Segment 2 Length...");
  mcp2515_register_write(MCP2515_CNF3_REG, mcp2515_register_read(MCP2515_CNF3_REG) & 0b11111000);
  mcp2515_register_write(MCP2515_CNF3_REG, mcp2515_register_read(MCP2515_CNF3_REG) | 0b00000101);
  if (debug) {
    Serial.print("Configuration Register 3:\t\t");
    print_byte(mcp2515_register_read(MCP2515_CNF3_REG));
  }

  // -------------------- Disable Clock out pin -------------------- //
  // CANCTRL[2] = 0
  if (debug) Serial.println("\nDisabling Clock out pin...");
  mcp2515_register_write(MCP2515_CANCTRL_REG, mcp2515_register_read(MCP2515_CANCTRL_REG) & 0b11111011);
  if (debug) {
    Serial.print("CAN Control Register: ");
    print_byte(mcp2515_register_read(MCP2515_CANCTRL_REG));
  }

  // -------------------- Set to Normal Mode -------------------- //
  // REQOP[2:0] = 000 to operating in normal mode
  // REQOP[2:0] are bits CANCTRL[7:5]
  if (debug) Serial.println("\nRequesting 'Normal Mode' Operation Mode");
  mcp2515_register_write(MCP2515_CANCTRL_REG, mcp2515_register_read(MCP2515_CANCTRL_REG) & 0b00011111);
  //mcp2515_register_write(MCP2515_CANCTRL_REG, mcp2515_register_read(MCP2515_CANCTRL_REG) | 0b01100000);
  if (debug) {
    Serial.print("CAN Control Register:\t\t\t");
    print_byte(mcp2515_register_read(MCP2515_CANCTRL_REG));
    Serial.println("------ initialization complete ------");
  }
  //delay(1000);
}

/*
 * Sends the data stored in the parameter "data" using the
 * transmit buffer value stored in the parameter "tx_buffer".
 */
void TR_CAN_Shield::can_send(byte tx_buffer, byte send_buf[8]) {
  byte mcp2515_tx_regs[3][8] = {{MCP2515_TXB0D0_REG, MCP2515_TXB0D1_REG, MCP2515_TXB0D2_REG, MCP2515_TXB0D3_REG,
                    MCP2515_TXB0D4_REG, MCP2515_TXB0D5_REG, MCP2515_TXB0D6_REG, MCP2515_TXB0D7_REG},
                        {MCP2515_TXB1D0_REG, MCP2515_TXB1D1_REG, MCP2515_TXB1D2_REG, MCP2515_TXB1D3_REG,
                    MCP2515_TXB1D4_REG, MCP2515_TXB1D5_REG, MCP2515_TXB1D6_REG, MCP2515_TXB1D7_REG},
                        {MCP2515_TXB2D0_REG, MCP2515_TXB2D1_REG, MCP2515_TXB2D2_REG, MCP2515_TXB2D3_REG,
                    MCP2515_TXB2D4_REG, MCP2515_TXB2D5_REG, MCP2515_TXB2D6_REG, MCP2515_TXB2D0_REG}};

    if (tx_buffer != 0 && tx_buffer != 1 && tx_buffer != 2) {
    Serial.println("TX BUFFER CAN BE 0, 1, 0R 2");
    return;
  }

  // Wait for the TXBnRTS bit to be cleared
  while(1) {
    if (tx_buffer == 0 && (mcp2515_register_read(MCP2515_TXB0CTRL_REG) & 0b00001000) == 0) break;
    if (tx_buffer == 1 && (mcp2515_register_read(MCP2515_TXB1CTRL_REG) & 0b00001000) == 0) break;
    if (tx_buffer == 2 && (mcp2515_register_read(MCP2515_TXB2CTRL_REG) & 0b00001000) == 0) break;
    Serial.println("Waiting to send...");
  }

  // Write the send_data[] array to the tranmission buffer specified
  for (int i = 0; i < 8; i++) {
    mcp2515_register_write(mcp2515_tx_regs[tx_buffer][i], send_buf[i]);
  }

  // Initiate transmission by setting the TXREQ bit in the TXB0CTRL register
  // I'll do this by sending the SPI Request-to-Send Instruction (there are
  //    other ways - see MCP2515 datasheet)
  digitalWrite(MCP2515_CS_PIN, LOW);
  SPI.transfer(MCP2515_RTS_INSTR(0));
  digitalWrite(MCP2515_CS_PIN, HIGH);
}

/*
 * Populates the parameter "data" with the value stored
 * in the receive buffer specified with the parameter "rx_buffer".
 * 
 * Returns the CANID of the device that sent the message that was received.
 */
short TR_CAN_Shield::can_receive(byte rx_buffer, byte receive_buf[8]) {
  byte mcp2515_rx_regs[2][8] = {{MCP2515_RXB0D0_REG, MCP2515_RXB0D1_REG, MCP2515_RXB0D2_REG, MCP2515_RXB0D3_REG,
                        MCP2515_RXB0D4_REG, MCP2515_RXB0D5_REG, MCP2515_RXB0D6_REG, MCP2515_RXB0D7_REG},
                            {MCP2515_RXB1D0_REG, MCP2515_RXB1D1_REG, MCP2515_RXB1D2_REG, MCP2515_RXB1D3_REG,
                        MCP2515_RXB1D4_REG, MCP2515_RXB1D5_REG, MCP2515_RXB1D6_REG, MCP2515_RXB1D7_REG}};

  short received_id;

  byte received_id_low;
  byte received_id_high;

  if (rx_buffer != 0 && rx_buffer != 1) {
    Serial.println("RECEIVE BUFFER CAN BE 0 OR 1");
    return 0xffff;
  }

  // Wait for a new message in RXB0
  //while (digitalRead(MCP2515_INT_PIN)) {
  while (!(mcp2515_register_read(MCP2515_CANINTF_REG) & 0b00000001)) {
    // Serial.println("Waiting to receive message");
    // Serial.println(mcp2515_register_read(MCP2515_CANINTF_REG));
    // delay(100);
  }

  // Read the data from the registers into the data[] array
  for (int i = 0; i < 8; i++) {
    receive_buf[i] = mcp2515_register_read(mcp2515_rx_regs[rx_buffer][i]);
  }

  received_id_low = mcp2515_register_read(MCP2515_RXB1SIDL_REG);
  received_id_high = mcp2515_register_read(MCP2515_RXB0SIDH_REG);

  received_id = (((short)received_id_high) << 8) & received_id_low;

  // reset the received message flag bit
  mcp2515_register_write(MCP2515_CANINTF_REG, mcp2515_register_read(MCP2515_CANINTF_REG) & 0b111111110); // read the value and clear bit 0

  return received_id;
}

/*
 * Interaction is 2-bytes long but the first 3 bits sent are the channel selection.
 * The address is sent out and the last 4 bits of the received byte from that
 * "transfer()" are the first 4 bits of the reading. The last 8 bits are shifted out
 * during the subsquent "shiftIn()" call.
 */
int TR_CAN_Shield::analogRead(byte channel) {
  int adc_value = 0; // some random thing to show if its working

  byte adc_hi = 0;
  byte adc_lo = 0;

  // Begin the SPI interaction
  digitalWrite(MCP3208_CS_PIN, LOW);
  // send out the bits to indicate single-ended Channel 0: 1000000
  // Bit timing for sampling a channel - see page 21 of the MCP3208 datasheet
  // send leading zeros to allow for 3 groups of 8 bits.

  // send 5 leading zeros, the start bit, and the selection of single-ended measurement, and the channel D2 bit
  SPI.transfer(0b00000110 | ((channel >> 2) & 0b00000001));
  adc_hi = SPI.transfer((channel << 6) & (0b11000000)); // channel D1, D0, and first 4 bits of the data
  adc_lo = SPI.transfer(0b00000000); // Last 8 bits of the data

  // Cut out any crazy don't care bits
  adc_hi = adc_hi & 0b00001111;

  // Construct the final value
  adc_value = (int)adc_hi;
  adc_value = adc_value << 8;
  adc_value = adc_value | (int)adc_lo;

  digitalWrite(MCP3208_CS_PIN, HIGH);

  return adc_value;
}

/*
 * Takes the address of a register and a byte to be written
 * to the register and send the instruction to write to that
 * register address.
 */
void TR_CAN_Shield::mcp2515_register_write(byte addr, byte data) {
  // select the chip
  digitalWrite(MCP2515_CS_PIN, LOW);

  // Write the sequence of bytes
  SPI.transfer(MCP2515_WRITE_INSTR);
  SPI.transfer(addr);
  SPI.transfer(data);

  // deselect the chips
  digitalWrite(MCP2515_CS_PIN, HIGH);
}

/*
 * Takes the address of the register to be read and returns the
 * byte stored in that register.
 */
byte TR_CAN_Shield::mcp2515_register_read(byte addr) {
  byte received_value;
  
  //select the chip
  digitalWrite(MCP2515_CS_PIN, LOW);

  // send the instruction
  SPI.transfer(MCP2515_READ_INSTR);
  SPI.transfer(addr);
  received_value = SPI.transfer(0x0); // transfer to get the value

  // deselect the chip
  digitalWrite(MCP2515_CS_PIN, HIGH);

  return received_value;
}

void TR_CAN_Shield::byte_to_string(byte b, char result[9]) {
  result[8] = '\0';
  for (int i = 0; i < 8; i++) {
    result[i] = 'x';
  }

  for (int i = 7; i >= 0; i--) {
    if ((b & 0b00000001) == 1) {
      result[i] = '1';
    } else {
      result[i] = '0';
    }
    b = b >> 1;
  }
}

void TR_CAN_Shield::print_byte(byte b) {
  char byte_string[9];
  
  byte_to_string(b, byte_string);
  Serial.print("0b");
  Serial.println(byte_string);
}

// void TR_CAN_Shield::print_data() {
//   for (int i = 0; i < 8; i++) {
//     Serial.print("; data[");
//     Serial.print(i);
//     Serial.print("] = ");
//     Serial.print(received_data[i]);
//   }
//   Serial.println("");
// }
