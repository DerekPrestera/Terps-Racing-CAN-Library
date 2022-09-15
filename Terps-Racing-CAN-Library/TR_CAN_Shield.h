/*
 * TR_CAN_Shield.h
 */

#ifndef TR_CAN_Shield_h
#define TR_CAN_Shield_h

#include <Arduino.h>
#include <SPI.h>

// Define all the pin numbers for the MCP2515 CAN controller
#define MCP2515_RESET_PIN   5  // reset input pin
#define MCP2515_CS_PIN      6  // chip select pin
#define MCP2515_INT_PIN     9  // interrupt flag - for this shield specifically
#define MCP2515_RX0BF_PIN   8  // receive buffer 0 full flag
#define MCP2515_RX1BF_PIN   7  // receive buffer 1 full flag
#define MCP2515_TX0RTS_PIN  4  // transmit buffer 0 request to send
#define MCP2515_TX1RTS_PIN  3  // transmit buffer 1 request to send
#define MCP2515_TX2RTS_PIN  2  // transmit buffer 2 request to send

// Define the MCP2515 instruction bytes
#define MCP2515_RESET_INSTR       0xc0
#define MCP2515_READ_INSTR        0x03
#define MCP2515_READ_RX_BUF_INSTR(n,m) ((0x90) + (0B ## (n ## (m ## 0))))
#define MCP2515_WRITE_INSTR       0x02
#define MCP2515_LOAD_TX_BUF_INSTR(a,b,c) ((0x40) + (0B ## (a ## (b ## c))))
#define MCP2515_RTS_INSTR(n)      ((0x80) + (1 << n))
#define MCP2515_READ_STATUS_INSTR 0xa0
#define MCP2515_RX_STATUS_INSTR   0xb0
#define MCP2515_BIT_MODIFY_INSTR  0x05

// mcp2515 registers - I just went down each column of the chart in the data sheet
#define MCP2515_RXF0SIDH_REG    0x00    // Receive filter 0 short ID high
#define MCP2515_RXF0SIDL_REG    0x01    // Receive filter 0 short ID low
#define MCP2515_RXF0EID8_REG    0x02    // Receive filter 0 extended ID 8
#define MCP2515_RXF0EID0_REG    0x03    // Receive filter 0 extended ID 0

#define MCP2515_RXF1SIDH_REG    0x04    // Receive filter 1 short ID high
#define MCP2515_RXF1SIDL_REG    0x05    // Receive filter 1 short ID low
#define MCP2515_RXF1EID8_REG    0x06    // Receive filter 1 extended ID 8
#define MCP2515_RXF1EID0_REG    0x07    // Receive filter 1 extended ID 0

#define MCP2515_RXF2SIDH_REG    0x08    // Receive filter 2 short ID high
#define MCP2515_RXF2SIDL_REG    0x09    // Receive filter 2 short ID low
#define MCP2515_RXF2EID8_REG    0x0a    // Receive filter 2 extended ID 8
#define MCP2515_RXF2EID0_REG    0x0b    // Receive filter 2 extended ID 0

#define MCP2515_BFPCTRL_REG     0x0c    // Buffer Flag Pin Control
#define MCP2515_TXRTSCTRL_REG   0x0d    // TX Request To Send Control

#define MCP2515_CANSTAT_REG     0x0e    // CAN status
#define MCP2515_CANCTRL_REG     0x0f    // CAN control

#define MCP2515_RXF3SIDH_REG    0x10    // Receive filter 3 short ID high
#define MCP2515_RXF3SIDL_REG    0x11    // Receive filter 3 short ID low
#define MCP2515_RXF3EID8_REG    0x12    // Receive filter 3 extended ID 8
#define MCP2515_RXF3EID0_REG    0x13    // Receive filter 3 extended ID 0

#define MCP2515_RXF4SIDH_REG    0x14    // Receive filter 4 short ID high
#define MCP2515_RXF4SIDL_REG    0x15    // Receive filter 4 short ID low
#define MCP2515_RXF4EID8_REG    0x16    // Receive filter 4 extended ID 8
#define MCP2515_RXF4EID0_REG    0x17    // Receive filter 4 extended ID 0

#define MCP2515_RXF5SIDH_REG    0x18    // Receive filter 5 short ID high
#define MPC2515_RXF5SIDL_REG    0x19    // Receive filter 5 short ID low
#define MCP2515_RXF5EID8_REG    0x1a    // Receive filter 5 extended ID 8
#define MCP2515_RXF5EID0_REG    0x1b    // Receive filter 5 extended ID 0

#define MCP2515_TEC_REG         0x1c    // Transmit error counter
#define MCP2515_REC_REG         0x1d    // Receive error counter

// CANSTAT and CANCTRL occupy all addresses with 0xnE or 0xnF

#define MCP2515_RXM0SIDH_REG    0x20    // Receive mask 0 short ID high
#define MCP2515_RXM0SIDL_REG    0x21    // Receive mask 0 short ID low
#define MCP2515_RXM0EID8_REG    0x22    // Receive mask 0 extended ID 8
#define MCP2515_RXM0EID0_REG    0x23    // Receive mask 0 extended ID 0

#define MCP2515_RXM1SIDH_REG    0x24    // Receive mask 1 short ID high
#define MCP2515_RXM1SIDL_REG    0x25    // Receive mask 1 short ID low
#define MCP2515_RXM1EID8_REG    0x26    // Receive mask 1 extended ID 8
#define MCP2515_RXM1EID0_REG    0x27    // Receive mask 1 extended ID 0

// Bit timing configuration registers
#define MCP2515_CNF3_REG        0x28    // Bit timing configuration 3
#define MCP2515_CNF2_REG        0x29    // Bit timing configuration 2
#define MCP2515_CNF1_REG        0x2a    // Bit timing configuration 1

// CAN interrupt enable and flag registers
#define MCP2515_CANINTE_REG     0x2b    // CAN interrupt enable
#define MCP2515_CANINTF_REG     0x2c    // CAN interrupt flag

#define MCP2515_EFLG_REG        0x2d    // Error flag register

#define MCP2515_TXB0CTRL_REG    0x30    // Transmit buffer 0 control

#define MCP2515_TXB0SIDH_REG    0x31    // Transmit buffer 0 short ID high
#define MCP2515_TXB0SIDL_REG    0x32    // Transmit buffer 0 short ID low

#define MCP2515_TXB0EID8_REG    0x33    // Transmit buffer 0 extended ID 8
#define MCP2515_TXB0EID0_REG    0x34    // Transmit buffer 0 extended ID 0

#define MCP2515_TXB0DLC_REG     0x35    // Transmit buffer 0 data length code

#define MCP2515_TXB0D0_REG      0x36    // Transmit buffer 0 data registers...
#define MCP2515_TXB0D1_REG      0x37
#define MCP2515_TXB0D2_REG      0x38
#define MCP2515_TXB0D3_REG      0x39
#define MCP2515_TXB0D4_REG      0x3a
#define MCP2515_TXB0D5_REG      0x3b
#define MCP2515_TXB0D6_REG      0x3c
#define MCP2515_TXB0D7_REG      0x3d

#define MCP2515_TXB1CTRL_REG    0x40    // Transmit buffer 1 control

#define MCP2515_TXB1SIDH_REG    0x41    // Transmit buffer 1 short ID high
#define MCP2515_TXB1SIDL_REG    0x42    // Transmit buffer 1 short ID low

#define MCP2515_TXB1EID8_REG    0x43    // Transmit buffer 1 short ID 8
#define MCP2515_TXB1EID0_REG    0x44    // Transmit buffer 1 short ID 0

#define MCP2515_TXB1DLC_REG     0x45    // Transmit buffer 1 data length code

#define MCP2515_TXB1D0_REG      0x46    // Transmit buffer 1 data registers...
#define MCP2515_TXB1D1_REG      0x47
#define MCP2515_TXB1D2_REG      0x48
#define MCP2515_TXB1D3_REG      0x49
#define MCP2515_TXB1D4_REG      0x4a
#define MCP2515_TXB1D5_REG      0x4b
#define MCP2515_TXB1D6_REG      0x4c
#define MCP2515_TXB1D7_REG      0x4d

#define MCP2515_TXB2CTRL_REG    0x50    // Transmit buffer 2 control

#define MCP2515_TXB2SIDH_REG    0x51    // Transmit buffer 2 short ID high
#define MCP2515_TXB2SIDL_REG    0x52    // Transmit buffer 2 short ID low

#define MCP2515_TXB2EID8_REG    0x53    // Transmit buffer 2 extended ID 8
#define MCP2515_TXB2EID0_REG    0x54    // Transmit buffer 2 extended ID 0

#define MCP2515_TXB2DLC_REG     0x55    // Transmit buffer 2 data length code

#define MCP2515_TXB2D0_REG      0x56    // Transmit buffer 2 data registers...
#define MCP2515_TXB2D1_REG      0x57
#define MCP2515_TXB2D2_REG      0x58
#define MCP2515_TXB2D3_REG      0x59
#define MCP2515_TXB2D4_REG      0x5a
#define MCP2515_TXB2D5_REG      0x5b
#define MCP2515_TXB2D6_REG      0x5c
#define MCP2515_TXB2D7_REG      0x5d

// Receive buffer addresses
#define MCP2515_RXB0CTRL_REG    0x60    // Receive buffer 0 control

#define MCP2515_RXB0SIDH_REG    0x61    // Receive buffer 0 short ID high
#define MCP2515_RXB0SIDL_REG    0x62    // Receive buffer 0 short ID low

#define MCP2515_RXB0EID8_REG    0x63    // Receive buffer 0 extended ID 8
#define MCP2515_RXB0EID0_REG    0x64    // Receive buffer 0 extended ID 0

#define MCP2515_RXB0DLC_REG     0x65    // Receive buffer 0 data length code

#define MCP2515_RXB0D0_REG      0x66    // Receive buffer 0 data registers...
#define MCP2515_RXB0D1_REG      0x67
#define MCP2515_RXB0D2_REG      0x68
#define MCP2515_RXB0D3_REG      0x69
#define MCP2515_RXB0D4_REG      0x6a
#define MCP2515_RXB0D5_REG      0x6b
#define MCP2515_RXB0D6_REG      0x6c
#define MCP2515_RXB0D7_REG      0x6d

#define MCP2515_RXB1CTRL_REG    0x70    // Receive buffer 1 control

#define MCP2515_RXB1SIDH_REG    0x71    // Receive buffer 1 short ID high
#define MCP2515_RXB1SIDL_REG    0x72    // Receive buffer 1 short ID low

#define MCP2515_RXB1EID8_REG    0x73    // Receive buffer 1 extended ID 8
#define MCP2515_RXB1EID0_REG    0x74    // Receive buffer 1 extended ID 0

#define MCP2515_RXB1DLC_REG     0x75    // Receive buffer 1 data length code

#define MCP2515_RXB1D0_REG      0x76    // Receive buffer 1 data registers...
#define MCP2515_RXB1D1_REG      0x77
#define MCP2515_RXB1D2_REG      0x78
#define MCP2515_RXB1D3_REG      0x79
#define MCP2515_RXB1D4_REG      0x7a
#define MCP2515_RXB1D5_REG      0x7b
#define MCP2515_RXB1D6_REG      0x7c
#define MCP2515_RXB1D7_REG      0x7d

// Define the chip select pin for the MCP3208 12-bit ADC chip
#define MCP3208_CS_PIN 10

class TR_CAN_Shield {
    public:
        TR_CAN_Shield(byte can_id, bool debug);
        void can_send(byte tx_buffer, byte send_buf[8]);
        short can_receive(byte rx_buffer_num, byte receive_buf[8]);
        int analogRead(byte channel);

    private:
        void mcp2515_register_write(byte addr, byte data);
        byte mcp2515_register_read(byte addr);
        void byte_to_string(byte b, char result[9]);
        void print_byte(byte b);
        void print_data();
};

#endif