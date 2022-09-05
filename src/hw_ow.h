/*
 * DallasOneWire.h
 *
 *  Created on: 28 серп. 2015 р.
 *      Author: sd
 */
#ifndef DALLASONEWIRE_DALLASONEWIRE_H_
#define DALLASONEWIRE_DALLASONEWIRE_H_

#include "driver/uart.h"
#include "ow_rom.h"

// definitions
#define UART_BASE_SPEED 9600
#define UART_BREAK_SPEED 4800
#define UART_RX_BUFFER 32
#define UART_TX_BUFFER 16

#define FALSE 0
#define TRUE 1

#define WRITE_FUNCTION 0
#define READ_FUNCTION 1

// Mode Commands
#define MODE_DATA 0xE1
#define MODE_COMMAND 0xE3
#define MODE_STOP_PULSE 0xF1

// Return byte value
#define RB_CHIPID_MASK 0x1C
#define RB_RESET_MASK 0x03
#define RB_1WIRESHORT 0x00
#define RB_PRESENCE 0x01
#define RB_ALARMPRESENCE 0x02
#define RB_NOPRESENCE 0x03

#define RB_BIT_MASK 0x03
#define RB_BIT_ONE 0x03
#define RB_BIT_ZERO 0x00

// Masks for all bit ranges
#define CMD_MASK 0x80
#define FUNCTSEL_MASK 0x60
#define BITPOL_MASK 0x10
#define SPEEDSEL_MASK 0x0C
#define MODSEL_MASK 0x02
#define PARMSEL_MASK 0x70
#define PARMSET_MASK 0x0E

// Command or config bit
#define CMD_COMM 0x81
#define CMD_CONFIG 0x01

// Function select bits
#define FUNCTSEL_BIT 0x00
#define FUNCTSEL_SEARCHON 0x30
#define FUNCTSEL_SEARCHOFF 0x20
#define FUNCTSEL_RESET 0x40
#define FUNCTSEL_CHMOD 0x60

// Bit polarity/Pulse voltage bits
#define BITPOL_ONE 0x10
#define BITPOL_ZERO 0x00
#define BITPOL_5V 0x00
#define BITPOL_12V 0x10

// One Wire speed bits
#define SPEEDSEL_STD 0x00
#define SPEEDSEL_FLEX 0x04
#define SPEEDSEL_OD 0x08
#define SPEEDSEL_PULSE 0x0C

// Data/Command mode select bits
#define MODSEL_DATA 0x00
#define MODSEL_COMMAND 0x02

// 5V Follow Pulse select bits
#define PRIME5V_TRUE 0x02
#define PRIME5V_FALSE 0x00

// Parameter select bits
#define PARMSEL_PARMREAD 0x00
#define PARMSEL_SLEW 0x10
#define PARMSEL_12VPULSE 0x20
#define PARMSEL_5VPULSE 0x30
#define PARMSEL_WRITE1LOW 0x40
#define PARMSEL_SAMPLEOFFSET 0x50
#define PARMSEL_ACTIVEPULLUPTIME 0x60
#define PARMSEL_BAUDRATE 0x70

// Pull down slew rate.
#define PARMSET_Slew15Vus 0x00
#define PARMSET_Slew2p2Vus 0x02
#define PARMSET_Slew1p65Vus 0x04
#define PARMSET_Slew1p37Vus 0x06
#define PARMSET_Slew1p1Vus 0x08
#define PARMSET_Slew0p83Vus 0x0A
#define PARMSET_Slew0p7Vus 0x0C
#define PARMSET_Slew0p55Vus 0x0E

// 12V programming pulse time table
#define PARMSET_32us 0x00
#define PARMSET_64us 0x02
#define PARMSET_128us 0x04
#define PARMSET_256us 0x06
#define PARMSET_512us 0x08
#define PARMSET_1024us 0x0A
#define PARMSET_2048us 0x0C
#define PARMSET_infinite 0x0E

// 5V strong pull up pulse time table
#define PARMSET_16p4ms 0x00
#define PARMSET_65p5ms 0x02
#define PARMSET_131ms 0x04
#define PARMSET_262ms 0x06
#define PARMSET_524ms 0x08
#define PARMSET_1p05s 0x0A
#define PARMSET_dynamic 0x0C
#define PARMSET_infinite 0x0E

// Write 1 low time
#define PARMSET_Write8us 0x00
#define PARMSET_Write9us 0x02
#define PARMSET_Write10us 0x04
#define PARMSET_Write11us 0x06
#define PARMSET_Write12us 0x08
#define PARMSET_Write13us 0x0A
#define PARMSET_Write14us 0x0C
#define PARMSET_Write15us 0x0E

// Data sample offset and Write 0 recovery time
#define PARMSET_SampOff3us 0x00
#define PARMSET_SampOff4us 0x02
#define PARMSET_SampOff5us 0x04
#define PARMSET_SampOff6us 0x06
#define PARMSET_SampOff7us 0x08
#define PARMSET_SampOff8us 0x0A
#define PARMSET_SampOff9us 0x0C
#define PARMSET_SampOff10us 0x0E

// Active pull up on time
#define PARMSET_PullUp0p0us 0x00
#define PARMSET_PullUp0p5us 0x02
#define PARMSET_PullUp1p0us 0x04
#define PARMSET_PullUp1p5us 0x06
#define PARMSET_PullUp2p0us 0x08
#define PARMSET_PullUp2p5us 0x0A
#define PARMSET_PullUp3p0us 0x0C
#define PARMSET_PullUp3p5us 0x0E

// Baud rate bits
#define PARMSET_9600 0x00
#define PARMSET_19200 0x02
#define PARMSET_57600 0x04
#define PARMSET_115200 0x06

// DS2480B program voltage available
#define DS2480BPROG_MASK 0x20

// mode bit flags
#define MODE_NORMAL 0x00
#define MODE_OVERDRIVE 0x01
#define MODE_STRONG5 0x02
#define MODE_PROGRAM 0x04
#define MODE_BREAK 0x08

#define MAX_BAUD PARMSET_115200

typedef struct hw_ow_type {
    uart_port_t uart_num;
    int8_t en_pin;

    rom_t rom;

    // search state

    int LastDiscrepancy;
    int LastFamilyDiscrepancy;
    int LastDeviceFlag;
    unsigned char crc8;

    // DS2480B state
    int ULevel;  // 1-Wire level
    int UBaud;   // baud rate
    int UMode;   // command or data mode state
    int USpeed;  // 1-Wire communication speed
} hw_ow_t;

/* Hardwre control functions */
hw_ow_t* hw_ow_new(uart_port_t uart_num, int tx_gpio, int rx_gpio, int en_gpio);
int hw_ow_probe(hw_ow_t* hw_ow);
int hw_ow_change_baud(hw_ow_t* hw_ow, unsigned char newbaud);
void hw_ow_hardware_reset(hw_ow_t* hw_ow);
void hw_ow_delete(hw_ow_t* hw_ow);

/* Basic 1-Wire functions */
int OWReset(hw_ow_t* hw_ow);
unsigned char OWTouchBit(hw_ow_t* hw_ow, unsigned char sendbit);
unsigned char OWTouchByte(hw_ow_t* hw_ow, unsigned char sendbyte);
void OWWriteByte(hw_ow_t* hw_ow, unsigned char byte_value);
void OWWriteBit(hw_ow_t* hw_ow, unsigned char bit_value);
unsigned char OWReadBit(hw_ow_t* hw_ow);
unsigned char OWReadByte(hw_ow_t* hw_ow);
int OWBlock(hw_ow_t* hw_ow, unsigned char* tran_buf, int tran_len);
int OWSearch(hw_ow_t* hw_ow);
int OWFirst(hw_ow_t* hw_ow);
int OWNext(hw_ow_t* hw_ow);
int OWVerify(hw_ow_t* hw_ow);
void OWTargetSetup(hw_ow_t* hw_ow, unsigned char family_code);
void OWFamilySkipSetup(hw_ow_t* hw_ow);

/* Extended 1-Wire functions */
int OWSpeed(hw_ow_t* hw_ow, int new_speed);
int OWLevel(hw_ow_t* hw_ow, int level);
int OWProgramPulse(hw_ow_t* hw_ow);
int OWWriteBytePower(hw_ow_t* hw_ow, int sendbyte);
int OWReadBitPower(hw_ow_t* hw_ow, int applyPowerResponse);

#endif /* DALLASONEWIRE_DALLASONEWIRE_H_ */
