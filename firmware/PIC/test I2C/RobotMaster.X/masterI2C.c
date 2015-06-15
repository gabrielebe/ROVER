#include "masterI2C.h"
#include <pic18f47j53.h>
#include <xc.h>

// I2C Bus Control Definition
#define I2C_DATA_ACK 0
#define I2C_DATA_NOACK 1
#define I2C_WRITE_CMD 0
#define I2C_READ_CMD 1

#define I2C_START_CMD 0
#define I2C_REP_START_CMD 1
#define I2C_REQ_ACK 0
#define I2C_REQ_NOACK 0

#define I2C_WRITE 0
#define I2C_READ 1

void i2c_init(void) {
  
  TRISBbits.TRISB4 = 1;
  TRISBbits.TRISB5 = 1;

  // Initial the PIC18F14K22 MSSP Peripheral I2C Master Mode
  // I2C Master Clock Speed: 16000000 / ((4 * (SSPADD + 1)) = 16000000 / (4 * (39 + 1))
  SSPSTATbits.SMP = 0; //working at 400KHz
  SSPCON1 = 0x28;      // Enable SDA and SCL, I2C Master mode, clock = FOSC/(4 * (SSPADD + 1))
  SSPCON2 = 0x00;      // Reset MSSP Control Register
  SSPADD = 7;         // Standard I2C Clock speed: 100 kHz

  PIR1bits.SSPIF=0;    // Clear MSSP Interrupt Flag
}

void i2c_idle(void)
{
  // Wait I2C Bus and Status Idle (i.e. ACKEN, RCEN, PEN, RSEN, SEN)
  while (( SSPCON2 & 0x1F ) || ( SSPSTATbits.R_nW));
}

void i2c_start(unsigned char stype)
{
  i2c_idle();                     // Ensure the I2C module is idle
  if (stype == I2C_START_CMD) {
    SSPCON2bits.SEN = 1;          // Start I2C Transmission
    while(SSPCON2bits.SEN);
  } else {
    SSPCON2bits.RSEN = 1;         // ReStart I2C Transmission
    while(SSPCON2bits.RSEN);
  }
}

void i2c_stop(void)
{
  // Stop I2C Transmission
  SSPCON2bits.PEN = 1;
  while(SSPCON2bits.PEN);
}

unsigned char i2c_slave_ack(void)
{
  // Return: 1 = Acknowledge was not received from slave
  //         0 = Acknowledge was received from slave
  return(SSPCON2bits.ACKSTAT);
}

void i2c_write(unsigned char data)
{
  // Send the Data to I2C Bus
  SSPBUF = data;
  if (SSPCON1bits.WCOL)         // Check for write collision
    return;

  while(SSPSTATbits.BF);        // Wait until write cycle is complete
  i2c_idle();                   // Ensure the I2C module is idle
}

void i2c_master_ack(unsigned char ack_type)
{
  SSPCON2bits.ACKDT = ack_type;   // 1 = Not Acknowledge, 0 = Acknowledge
  SSPCON2bits.ACKEN = 1;          // Enable Acknowledge
  while (SSPCON2bits.ACKEN == 1);
}

unsigned char i2c_read(void)
{
  // Ensure the I2C module is idle
  i2c_idle();

  // Enable Receive Mode
  SSPCON2bits.RCEN = 1;           // Enable master for 1 byte reception
  while(!SSPSTATbits.BF);         // Wait until buffer is full
  return(SSPBUF);
}

void sendByte(unsigned char data, unsigned char address)
{
      // Start the I2C Write Transmission
  i2c_start(I2C_START_CMD);

  // Write the address
  i2c_write(address|I2C_WRITE_CMD);

  // Write data to PCA8574 Register
  i2c_write(data);

  // Send No Acknowledge to the Slave
  i2c_master_ack(I2C_DATA_NOACK);

  // Stop I2C Transmission
  i2c_stop();
}
unsigned char readByte(unsigned char address)
{
  unsigned char data;
// Start the I2C Write Transmission
  i2c_start(I2C_START_CMD);

  // read the given node
  i2c_write(address|I2C_READ_CMD);

  // Get the High Byte of MCP9801 I2C Temp Sensor
  data=i2c_read();

  // Send No Acknowledge to the Slave
  i2c_master_ack(I2C_DATA_NOACK);

  // Stop I2C Transmission
  i2c_stop();
  return data;
}