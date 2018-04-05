// SoftwareWire.h
//
// 2008, Raul wrote a I2C with bit banging as an exercise.
// http://codinglab.blogspot.nl/2008/10/i2c-on-avr-using-bit-banging.html
//
// 2010-2012, Tod E. Kurt takes some tricks from Raul,
// and wrote the SoftI2CMaster library for the Arduino environment.
// https://github.com/todbot/SoftI2CMaster
// http://todbot.com/blog/
//
// 2014-2015, Testato updates the SoftI2CMaster library to make it faster
// and to make it compatible with the Arduino 1.x API
// Also changed I2C waveform and added speed selection.
//
// 2015, Peter_n renames the library into "SoftwareWire",
// and made it a drop-in replacement for the Wire library.
//
// 2018, John Gillis updated the library to work beyond Arduino


#ifndef SoftwareWire_h
#define SoftwareWire_h

#include <stdlib.h>
#include <avr/io.h>

// Transmission status error, the return value of endTransmission()
#define SOFTWAREWIRE_NO_ERROR       0
#define SOFTWAREWIRE_BUFFER_FULL    1
#define SOFTWAREWIRE_ADDRESS_NACK   2
#define SOFTWAREWIRE_DATA_NACK      3
#define SOFTWAREWIRE_OTHER          4

#define SOFTWAREWIRE_BUFSIZE 32        // same as buffer size of Arduino Wire library

// Pins are on Port B
void softwareWire_init_long(uint8_t sdaPin, uint8_t sclPin, int pullups);
void softwareWire_init(uint8_t sdaPin, uint8_t sclPin);

void begin(void);
void setClock(uint32_t clock);
void beginTransmission(uint8_t address);
uint8_t endTransmission(int sendStop);
uint8_t requestFrom(uint8_t address, uint8_t size, int sendStop);
uint8_t write(uint8_t data);
int available(void);
int read(void);
int readBytes(uint8_t* buf, uint8_t size);
int peek(void);
void setTimeout(long timeout);           // timeout to wait for the I2C bus

// Class replacement struct
typedef struct SoftwareWireVars {
	uint8_t _sdaPin;
  	uint8_t _sclPin;
  	uint8_t _sdaBitMask;
  	uint8_t _sclBitMask;
  
  	volatile uint8_t *_sdaPortReg;
  	volatile uint8_t *_sclPortReg;
  	volatile uint8_t *_sdaDirReg;
  	volatile uint8_t *_sclDirReg;
  	volatile uint8_t *_sdaPinReg;
  	volatile uint8_t *_sclPinReg;

	uint8_t _transmission;      // transmission status, returned by endTransmission(). 0 is no error.
  	uint16_t _i2cdelay;         // delay in micro seconds for sda and scl bits.
  	int _pullups;           // using the internal pullups or not
  	int _stretch;           // should code handle clock stretching by the slave or not.
  	unsigned long _timeout;     // timeout in ms. When waiting for a clock pulse stretch. 2017, Fix issue #6

  	uint8_t rxBuf[SOFTWAREWIRE_BUFSIZE];   // buffer inside this class, a buffer per SoftwareWire.
  	uint8_t rxBufPut;           // index to rxBuf, just after the last valid byte.
  	uint8_t rxBufGet;           // index to rxBuf, the first new to be read byte.

} SoftwareWireVars;

  
// private methods  
void i2c_writebit( uint8_t c );
uint8_t i2c_readbit(void);
void i2c_init(void);
int i2c_start(void);
void i2c_repstart(void);
void i2c_stop(void);
uint8_t i2c_write(uint8_t c);
uint8_t i2c_read(int ack);


/* --- Arduino Code Reimplemented ---*/

// safe access to millis counter
uint64_t millis(void);

#endif // SoftwareWire_h
