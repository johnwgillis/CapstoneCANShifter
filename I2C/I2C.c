// SoftwareWire.cpp
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
// 21 sep 2015: 
//  added code to i2c_stop(), since a problem was reported here: 
//  http://forum.arduino.cc/index.php?topic=348337.0
//  Added lines have keyword "ADDED1".
//
// 2018:
//  John Gillis made lots of changes to get working beyond Arduino. The inline documentation was not completely updated.

// Signal differences
// ------------------
//    When the AVR microcontroller is set into hardwere I2C mode, 
//    the pins follow the I2C specifications for voltage levels and current.
//    For example the current to pull SDA or SCL low is maximum 3mA.
//
//    With the Software I2C, a normal pin is used which can sink/source 40mA for a ATmega328P.
//    That could increase the voltage spikes and could increase interference between sda and scl.
//    The voltage levels are different.
//    The timing of the sofware I2C is also different.
//
//    In most cases the software I2C should work.
//    With longer wires or with non-matching voltage levels, the result is unpredictable.
//    
//
//
// Clock pulse stretching
// ----------------------
//    An I2C Slave could stretch the clock signal by keeping the SCL low.
//    This happens for example when a Slave is an Arduino which can be
//    busy doing other things like handling other interrupts.
//    Adding a check for the clock stretching should make the transmission
//    completely reliable without any loss.
//    Only an Arduino as Slave would do clock stretching, normal devices
//    like sensors and I2C EEPROM don't use clock stretching.
//    The extra check for clock stretching slows down the transfer rate.
//
//    Using millis() instead of micros() is faster.
//    That is why millis() is used for the timeout of the clock pulse stretching.
//    
//
//
// Arduino Stream class
// --------------------
//    The Arduino Stream class is used by many Arduino objects.
//    For the I2C bus, the benefits are less obvious.
//    For example the parseInt() function is not used with I2C.
//    At this moment the Stream class is not used.
//
//
// Multiple Slaves with the same I2C address
// -----------------------------------------
//    The SoftwareWire can be declared more than once,
//    to create a number of software i2c busses.
//    That makes it possible to use a number of I2C devices,
//    which have the same I2C address.
//    Every software i2c bus requires 2 pins, 
//    and every SoftwareWire object requires 59 bytes at the moment.
//
//

#include <stdlib.h>
#include <avr/io.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include "FreeRTOS.h"
#include "task.h"
#include "I2C.h"

#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#define __DELAY_BACKWARD_COMPATIBLE__
#include <util/delay.h>


// Sets SDA low and drives output.
// The SDA may not be HIGH output, so first the output register is cleared 
// (clearing internal pullup resistor), after that the SDA is set as output.
#define i2c_sda_lo()              \
  *softwareWireVars._sdaPortReg &= ~softwareWireVars._sdaBitMask;   \
  *softwareWireVars._sdaDirReg  |=  softwareWireVars._sdaBitMask;
  

// sets SCL low and drives output.
// The SCL may not be HIGH output, so first the output register is cleared 
// (clearing internal pullup resistor), after that the SCL is set as output.
#define i2c_scl_lo()              \
  *softwareWireVars._sclPortReg &= ~softwareWireVars._sclBitMask;   \
  *softwareWireVars._sclDirReg  |=  softwareWireVars._sclBitMask;


// Set SDA high and to input (releases pin) (i.e. change to input,turnon pullup).
// The SDA may not become HIGH output. Therefor the pin is first set to input,
// after that, a pullup resistor is switched on if needed.
#define i2c_sda_hi()              \
  *softwareWireVars._sdaDirReg &= ~softwareWireVars._sdaBitMask;    \
  if(softwareWireVars._pullups) { *softwareWireVars._sdaPortReg |= softwareWireVars._sdaBitMask; }


// set SCL high and to input (releases pin) (i.e. change to input,turnon pullup)
// The SCL may not become HIGH output. Therefor the pin is first set to input,
// after that, a pullup resistor is switched on if needed.
#define i2c_scl_hi()              \
  *softwareWireVars._sclDirReg &= ~softwareWireVars._sclBitMask;    \
  if(softwareWireVars._pullups) { *softwareWireVars._sclPortReg |= softwareWireVars._sclBitMask; }


// Read the bit value of the pin
// Note that is the pin can also be read when it is an output.
#define i2c_sda_read()   ((uint8_t) (*softwareWireVars._sdaPinReg & softwareWireVars._sdaBitMask) ? 1 : 0)
#define i2c_scl_read()   ((uint8_t) (*softwareWireVars._sclPinReg & softwareWireVars._sclBitMask) ? 1 : 0)


// Stores the singleton class replacement struct
static SoftwareWireVars softwareWireVars;

void softwareWire_init(uint8_t sdaPin, uint8_t sclPin) {
	softwareWire_init_long(sdaPin, sclPin, pdFALSE);
}

//
// Constructor
//
// The pins are not activated until begin() is called.
//
void softwareWire_init_long(uint8_t sdaPin, uint8_t sclPin, int pullups)
{
  softwareWireVars._sdaPin = sdaPin;
  softwareWireVars._sclPin = sclPin;
  softwareWireVars._pullups = pullups;
  softwareWireVars._stretch = pdFALSE; // forced false since millis() isn't properly defined
  
  setClock( 100000UL);       // set default 100kHz
  
  // Set default timeout to 1000 ms. 
  // 1 second is very long, 10ms would be more appropriate.
  // However, the Arduino libraries use often a default timeout of 1 second.
  setTimeout( 1000L);        

  // Turn Arduino pin numbers into PORTx, DDRx, and PINx
  const uint8_t digital_pin_to_bit_mask[8] = {
        _BV(0), /* 0, port B */
        _BV(1),
        _BV(2),
        _BV(3),
        _BV(4),
        _BV(5),
        _BV(6),
        _BV(7)
  };

  softwareWireVars._sdaBitMask  = digital_pin_to_bit_mask[softwareWireVars._sdaPin];
  softwareWireVars._sdaPortReg  = &PORTB;
  softwareWireVars._sdaDirReg   = &DDRB;
  softwareWireVars._sdaPinReg   = &PINB;      // PinReg is the input register, not the Arduino pin.

  softwareWireVars._sclBitMask  = digital_pin_to_bit_mask[softwareWireVars._sclPin];
  softwareWireVars._sclPortReg  = &PORTB;
  softwareWireVars._sclDirReg   = &DDRB;
  softwareWireVars._sclPinReg   = &PINB;
}


// The pins are not changed until begin() is called.
void begin(void)
{
  softwareWireVars.rxBufPut = 0;          // nothing in the softwareWireVars.rxBuf
  softwareWireVars.rxBufGet = 0;

  i2c_init();            // relase the sda and scl (the pullup resistors pull them high)

  // Some tests could be added here, to check if the SDA and SCL are really turning high.
  // Even some tests for shortcuts could be done here.
  
  // When a I2C transmission would start immediate, it could fail when only the internal pullup resistors
  // are used, and the signals were just now turned high with i2c_init().
  if( softwareWireVars._pullups)
    _delay_ms(2);           // 1ms didn't always work.
}

//
// beginTransmission starts the I2C transmission immediate.
//
void beginTransmission(uint8_t address)
{
  // Reset error returned by endTransmission.
  softwareWireVars._transmission = SOFTWAREWIRE_NO_ERROR;
  
  // check return value of the start condition.
  // It indicates if the i2c bus is okay.
  if(i2c_start())             
  {
    uint8_t rc = i2c_write((address << 1) | 0);       // The r/w bit is zero for write
    
    if( rc == 0)                                      // a sda zero from Slave for the 9th bit is ack
    {
      softwareWireVars._transmission = SOFTWAREWIRE_NO_ERROR;
    }
    else
    {
      softwareWireVars._transmission = SOFTWAREWIRE_ADDRESS_NACK;
    }
  }
  else
  {
    // If the bus was not okay, the scl or sda didn't work.
    softwareWireVars._transmission = SOFTWAREWIRE_OTHER;
  }
}

//
uint8_t endTransmission(int sendStop)
{
  if(sendStop)
    i2c_stop();
  else
    i2c_repstart();
  
  return(softwareWireVars._transmission);          // return the transmission status that was set during writing address and data
}


//
// The requestFrom() read the data from the I2C bus and stores it in a buffer.
//
uint8_t requestFrom(uint8_t address, uint8_t size, int sendStop)
{
  uint8_t n=0;             // number of valid received bytes. Start with 0 bytes.

  // The stransmission status is set, allthough it is not returned.
  // Start with the status : no error
  softwareWireVars._transmission = SOFTWAREWIRE_NO_ERROR;


  // Clear the RX buffer
  softwareWireVars.rxBufPut = 0;
  softwareWireVars.rxBufGet = 0;

  int bus_okay = i2c_start();
  
  if(bus_okay)
  {
    uint8_t rc = i2c_write((address << 1) | 1);          // The r/w bit is '1' to read
    
    if( rc == 0)                                         // a sda zero from Slave for the 9th bit is ack
    {
      softwareWireVars._transmission = SOFTWAREWIRE_NO_ERROR;
  
      // TODO: check if the Slave returns less bytes than requested.
      
      for(; n<size; n++)
      {
        if( n < (size - 1))
          softwareWireVars.rxBuf[n] = i2c_read(pdTRUE);        // read with ack
        else
          softwareWireVars.rxBuf[n] = i2c_read(pdFALSE);       // last byte, read with nack
      }
      softwareWireVars.rxBufPut = n;
    }
    else
    {
      softwareWireVars._transmission = SOFTWAREWIRE_ADDRESS_NACK;
    }
  }
  else
  {
    // There was a bus error.
    softwareWireVars._transmission = SOFTWAREWIRE_OTHER;
  }
  
  if(sendStop || softwareWireVars._transmission != SOFTWAREWIRE_NO_ERROR)
    i2c_stop();
  else
    i2c_repstart();

  return( n);
}


// must be called in:
// slave tx event callback
// or after beginTransmission(address)
uint8_t write(uint8_t data)
{
  // When there was an error during the transmission, no more bytes are transmitted.
  if( softwareWireVars._transmission == SOFTWAREWIRE_NO_ERROR)
  {
    if( i2c_write(data) == 0)                // a sda zero from Slave for the 9th bit is ack
    {
      softwareWireVars._transmission = SOFTWAREWIRE_NO_ERROR;
    }
    else
    {
      softwareWireVars._transmission = SOFTWAREWIRE_ADDRESS_NACK;
    }
  }
    
  return(1);             // ignore any errors, return the number of bytes that are written.
}

//
// The read() reads the buffer, not the I2C bus.
//
int read(void)
{
  int data;
  
  if( softwareWireVars.rxBufPut > softwareWireVars.rxBufGet)
  {
    data = softwareWireVars.rxBuf[softwareWireVars.rxBufGet];
    softwareWireVars.rxBufGet++;
  }
  else
  {
    data = -1;
  }
  
  return(data);
}


int readBytes(uint8_t* buf, uint8_t size)
{
  int data;
  int n;
  
  for( n=0; n<size; n++)
  {
    data = read();
    if( data == -1)
      break;
    else
      buf[n] = (uint8_t) data;
  }
  
  return(n);
}

//
// Set the clock speed for the I2C bus.
// Default is 100000 (100kHz).
// A speed of 1Hz is possible with this software I2C library (but not with the Arduino Wire library).
// A speed of 200kHz or higher will remove the delay on an Arduino Uno.
// Without the delay, the functions are free running, using the execution timing of the code.
//
void setClock(uint32_t clock)
{
  // Tested values with an earlier version of this library.
  //   Value 0 is without delay, the result depends on the microcontroller and the cpu clock.
  //   0=maxspeed=140kHz (tested on 328P@16MHz)
  //   1=120kHz
  //   2=100kHz (default)
  //   7=50kHz
  //   47=10kHz
  //   97=5kHz
  //   500=1kHz
  //   5000=100Hz
  //   16383=minspeed=30Hz  - _delay_us() max value reference arduino
  // 

  // The softwareWireVars._i2cdelay is an uint16_t
  softwareWireVars._i2cdelay = ( (F_CPU / 32L) / clock );               // The delay in microseconds, '32' is for this code.
  unsigned int delayByCode = (F_CPU / 5000000L);       // Add some delay for the code, just a guess
  
  if( softwareWireVars._i2cdelay > delayByCode)
    softwareWireVars._i2cdelay -= delayByCode;
  else
    softwareWireVars._i2cdelay = 0;

}


// 
// Set the timeout in milliseconds.
// At this moment, it is only used for timeout when the Slave is stretching the clock pulse.
//
void setTimeout(long timeout)    
{
  // 2017, fix issue #6. 
  // A signed long as parameter to be compatible with Arduino libraries.
  // A unsigned long internal to avoid compiler warnings.
  softwareWireVars._timeout = (unsigned long) timeout;
}

//--------------------------------------------------------------------


//
// The i2c_writebit and i2c_readbit could be make "inline", but that
// didn't increase the speed, and the code size increases.
//
// The sda is low after the start condition.
// Therefor the sda is low for the first bit.
//
void i2c_writebit(uint8_t c)
{
  if(c==0)
  {
    i2c_sda_lo();
  }
  else
  {
    i2c_sda_hi();
  }
  
  if (softwareWireVars._i2cdelay != 0)               // This delay is not needed, but it makes it safer
    _delay_us(softwareWireVars._i2cdelay);   // This delay is not needed, but it makes it safer
  
  i2c_scl_hi();                     // clock high: the Slave will read the sda signal
  
  // Check if clock stretching by the Slave should be detected.
  if( softwareWireVars._stretch)
  {
    // If the Slave was strechting the clock pulse, the clock would not go high immediately.
    // For example if the Slave is an Arduino, that has other interrupts running (for example Serial data).
    unsigned long prevMillis = millis();
    while( i2c_scl_read() == 0)
    {
      if( millis() - prevMillis >= softwareWireVars._timeout)
        break;
    };
  }

  // After the clock stretching, the clock must be high for the normal duration.
  // That is why this delay has still to be done.
  if (softwareWireVars._i2cdelay != 0)
    _delay_us(softwareWireVars._i2cdelay);

  i2c_scl_lo();
  
  if (softwareWireVars._i2cdelay != 0)
    _delay_us(softwareWireVars._i2cdelay);
}


//
uint8_t i2c_readbit(void)
{
  i2c_sda_hi();            // 'hi' is the same as releasing the line
  i2c_scl_hi();

  // Check if clock stretching by the Slave should be detected.
  if( softwareWireVars._stretch)
  {
    // Wait until the clock is high, the Slave could keep it low for clock stretching.
    unsigned long prevMillis = millis();
    while( i2c_scl_read() == 0)
    {
      if( millis() - prevMillis >= softwareWireVars._timeout)
        break;
    };
  }

  // After the clock stretching, this delay has still be done before reading sda.
  if (softwareWireVars._i2cdelay != 0)
    _delay_us(softwareWireVars._i2cdelay);
  
  uint8_t c = i2c_sda_read();
  
  i2c_scl_lo();

  if (softwareWireVars._i2cdelay != 0)
    _delay_us(softwareWireVars._i2cdelay);

  return(c);
}


//
// Initializes the Software I2C.
//
// The original i2c_init sets the SDA and SCL high at the same time.
//
// The code has been changed, since the first data to the software i2c did fail sometimes.
// Changed into SCL high first, with a delay.
// That would send a STOP if the SDA happens to be low.
// Any Slave that was busy, will detect the STOP.
//
// After both lines are high, the delay is changed into 4 times the normal delay.
// That did reduce the error with the first tranmission.
// It was tested with Arduino Uno with clock of 100kHz (softwareWireVars._i2cdelay=2).
// 
void i2c_init(void)
{
  i2c_scl_hi();

  if (softwareWireVars._i2cdelay != 0)
    _delay_us(softwareWireVars._i2cdelay);

  i2c_sda_hi();

  for( uint8_t i=0; i<4; i++)             // 4 times the normal delay, to claim the bus.
  {
    if (softwareWireVars._i2cdelay != 0)
      _delay_us(softwareWireVars._i2cdelay);
  }
}


//
// Send a START Condition
//
// The SDA and SCL should already be high.
// 
// The SDA and SCL will both be low after this function.
// When writing the address, the Master makes them high.
// 
// Return value:
//   pdTRUE  : software i2c bus is okay.
//   pdFALSE : failed, some kind of hardware bus error.
//
int i2c_start(void)
{
  i2c_sda_hi();              // can perhaps be removed some day ? if the rest of the code is okay
  i2c_scl_hi();              // can perhaps be removed some day ? if the rest of the code is okay

  if (softwareWireVars._i2cdelay != 0)
    _delay_us(softwareWireVars._i2cdelay);
    
  // Both the sda and scl should be high.
  // If not, there might be a hardware problem with the i2c bus signal lines.
  // This check was added to prevent that a shortcut of sda would be seen as a valid ACK
  // from a i2c Slave.
  uint8_t sda_status = i2c_sda_read();
  uint8_t scl_status = i2c_scl_read();
  if(sda_status == 0 || scl_status == 0)
  {
    return(pdFALSE);
  }
  else
  {
    i2c_sda_lo();
    
    if (softwareWireVars._i2cdelay != 0)
      _delay_us(softwareWireVars._i2cdelay);
  
    i2c_scl_lo();
    
    if (softwareWireVars._i2cdelay != 0)
      _delay_us(softwareWireVars._i2cdelay);
  }
  return(pdTRUE);
}


//
// Repeated START instead of a STOP
// 
// TODO: check if the repeated start actually works.
//
void i2c_repstart(void)
{
  i2c_sda_hi();
//  i2c_scl_hi();               // ??????

  i2c_scl_lo();                         // force SCL low
  
  if (softwareWireVars._i2cdelay != 0)
    _delay_us(softwareWireVars._i2cdelay);

  i2c_sda_hi();                        // release SDA
  
  if (softwareWireVars._i2cdelay != 0)
    _delay_us(softwareWireVars._i2cdelay);

  i2c_scl_hi();                        // release SCL
  
  // Check if clock stretching by the Slave should be detected.
  if( softwareWireVars._stretch)
  {
    // If the Slave was strechting the clock pulse, the clock would not go high immediately.
    // For example if the Slave is an Arduino, that has other interrupts running (for example Serial data).
    unsigned long prevMillis = millis();
    while( i2c_scl_read() == 0)
    {
      if( millis() - prevMillis >= softwareWireVars._timeout)
        break;
    };
  }
  
  if (softwareWireVars._i2cdelay != 0)
    _delay_us(softwareWireVars._i2cdelay);

  i2c_sda_lo();                        // force SDA low
  
  if (softwareWireVars._i2cdelay != 0)
    _delay_us(softwareWireVars._i2cdelay);
}


// Send a STOP Condition
//
// The stop was not recognized by every chip.
// Some code has been added (with comment "ADDED1"),
// to be sure that the levels are okay with delays in between.
void i2c_stop(void)
{
  i2c_scl_lo();         // ADDED1, it should already be low.
  i2c_sda_lo();
  
  // ADDED1, wait to be sure that the slave knows that both are low
  if (softwareWireVars._i2cdelay != 0)              // ADDED1
    _delay_us(softwareWireVars._i2cdelay);  // ADDED1
  
  // For a stop, make SCL high wile SDA is still low
  i2c_scl_hi();
  
  // Check if clock stretching by the Slave should be detected.
  if( softwareWireVars._stretch)
  {
    // Wait until the clock is high, the Slave could keep it low for clock stretching.
    // Clock pulse stretching during a stop condition seems odd, but when
    // the Slave is an Arduino, it might happen.
    unsigned long prevMillis = millis();
    while( i2c_scl_read() == 0)
    {
      if( millis() - prevMillis >= softwareWireVars._timeout)
        break;
    };
  }
   
  if (softwareWireVars._i2cdelay != 0)
    _delay_us(softwareWireVars._i2cdelay);

  // complete the STOP by setting SDA high
  i2c_sda_hi();
  
  // A delay after the STOP for safety.
  // It is not known how fast the next START will happen.
  if (softwareWireVars._i2cdelay != 0)
    _delay_us(softwareWireVars._i2cdelay);
}


//
// Write a byte to the I2C slave device
// The returned bit is 0 for ACK and 1 for NACK
//
uint8_t i2c_write( uint8_t c )
{
  for ( uint8_t i=0; i<8; i++) 
  {
    i2c_writebit(c & 0x80);           // highest bit first
    c <<= 1;
  }

  return(i2c_readbit());
}


//
// read a byte from the I2C slave device
//
uint8_t i2c_read(int ack)
{
  uint8_t res = 0;

  for(uint8_t i=0; i<8; i++) 
  {
    res <<= 1;
    res |= i2c_readbit();
  }

  if(ack)
  {
    i2c_writebit(0);
  }
  else
  {
    i2c_writebit(1);
  }

  if(softwareWireVars._i2cdelay != 0)
    _delay_us(softwareWireVars._i2cdelay);

  return(res);
}



/* --- Arduino Code Reimplemented ---*/

// safe access to millis counter
uint64_t millis(void) {
  return 0;
}
