/*
    This file is part of Repetier-Firmware.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

    This firmware is a nearly complete rewrite of the sprinter firmware
    by kliment (https://github.com/kliment/Sprinter)
    which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.

  Functions in this file are used to communicate using ascii or repetier protocol.
*/

#ifndef HAL_H
#define HAL_H

/**
  This is the main Hardware Abstraction Layer (HAL).
  To make the firmware work with different processors and toolchains,
  all hardware related code should be packed into the hal files.
*/

// Hack to make 84 MHz Due clock work without changes to pre-existing code
// which would otherwise have problems with int overflow.
// Timer routines must have 4x correction factor to compensate.
#define F_CPU       21000000
#define F_CPU_TRUE  84000000

#define PROGMEM
#define PGM_P const char *
#define PSTR(s) s
#define pgm_read_byte_near(x) (*(char*)x)
#define pgm_read_byte(x) (*(char*)x)
#define pgm_read_float(addr) (*(const float *)(addr))
#define pgm_read_word(addr) (*(const unsigned int *)(addr))
#define pgm_read_word_near(addr) pgm_read_word(addr)
#define pgm_read_dword(addr) (*(const unsigned long *)(addr))
#define pgm_read_dword_near(addr) pgm_read_dword(addr)

#define FSTRINGVALUE(var,value) const char var[] PROGMEM = value;
#define FSTRINGVAR(var) static const char var[] PROGMEM;
#define FSTRINGPARAM(var) PGM_P var

#include <inttypes.h>

#if MOTHERBOARD == 401
#define EXTRUDER_TIMER          TC0
#define EXTRUDER_TIMER_CHANNEL  0
#define EXTRUDER_TIMER_IRQ      ID_TC0
#define EXTRUDER_TIMER_VECTOR   TC0_Handler
#define PWM_TIMER               TC0
#define PWM_TIMER_CHANNEL       1
#define PWM_TIMER_IRQ           ID_TC1
#define PWM_TIMER_VECTOR        TC1_Handler
#define TIMER1_TIMER            TC2
#define TIMER1_TIMER_CHANNEL    2
#define TIMER1_TIMER_IRQ        ID_TC8
#define TIMER1_COMPA_VECTOR     TC8_Handler
#define SERVO_TIMER             TC2
#define SERVO_TIMER_CHANNEL     0
#define SERVO_TIMER_IRQ         ID_TC6
#define SERVO_COMPA_VECTOR      TC6_Handler
#define BEEPER_TIMER            TC1
#define BEEPER_TIMER_CHANNEL    0
#define BEEPER_TIMER_IRQ        ID_TC3
#define BEEPER_TIMER_VECTOR     TC3_Handler


#define EXTRUDER_CLOCK_FREQ     244    // don't know what this should be
#define PWM_CLOCK_FREQ          3096
#define TIMER1_CLOCK_FREQ       244
#define SERVO_CLOCK_FREQ        0

#define AD_PRESCALE_FACTOR      41  // 1 MHz ADC clock 
#define AD_TRACKING_CYCLES      0   // 0 - 15     + 1 adc clock cycles
#define AD_TRANSFER_CYCLES      1   // 0 - 3      * 2 + 3 adc clock cycles

#define ADC_ISR_EOC(channel)    (0x1u << channel) 
#define ENABLED_ADC_CHANNELS    {TEMP_0_PIN, TEMP_1_PIN, TEMP_2_PIN}  

#define PULLUP(IO,v)            WRITE(IO, v)

#define TWI_CLOCK               204

#endif

#include "pins.h"

#ifndef SOFTWARE_SPI
#include <SPI.h>
#endif


#include "Print.h"

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#define COMPAT_PRE1
#endif
#define	READ(IO)  digitalRead(IO)
#define	WRITE(IO, v)  digitalWrite(IO, v)
#define	SET_INPUT(IO)  pinMode(IO, INPUT)
#define	SET_OUTPUT(IO)  pinMode(IO, OUTPUT)

#define BEGIN_INTERRUPT_PROTECTED noInterrupts();
#define END_INTERRUPT_PROTECTED interrupts();
#define ESCAPE_INTERRUPT_PROTECTED  interrupts();

#define EEPROM_OFFSET               0
#define SECONDS_TO_TICKS(s) (unsigned long)(s*(float)F_CPU)
#define ANALOG_REDUCE_BITS 0
#define ANALOG_REDUCE_FACTOR 1

#define MAX_RAM 98303

#define bit_clear(x,y) x &= ~(1<<y) //cbi(x,y)
#define bit_set(x,y)   x |= (1<<y)//sbi(x,y)

/** defines the data direction (reading from I2C device) in i2cStart(),i2cRepStart() */
#define I2C_READ    1
/** defines the data direction (writing to I2C device) in i2cStart(),i2cRepStart() */
#define I2C_WRITE   0

#if ANALOG_INPUTS>0
static const uint32_t adcChannel[] = ENABLED_ADC_CHANNELS;
#endif

static uint32_t    tone_pin;

typedef unsigned int speed_t;
typedef unsigned long ticks_t;
typedef unsigned long millis_t;

#define RFSERIAL Serial

#define OUT_P_I(p,i) Com::printF(PSTR(p),(int)(i))
#define OUT_P_I_LN(p,i) Com::printFLN(PSTR(p),(int)(i))
#define OUT_P_L(p,i) Com::printF(PSTR(p),(long)(i))
#define OUT_P_L_LN(p,i) Com::printFLN(PSTR(p),(long)(i))
#define OUT_P_F(p,i) Com::printF(PSTR(p),(float)(i))
#define OUT_P_F_LN(p,i) Com::printFLN(PSTR(p),(float)(i))
#define OUT_P_FX(p,i,x) Com::printF(PSTR(p),(float)(i),x)
#define OUT_P_FX_LN(p,i,x) Com::printFLN(PSTR(p),(float)(i),x)
#define OUT_P(p) Com::printF(PSTR(p))
#define OUT_P_LN(p) Com::printFLN(PSTR(p))
#define OUT_ERROR_P(p) Com::printErrorF(PSTR(p))
#define OUT_ERROR_P_LN(p) {Com::printErrorF(PSTR(p));Com::println();}
#define OUT(v) Com::print(v)
#define OUT_LN Com::println()

class HAL
{
public:
    HAL();
    virtual ~HAL();
    // return val'val
    static inline unsigned long U16SquaredToU32(unsigned int val)
    {
        return (unsigned long) val * (unsigned long) val;
    }
    static inline unsigned int ComputeV(long timer,long accel)
    {
        return ((timer>>8)*accel)>>10;
    }
// Multiply two 16 bit values and return 32 bit result
    static inline unsigned long mulu16xu16to32(unsigned int a,unsigned int b)
    {
        return (unsigned long) a * (unsigned long) b;
    }
// Multiply two 16 bit values and return 32 bit result
    static inline unsigned int mulu6xu16shift16(unsigned int a,unsigned int b)
    {
        return ((unsigned long)a*(unsigned long)b)>>16;
    }
    static inline long Div4U2U(unsigned long a,unsigned int b) {
        return (unsigned long) a / (unsigned long) b;
    }
    static inline void digitalWrite(byte pin,byte value)
    {
        ::digitalWrite(pin,value);
    }
    static inline byte digitalRead(byte pin)
    {
        return ::digitalRead(pin);
    }
    static inline void pinMode(byte pin,byte mode)
    {
        ::pinMode(pin,mode);
    }
    static long CPUDivU2(unsigned int divisor);
    static inline void delayMicroseconds(unsigned int delayUs)
    {
        ::delayMicroseconds(delayUs);
    }
    static inline void delayMilliseconds(unsigned int delayMs)
    {
        ::delay(delayMs);
    }
    static inline void tone(byte pin,int frequency) {
        // set up timer counter 1 channel 0 to generate interrupts for
        // toggling output pin.  
        SET_OUTPUT(pin);
        tone_pin = pin;
        pmc_set_writeprotect(false);
        pmc_enable_periph_clk((uint32_t)BEEPER_TIMER_IRQ);
        // set interrupt to lowest possible priority
        NVIC_SetPriority((IRQn_Type)EXTRUDER_TIMER_IRQ, NVIC_EncodePriority(4, 6, 3));
        TC_Configure(BEEPER_TIMER, BEEPER_TIMER_CHANNEL, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | 
                     TC_CMR_TCCLKS_TIMER_CLOCK4);  // TIMER_CLOCK4 -> 128 divisor
        uint32_t rc = VARIANT_MCK / 128 / frequency; 
        TC_SetRA(BEEPER_TIMER, BEEPER_TIMER_CHANNEL, rc/2);                     // 50% duty cycle
        TC_SetRC(BEEPER_TIMER, BEEPER_TIMER_CHANNEL, rc);
        TC_Start(BEEPER_TIMER, BEEPER_TIMER_CHANNEL);
        BEEPER_TIMER->TC_CHANNEL[BEEPER_TIMER_CHANNEL].TC_IER=TC_IER_CPCS;
        BEEPER_TIMER->TC_CHANNEL[BEEPER_TIMER_CHANNEL].TC_IDR=~TC_IER_CPCS;
        NVIC_EnableIRQ((IRQn_Type)BEEPER_TIMER_IRQ);
    }
    static inline void noTone(byte pin) {
        TC_Stop(TC1, 0); 
        WRITE(pin, LOW);
    }
    static inline void epr_set_byte(unsigned int pos,byte value)
    {
//        eeprom_write_byte((unsigned char *)(EEPROM_OFFSET+pos), value);
    }
    static inline void epr_set_int(unsigned int pos,int value)
    {
//        eeprom_write_word((unsigned int*)(EEPROM_OFFSET+pos),value);
    }
    static inline void epr_set_long(unsigned int pos,long value)
    {
//        eeprom_write_dword((unsigned long*)(EEPROM_OFFSET+pos),value);
    }
    static inline void epr_set_float(unsigned int pos,float value)
    {
//        eeprom_write_block(&value,(void*)(EEPROM_OFFSET+pos), 4);
    }
    static inline byte epr_get_byte(unsigned int pos)
    {
//        return eeprom_read_byte ((unsigned char *)(EEPROM_OFFSET+pos));
        return 0;
    }
    static inline int epr_get_int(unsigned int pos)
    {
//        return eeprom_read_word((unsigned int *)(EEPROM_OFFSET+pos));
        return 0;
    }
    static inline long epr_get_long(unsigned int pos)
    {
//        return eeprom_read_dword((unsigned long*)(EEPROM_OFFSET+pos));
        return 0;
    }
    static inline float epr_get_float(unsigned int pos)
    {
        float v = 0.0;
//        eeprom_read_block(&v,(void *)(EEPROM_OFFSET+pos),4); // newer gcc have eeprom_read_block but not arduino 22
        return v;
    }
    static inline void allowInterrupts()
    {
//        __enable_irq();
    }
    static inline void forbidInterrupts()
    {
//        __disable_irq();
    }
    static inline unsigned long timeInMilliseconds()
    {
        return millis();
    }
    static inline char readFlashByte(PGM_P ptr)
    {
        return pgm_read_byte(ptr);
    }
    static inline void serialSetBaudrate(long baud)
    {
        RFSERIAL.begin(baud);
    }
    static inline bool serialByteAvailable()
    {
        return RFSERIAL.available();
    }
    static inline byte serialReadByte()
    {
        return RFSERIAL.read();
    }
    static inline void serialWriteByte(char b)
    {
        RFSERIAL.write(b);
    }
    static inline void serialFlush()
    {
        RFSERIAL.flush();
    }
    static void setupTimer();
    static void showStartReason();
    static int getFreeRam();
    static void resetHardware();

    // SPI related functions

#ifdef SOFTWARE_SPI
    // bitbanging free-running transfer
    // Too fast?  Too slow?

    static byte spiTransfer(byte b)  // using Mode 0
    {
        for (int bits = 0; bits < 8; bits++) {
            if (b & 0x80) {
                WRITE(MOSI, HIGH);
            } else {
                WRITE(MOSI, LOW);
            }
            b <<= 1;
            WRITE(SCK_PIN, HIGH);

            if(READ(MISO)) {
                b |= 1;
            }
            WRITE(SCK_PIN, LOW);
        }
        return b;
    }

    static inline void spiInit(byte spiClock) 
   {
       SET_OUTPUT(SCK_PIN);
       SET_INPUT(MISO_PIN);
       SET_OUTPUT(MOSI_PIN);
       SET_OUTPUT(SPI_PIN);

       WRITE(SPI_PIN, HIGH);
       WRITE(SCK_PIN, LOW);
   }
   static inline byte spiReceive()
   {
       WRITE(SPI_PIN, LOW);
       byte b = spiTransfer(0);       
       WRITE(SPI_PIN, HIGH);
       return b;
   }
   static inline void spiReadBlock(byte*buf,uint16_t nbyte) 
   {   
       WRITE(SPI_PIN, LOW);  
       for (uint16_t i = 0; i < nbyte; i++)
        {
            buf[i] = spiTransfer(0);  
        }
       WRITE(SPI_PIN, HIGH);

   }
   static inline void spiSend(byte b) {
       WRITE(SPI_PIN, LOW);
       byte response = spiTransfer(b);
       WRITE(SPI_PIN, HIGH);
   }

   inline __attribute__((always_inline))
   static void spiSendBlock(uint8_t token, const uint8_t* buf)
   {
       byte response;

       WRITE(SPI_PIN, LOW);
       response = spiTransfer(token);

       for (uint16_t i = 0; i < 512; i++)
       {
           response = spiTransfer(buf[i]);  
       }
       WRITE(SPI_PIN, HIGH);
   }
   
#else  /*SOFTWARE_SPI*/
   // hardware SPI

    static inline void spiInit(byte spiClock) 
   {
       SPI.begin(SPI_PIN);
       SPI.setBitOrder(SPI_PIN, MSBFIRST);
       SPI.setDataMode(SPI_PIN, SPI_MODE0);
       SPI.setClockDivider(SPI_PIN, F_CPU / spiClock);
   }
   static inline byte spiReceive()
   {
       return SPI.transfer(SPI_PIN, 0x00);
   }
   static inline void spiReadBlock(byte*buf,uint16_t nbyte) 
   {     
       nbyte--;
       for (uint16_t i = 0; i < nbyte; i++)
        {
            buf[i] = SPI.transfer(SPI_PIN, 0, SPI_CONTINUE);  
        }
       buf[nbyte] = SPI.transfer(SPI_PIN, 0, SPI_LAST);  
   }
   static inline void spiSend(byte b) {
       byte response = SPI.transfer(SPI_PIN, b);
   }

   static inline __attribute__((always_inline))
   void spiSendBlock(uint8_t token, const uint8_t* buf)
   {
       byte response;

       response = SPI.transfer(SPI_PIN, token, SPI_CONTINUE);
       for (uint16_t i = 0; i < 511; i++)
       {
           response = SPI.transfer(SPI_PIN, buf[i], SPI_CONTINUE);  
       }
       response = SPI.transfer(SPI_PIN, buf[511], SPI_LAST);
   }
#endif  /*SOFTWARE_SPI*/

    // I2C Support
    static void i2cInit(unsigned long clockSpeedHz);
    static unsigned char i2cStart(unsigned char address);
    static void i2cStartWait(unsigned char address);
    static void i2cStop(void);
    static unsigned char i2cWrite( unsigned char data );
    static unsigned char i2cReadAck(void);
    static unsigned char i2cReadNak(void);


#if FEATURE_SERVO
    static unsigned int servoTimings[4];
    static void servoMicroseconds(byte servo,int ms);
#endif

#if ANALOG_INPUTS>0
    static void analogStart(void);
//    uint32_t    adcChannel[] = ENABLED_ADC_CHANNELS;
#endif
    
protected:
private:
    static uint32_t    currentTWIaddress;
    static uint32_t    twiDirection;
    static bool        twiMultipleRead;
};

#endif // HAL_H
