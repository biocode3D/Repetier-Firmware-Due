#include "Repetier.h"
#include <../Wire/Wire.h>

#include <malloc.h>

//extern "C" void __cxa_pure_virtual() { }

HAL::HAL()
{
    //ctor
}

HAL::~HAL()
{
    //dtor
}


/** \brief Optimized division

Normally the C compiler will compute a long/long division, which takes ~670 Ticks.
This version is optimized for a 16 bit dividend and recognises the special cases
of a 24 bit and 16 bit dividend, which offen, but not always occur in updating the
interval.
*/
inline long Div4U2U(unsigned long a,unsigned int b)
{
    return a/b;
}

const uint16_t fast_div_lut[17] PROGMEM = {0,F_CPU/4096,F_CPU/8192,F_CPU/12288,F_CPU/16384,F_CPU/20480,F_CPU/24576,F_CPU/28672,F_CPU/32768,F_CPU/36864
        ,F_CPU/40960,F_CPU/45056,F_CPU/49152,F_CPU/53248,F_CPU/57344,F_CPU/61440,F_CPU/65536
                                          };

const uint16_t slow_div_lut[257] PROGMEM = {0,F_CPU/32,F_CPU/64,F_CPU/96,F_CPU/128,F_CPU/160,F_CPU/192,F_CPU/224,F_CPU/256,F_CPU/288,F_CPU/320,F_CPU/352
        ,F_CPU/384,F_CPU/416,F_CPU/448,F_CPU/480,F_CPU/512,F_CPU/544,F_CPU/576,F_CPU/608,F_CPU/640,F_CPU/672,F_CPU/704,F_CPU/736,F_CPU/768,F_CPU/800,F_CPU/832
        ,F_CPU/864,F_CPU/896,F_CPU/928,F_CPU/960,F_CPU/992,F_CPU/1024,F_CPU/1056,F_CPU/1088,F_CPU/1120,F_CPU/1152,F_CPU/1184,F_CPU/1216,F_CPU/1248,F_CPU/1280,F_CPU/1312
        ,F_CPU/1344,F_CPU/1376,F_CPU/1408,F_CPU/1440,F_CPU/1472,F_CPU/1504,F_CPU/1536,F_CPU/1568,F_CPU/1600,F_CPU/1632,F_CPU/1664,F_CPU/1696,F_CPU/1728,F_CPU/1760,F_CPU/1792
        ,F_CPU/1824,F_CPU/1856,F_CPU/1888,F_CPU/1920,F_CPU/1952,F_CPU/1984,F_CPU/2016
        ,F_CPU/2048,F_CPU/2080,F_CPU/2112,F_CPU/2144,F_CPU/2176,F_CPU/2208,F_CPU/2240,F_CPU/2272,F_CPU/2304,F_CPU/2336,F_CPU/2368,F_CPU/2400
        ,F_CPU/2432,F_CPU/2464,F_CPU/2496,F_CPU/2528,F_CPU/2560,F_CPU/2592,F_CPU/2624,F_CPU/2656,F_CPU/2688,F_CPU/2720,F_CPU/2752,F_CPU/2784,F_CPU/2816,F_CPU/2848,F_CPU/2880
        ,F_CPU/2912,F_CPU/2944,F_CPU/2976,F_CPU/3008,F_CPU/3040,F_CPU/3072,F_CPU/3104,F_CPU/3136,F_CPU/3168,F_CPU/3200,F_CPU/3232,F_CPU/3264,F_CPU/3296,F_CPU/3328,F_CPU/3360
        ,F_CPU/3392,F_CPU/3424,F_CPU/3456,F_CPU/3488,F_CPU/3520,F_CPU/3552,F_CPU/3584,F_CPU/3616,F_CPU/3648,F_CPU/3680,F_CPU/3712,F_CPU/3744,F_CPU/3776,F_CPU/3808,F_CPU/3840
        ,F_CPU/3872,F_CPU/3904,F_CPU/3936,F_CPU/3968,F_CPU/4000,F_CPU/4032,F_CPU/4064
        ,F_CPU/4096,F_CPU/4128,F_CPU/4160,F_CPU/4192,F_CPU/4224,F_CPU/4256,F_CPU/4288,F_CPU/4320,F_CPU/4352,F_CPU/4384,F_CPU/4416,F_CPU/4448,F_CPU/4480,F_CPU/4512,F_CPU/4544
        ,F_CPU/4576,F_CPU/4608,F_CPU/4640,F_CPU/4672,F_CPU/4704,F_CPU/4736,F_CPU/4768,F_CPU/4800,F_CPU/4832,F_CPU/4864,F_CPU/4896,F_CPU/4928,F_CPU/4960,F_CPU/4992,F_CPU/5024
        ,F_CPU/5056,F_CPU/5088,F_CPU/5120,F_CPU/5152,F_CPU/5184,F_CPU/5216,F_CPU/5248,F_CPU/5280,F_CPU/5312,F_CPU/5344,F_CPU/5376,F_CPU/5408,F_CPU/5440,F_CPU/5472,F_CPU/5504
        ,F_CPU/5536,F_CPU/5568,F_CPU/5600,F_CPU/5632,F_CPU/5664,F_CPU/5696,F_CPU/5728,F_CPU/5760,F_CPU/5792,F_CPU/5824,F_CPU/5856,F_CPU/5888,F_CPU/5920,F_CPU/5952,F_CPU/5984
        ,F_CPU/6016,F_CPU/6048,F_CPU/6080,F_CPU/6112,F_CPU/6144,F_CPU/6176,F_CPU/6208,F_CPU/6240,F_CPU/6272,F_CPU/6304,F_CPU/6336,F_CPU/6368,F_CPU/6400,F_CPU/6432,F_CPU/6464
        ,F_CPU/6496,F_CPU/6528,F_CPU/6560,F_CPU/6592,F_CPU/6624,F_CPU/6656,F_CPU/6688,F_CPU/6720,F_CPU/6752,F_CPU/6784,F_CPU/6816,F_CPU/6848,F_CPU/6880,F_CPU/6912,F_CPU/6944
        ,F_CPU/6976,F_CPU/7008,F_CPU/7040,F_CPU/7072,F_CPU/7104,F_CPU/7136,F_CPU/7168,F_CPU/7200,F_CPU/7232,F_CPU/7264,F_CPU/7296,F_CPU/7328,F_CPU/7360,F_CPU/7392,F_CPU/7424
        ,F_CPU/7456,F_CPU/7488,F_CPU/7520,F_CPU/7552,F_CPU/7584,F_CPU/7616,F_CPU/7648,F_CPU/7680,F_CPU/7712,F_CPU/7744,F_CPU/7776,F_CPU/7808,F_CPU/7840,F_CPU/7872,F_CPU/7904
        ,F_CPU/7936,F_CPU/7968,F_CPU/8000,F_CPU/8032,F_CPU/8064,F_CPU/8096,F_CPU/8128,F_CPU/8160,F_CPU/8192
                                           };
/** \brief approximates division of F_CPU/divisor

In the stepper interrupt a division is needed, which is a slow operation.
The result is used for timer calculation where small errors are ok. This
function uses lookup tables to find a fast approximation of the result.

*/
long HAL::CPUDivU2(unsigned int divisor)
{
#if CPU_ARCH==ARCH_AVR
    long res;
    unsigned short table;
    if(divisor<8192)
    {
        if(divisor<512)
        {
            if(divisor<10) divisor = 10;
            return Div4U2U(F_CPU,divisor); // These entries have overflows in lookuptable!
        }
        table = (unsigned short)&slow_div_lut[0];
        __asm__ __volatile__( // needs 64 ticks neu 49 Ticks
            "mov r18,%A1 \n\t"
            "andi r18,31 \n\t"  // divisor & 31 in r18
            "lsr %B1 \n\t" // divisor >> 4
            "ror %A1 \n\t"
            "lsr %B1 \n\t"
            "ror %A1 \n\t"
            "lsr %B1 \n\t"
            "ror %A1 \n\t"
            "lsr %B1 \n\t"
            "ror %A1 \n\t"
            "andi %A1,254 \n\t"
            "add %A2,%A1 \n\t" // table+divisor>>3
            "adc %B2,%B1 \n\t"
            "lpm %A0,Z+ \n\t" // y0 in res
            "lpm %B0,Z+ \n\t"  // %C0,%D0 are 0
            "movw r4,%A0 \n\t" // y0 nach gain (r4-r5)
            "lpm r0,Z+ \n\t" // gain = gain-y1
            "sub r4,r0 \n\t"
            "lpm r0,Z+ \n\t"
            "sbc r5,r0 \n\t"
            "mul r18,r4 \n\t" // gain*(divisor & 31)
            "movw %A1,r0 \n\t" // divisor not needed any more, use for byte 0,1 of result
            "mul r18,r5 \n\t"
            "add %B1,r0 \n\t"
            "mov %A2,r1 \n\t"
            "lsl %A1 \n\t"
            "rol %B1 \n\t"
            "rol %A2 \n\t"
            "lsl %A1 \n\t"
            "rol %B1 \n\t"
            "rol %A2 \n\t"
            "lsl %A1 \n\t"
            "rol %B1 \n\t"
            "rol %A2 \n\t"
            "sub %A0,%B1 \n\t"
            "sbc %B0,%A2 \n\t"
            "clr %C0 \n\t"
            "clr %D0 \n\t"
            "clr r1 \n\t"
            : "=&r" (res),"=&d"(divisor),"=&z"(table) : "1"(divisor),"2"(table) : "r18","r4","r5");
        return res;
        /*unsigned short adr0 = (unsigned short)&slow_div_lut+(divisor>>4)&1022;
        long y0=	pgm_read_dword_near(adr0);
        long gain = y0-pgm_read_dword_near(adr0+2);
        return y0-((gain*(divisor & 31))>>5);*/
    }
    else
    {
        table = (unsigned short)&fast_div_lut[0];
        __asm__ __volatile__( // needs 49 ticks
            "movw r18,%A1 \n\t"
            "andi r19,15 \n\t"  // divisor & 4095 in r18,r19
            "lsr %B1 \n\t" // divisor >> 3, then %B1 is 2*(divisor >> 12)
            "lsr %B1 \n\t"
            "lsr %B1 \n\t"
            "andi %B1,254 \n\t"
            "add %A2,%B1 \n\t" // table+divisor>>11
            "adc %B2,r1 \n\t" //
            "lpm %A0,Z+ \n\t" // y0 in res
            "lpm %B0,Z+ \n\t"
            "movw r4,%A0 \n\t" // y0 to gain (r4-r5)
            "lpm r0,Z+ \n\t" // gain = gain-y1
            "sub r4,r0 \n\t"
            "lpm r0,Z+ \n\t"
            "sbc r5,r0 \n\t" // finished - result has max. 16 bit
            "mul r18,r4 \n\t" // gain*(divisor & 4095)
            "movw %A1,r0 \n\t" // divisor not needed any more, use for byte 0,1 of result
            "mul r19,r5 \n\t"
            "mov %A2,r0 \n\t" // %A2 = byte 3 of result
            "mul r18,r5 \n\t"
            "add %B1,r0 \n\t"
            "adc %A2,r1 \n\t"
            "mul r19,r4 \n\t"
            "add %B1,r0 \n\t"
            "adc %A2,r1 \n\t"
            "andi %B1,240 \n\t" // >> 12
            "swap %B1 \n\t"
            "swap %A2 \r\n"
            "mov %A1,%A2 \r\n"
            "andi %A1,240 \r\n"
            "or %B1,%A1 \r\n"
            "andi %A2,15 \r\n"
            "sub %A0,%B1 \n\t"
            "sbc %B0,%A2 \n\t"
            "clr %C0 \n\t"
            "clr %D0 \n\t"
            "clr r1 \n\t"
            : "=&r" (res),"=&d"(divisor),"=&z"(table) : "1"(divisor),"2"(table) : "r18","r19","r4","r5");
        return res;
        /*
        // The asm mimics the following code
        unsigned short adr0 = (unsigned short)&fast_div_lut+(divisor>>11)&254;
        unsigned short y0=	pgm_read_word_near(adr0);
        unsigned short gain = y0-pgm_read_word_near(adr0+2);
        return y0-(((long)gain*(divisor & 4095))>>12);*/
    }
#else
    return F_CPU/divisor;
#endif
}

void HAL::setupTimer() {
    uint32_t     tc_count, tc_clock;

    pmc_set_writeprotect(false);

#if defined(USE_ADVANCE)
    pmc_enable_periph_clk(EXTRUDER_TIMER_IRQ);  // enable power to timer

    // count up to value in RC register using given clock
    TC_Configure(EXTRUDER_TIMER, EXTRUDER_TIMER_CHANNEL, TC_CMR_WAVSEL_UP_RC | TC_CMR_WAVE | TC_CMR_TCCLKS_TIMER_CLOCK4);

    TC_SetRC(EXTRUDER_TIMER, EXTRUDER_TIMER_CHANNEL, (F_CPU / 128) / EXTRUDER_CLOCK_FREQ); // set frequency
    TC_Start(EXTRUDER_TIMER, EXTRUDER_TIMER_CHANNEL);           // start timer running
    
    // enable RC compare interrupt
    EXTRUDER_TIMER->TC_CHANNEL[EXTRUDER_TIMER_CHANNEL].TC_IER = TC_IER_CPCS;
    // clear the "disable RC compare" interrupt
    EXTRUDER_TIMER->TC_CHANNEL[EXTRUDER_TIMER_CHANNEL].TC_IDR = ~TC_IER_CPCS;

    // allow interrupts on timer
    NVIC_EnableIRQ((IRQn_Type)EXTRUDER_TIMER_IRQ);
#endif
    pmc_enable_periph_clk(PWM_TIMER_IRQ);
   
    TC_FindMckDivisor(PWM_CLOCK_FREQ, F_CPU, &tc_count, &tc_clock, F_CPU);  
    TC_Configure(PWM_TIMER, PWM_TIMER_CHANNEL, TC_CMR_WAVSEL_UP_RC | TC_CMR_WAVE | tc_clock);

    TC_SetRC(PWM_TIMER, PWM_TIMER_CHANNEL, tc_count);
    TC_Start(PWM_TIMER, PWM_TIMER_CHANNEL);
 
    PWM_TIMER->TC_CHANNEL[PWM_TIMER_CHANNEL].TC_IER = TC_IER_CPCS;
    PWM_TIMER->TC_CHANNEL[PWM_TIMER_CHANNEL].TC_IDR = ~TC_IER_CPCS;
    NVIC_EnableIRQ((IRQn_Type)PWM_TIMER_IRQ);

    //
    pmc_enable_periph_clk(TIMER1_TIMER_IRQ );
      
    TC_Configure(TIMER1_TIMER, TIMER1_TIMER_CHANNEL, TC_CMR_WAVSEL_UP_RC | 
                 TC_CMR_WAVE | TC_CMR_TCCLKS_TIMER_CLOCK4);

    TC_SetRC(TIMER1_TIMER, TIMER1_TIMER_CHANNEL, (F_CPU / 128) / TIMER1_CLOCK_FREQ);
    TC_Start(TIMER1_TIMER, TIMER1_TIMER_CHANNEL);

    TIMER1_TIMER->TC_CHANNEL[TIMER1_TIMER_CHANNEL].TC_IER = TC_IER_CPCS;
    TIMER1_TIMER->TC_CHANNEL[TIMER1_TIMER_CHANNEL].TC_IDR = ~TC_IER_CPCS;
    NVIC_EnableIRQ((IRQn_Type)TIMER1_TIMER_IRQ);

#if FEATURE_SERVO
#if SERVO0_PIN>-1
    SET_OUTPUT(SERVO0_PIN);
    WRITE(SERVO0_PIN,LOW);
#endif
#if SERVO1_PIN>-1
    SET_OUTPUT(SERVO1_PIN);
    WRITE(SERVO1_PIN,LOW);
#endif
#if SERVO2_PIN>-1
    SET_OUTPUT(SERVO2_PIN);
    WRITE(SERVO2_PIN,LOW);
#endif
#if SERVO3_PIN>-1
    SET_OUTPUT(SERVO3_PIN);
    WRITE(SERVO3_PIN,LOW);
#endif
    pmc_enable_periph_clk(TIMER1_TIMER_IRQ );
      
    TC_FindMckDivisor(EXTRUDER_CLOCK_FREQ, F_CPU, &tc_count, &tc_clock, F_CPU);
    TC_Configure(SERVO_TIMER, SERVO_TIMER_CHANNEL, TC_CMR_WAVSEL_UP_RC | 
                 TC_CMR_WAVE | tc_clock);

    TC_SetRC(SERVO_TIMER, SERVO_TIMER_CHANNEL, (F_CPU / 128) / tc_count);

    SERVO_TIMER->TC_CHANNEL[SERVO_TIMER_CHANNEL].TC_IER = TC_IER_CPCS;
    SERvo_TIMER->TC_CHANNEL[SERVO_TIMER_CHANNEL].TC_IDR = ~TC_IER_CPCS;
    NVIC_EnableIRQ((IRQn_Type)SERVO_TIMER_IRQ);
#endif
}

void HAL::showStartReason() {
    int mcu = (RSTC->RSTC_SR & RSTC_SR_RSTTYP_Msk) >> RSTC_SR_RSTTYP_Pos;
    switch (mcu){
    case 0:
        Com::printInfoFLN(Com::tPowerUp);
        break;
    case 1:
        // this is return from backup mode on SAM
        Com::printInfoFLN(Com::tBrownOut);
    case 2:
        Com::printInfoFLN(Com::tWatchdog);
        break;
    case 3:
        Com::printInfoFLN(Com::tSoftwareReset);
        break;
    case 4:
        Com::printInfoFLN(Com::tExternalReset);
    } 
}
int HAL::getFreeRam() {
    struct mallinfo memstruct = mallinfo();
    return memstruct.fordblks;
}

void(* resetFunc) (void) = 0; //declare reset function @ address 0

void HAL::resetHardware() {
    resetFunc();
}

/*************************************************************************
* Title:    I2C master library using hardware TWI interface
* Author:   Peter Fleury <pfleury@gmx.ch>  http://jump.to/fleury
* File:     $Id: twimaster.c,v 1.3 2005/07/02 11:14:21 Peter Exp $
* Software: AVR-GCC 3.4.3 / avr-libc 1.2.3
* Target:   any AVR device with hardware TWI
* Usage:    API compatible with I2C Software Library i2cmaster.h
**************************************************************************/
#if (__GNUC__ * 100 + __GNUC_MINOR__) < 304
#error "This library requires AVR-GCC 3.4 or later, update to newer AVR-GCC compiler !"
#endif

/*************************************************************************
 Initialization of the I2C bus interface. Need to be called only once
*************************************************************************/
void HAL::i2cInit(unsigned long clockSpeedHz)
{
    Wire.begin();
}


/*************************************************************************
  Issues a start condition and sends address and transfer direction.
  return 0 = device accessible, 1= failed to access device
*************************************************************************/
unsigned char HAL::i2cStart(unsigned char address)
{
 /*   uint8_t   twst;

    // send START condition
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

    // wait until transmission completed
    while(!(TWCR & (1<<TWINT)));

    // check value of TWI Status Register. Mask prescaler bits.
    twst = TW_STATUS & 0xF8;
    if ( (twst != TW_START) && (twst != TW_REP_START)) return 1;

    // send device address
    TWDR = address;
    TWCR = (1<<TWINT) | (1<<TWEN);

    // wail until transmission completed and ACK/NACK has been received
    while(!(TWCR & (1<<TWINT)));

    // check value of TWI Status Register. Mask prescaler bits.
    twst = TW_STATUS & 0xF8;
    if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;

    return 0;

*/


    Wire.beginTransmission(address);
}


/*************************************************************************
 Issues a start condition and sends address and transfer direction.
 If device is busy, use ack polling to wait until device is ready

 Input:   address and transfer direction of I2C device
*************************************************************************/
void HAL::i2cStartWait(unsigned char address)
{
/*    uint8_t   twst;
    while ( 1 )
    {
        // send START condition
        TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

        // wait until transmission completed
        while(!(TWCR & (1<<TWINT)));

        // check value of TWI Status Register. Mask prescaler bits.
        twst = TW_STATUS & 0xF8;
        if ( (twst != TW_START) && (twst != TW_REP_START)) continue;

        // send device address
        TWDR = address;
        TWCR = (1<<TWINT) | (1<<TWEN);

        // wail until transmission completed
        while(!(TWCR & (1<<TWINT)));

        // check value of TWI Status Register. Mask prescaler bits.
        twst = TW_STATUS & 0xF8;
        if ( (twst == TW_MT_SLA_NACK )||(twst ==TW_MR_DATA_NACK) )
        {
            /* device busy, send stop condition to terminate write operation */
/*            TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);

            // wait until stop condition is executed and bus released
            while(TWCR & (1<<TWSTO));

            continue;
        }
        //if( twst != TW_MT_SLA_ACK) return 1;
        break;
    }
*/
    Wire.beginTransmission(address);
}


/*************************************************************************
 Terminates the data transfer and releases the I2C bus
*************************************************************************/
void HAL::i2cStop(void)
{
    /* send stop condition */
/*    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
    // wait until stop condition is executed and bus released
    while(TWCR & (1<<TWSTO));
*/

    Wire.endTransmission();
}


/*************************************************************************
  Send one byte to I2C device

  Input:    byte to be transfered
  Return:   0 write successful
            1 write failed
*************************************************************************/
unsigned char HAL::i2cWrite( unsigned char data )
{
/*    uint8_t   twst;
    // send data to the previously addressed device
    TWDR = data;
    TWCR = (1<<TWINT) | (1<<TWEN);
    // wait until transmission completed
    while(!(TWCR & (1<<TWINT)));
    // check value of TWI Status Register. Mask prescaler bits
    twst = TW_STATUS & 0xF8;
    if( twst != TW_MT_DATA_ACK) return 1;
    return 0;
*/

    Wire.write(data);
    return 0;
}


/*************************************************************************
 Read one byte from the I2C device, request more data from device
 Return:  byte read from I2C device
*************************************************************************/
unsigned char HAL::i2cReadAck(void)
{
/*    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
    while(!(TWCR & (1<<TWINT)));
    return TWDR;
*/

    return Wire.read();
}

/*************************************************************************
 Read one byte from the I2C device, read is followed by a stop condition

 Return:  byte read from I2C device
*************************************************************************/
unsigned char HAL::i2cReadNak(void)
{
/*    TWCR = (1<<TWINT) | (1<<TWEN);
    while(!(TWCR & (1<<TWINT)));
    return TWDR;
*/
    return Wire.read();
}

#if FEATURE_SERVO
#if defined (__SAM3X8E__) || (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__) || defined(__AVR_ATmega128__) ||defined(__AVR_ATmega1281__)||defined(__AVR_ATmega2561__)
#define SERVO2500US F_CPU/3200
#define SERVO5000US F_CPU/1600
unsigned int HAL::servoTimings[4] = {0,0,0,0};
static byte servoIndex = 0;
void HAL::servoMicroseconds(byte servo,int ms) {
    if(ms<500) ms = 0;
    if(ms>2500) ms = 2500;
    servoTimings[servo] = (unsigned int)(((F_CPU/1000000)*(long)ms)>>3);
}


// Servo timer Interrupt handler
void SERVO_COMPA_VECTOR ()
{
  switch(servoIndex) {
  case 0:
      TCNT3 = 0;
      if(HAL::servoTimings[0]) {
#if SERVO0_PIN>-1
        WRITE(SERVO0_PIN,HIGH);
#endif
        TC_SetRC(SERVO_TIMER, SERVO_TIMER_CHANNEL, HAL::servoTimings[0]);
      } else TC_SetRC(SERVO_TIMER, SERVO_TIMER_CHANNEL, SERVO2500US);
    break;
  case 1:
#if SERVO0_PIN>-1
      WRITE(SERVO0_PIN,LOW);
#endif
      TC_SetRC(SERVO_TIMER, SERVO_TIMER_CHANNEL, SERVO5000US);
    break;
  case 2:
      TCNT3 = 0;
      if(HAL::servoTimings[1]) {
#if SERVO1_PIN>-1
        WRITE(SERVO1_PIN,HIGH);
#endif
        TC_SetRC(SERVO_TIMER, SERVO_TIMER_CHANNEL, HAL::servoTimings[1]);
      } else TC_SetRC(SERVO_TIMER, SERVO_TIMER_CHANNEL, SERVO2500US);
    break;
  case 3:
#if SERVO1_PIN>-1
      WRITE(SERVO1_PIN,LOW);
#endif
      TC_SetRC(SERVO_TIMER, SERVO_TIMER_CHANNEL, SERVO5000US);
    break;
  case 4:
      TCNT3 = 0;
      if(HAL::servoTimings[2]) {
#if SERVO2_PIN>-1
        WRITE(SERVO2_PIN,HIGH);
#endif
        TC_SetRC(SERVO_TIMER, SERVO_TIMER_CHANNEL, HAL::servoTimings[2]);
      } else TC_SetRC(SERVO_TIMER, SERVO_TIMER_CHANNEL, SERVO2500US);
    break;
  case 5:
#if SERVO2_PIN>-1
      WRITE(SERVO2_PIN,LOW);
#endif
      TC_SetRC(SERVO_TIMER, SERVO_TIMER_CHANNEL, SERVO5000US);
    break;
  case 6:
      TCNT3 = 0;
      if(HAL::servoTimings[3]) {
#if SERVO3_PIN>-1
        WRITE(SERVO3_PIN,HIGH);
#endif
        TC_SetRC(SERVO_TIMER, SERVO_TIMER_CHANNEL, HAL::servoTimings[3]);
      } else TC_SetRC(SERVO_TIMER, SERVO_TIMER_CHANNEL, SERVO2500US);
    break;
  case 7:
#if SERVO3_PIN>-1
      WRITE(SERVO3_PIN,LOW);
#endif
      TC_SetRC(SERVO_TIMER, SERVO_TIMER_CHANNEL, SERVO5000US);
    break;
  }
  servoIndex++;
  if(servoIndex>7)
    servoIndex = 0;
}
#else
#error No servo support for your board, please diable FEATURE_SERVO
#endif
#endif

// ================== Interrupt handling ======================

/** \brief Sets the timer 1 compare value to delay ticks.
*/
inline void setTimer(unsigned long delay)
{
    // convert old AVR timer delay value for SAM timers
    uint32_t timer_count =  ((F_CPU / 16000000) * delay) / 128;   

    TC_SetRC(TIMER1_TIMER, TIMER1_TIMER_CHANNEL, timer_count);
    TC_Start(TIMER1_TIMER, TIMER1_TIMER_CHANNEL);
}

volatile byte insideTimer1=0;
extern long bresenham_step();
/** \brief Timer interrupt routine to drive the stepper motors.
*/
void TIMER1_COMPA_VECTOR ()
{
    if(insideTimer1) return;
    insideTimer1 = 1;

    if(PrintLine::hasLines())
    {
        setTimer(PrintLine::bresenhamStep());
    }
    else
    {
        if(waitRelax==0)
        {
#ifdef USE_ADVANCE
            if(Printer::advance_steps_set)
            {
                Printer::extruderStepsNeeded-=Printer::advance_steps_set;
#ifdef ENABLE_QUADRATIC_ADVANCE
                Printer::advance_executed = 0;
#endif
                Printer::advance_steps_set = 0;
            }
            if((!Printer::extruderStepsNeeded) && (DISABLE_E)) 
                Extruder::disableCurrentExtruderMotor();
#else
            if(DISABLE_E) extruder_disable();
#endif
        }
        else waitRelax--;
    }
    DEBUG_MEMORY;
    insideTimer1=0;
}

/**
This timer is called 3906 times per second. It is used to update
pwm values for heater and some other frequent jobs. 
*/
void PWM_TIMER_VECTOR ()
{
    static byte pwm_count = 0;
    static byte pwm_pos_set[NUM_EXTRUDER+3];
    static byte pwm_cooler_pos_set[NUM_EXTRUDER];

    if(pwm_count==0)
    {
#if EXT0_HEATER_PIN>-1
        if((pwm_pos_set[0] = pwm_pos[0])>0) WRITE(EXT0_HEATER_PIN,1);
#if EXT0_EXTRUDER_COOLER_PIN>-1
        if((pwm_cooler_pos_set[0] = extruder[0].coolerPWM)>0) WRITE(EXT0_EXTRUDER_COOLER_PIN,1);
#endif
#endif
#if defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN>-1 && NUM_EXTRUDER>1
        if((pwm_pos_set[1] = pwm_pos[1])>0) WRITE(EXT1_HEATER_PIN,1);
#if EXT1_EXTRUDER_COOLER_PIN>-1
        if((pwm_cooler_pos_set[1] = extruder[1].coolerPWM)>0) WRITE(EXT1_EXTRUDER_COOLER_PIN,1);
#endif
#endif
#if defined(EXT2_HEATER_PIN) && EXT2_HEATER_PIN>-1 && NUM_EXTRUDER>2
        if((pwm_pos_set[2] = pwm_pos[2])>0) WRITE(EXT2_HEATER_PIN,1);
#if EXT2_EXTRUDER_COOLER_PIN>-1
        if((pwm_cooler_pos_set[2] = extruder[2].coolerPWM)>0) WRITE(EXT2_EXTRUDER_COOLER_PIN,1);
#endif
#endif
#if defined(EXT3_HEATER_PIN) && EXT3_HEATER_PIN>-1 && NUM_EXTRUDER>3
        if((pwm_pos_set[3] = pwm_pos[3])>0) WRITE(EXT3_HEATER_PIN,1);
#if EXT3_EXTRUDER_COOLER_PIN>-1
        if((pwm_cooler_pos_set[3] = extruder[3].coolerPWM)>0) WRITE(EXT3_EXTRUDER_COOLER_PIN,1);
#endif
#endif
#if defined(EXT4_HEATER_PIN) && EXT4_HEATER_PIN>-1 && NUM_EXTRUDER>4
        if((pwm_pos_set[4] = pwm_pos[4])>0) WRITE(EXT4_HEATER_PIN,1);
#if EXT4_EXTRUDER_COOLER_PIN>-1
        if((pwm_cooler_pos_set[4] = pwm_pos[4].coolerPWM)>0) WRITE(EXT4_EXTRUDER_COOLER_PIN,1);
#endif
#endif
#if defined(EXT5_HEATER_PIN) && EXT5_HEATER_PIN>-1 && NUM_EXTRUDER>5
        if((pwm_pos_set[5] = pwm_pos[5])>0) WRITE(EXT5_HEATER_PIN,1);
#if EXT5_EXTRUDER_COOLER_PIN>-1
        if((pwm_cooler_pos_set[5] = extruder[5].coolerPWM)>0) WRITE(EXT5_EXTRUDER_COOLER_PIN,1);
#endif
#endif
#if FAN_BOARD_PIN>-1
        if((pwm_pos_set[NUM_EXTRUDER+1] = pwm_pos[NUM_EXTRUDER+1])>0) WRITE(FAN_BOARD_PIN,1);
#endif
#if FAN_PIN>-1 && FEATURE_FAN_CONTROL
        if((pwm_pos_set[NUM_EXTRUDER+2] = pwm_pos[NUM_EXTRUDER+2])>0) WRITE(FAN_PIN,1);
#endif
#if HEATED_BED_HEATER_PIN>-1 && HAVE_HEATED_BED
        if((pwm_pos_set[NUM_EXTRUDER] = pwm_pos[NUM_EXTRUDER])>0) WRITE(HEATED_BED_HEATER_PIN,1);
#endif
    }
#if EXT0_HEATER_PIN>-1
    if(pwm_pos_set[0] == pwm_count && pwm_pos_set[0]!=255) WRITE(EXT0_HEATER_PIN,0);
#if EXT0_EXTRUDER_COOLER_PIN>-1
    if(pwm_cooler_pos_set[0] == pwm_count && pwm_cooler_pos_set[0]!=255) WRITE(EXT0_EXTRUDER_COOLER_PIN,0);
#endif
#endif
#if defined(EXT1_HEATER_PIN) && EXT1_HEATER_PIN>-1 && NUM_EXTRUDER>1
    if(pwm_pos_set[1] == pwm_count && pwm_pos_set[1]!=255) WRITE(EXT1_HEATER_PIN,0);
#if EXT1_EXTRUDER_COOLER_PIN>-1
    if(pwm_cooler_pos_set[1] == pwm_count && pwm_cooler_pos_set[1]!=255) WRITE(EXT1_EXTRUDER_COOLER_PIN,0);
#endif
#endif
#if defined(EXT2_HEATER_PIN) && EXT2_HEATER_PIN>-1 && NUM_EXTRUDER>2
    if(pwm_pos_set[2] == pwm_count && pwm_pos_set[2]!=255) WRITE(EXT2_HEATER_PIN,0);
#if EXT2_EXTRUDER_COOLER_PIN>-1
    if(pwm_cooler_pos_set[2] == pwm_count && pwm_cooler_pos_set[2]!=255) WRITE(EXT2_EXTRUDER_COOLER_PIN,0);
#endif
#endif
#if defined(EXT3_HEATER_PIN) && EXT3_HEATER_PIN>-1 && NUM_EXTRUDER>3
    if(pwm_pos_set[3] == pwm_count && pwm_pos_set[3]!=255) WRITE(EXT3_HEATER_PIN,0);
#if EXT3_EXTRUDER_COOLER_PIN>-1
    if(pwm_cooler_pos_set[3] == pwm_count && pwm_cooler_pos_set[3]!=255) WRITE(EXT3_EXTRUDER_COOLER_PIN,0);
#endif
#endif
#if defined(EXT4_HEATER_PIN) && EXT4_HEATER_PIN>-1 && NUM_EXTRUDER>4
    if(pwm_pos_set[4] == pwm_count && pwm_pos_set[4]!=255) WRITE(EXT4_HEATER_PIN,0);
#if EXT4_EXTRUDER_COOLER_PIN>-1
    if(pwm_cooler_pos_set[4] == pwm_count && pwm_cooler_pos_set[4]!=255) WRITE(EXT4_EXTRUDER_COOLER_PIN,0);
#endif
#endif
#if defined(EXT5_HEATER_PIN) && EXT5_HEATER_PIN>-1 && NUM_EXTRUDER>5
    if(pwm_pos_set[5] == pwm_count && pwm_pos_set[5]!=255) WRITE(EXT5_HEATER_PIN,0);
#if EXT5_EXTRUDER_COOLER_PIN>-1
    if(pwm_cooler_pos_set[5] == pwm_count && pwm_cooler_pos_set[5]!=255) WRITE(EXT5_EXTRUDER_COOLER_PIN,0);
#endif
#endif
#if FAN_BOARD_PIN>-1
    if(pwm_pos_set[NUM_EXTRUDER+2] == pwm_count && pwm_pos_set[NUM_EXTRUDER+2]!=255) WRITE(FAN_BOARD_PIN,0);
#endif
#if FAN_PIN>-1 && FEATURE_FAN_CONTROL
    if(pwm_pos_set[NUM_EXTRUDER+2] == pwm_count && pwm_pos_set[NUM_EXTRUDER+2]!=255) WRITE(FAN_PIN,0);
#endif
#if HEATED_BED_HEATER_PIN>-1 && HAVE_HEATED_BED
    if(pwm_pos_set[NUM_EXTRUDER] == pwm_count && pwm_pos_set[NUM_EXTRUDER]!=255) WRITE(HEATED_BED_HEATER_PIN,0);
#endif
    HAL::allowInterrupts();
    counter_periodical++; // Appxoimate a 100ms timer
    if(counter_periodical>=(int)(F_CPU/40960))
    {
        counter_periodical=0;
        execute_periodical=1;
    }
// read analog values -- only read one per interrupt
#if ANALOG_INPUTS>0
        
    // conversion finished?
    if(ADC->ADC_ISR & ADC_ISR_EOC(adcChannel[osAnalogInputPos])) 
    {                
        osAnalogInputValues[osAnalogInputPos] = ADC->ADC_CDR[adcChannel[osAnalogInputPos]];    

        // Start next conversion cycle
        if(++osAnalogInputPos>=ANALOG_INPUTS) { 
            osAnalogInputPos = 0;
            ADC->ADC_CR = ADC_CR_START;
        }
    }
#endif

    UI_FAST; // Short timed user interface action
    pwm_count++;
}
#if defined(USE_ADVANCE)
byte extruder_wait_dirchange=0; ///< Wait cycles, if direction changes. Prevents stepper from loosing steps.
char extruder_last_dir = 0;
byte extruder_speed = 0;
#endif

/** \brief Timer routine for extruder stepper.

Several methods need to move the extruder. To get a optima result,
all methods update the printer_state.extruderStepsNeeded with the
number of additional steps needed. During this interrupt, one step
is executed. This will keep the extruder moving, until the total
wanted movement is achieved. This will be done with the maximum
allowable speed for the extruder.
*/
#if defined(USE_ADVANCE)
// EXTRUDER_TIMER IRQ handler
void EXTRUDER_TIMER_VECTOR ()
{
    if(!Printer::isAdvanceActivated()) return; // currently no need

    uint32_t timer = EXTRUDER_TIMER->TC_CHANNEL[EXTRUDER_TIMER_CHANNEL].TC_RC;
    // have to convert old AVR delay values for Due timers
    timer +=  ((F_CPU / 16000000) * Printer::maxExtruderSpeed) / 128; 

    bool increasing = Printer::extruderStepsNeeded>0;

    // Require at least 2 steps in one direction before going to action
    if(abs(Printer::extruderStepsNeeded)<2)
    {
        TC_SetRC(EXTRUDER_TIMER, EXTRUDER_TIMER_CHANNEL, timer);
        ANALYZER_OFF(ANALYZER_CH2);
        extruder_last_dir = 0;
        return;
    }

    if(extruder_last_dir==0)
    {
        Extruder::setDirection(increasing ? 1 : 0);
        extruder_last_dir = (increasing ? 1 : -1);
    }
    Extruder::step();
    Printer::extruderStepsNeeded-=extruder_last_dir;
#if STEPPER_HIGH_DELAY>0
    HAL::delayMicroseconds(STEPPER_HIGH_DELAY);
#endif
    Extruder::unstep();

    TC_SetRC(EXTRUDER_TIMER, EXTRUDER_TIMER_CHANNEL, timer);
}
#endif

#ifndef EXTERNALSERIAL
// Implement serial communication for one stream only!
/*
  HardwareSerial.h - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 28 September 2010 by Mark Sproul

  Modified to use only 1 queue with fixed length by Repetier
*/

ring_buffer rx_buffer = { { 0 }, 0, 0};
ring_buffer tx_buffer = { { 0 }, 0, 0};

inline void rf_store_char(unsigned char c, ring_buffer *buffer)
{
  int i = (unsigned int)(buffer->head + 1) & SERIAL_BUFFER_MASK;

  // if we should be storing the received character into the location
  // just before the tail (meaning that the head would advance to the
  // current location of the tail), we're about to overflow the buffer
  // and so we don't write the character or advance the head.
  if (i != buffer->tail) {
    buffer->buffer[buffer->head] = c;
    buffer->head = i;
  }
}
#if !defined(USART0_RX_vect) && defined(USART1_RX_vect)
// do nothing - on the 32u4 the first USART is USART1
#else
  void rfSerialEvent() __attribute__((weak));
  void rfSerialEvent() {}
  #define serialEvent_implemented
#if defined(USART_RX_vect)
  SIGNAL(USART_RX_vect)
#elif defined(USART0_RX_vect)
  SIGNAL(USART0_RX_vect)
#else
#if defined(SIG_USART0_RECV)
  SIGNAL(SIG_USART0_RECV)
#elif defined(SIG_UART0_RECV)
  SIGNAL(SIG_UART0_RECV)
#elif defined(SIG_UART_RECV)
  SIGNAL(SIG_UART_RECV)
#else
  #error "Don't know what the Data Received vector is called for the first UART"
#endif
#endif
  {
  #if defined(UDR0)
    unsigned char c  =  UDR0;
  #elif defined(UDR)
    unsigned char c  =  UDR;
  #else
    #error UDR not defined
  #endif
    rf_store_char(c, &rx_buffer);
  }
#endif

#if !defined(USART0_UDRE_vect) && defined(USART1_UDRE_vect)
// do nothing - on the 32u4 the first USART is USART1
#else
#if !defined(UART0_UDRE_vect) && !defined(UART_UDRE_vect) && !defined(USART0_UDRE_vect) && !defined(USART_UDRE_vect)
  #error "Don't know what the Data Register Empty vector is called for the first UART"
#else
#if defined(UART0_UDRE_vect)
ISR(UART0_UDRE_vect)
#elif defined(UART_UDRE_vect)
ISR(UART_UDRE_vect)
#elif defined(USART0_UDRE_vect)
ISR(USART0_UDRE_vect)
#elif defined(USART_UDRE_vect)
ISR(USART_UDRE_vect)
#endif
{
  if (tx_buffer.head == tx_buffer.tail) {
	// Buffer empty, so disable interrupts
#if defined(UCSR0B)
    bit_clear(UCSR0B, UDRIE0);
#else
    bit_clear(UCSRB, UDRIE);
#endif
  }
  else {
    // There is more data in the output buffer. Send the next byte
    unsigned char c = tx_buffer.buffer[tx_buffer.tail];
    tx_buffer.tail = (tx_buffer.tail + 1) & SERIAL_BUFFER_MASK;

  #if defined(UDR0)
    UDR0 = c;
  #elif defined(UDR)
    UDR = c;
  #else
    #error UDR not defined
  #endif
  }
}
#endif
#endif


// Constructors ////////////////////////////////////////////////////////////////

RFHardwareSerial::RFHardwareSerial(ring_buffer *rx_buffer, ring_buffer *tx_buffer,
  volatile uint8_t *ubrrh, volatile uint8_t *ubrrl,
  volatile uint8_t *ucsra, volatile uint8_t *ucsrb,
  volatile uint8_t *udr,
  uint8_t rxen, uint8_t txen, uint8_t rxcie, uint8_t udrie, uint8_t u2x)
{
  _rx_buffer = rx_buffer;
  _tx_buffer = tx_buffer;
  _ubrrh = ubrrh;
  _ubrrl = ubrrl;
  _ucsra = ucsra;
  _ucsrb = ucsrb;
  _udr = udr;
  _rxen = rxen;
  _txen = txen;
  _rxcie = rxcie;
  _udrie = udrie;
  _u2x = u2x;
}

// Public Methods //////////////////////////////////////////////////////////////

void RFHardwareSerial::begin(unsigned long baud)
{
  uint16_t baud_setting;
  bool use_u2x = true;

#if F_CPU == 16000000UL
  // hardcoded exception for compatibility with the bootloader shipped
  // with the Duemilanove and previous boards and the firmware on the 8U2
  // on the Uno and Mega 2560.
  if (baud == 57600) {
    use_u2x = false;
  }
#endif

try_again:

  if (use_u2x) {
    *_ucsra = 1 << _u2x;
    baud_setting = (F_CPU / 4 / baud - 1) / 2;
  } else {
    *_ucsra = 0;
    baud_setting = (F_CPU / 8 / baud - 1) / 2;
  }

  if ((baud_setting > 4095) && use_u2x)
  {
    use_u2x = false;
    goto try_again;
  }

  // assign the baud_setting, a.k.a. ubbr (USART Baud Rate Register)
  *_ubrrh = baud_setting >> 8;
  *_ubrrl = baud_setting;

  bit_set(*_ucsrb, _rxen);
  bit_set(*_ucsrb, _txen);
  bit_set(*_ucsrb, _rxcie);
  bit_clear(*_ucsrb, _udrie);
}

void RFHardwareSerial::end()
{
  // wait for transmission of outgoing data
  while (_tx_buffer->head != _tx_buffer->tail)
    ;

  bit_clear(*_ucsrb, _rxen);
  bit_clear(*_ucsrb, _txen);
  bit_clear(*_ucsrb, _rxcie);
  bit_clear(*_ucsrb, _udrie);

  // clear a  ny received data
  _rx_buffer->head = _rx_buffer->tail;
}

int RFHardwareSerial::available(void)
{
  return (unsigned int)(SERIAL_BUFFER_SIZE + _rx_buffer->head - _rx_buffer->tail) & SERIAL_BUFFER_MASK;
}

int RFHardwareSerial::peek(void)
{
  if (_rx_buffer->head == _rx_buffer->tail) {
    return -1;
  } else {
    return _rx_buffer->buffer[_rx_buffer->tail];
  }
}

int RFHardwareSerial::read(void)
{
  // if the head isn't ahead of the tail, we don't have any characters
  if (_rx_buffer->head == _rx_buffer->tail) {
    return -1;
  } else {
    unsigned char c = _rx_buffer->buffer[_rx_buffer->tail];
    _rx_buffer->tail = (unsigned int)(_rx_buffer->tail + 1) & SERIAL_BUFFER_MASK;
    return c;
  }
}

void RFHardwareSerial::flush()
{
  while (_tx_buffer->head != _tx_buffer->tail)
    ;
}
#ifdef COMPAT_PRE1
  void
#else
  size_t
#endif
RFHardwareSerial::write(uint8_t c)
{
  int i = (_tx_buffer->head + 1) & SERIAL_BUFFER_MASK;

  // If the output buffer is full, there's nothing for it other than to
  // wait for the interrupt handler to empty it a bit
  // ???: return 0 here instead?
  while (i == _tx_buffer->tail)
    ;

  _tx_buffer->buffer[_tx_buffer->head] = c;
  _tx_buffer->head = i;

  bit_set(*_ucsrb, _udrie);
#ifndef COMPAT_PRE1
  return 1;
#endif
}

// Preinstantiate Objects //////////////////////////////////////////////////////

#if defined(UBRRH) && defined(UBRRL)
  RFHardwareSerial RFSerial(&rx_buffer, &tx_buffer, &UBRRH, &UBRRL, &UCSRA, &UCSRB, &UDR, RXEN, TXEN, RXCIE, UDRIE, U2X);
#elif defined(UBRR0H) && defined(UBRR0L)
  RFHardwareSerial RFSerial(&rx_buffer, &tx_buffer, &UBRR0H, &UBRR0L, &UCSR0A, &UCSR0B, &UDR0, RXEN0, TXEN0, RXCIE0, UDRIE0, U2X0);
#elif defined(USBCON)
  // do nothing - Serial object and buffers are initialized in CDC code
#else
  #error no serial port defined  (port 0)
#endif

#endif

#if ANALOG_INPUTS>0
void HAL::analogStart(void)
{
  uint32_t  adcEnable = 0;

  for(int i=0; i<ANALOG_INPUTS; i++)
  {
      osAnalogInputCounter[i] = 0;
      osAnalogInputValues[i] = 0;

      adcEnable &= (0x1u << adcChannel[i]);
  }
  // enable channels
  ADC->ADC_CHER = adcEnable;
  ADC->ADC_CHDR = !adcEnable;

  // Initialize ADC mode register (some of the following params are not used here)
  // HW trigger disabled, use external Trigger, 10 bit resolution
  // core and ref voltage stays on, normal sleep mode, normal not free-run mode
  // startup time 16 clocks, settling time 17 clocks, no changes on channel switch
  // convert channels in numeric order
  // set prescaler rate  MCK/((PRESCALE+1) * 2)
  // set tracking time  (TRACKTIM+1) * clock periods
  // set transfer period  (TRANSFER * 2 + 3) 
  ADC->ADC_MR = ADC_MR_TRGEN_DIS | ADC_MR_TRGSEL_ADC_TRIG0 | ADC_MR_LOWRES_BITS_10 |
            ADC_MR_SLEEP_NORMAL | ADC_MR_FWUP_OFF | ADC_MR_FREERUN_OFF |
            ADC_MR_STARTUP_SUT16 | ADC_MR_SETTLING_AST17 | ADC_MR_ANACH_NONE |
            ADC_MR_USEQ_NUM_ORDER |
            ADC_MR_PRESCAL(AD_PRESCALE_FACTOR) |
            ADC_MR_TRACKTIM(AD_TRACKING_CYCLES) |
            ADC_MR_TRANSFER(AD_TRANSFER_CYCLES);

  ADC->ADC_IER = 0;              // no ADC interrupts

  // start first conversion
  ADC->ADC_CR = ADC_CR_START;
}

#if ANALOG_INPUTS>0
uint8_t osAnalogInputCounter[ANALOG_INPUTS];
uint8_t osAnalogInputPos=0; // Current sampling position
volatile uint16_t osAnalogInputValues[ANALOG_INPUTS];
#endif

#endif


