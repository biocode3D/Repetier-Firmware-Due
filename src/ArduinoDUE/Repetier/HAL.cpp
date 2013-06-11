#include "Repetier.h"
#include <malloc.h>

//extern "C" void __cxa_pure_virtual() { }
extern "C" char *sbrk(int i);

HAL::HAL()
{
    //ctor
}

HAL::~HAL()
{
    //dtor
}


long HAL::CPUDivU2(unsigned int divisor)
{
    return F_CPU/divisor;
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
//    NVIC_EnableIRQ((IRQn_Type)TIMER1_TIMER_IRQ); 

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
    register char * stack_ptr asm ("sp");

    // avail mem in heap + (bottom of stack addr - end of heap addr)
    return (memstruct.fordblks + (int)stack_ptr -  (int)sbrk(0));
}

void(* resetFunc) (void) = 0; //declare reset function @ address 0

void HAL::resetHardware() {
    resetFunc();
}


/*************************************************************************
 Initialization of the I2C bus interface. Need to be called only once
*************************************************************************/
void HAL::i2cInit(unsigned long clockSpeedHz)
{
	PIO_Configure(g_APinDescription[SDA_PIN].pPort,
                  g_APinDescription[SDA_PIN].ulPinType,
                  g_APinDescription[SDA_PIN].ulPin,
                  g_APinDescription[SDA_PIN].ulPinConfiguration);
	PIO_Configure(g_APinDescription[SCL_PIN].pPort,
                  g_APinDescription[SCL_PIN].ulPinType,
                  g_APinDescription[SCL_PIN].ulPin,
                  g_APinDescription[SCL_PIN].ulPinConfiguration);
	pmc_enable_periph_clk(TWI_ID);

    uint divisor = (uint)((clockSpeedHz == 400000) ? 1 : 4);
    
    TWI_INTERFACE->TWI_CWGR = (divisor << TWI_CWGR_CKDIV_Pos) | (TWI_CLOCK << TWI_CWGR_CHDIV_Pos) | (TWI_CLOCK << TWI_CWGR_CLDIV_Pos);
}

/*************************************************************************
  Issues a start condition and sends address and transfer direction.
  return 0 = device accessible, 1= failed to access device
*************************************************************************/
unsigned char HAL::i2cStart(unsigned char address_and_direction)
{
    twiMultipleRead = false;
    twiDirection = (address_and_direction & I2C_READ) << 12;

    // address = address_and_direction >> 1;
    // currentTWIaddress = address << TWI_MMR_DADR_Pos;
    // assumes TWI_MMR_DADR_Pos remains >0
    currentTWIaddress = address_and_direction << (TWI_MMR_DADR_Pos - 1);

    // set to master mode
    TWI_INTERFACE->TWI_CR = TWI_CR_MSEN | TWI_CR_SVDIS;

    // set master mode register with no internal address
    TWI_INTERFACE->TWI_MMR = twiDirection | TWI_MMR_IADRSZ_NONE | currentTWIaddress ;
    
    // returning readiness to send/recieve not device accessibility
    // return value not used in code anyway
    return !(TWI_INTERFACE->TWI_SR & TWI_SR_TXCOMP);
}


/*************************************************************************
 Issues a start condition and sends address and transfer direction.
 If device is busy, use ack polling to wait until device is ready

 Input:   address and transfer direction of I2C device
*************************************************************************/
void HAL::i2cStartWait(unsigned char address_and_direction)
{
    twiMultipleRead = false;
    twiDirection = (address_and_direction & I2C_READ) << 12;

    // address = address_and_direction >> 1;
    // currentTWIaddress = address << TWI_MMR_DADR_Pos;
    currentTWIaddress = address_and_direction << (TWI_MMR_DADR_Pos - 1);

    while(1) {
        if (TWI_INTERFACE->TWI_SR & TWI_SR_TXCOMP) break;
    }
    TWI_INTERFACE->TWI_CR = TWI_CR_MSEN | TWI_CR_SVDIS;

    // set master mode register with no internal address
    TWI_INTERFACE->TWI_MMR = twiDirection | TWI_MMR_IADRSZ_NONE | currentTWIaddress ;
}


/*************************************************************************
 Terminates the data transfer and releases the I2C bus
*************************************************************************/
void HAL::i2cStop(void)
{
    int     i;
    // NOP if end of read, send Stop if end of write
    if(twiDirection == 0) {
        TWI_INTERFACE->TWI_CR = TWI_CR_STOP;

        while(!(TWI_INTERFACE->TWI_SR & TWI_SR_TXCOMP));
    }
}


/*************************************************************************
  Send one byte to I2C device

  Input:    byte to be transfered
  Return:   0 write successful
            1 write failed
*************************************************************************/
unsigned char HAL::i2cWrite( unsigned char data )
{    
    TWI_INTERFACE->TWI_THR = data;         

    while(!(TWI_INTERFACE->TWI_SR & TWI_SR_TXRDY));

    return ((TWI_INTERFACE->TWI_SR & TWI_SR_NACK) == TWI_SR_NACK);
}


/*************************************************************************
 Read one byte from the I2C device, request more data from device
 Return:  byte read from I2C device
*************************************************************************/
unsigned char HAL::i2cReadAck(void)
{
    twiMultipleRead = true;

    TWI_INTERFACE->TWI_CR = TWI_CR_START;
    while(!(TWI_INTERFACE->TWI_SR & TWI_SR_RXRDY));

    unsigned char rcvd = TWI_INTERFACE->TWI_RHR;
    while(!(TWI_INTERFACE->TWI_SR & TWI_SR_TXCOMP));

    return rcvd;
}

/*************************************************************************
 Read one byte from the I2C device, read is followed by a stop condition

 Return:  byte read from I2C device
*************************************************************************/
unsigned char HAL::i2cReadNak(void)
{
    if(twiMultipleRead) {
        twiMultipleRead = false;
        TWI_INTERFACE->TWI_CR = TWI_CR_STOP;
    }else {
        TWI_INTERFACE->TWI_CR = TWI_CR_START | TWI_CR_STOP;
    }
    while(!(TWI_INTERFACE->TWI_SR & TWI_SR_RXRDY));

    unsigned char rcvd = TWI_INTERFACE->TWI_RHR;
    while(!(TWI_INTERFACE->TWI_SR & TWI_SR_TXCOMP));

    return rcvd;
}


#if FEATURE_SERVO
// may need further restrictions here in the future
#if defined (__SAM3X8E__)
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
  // apparently have to read status register
  int dummy = SERVO_COMPA_STATUS;

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
    // apparently have to read status register
    int dummy = TIMER1_COMPA_STATUS;

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
    // apparently have to read status register
    int dummy = PWM_TIMER_STATUS;

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
    // apparently have to read status register
    int dummy = EXTRUDER_TIMER_STATUS;

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


