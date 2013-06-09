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

#include "Repetier.h"


void EEPROM::update(GCode *com)
{
#if EEPROM_MODE!=0
    if(com->hasT() && com->hasP()) switch(com->T)
        {
        case 0:
            if(com->hasS()) HAL::epr_set_byte(com->P,(byte)com->S);
            break;
        case 1:
            if(com->hasS()) HAL::epr_set_int(com->P,(int)com->S);
            break;
        case 2:
            if(com->hasS()) HAL::epr_set_long(com->P,(long)com->S);
            break;
        case 3:
            if(com->hasX()) HAL::epr_set_float(com->P,com->X);
            break;
        }
    byte newcheck = computeChecksum();
    if(newcheck!=HAL::epr_get_byte(EPR_INTEGRITY_BYTE))
        HAL::epr_set_byte(EPR_INTEGRITY_BYTE,newcheck);
    readDataFromEEPROM();
#else
    Com::printErrorF(Com::tNoEEPROMSupport);
#endif
}

void EEPROM::restoreEEPROMSettingsFromConfiguration()
{
#if EEPROM_MODE!=0
    byte version = EEPROM_PROTOCOL_VERSION;
    baudrate = BAUDRATE;
    max_inactive_time = MAX_INACTIVE_TIME*1000L;
    stepper_inactive_time = STEPPER_INACTIVE_TIME*1000L;
    Printer::axisStepsPerMM[0] = XAXIS_STEPS_PER_MM;
    Printer::axisStepsPerMM[1] = YAXIS_STEPS_PER_MM;
    Printer::axisStepsPerMM[2] = ZAXIS_STEPS_PER_MM;
    Printer::axisStepsPerMM[3] = 1;
    Printer::maxFeedrate[0] = MAX_FEEDRATE_X;
    Printer::maxFeedrate[1] = MAX_FEEDRATE_Y;
    Printer::maxFeedrate[2] = MAX_FEEDRATE_Z;
    Printer::homingFeedrate[0] = HOMING_FEEDRATE_X;
    Printer::homingFeedrate[1] = HOMING_FEEDRATE_Y;
    Printer::homingFeedrate[2] = HOMING_FEEDRATE_Z;
    Printer::maxJerk = MAX_JERK;
    Printer::maxZJerk = MAX_ZJERK;
#ifdef RAMP_ACCELERATION
    Printer::maxAccelerationMMPerSquareSecond[0] = maxAccelerationMMPerSquareSecond_X;
    Printer::maxAccelerationMMPerSquareSecond[1] = maxAccelerationMMPerSquareSecond_Y;
    Printer::maxAccelerationMMPerSquareSecond[2] = maxAccelerationMMPerSquareSecond_Z;
    Printer::maxTravelAccelerationMMPerSquareSecond[0] = maxTravelAccelerationMMPerSquareSecond_X;
    Printer::maxTravelAccelerationMMPerSquareSecond[1] = maxTravelAccelerationMMPerSquareSecond_Y;
    Printer::maxTravelAccelerationMMPerSquareSecond[2] = maxTravelAccelerationMMPerSquareSecond_Z;
#endif
#if USE_OPS==1
    Printer::opsMode = OPS_MODE;
    Printer::opsMinDistance = OPS_MIN_DISTANCE;
    Printer::opsRetractDistance = OPS_RETRACT_DISTANCE;
    Printer::opsRetractBacklash = OPS_RETRACT_BACKLASH;
    Printer::opsMoveAfter = OPS_MOVE_AFTER;
#endif
#if HAVE_HEATED_BED
    heatedBedController.heatManager= HEATED_BED_HEAT_MANAGER;
#ifdef TEMP_PID
    heatedBedController.pidDriveMax = HEATED_BED_PID_INTEGRAL_DRIVE_MAX;
    heatedBedController.pidDriveMin = HEATED_BED_PID_INTEGRAL_DRIVE_MIN;
    heatedBedController.pidPGain = HEATED_BED_PID_PGAIN;
    heatedBedController.pidIGain = HEATED_BED_PID_IGAIN;
    heatedBedController.pidDGain = HEATED_BED_PID_DGAIN;
    heatedBedController.pidMax = HEATED_BED_PID_MAX;
#endif
#endif
    Printer::xLength = X_MAX_LENGTH;
    Printer::yLength = Y_MAX_LENGTH;
    Printer::zLength = Z_MAX_LENGTH;
    Printer::xMin = X_MIN_POS;
    Printer::yMin = Y_MIN_POS;
    Printer::zMin = Z_MIN_POS;
#if ENABLE_BACKLASH_COMPENSATION
    Printer::backlashX = X_BACKLASH;
    Printer::backlashY = Y_BACKLASH;
    Printer::backlashZ = Z_BACKLASH;
#endif
    Extruder *e;
#if NUM_EXTRUDER>0
    e = &extruder[0];
    e->stepsPerMM = EXT0_STEPS_PER_MM;
    e->maxFeedrate = EXT0_MAX_FEEDRATE;
    e->maxStartFeedrate = EXT0_MAX_START_FEEDRATE;
    e->maxAcceleration = EXT0_MAX_ACCELERATION;
    e->tempControl.heatManager = EXT0_HEAT_MANAGER;
#ifdef TEMP_PID
    e->tempControl.pidDriveMax = EXT0_PID_INTEGRAL_DRIVE_MAX;
    e->tempControl.pidDriveMin = EXT0_PID_INTEGRAL_DRIVE_MIN;
    e->tempControl.pidPGain = EXT0_PID_P;
    e->tempControl.pidIGain = EXT0_PID_I;
    e->tempControl.pidDGain = EXT0_PID_D;
    e->tempControl.pidMax = EXT0_PID_MAX;
#endif
    e->yOffset = EXT0_Y_OFFSET;
    e->xOffset = EXT0_X_OFFSET;
    e->watchPeriod = EXT0_WATCHPERIOD;
#if RETRACT_DURING_HEATUP
    e->waitRetractTemperature = EXT0_WAIT_RETRACT_TEMP;
    e->waitRetractUnits = EXT0_WAIT_RETRACT_UNITS;
#endif
    e->coolerSpeed = EXT0_EXTRUDER_COOLER_SPEED;
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    e->advanceK = EXT0_ADVANCE_K;
#endif
    e->advanceL = EXT0_ADVANCE_L;
#endif
#endif // NUM_EXTRUDER>0
#if NUM_EXTRUDER>1
    e = &extruder[1];
    e->stepsPerMM = EXT1_STEPS_PER_MM;
    e->maxFeedrate = EXT1_MAX_FEEDRATE;
    e->maxStartFeedrate = EXT1_MAX_START_FEEDRATE;
    e->maxAcceleration = EXT1_MAX_ACCELERATION;
    e->tempControl.heatManager = EXT1_HEAT_MANAGER;
#ifdef TEMP_PID
    e->tempControl.pidDriveMax = EXT1_PID_INTEGRAL_DRIVE_MAX;
    e->tempControl.pidDriveMin = EXT1_PID_INTEGRAL_DRIVE_MIN;
    e->tempControl.pidPGain = EXT1_PID_P;
    e->tempControl.pidIGain = EXT1_PID_I;
    e->tempControl.pidDGain = EXT1_PID_D;
    e->tempControl.pidMax = EXT1_PID_MAX;
#endif
    e->yOffset = EXT1_Y_OFFSET;
    e->xOffset = EXT1_X_OFFSET;
    e->watchPeriod = EXT1_WATCHPERIOD;
#if RETRACT_DURING_HEATUP
    e->waitRetractTemperature = EXT1_WAIT_RETRACT_TEMP;
    e->waitRetractUnits = EXT1_WAIT_RETRACT_UNITS;
#endif
    e->coolerSpeed = EXT1_EXTRUDER_COOLER_SPEED;
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    e->advanceK = EXT1_ADVANCE_K;
#endif
    e->advanceL = EXT1_ADVANCE_L;
#endif
#endif // NUM_EXTRUDER > 1
#if NUM_EXTRUDER>2
    e = &extruder[2];
    e->stepsPerMM = EXT2_STEPS_PER_MM;
    e->maxFeedrate = EXT2_MAX_FEEDRATE;
    e->maxStartFeedrate = EXT2_MAX_START_FEEDRATE;
    e->maxAcceleration = EXT2_MAX_ACCELERATION;
    e->tempControl.heatManager = EXT2_HEAT_MANAGER;
#ifdef TEMP_PID
    e->tempControl.pidDriveMax = EXT2_PID_INTEGRAL_DRIVE_MAX;
    e->tempControl.pidDriveMin = EXT2_PID_INTEGRAL_DRIVE_MIN;
    e->tempControl.pidPGain = EXT2_PID_P;
    e->tempControl.pidIGain = EXT2_PID_I;
    e->tempControl.pidDGain = EXT2_PID_D;
    e->tempControl.pidMax = EXT2_PID_MAX;
#endif
    e->yOffset = EXT2_Y_OFFSET;
    e->xOffset = EXT2_X_OFFSET;
    e->watchPeriod = EXT2_WATCHPERIOD;
#if RETRACT_DURING_HEATUP
    e->waitRetractTemperature = EXT2_WAIT_RETRACT_TEMP;
    e->waitRetractUnits = EXT2_WAIT_RETRACT_UNITS;
#endif
    e->coolerSpeed = EXT2_EXTRUDER_COOLER_SPEED;
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    e->advanceK = EXT2_ADVANCE_K;
#endif
    e->advanceL = EXT2_ADVANCE_L;
#endif
#endif // NUM_EXTRUDER > 2
#if NUM_EXTRUDER>3
    e = &extruder[3];
    e->stepsPerMM = EXT3_STEPS_PER_MM;
    e->maxFeedrate = EXT3_MAX_FEEDRATE;
    e->maxStartFeedrate = EXT3_MAX_START_FEEDRATE;
    e->maxAcceleration = EXT3_MAX_ACCELERATION;
    e->tempControl.heatManager = EXT3_HEAT_MANAGER;
#ifdef TEMP_PID
    e->tempControl.pidDriveMax = EXT3_PID_INTEGRAL_DRIVE_MAX;
    e->tempControl.pidDriveMin = EXT3_PID_INTEGRAL_DRIVE_MIN;
    e->tempControl.pidPGain = EXT3_PID_P;
    e->tempControl.pidIGain = EXT3_PID_I;
    e->tempControl.pidDGain = EXT3_PID_D;
    e->tempControl.pidMax = EXT3_PID_MAX;
#endif
    e->yOffset = EXT3_Y_OFFSET;
    e->xOffset = EXT3_X_OFFSET;
    e->watchPeriod = EXT3_WATCHPERIOD;
#if RETRACT_DURING_HEATUP
    e->waitRetractTemperature = EXT3_WAIT_RETRACT_TEMP;
    e->waitRetractUnits = EXT3_WAIT_RETRACT_UNITS;
#endif
    e->coolerSpeed = EXT3_EXTRUDER_COOLER_SPEED;
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    e->advanceK = EXT3_ADVANCE_K;
#endif
    e->advanceL = EXT3_ADVANCE_L;
#endif
#endif // NUM_EXTRUDER > 3
#if NUM_EXTRUDER>4
    e = &extruder[4];
    e->stepsPerMM = EXT4_STEPS_PER_MM;
    e->maxFeedrate = EXT4_MAX_FEEDRATE;
    e->maxStartFeedrate = EXT4_MAX_START_FEEDRATE;
    e->maxAcceleration = EXT4_MAX_ACCELERATION;
    e->tempControl.heatManager = EXT4_HEAT_MANAGER;
#ifdef TEMP_PID
    e->tempControl.pidDriveMax = EXT4_PID_INTEGRAL_DRIVE_MAX;
    e->tempControl.pidDriveMin = EXT4_PID_INTEGRAL_DRIVE_MIN;
    e->tempControl.pidPGain = EXT4_PID_P;
    e->tempControl.pidIGain = EXT4_PID_I;
    e->tempControl.pidDGain = EXT4_PID_D;
    e->tempControl.pidMax = EXT4_PID_MAX;
#endif
    e->yOffset = EXT4_Y_OFFSET;
    e->xOffset = EXT4_X_OFFSET;
    e->watchPeriod = EXT4_WATCHPERIOD;
#if RETRACT_DURING_HEATUP
    e->waitRetractTemperature = EXT4_WAIT_RETRACT_TEMP;
    e->waitRetractUnits = EXT4_WAIT_RETRACT_UNITS;
#endif
    e->coolerSpeed = EXT4_EXTRUDER_COOLER_SPEED;
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    e->advanceK = EXT4_ADVANCE_K;
#endif
    e->advanceL = EXT4_ADVANCE_L;
#endif
#endif // NUM_EXTRUDER > 4
#if NUM_EXTRUDER>5
    e = &extruder[5];
    e->stepsPerMM = EXT5_STEPS_PER_MM;
    e->maxFeedrate = EXT5_MAX_FEEDRATE;
    e->maxStartFeedrate = EXT5_MAX_START_FEEDRATE;
    e->maxAcceleration = EXT5_MAX_ACCELERATION;
    e->tempControl.heatManager = EXT5_HEAT_MANAGER;
#ifdef TEMP_PID
    e->tempControl.pidDriveMax = EXT5_PID_INTEGRAL_DRIVE_MAX;
    e->tempControl.pidDriveMin = EXT5_PID_INTEGRAL_DRIVE_MIN;
    e->tempControl.pidPGain = EXT5_PID_P;
    e->tempControl.pidIGain = EXT5_PID_I;
    e->tempControl.pidDGain = EXT5_PID_D;
    e->tempControl.pidMax = EXT5_PID_MAX;
#endif
    e->yOffset = EXT5_Y_OFFSET;
    e->xOffset = EXT5_X_OFFSET;
    e->watchPeriod = EXT5_WATCHPERIOD;
#if RETRACT_DURING_HEATUP
    e->waitRetractTemperature = EXT5_WAIT_RETRACT_TEMP;
    e->waitRetractUnits = EXT5_WAIT_RETRACT_UNITS;
#endif
    e->coolerSpeed = EXT5_EXTRUDER_COOLER_SPEED;
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    e->advanceK = EXT5_ADVANCE_K;
#endif
    e->advanceL = EXT5_ADVANCE_L;
#endif
#endif // NUM_EXTRUDER > 5
#if FEATURE_AUTOLEVEL
    Printer::setAutolevelActive(false);
    Printer::resetTransformationMatrix(true);
#endif
    initalizeUncached();
    Extruder::selectExtruderById(current_extruder->id);
    Printer::updateDerivedParameter();
    Extruder::initHeatedBed();
    Com::printInfoF(Com::tEPRConfigResetDefaults);
#else
    Com::printErrorF(Com::tNoEEPROMSupport);
#endif

}

void EEPROM::storeDataIntoEEPROM(byte corrupted)
{
#if EEPROM_MODE!=0
    HAL::epr_set_long(EPR_BAUDRATE,baudrate);
    HAL::epr_set_long(EPR_MAX_INACTIVE_TIME,max_inactive_time);
    HAL::epr_set_long(EPR_STEPPER_INACTIVE_TIME,stepper_inactive_time);
//#define EPR_ACCELERATION_TYPE 1
    HAL::epr_set_float(EPR_XAXIS_STEPS_PER_MM,Printer::axisStepsPerMM[0]);
    HAL::epr_set_float(EPR_YAXIS_STEPS_PER_MM,Printer::axisStepsPerMM[1]);
    HAL::epr_set_float(EPR_ZAXIS_STEPS_PER_MM,Printer::axisStepsPerMM[2]);
    HAL::epr_set_float(EPR_X_MAX_FEEDRATE,Printer::maxFeedrate[0]);
    HAL::epr_set_float(EPR_Y_MAX_FEEDRATE,Printer::maxFeedrate[1]);
    HAL::epr_set_float(EPR_Z_MAX_FEEDRATE,Printer::maxFeedrate[2]);
    HAL::epr_set_float(EPR_X_HOMING_FEEDRATE,Printer::homingFeedrate[0]);
    HAL::epr_set_float(EPR_Y_HOMING_FEEDRATE,Printer::homingFeedrate[1]);
    HAL::epr_set_float(EPR_Z_HOMING_FEEDRATE,Printer::homingFeedrate[2]);
    HAL::epr_set_float(EPR_MAX_JERK,Printer::maxJerk);
    HAL::epr_set_float(EPR_MAX_ZJERK,Printer::maxZJerk);
#ifdef RAMP_ACCELERATION
    HAL::epr_set_float(EPR_X_MAX_ACCEL,Printer::maxAccelerationMMPerSquareSecond[0]);
    HAL::epr_set_float(EPR_Y_MAX_ACCEL,Printer::maxAccelerationMMPerSquareSecond[1]);
    HAL::epr_set_float(EPR_Z_MAX_ACCEL,Printer::maxAccelerationMMPerSquareSecond[2]);
    HAL::epr_set_float(EPR_X_MAX_TRAVEL_ACCEL,Printer::maxTravelAccelerationMMPerSquareSecond[0]);
    HAL::epr_set_float(EPR_Y_MAX_TRAVEL_ACCEL,Printer::maxTravelAccelerationMMPerSquareSecond[1]);
    HAL::epr_set_float(EPR_Z_MAX_TRAVEL_ACCEL,Printer::maxTravelAccelerationMMPerSquareSecond[2]);
#endif
#if USE_OPS==1
    HAL::epr_set_float(EPR_OPS_MIN_DISTANCE,Printer::opsMinDistance);
    HAL::epr_set_byte(EPR_OPS_MODE,Printer::opsMode);
    HAL::epr_set_float(EPR_OPS_MOVE_AFTER,Printer::opsMoveAfter);
    HAL::epr_set_float(EPR_OPS_RETRACT_DISTANCE,Printer::opsRetractDistance);
    HAL::epr_set_float(EPR_OPS_RETRACT_BACKLASH,Printer::opsRetractBacklash);
#else
    HAL::epr_set_float(EPR_OPS_MIN_DISTANCE,OPS_MIN_DISTANCE);
    HAL::epr_set_byte(EPR_OPS_MODE,OPS_MODE);
    HAL::epr_set_float(EPR_OPS_MOVE_AFTER,OPS_MOVE_AFTER);
    HAL::epr_set_float(EPR_OPS_RETRACT_DISTANCE,OPS_RETRACT_DISTANCE);
    HAL::epr_set_float(EPR_OPS_RETRACT_BACKLASH,OPS_RETRACT_BACKLASH);
#endif
#if HAVE_HEATED_BED
    HAL::epr_set_byte(EPR_BED_HEAT_MANAGER,heatedBedController.heatManager);
#else
    HAL::epr_set_byte(EPR_BED_HEAT_MANAGER,HEATED_BED_HEAT_MANAGER);
#endif
#if defined(TEMP_PID) && HAVE_HEATED_BED
    HAL::epr_set_byte(EPR_BED_DRIVE_MAX,heatedBedController.pidDriveMax);
    HAL::epr_set_byte(EPR_BED_DRIVE_MIN,heatedBedController.pidDriveMin);
    HAL::epr_set_float(EPR_BED_PID_PGAIN,heatedBedController.pidPGain);
    HAL::epr_set_float(EPR_BED_PID_IGAIN,heatedBedController.pidIGain);
    HAL::epr_set_float(EPR_BED_PID_DGAIN,heatedBedController.pidDGain);
    HAL::epr_set_byte(EPR_BED_PID_MAX,heatedBedController.pidMax);
#else
    HAL::epr_set_byte(EPR_BED_DRIVE_MAX,HEATED_BED_PID_INTEGRAL_DRIVE_MAX);
    HAL::epr_set_byte(EPR_BED_DRIVE_MIN,HEATED_BED_PID_INTEGRAL_DRIVE_MIN);
    HAL::epr_set_float(EPR_BED_PID_PGAIN,HEATED_BED_PID_PGAIN);
    HAL::epr_set_float(EPR_BED_PID_IGAIN,HEATED_BED_PID_IGAIN);
    HAL::epr_set_float(EPR_BED_PID_DGAIN,HEATED_BED_PID_DGAIN);
    HAL::epr_set_byte(EPR_BED_PID_MAX,HEATED_BED_PID_MAX);
#endif
    HAL::epr_set_float(EPR_X_HOME_OFFSET,Printer::xMin);
    HAL::epr_set_float(EPR_Y_HOME_OFFSET,Printer::yMin);
    HAL::epr_set_float(EPR_Z_HOME_OFFSET,Printer::zMin);
    HAL::epr_set_float(EPR_X_LENGTH,Printer::xLength);
    HAL::epr_set_float(EPR_Y_LENGTH,Printer::yLength);
    HAL::epr_set_float(EPR_Z_LENGTH,Printer::zLength);
#if ENABLE_BACKLASH_COMPENSATION
    HAL::epr_set_float(EPR_BACKLASH_X,Printer::backlashX);
    HAL::epr_set_float(EPR_BACKLASH_Y,Printer::backlashY);
    HAL::epr_set_float(EPR_BACKLASH_Z,Printer::backlashZ);
#else
    HAL::epr_set_float(EPR_BACKLASH_X,0);
    HAL::epr_set_float(EPR_BACKLASH_Y,0);
    HAL::epr_set_float(EPR_BACKLASH_Z,0);
#endif
#if FEATURE_AUTOLEVEL
    HAL::epr_set_byte(EPR_AUTOLEVEL_ACTIVE,Printer::isAutolevelActive());
    for(byte i=0; i<9; i++)
        HAL::epr_set_float(EPR_AUTOLEVEL_MATRIX+((int)i)<<2,Printer::autolevelTransformation[i]);
#endif
    // now the extruder
    for(byte i=0; i<NUM_EXTRUDER; i++)
    {
        int o=i*EEPROM_EXTRUDER_LENGTH+EEPROM_EXTRUDER_OFFSET;
        Extruder *e = &extruder[i];
        HAL::epr_set_float(o+EPR_EXTRUDER_STEPS_PER_MM,e->stepsPerMM);
        HAL::epr_set_float(o+EPR_EXTRUDER_MAX_FEEDRATE,e->maxFeedrate);
        HAL::epr_set_float(o+EPR_EXTRUDER_MAX_START_FEEDRATE,e->maxStartFeedrate);
        HAL::epr_set_float(o+EPR_EXTRUDER_MAX_ACCELERATION,e->maxAcceleration);
        HAL::epr_set_byte(o+EPR_EXTRUDER_HEAT_MANAGER,e->tempControl.heatManager);
#ifdef TEMP_PID
        HAL::epr_set_byte(o+EPR_EXTRUDER_DRIVE_MAX,e->tempControl.pidDriveMax);
        HAL::epr_set_byte(o+EPR_EXTRUDER_DRIVE_MIN,e->tempControl.pidDriveMin);
        HAL::epr_set_float(o+EPR_EXTRUDER_PID_PGAIN,e->tempControl.pidPGain);
        HAL::epr_set_float(o+EPR_EXTRUDER_PID_IGAIN,e->tempControl.pidIGain);
        HAL::epr_set_float(o+EPR_EXTRUDER_PID_DGAIN,e->tempControl.pidDGain);
        HAL::epr_set_byte(o+EPR_EXTRUDER_PID_MAX,e->tempControl.pidMax);
#endif
        HAL::epr_set_long(o+EPR_EXTRUDER_X_OFFSET,e->xOffset);
        HAL::epr_set_long(o+EPR_EXTRUDER_Y_OFFSET,e->yOffset);
        HAL::epr_set_int(o+EPR_EXTRUDER_WATCH_PERIOD,e->watchPeriod);
#if RETRACT_DURING_HEATUP
        HAL::epr_set_int(o+EPR_EXTRUDER_WAIT_RETRACT_TEMP,e->waitRetractTemperature);
        HAL::epr_set_int(o+EPR_EXTRUDER_WAIT_RETRACT_UNITS,e->waitRetractUnits);
#else
        HAL::epr_set_int(o+EPR_EXTRUDER_WAIT_RETRACT_TEMP,EXT0_WAIT_RETRACT_TEMP);
        HAL::epr_set_int(o+EPR_EXTRUDER_WAIT_RETRACT_UNITS,EXT0_WAIT_RETRACT_UNITS);
#endif
        HAL::epr_set_byte(o+EPR_EXTRUDER_COOLER_SPEED,e->coolerSpeed);
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
        HAL::epr_set_float(o+EPR_EXTRUDER_ADVANCE_K,e->advanceK);
#else
        HAL::epr_set_float(o+EPR_EXTRUDER_ADVANCE_K,0);
#endif
        HAL::epr_set_float(o+EPR_EXTRUDER_ADVANCE_L,e->advanceL);
#else
        HAL::epr_set_float(o+EPR_EXTRUDER_ADVANCE_K,0);
        HAL::epr_set_float(o+EPR_EXTRUDER_ADVANCE_L,0);
#endif
    }
    if(corrupted)
    {
        HAL::epr_set_long(EPR_PRINTING_TIME,0);
        HAL::epr_set_float(EPR_PRINTING_DISTANCE,0);
        initalizeUncached();
    }
    // Save version and build checksum
    HAL::epr_set_byte(EPR_VERSION,EEPROM_PROTOCOL_VERSION);
    HAL::epr_set_byte(EPR_INTEGRITY_BYTE,computeChecksum());
#endif
}
void EEPROM::initalizeUncached()
{
    HAL::epr_set_float(EPR_Z_PROBE_HEIGHT,Z_PROBE_HEIGHT);
    HAL::epr_set_float(EPR_Z_PROBE_SPEED,Z_PROBE_SPEED);
    HAL::epr_set_float(EPR_Z_PROBE_XY_SPEED,Z_PROBE_XY_SPEED);
    HAL::epr_set_float(EPR_Z_PROBE_X_OFFSET,Z_PROBE_X_OFFSET);
    HAL::epr_set_float(EPR_Z_PROBE_Y_OFFSET,Z_PROBE_Y_OFFSET);
    HAL::epr_set_float(EPR_Z_PROBE_X1,Z_PROBE_X1);
    HAL::epr_set_float(EPR_Z_PROBE_Y1,Z_PROBE_Y1);
    HAL::epr_set_float(EPR_Z_PROBE_X2,Z_PROBE_X2);
    HAL::epr_set_float(EPR_Z_PROBE_Y2,Z_PROBE_Y2);
    HAL::epr_set_float(EPR_Z_PROBE_X3,Z_PROBE_X3);
    HAL::epr_set_float(EPR_Z_PROBE_Y3,Z_PROBE_Y3);
}

void EEPROM::readDataFromEEPROM()
{
#if EEPROM_MODE!=0
    byte version = HAL::epr_get_byte(EPR_VERSION); // This is the saved version. Don't copy data not set in older versions!
    baudrate = HAL::epr_get_long(EPR_BAUDRATE);
    max_inactive_time = HAL::epr_get_long(EPR_MAX_INACTIVE_TIME);
    stepper_inactive_time = HAL::epr_get_long(EPR_STEPPER_INACTIVE_TIME);
//#define EPR_ACCELERATION_TYPE 1
    Printer::axisStepsPerMM[0] = HAL::epr_get_float(EPR_XAXIS_STEPS_PER_MM);
    Printer::axisStepsPerMM[1] = HAL::epr_get_float(EPR_YAXIS_STEPS_PER_MM);
    Printer::axisStepsPerMM[2] = HAL::epr_get_float(EPR_ZAXIS_STEPS_PER_MM);
    Printer::maxFeedrate[0] = HAL::epr_get_float(EPR_X_MAX_FEEDRATE);
    Printer::maxFeedrate[1] = HAL::epr_get_float(EPR_Y_MAX_FEEDRATE);
    Printer::maxFeedrate[2] = HAL::epr_get_float(EPR_Z_MAX_FEEDRATE);
    Printer::homingFeedrate[0] = HAL::epr_get_float(EPR_X_HOMING_FEEDRATE);
    Printer::homingFeedrate[1] = HAL::epr_get_float(EPR_Y_HOMING_FEEDRATE);
    Printer::homingFeedrate[2] = HAL::epr_get_float(EPR_Z_HOMING_FEEDRATE);
    Printer::maxJerk = HAL::epr_get_float(EPR_MAX_JERK);
    Printer::maxZJerk = HAL::epr_get_float(EPR_MAX_ZJERK);
#ifdef RAMP_ACCELERATION
    Printer::maxAccelerationMMPerSquareSecond[0] = HAL::epr_get_float(EPR_X_MAX_ACCEL);
    Printer::maxAccelerationMMPerSquareSecond[1] = HAL::epr_get_float(EPR_Y_MAX_ACCEL);
    Printer::maxAccelerationMMPerSquareSecond[2] = HAL::epr_get_float(EPR_Z_MAX_ACCEL);
    Printer::maxTravelAccelerationMMPerSquareSecond[0] = HAL::epr_get_float(EPR_X_MAX_TRAVEL_ACCEL);
    Printer::maxTravelAccelerationMMPerSquareSecond[1] = HAL::epr_get_float(EPR_Y_MAX_TRAVEL_ACCEL);
    Printer::maxTravelAccelerationMMPerSquareSecond[2] = HAL::epr_get_float(EPR_Z_MAX_TRAVEL_ACCEL);
#endif
#if USE_OPS==1
    Printer::opsMode = HAL::epr_get_byte(EPR_OPS_MODE);
    Printer::opsMoveAfter = HAL::epr_get_float(EPR_OPS_MOVE_AFTER);
    Printer::opsMinDistance = HAL::epr_get_float(EPR_OPS_MIN_DISTANCE);
    Printer::opsRetractDistance = HAL::epr_get_float(EPR_OPS_RETRACT_DISTANCE);
    Printer::opsRetractBacklash = HAL::epr_get_float(EPR_OPS_RETRACT_BACKLASH);
#endif
#if HAVE_HEATED_BED
    heatedBedController.heatManager= HAL::epr_get_byte(EPR_BED_HEAT_MANAGER);
#ifdef TEMP_PID
    heatedBedController.pidDriveMax = HAL::epr_get_byte(EPR_BED_DRIVE_MAX);
    heatedBedController.pidDriveMin = HAL::epr_get_byte(EPR_BED_DRIVE_MIN);
    heatedBedController.pidPGain = HAL::epr_get_float(EPR_BED_PID_PGAIN);
    heatedBedController.pidIGain = HAL::epr_get_float(EPR_BED_PID_IGAIN);
    heatedBedController.pidDGain = HAL::epr_get_float(EPR_BED_PID_DGAIN);
    heatedBedController.pidMax = HAL::epr_get_byte(EPR_BED_PID_MAX);
#endif
#endif
    Printer::xMin = HAL::epr_get_float(EPR_X_HOME_OFFSET);
    Printer::yMin = HAL::epr_get_float(EPR_Y_HOME_OFFSET);
    Printer::zMin = HAL::epr_get_float(EPR_Z_HOME_OFFSET);
    Printer::xLength = HAL::epr_get_float(EPR_X_LENGTH);
    Printer::yLength = HAL::epr_get_float(EPR_Y_LENGTH);
    Printer::zLength = HAL::epr_get_float(EPR_Z_LENGTH);
#if ENABLE_BACKLASH_COMPENSATION
    Printer::backlashX = HAL::epr_get_float(EPR_BACKLASH_X);
    Printer::backlashY = HAL::epr_get_float(EPR_BACKLASH_Y);
    Printer::backlashZ = HAL::epr_get_float(EPR_BACKLASH_Z);
#endif
#if FEATURE_AUTOLEVEL
    if(version>2)
    {
        for(byte i=0; i<9; i++)
            Printer::autolevelTransformation[i] = HAL::epr_get_float(EPR_AUTOLEVEL_MATRIX+((int)i)<<2);
        Printer::setAutolevelActive(HAL::epr_get_byte(EPR_AUTOLEVEL_ACTIVE));
        Com::printArrayFLN(Com::tInfo,Printer::autolevelTransformation,9,6);
    }
#endif
    // now the extruder
    for(byte i=0; i<NUM_EXTRUDER; i++)
    {
        int o=i*EEPROM_EXTRUDER_LENGTH+EEPROM_EXTRUDER_OFFSET;
        Extruder *e = &extruder[i];
        e->stepsPerMM = HAL::epr_get_float(o+EPR_EXTRUDER_STEPS_PER_MM);
        e->maxFeedrate = HAL::epr_get_float(o+EPR_EXTRUDER_MAX_FEEDRATE);
        e->maxStartFeedrate = HAL::epr_get_float(o+EPR_EXTRUDER_MAX_START_FEEDRATE);
        e->maxAcceleration = HAL::epr_get_float(o+EPR_EXTRUDER_MAX_ACCELERATION);
        e->tempControl.heatManager = HAL::epr_get_byte(o+EPR_EXTRUDER_HEAT_MANAGER);
#ifdef TEMP_PID
        e->tempControl.pidDriveMax = HAL::epr_get_byte(o+EPR_EXTRUDER_DRIVE_MAX);
        e->tempControl.pidDriveMin = HAL::epr_get_byte(o+EPR_EXTRUDER_DRIVE_MIN);
        e->tempControl.pidPGain = HAL::epr_get_float(o+EPR_EXTRUDER_PID_PGAIN);
        e->tempControl.pidIGain = HAL::epr_get_float(o+EPR_EXTRUDER_PID_IGAIN);
        e->tempControl.pidDGain = HAL::epr_get_float(o+EPR_EXTRUDER_PID_DGAIN);
        e->tempControl.pidMax = HAL::epr_get_byte(o+EPR_EXTRUDER_PID_MAX);
#endif
        e->xOffset = HAL::epr_get_long(o+EPR_EXTRUDER_X_OFFSET);
        e->yOffset = HAL::epr_get_long(o+EPR_EXTRUDER_Y_OFFSET);
        e->watchPeriod = HAL::epr_get_int(o+EPR_EXTRUDER_WATCH_PERIOD);
#if RETRACT_DURING_HEATUP
        e->waitRetractTemperature = HAL::epr_get_int(o+EPR_EXTRUDER_WAIT_RETRACT_TEMP);
        e->waitRetractUnits = HAL::epr_get_int(o+EPR_EXTRUDER_WAIT_RETRACT_UNITS);
#endif
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
        e->advanceK = HAL::epr_get_float(o+EPR_EXTRUDER_ADVANCE_K);
#endif
        e->advanceL = HAL::epr_get_float(o+EPR_EXTRUDER_ADVANCE_L);
#endif
        if(version>1)
            e->coolerSpeed = HAL::epr_get_byte(o+EPR_EXTRUDER_COOLER_SPEED);
    }
    if(version!=EEPROM_PROTOCOL_VERSION)
    {
        Com::printInfoFLN(Com::tEPRProtocolChanged);
        if(version<3)
        {
            HAL::epr_set_float(EPR_Z_PROBE_HEIGHT,Z_PROBE_HEIGHT);
            HAL::epr_set_float(EPR_Z_PROBE_SPEED,Z_PROBE_SPEED);
            HAL::epr_set_float(EPR_Z_PROBE_XY_SPEED,Z_PROBE_XY_SPEED);
            HAL::epr_set_float(EPR_Z_PROBE_X_OFFSET,Z_PROBE_X_OFFSET);
            HAL::epr_set_float(EPR_Z_PROBE_Y_OFFSET,Z_PROBE_Y_OFFSET);
            HAL::epr_set_float(EPR_Z_PROBE_X1,Z_PROBE_X1);
            HAL::epr_set_float(EPR_Z_PROBE_Y1,Z_PROBE_Y1);
            HAL::epr_set_float(EPR_Z_PROBE_X2,Z_PROBE_X2);
            HAL::epr_set_float(EPR_Z_PROBE_Y2,Z_PROBE_Y2);
            HAL::epr_set_float(EPR_Z_PROBE_X3,Z_PROBE_X3);
            HAL::epr_set_float(EPR_Z_PROBE_Y3,Z_PROBE_Y3);
        }
        storeDataIntoEEPROM(false); // Store new fields for changed version
    }
    Extruder::selectExtruderById(current_extruder->id);
    Printer::updateDerivedParameter();
    Extruder::initHeatedBed();
#endif
}

void EEPROM::initBaudrate()
{
#if EEPROM_MODE!=0
    if(HAL::epr_get_byte(EPR_MAGIC_BYTE)==EEPROM_MODE)
    {
        baudrate = HAL::epr_get_long(EPR_BAUDRATE);
    }
#endif
}

void EEPROM::init()
{
#if EEPROM_MODE!=0
    byte check = computeChecksum();
    byte storedcheck = HAL::epr_get_byte(EPR_INTEGRITY_BYTE);
    if(HAL::epr_get_byte(EPR_MAGIC_BYTE)==EEPROM_MODE && storedcheck==check)
    {
        readDataFromEEPROM();
    }
    else
    {
        HAL::epr_set_byte(EPR_MAGIC_BYTE,EEPROM_MODE); // Make datachange permanent
        storeDataIntoEEPROM(storedcheck!=check);
    }
#endif
}

void EEPROM::updatePrinterUsage()
{
#if EEPROM_MODE!=0
    if(Printer::filamentPrinted==0) return; // No miles only enabled
    unsigned long seconds = (HAL::timeInMilliseconds()-Printer::msecondsPrinting)/1000;
    seconds += HAL::epr_get_long(EPR_PRINTING_TIME);
    HAL::epr_set_long(EPR_PRINTING_TIME,seconds);
    HAL::epr_set_float(EPR_PRINTING_DISTANCE,HAL::epr_get_float(EPR_PRINTING_DISTANCE)+Printer::filamentPrinted*0.001);
    Printer::filamentPrinted = 0;
    Printer::msecondsPrinting = HAL::timeInMilliseconds();
    byte newcheck = computeChecksum();
    if(newcheck!=HAL::epr_get_byte(EPR_INTEGRITY_BYTE))
        HAL::epr_set_byte(EPR_INTEGRITY_BYTE,newcheck);
    Commands::reportPrinterUsage();
#endif
}

/** \brief Writes all eeprom settings to serial console.

For each value stored, this function generates one line with syntax

EPR: pos type value description

With
- pos = Position in EEPROM, the data starts.
- type = Value type: 0 = byte, 1 = int, 2 = long, 3 = float
- value = The value currently stored
- description = Definition of the value
*/
void EEPROM::writeSettings()
{
#if EEPROM_MODE!=0
    writeLong(EPR_BAUDRATE,Com::tEPRBaudrate);
    writeFloat(EPR_PRINTING_DISTANCE,Com::tEPRFilamentPrinted);
    writeLong(EPR_PRINTING_TIME,Com::tEPRPrinterActive);
    writeLong(EPR_MAX_INACTIVE_TIME,Com::tEPRMaxInactiveTime);
    writeLong(EPR_STEPPER_INACTIVE_TIME,Com::tEPRStopAfterInactivty);
//#define EPR_ACCELERATION_TYPE 1
    writeFloat(EPR_XAXIS_STEPS_PER_MM,Com::tEPRXStepsPerMM);
    writeFloat(EPR_YAXIS_STEPS_PER_MM,Com::tEPRYStepsPerMM);
    writeFloat(EPR_ZAXIS_STEPS_PER_MM,Com::tEPRZStepsPerMM);
    writeFloat(EPR_X_MAX_FEEDRATE,Com::tEPRXMaxFeedrate);
    writeFloat(EPR_Y_MAX_FEEDRATE,Com::tEPRYMaxFeedrate);
    writeFloat(EPR_Z_MAX_FEEDRATE,Com::tEPRZMaxFeedrate);
    writeFloat(EPR_X_HOMING_FEEDRATE,Com::tEPRXHomingFeedrate);
    writeFloat(EPR_Y_HOMING_FEEDRATE,Com::tEPRYHomingFeedrate);
    writeFloat(EPR_Z_HOMING_FEEDRATE,Com::tEPRZHomingFeedrate);
    writeFloat(EPR_MAX_JERK,Com::tEPRMaxJerk);
    writeFloat(EPR_MAX_ZJERK,Com::tEPRMaxZJerk);
    writeFloat(EPR_X_HOME_OFFSET,Com::tEPRXHomePos);
    writeFloat(EPR_Y_HOME_OFFSET,Com::tEPRYHomePos);
    writeFloat(EPR_Z_HOME_OFFSET,Com::tEPRZHomePos);
    writeFloat(EPR_X_LENGTH,Com::tEPRXMaxLength);
    writeFloat(EPR_Y_LENGTH,Com::tEPRYMaxLength);
    writeFloat(EPR_Z_LENGTH,Com::tEPRZMaxLength);
#if ENABLE_BACKLASH_COMPENSATION
    writeFloat(EPR_BACKLASH_X,Com::tEPRXBacklash);
    writeFloat(EPR_BACKLASH_Y,Com::tEPRYBacklash);
    writeFloat(EPR_BACKLASH_Z,Com::tEPRZBacklash);
#endif

#ifdef RAMP_ACCELERATION
    //epr_out_float(EPR_X_MAX_START_SPEED,PSTR("X-axis start speed [mm/s]"));
    //epr_out_float(EPR_Y_MAX_START_SPEED,PSTR("Y-axis start speed [mm/s]"));
    //epr_out_float(EPR_Z_MAX_START_SPEED,PSTR("Z-axis start speed [mm/s]"));
    writeFloat(EPR_X_MAX_ACCEL,Com::tEPRXAcceleration);
    writeFloat(EPR_Y_MAX_ACCEL,Com::tEPRYAcceleration);
    writeFloat(EPR_Z_MAX_ACCEL,Com::tEPRZAcceleration);
    writeFloat(EPR_X_MAX_TRAVEL_ACCEL,Com::tEPRXTravelAcceleration);
    writeFloat(EPR_Y_MAX_TRAVEL_ACCEL,Com::tEPRYTravelAcceleration);
    writeFloat(EPR_Z_MAX_TRAVEL_ACCEL,Com::tEPRZTravelAcceleration);
#endif
#if USE_OPS==1
    writeByte(EPR_OPS_MODE,Com::tEPROPSMode);
    writeFloat(EPR_OPS_MOVE_AFTER,Com::tEPROPSMoveAfter);
    writeFloat(EPR_OPS_MIN_DISTANCE,Com::tEPROPSMinDistance);
    writeFloat(EPR_OPS_RETRACT_DISTANCE,Com::tEPROPSRetractionLength);
    writeFloat(EPR_OPS_RETRACT_BACKLASH,Com::tEPROPSRetractionBacklash);
#endif
#if FEATURE_Z_PROBE
    writeFloat(EPR_Z_PROBE_HEIGHT,Com::tZProbeHeight);
    writeFloat(EPR_Z_PROBE_SPEED,Com::tZProbeSpeed);
    writeFloat(EPR_Z_PROBE_XY_SPEED,Com::tZProbeSpeedXY);
    writeFloat(EPR_Z_PROBE_X_OFFSET,Com::tZProbeOffsetX);
    writeFloat(EPR_Z_PROBE_Y_OFFSET,Com::tZProbeOffsetY);
    writeFloat(EPR_Z_PROBE_X1,Com::tZProbeX1);
    writeFloat(EPR_Z_PROBE_Y1,Com::tZProbeY1);
    writeFloat(EPR_Z_PROBE_X2,Com::tZProbeX2);
    writeFloat(EPR_Z_PROBE_Y2,Com::tZProbeY2);
    writeFloat(EPR_Z_PROBE_X3,Com::tZProbeX3);
    writeFloat(EPR_Z_PROBE_Y3,Com::tZProbeY3);
#endif
#if FEATURE_AUTOLEVEL
    writeByte(EPR_AUTOLEVEL_ACTIVE,Com::tAutolevelActive);
#endif
#if HAVE_HEATED_BED
    writeByte(EPR_BED_HEAT_MANAGER,Com::tEPRBedHeatManager);
#ifdef TEMP_PID
    writeByte(EPR_BED_DRIVE_MAX,Com::tEPRBedPIDDriveMax);
    writeByte(EPR_BED_DRIVE_MIN,Com::tEPRBedPIDDriveMin);
    writeFloat(EPR_BED_PID_PGAIN,Com::tEPRBedPGain);
    writeFloat(EPR_BED_PID_IGAIN,Com::tEPRBedIGain);
    writeFloat(EPR_BED_PID_DGAIN,Com::tEPRBedDGain);
    writeByte(EPR_BED_PID_MAX,Com::tEPRBedPISMaxValue);
#endif
#endif
    // now the extruder
    for(byte i=0; i<NUM_EXTRUDER; i++)
    {
        int o=i*EEPROM_EXTRUDER_LENGTH+EEPROM_EXTRUDER_OFFSET;
        Extruder *e = &extruder[i];
        writeFloat(o+EPR_EXTRUDER_STEPS_PER_MM,Com::tEPRStepsPerMM);
        writeFloat(o+EPR_EXTRUDER_MAX_FEEDRATE,Com::tEPRMaxFeedrate);
        writeFloat(o+EPR_EXTRUDER_MAX_START_FEEDRATE,Com::tEPRStartFeedrate);
        writeFloat(o+EPR_EXTRUDER_MAX_ACCELERATION,Com::tEPRAcceleration);
        writeByte(o+EPR_EXTRUDER_HEAT_MANAGER,Com::tEPRHeatManager);
#ifdef TEMP_PID
        writeByte(o+EPR_EXTRUDER_DRIVE_MAX,Com::tEPRDriveMax);
        writeByte(o+EPR_EXTRUDER_DRIVE_MIN,Com::tEPRDriveMin);
        writeFloat(o+EPR_EXTRUDER_PID_PGAIN,Com::tEPRPGain);
        writeFloat(o+EPR_EXTRUDER_PID_IGAIN,Com::tEPRIGain);
        writeFloat(o+EPR_EXTRUDER_PID_DGAIN,Com::tEPRDGain);
        writeByte(o+EPR_EXTRUDER_PID_MAX,Com::tEPRPIDMaxValue);
#endif
        writeLong(o+EPR_EXTRUDER_X_OFFSET,Com::tEPRXOffset);
        writeLong(o+EPR_EXTRUDER_Y_OFFSET,Com::tEPRYOffset);
        writeInt(o+EPR_EXTRUDER_WATCH_PERIOD,Com::tEPRStabilizeTime);
#if RETRACT_DURING_HEATUP
        writeInt(o+EPR_EXTRUDER_WAIT_RETRACT_TEMP,Com::tEPRRetractionWhenHeating);
        writeInt(o+EPR_EXTRUDER_WAIT_RETRACT_UNITS,Com::tEPRDistanceRetractHeating);
#endif
        writeByte(o+EPR_EXTRUDER_COOLER_SPEED,Com::tEPRExtruderCoolerSpeed);
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
        writeFloat(o+EPR_EXTRUDER_ADVANCE_K,Com::tEPRAdvanceK);
#endif
        writeFloat(o+EPR_EXTRUDER_ADVANCE_L,Com::tEPRAdvanceL);
#endif
    }
#else
    Com::printErrorF(Com::tNoEEPROMSupport);
#endif
}

#if EEPROM_MODE!=0

byte EEPROM::computeChecksum()
{
    unsigned int i;
    byte checksum=0;
    for(i=0; i<2048; i++)
    {
        if(i==EEPROM_OFFSET+EPR_INTEGRITY_BYTE) continue;
        checksum += HAL::epr_get_byte(i);
    }
    return checksum;
}

void EEPROM::writeExtruderPrefix(uint pos)
{
    if(pos<EEPROM_EXTRUDER_OFFSET || pos>=800) return;
    int n = (pos-EEPROM_EXTRUDER_OFFSET)/EEPROM_EXTRUDER_LENGTH+1;
    Com::printF(Com::tExtrDot,n);
    Com::print(' ');
}

void EEPROM::writeFloat(uint pos,PGM_P text)
{
    Com::printF(Com::tEPR3,(int)pos);
    Com::print(' ');
    Com::printFloat(HAL::epr_get_float(pos),3);
    Com::print(' ');
    writeExtruderPrefix(pos);
    Com::printFLN(text);
}

void EEPROM::writeLong(uint pos,PGM_P text)
{
    Com::printF(Com::tEPR2,(int)pos);
    Com::print(' ');
    Com::print(HAL::epr_get_long(pos));
    Com::print(' ');
    writeExtruderPrefix(pos);
    Com::printFLN(text);
}

void EEPROM::writeInt(uint pos,PGM_P text)
{
    Com::printF(Com::tEPR1,(int)pos);
    Com::print(' ');
    Com::print(HAL::epr_get_int(pos));
    Com::print(' ');
    writeExtruderPrefix(pos);
    Com::printFLN(text);
}

void EEPROM::writeByte(uint pos,PGM_P text)
{
    Com::printF(Com::tEPR0,(int)pos);
    Com::print(' ');
    Com::print((int)HAL::epr_get_byte(pos));
    Com::print(' ');
    writeExtruderPrefix(pos);
    Com::printFLN(text);
}
#endif

