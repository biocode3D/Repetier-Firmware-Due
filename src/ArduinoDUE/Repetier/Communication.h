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
*/

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

class Com
{
    public:
FSTRINGVAR(tFirmware)
FSTRINGVAR(tOk)
FSTRINGVAR(tNewline)
FSTRINGVAR(tNAN)
FSTRINGVAR(tINF)
FSTRINGVAR(tError)
FSTRINGVAR(tInfo)
FSTRINGVAR(tWarning)
FSTRINGVAR(tResend)
FSTRINGVAR(tEcho)
FSTRINGVAR(tOkSpace)
//FSTRINGVAR(tNoEEPROMSupport)
FSTRINGVAR(tWrongChecksum)
FSTRINGVAR(tMissingChecksum)
FSTRINGVAR(tDonePrinting)
FSTRINGVAR(tX)
FSTRINGVAR(tY)
FSTRINGVAR(tZ)
FSTRINGVAR(tE)
FSTRINGVAR(tF)
FSTRINGVAR(tS)
FSTRINGVAR(tP)
FSTRINGVAR(tI)
FSTRINGVAR(tJ)
FSTRINGVAR(tR)
FSTRINGVAR(tSDReadError)
FSTRINGVAR(tExpectedLine)
FSTRINGVAR(tGot)
FSTRINGVAR(tSkip)
FSTRINGVAR(tBLK)
FSTRINGVAR(tStart)
FSTRINGVAR(tPowerUp)
FSTRINGVAR(tExternalReset)
FSTRINGVAR(tBrownOut)
FSTRINGVAR(tWatchdog)
FSTRINGVAR(tSoftwareReset)
FSTRINGVAR(tUnknownCommand)
FSTRINGVAR(tFreeRAM)
FSTRINGVAR(tXColon)
FSTRINGVAR(tSlash)
FSTRINGVAR(tSpaceXColon)
FSTRINGVAR(tSpaceYColon)
FSTRINGVAR(tSpaceZColon)
FSTRINGVAR(tSpaceEColon)
FSTRINGVAR(tTColon)
FSTRINGVAR(tSpaceBColon)
FSTRINGVAR(tSpaceAtColon)
FSTRINGVAR(tSpaceT)
FSTRINGVAR(tSpaceAt)
FSTRINGVAR(tColon)
FSTRINGVAR(tSpeedMultiply);
FSTRINGVAR(tFlowMultiply);
FSTRINGVAR(tFanspeed);
FSTRINGVAR(tPrintedFilament)
FSTRINGVAR(tPrintingTime)
FSTRINGVAR(tSpacem)
FSTRINGVAR(tSpaceDaysSpace)
FSTRINGVAR(tSpaceHoursSpace)
FSTRINGVAR(tSpaceMin)
FSTRINGVAR(tInvalidArc)
FSTRINGVAR(tComma)
FSTRINGVAR(tSpace)
FSTRINGVAR(tYColon)
FSTRINGVAR(tZColon)
FSTRINGVAR(tE0Colon)
FSTRINGVAR(tE1Colon)
FSTRINGVAR(tMS1MS2Pins)
FSTRINGVAR(tSetOutputSpace)
FSTRINGVAR(tSpaceToSpace)
FSTRINGVAR(tHSpace)
FSTRINGVAR(tLSpace)
FSTRINGVAR(tXMinColon)
FSTRINGVAR(tXMaxColon)
FSTRINGVAR(tYMinColon)
FSTRINGVAR(tYMaxColon)
FSTRINGVAR(tZMinColon)
FSTRINGVAR(tZMaxColon)
FSTRINGVAR(tJerkColon)
FSTRINGVAR(tZJerkColon)
FSTRINGVAR(tLinearStepsColon)
FSTRINGVAR(tQuadraticStepsColon)
FSTRINGVAR(tCommaSpeedEqual)
FSTRINGVAR(tOPSDisabled)
FSTRINGVAR(tOPSClassicMode)
FSTRINGVAR(tOPSFastMode)
FSTRINGVAR(tMinDistance)
FSTRINGVAR(tRetractEqual)
FSTRINGVAR(tBacklashEqual)
FSTRINGVAR(tMoveAfter)
FSTRINGVAR(tRetrSteps)
FSTRINGVAR(tPushBackSteps)
FSTRINGVAR(tMoveAfterSteps)
FSTRINGVAR(tLinearLColon)
FSTRINGVAR(tQuadraticKColon)

#if DRIVE_SYSTEM==3
FSTRINGVAR(tMeasurementReset)
FSTRINGVAR(tMeasureDeltaSteps)
FSTRINGVAR(tMeasureDelta)
FSTRINGVAR(tMeasureOriginReset)
FSTRINGVAR(tEEPROMUpdated)
FSTRINGVAR(tInvalidDeltaCoordinate)
FSTRINGVAR(tLevelingCalc)
FSTRINGVAR(tTower1)
FSTRINGVAR(tTower2)
FSTRINGVAR(tTower3)
FSTRINGVAR(tDBGDeltaNoMoveinDSegment)
#endif // DRIVE_SYSTEM
#ifdef DEBUG_GENERIC
FSTRINGVAR(tGenTemp)
#endif // DEBUG_GENERICFSTRINGVALUE(Com::,"")
FSTRINGVAR(tTargetExtr)
FSTRINGVAR(tTargetBedColon)
FSTRINGVAR(tPIDAutotuneStart)
FSTRINGVAR(tAPIDBias)
FSTRINGVAR(tAPIDD)
FSTRINGVAR(tAPIDMin)
FSTRINGVAR(tAPIDMax)
FSTRINGVAR(tAPIDKu)
FSTRINGVAR(tAPIDTu)
FSTRINGVAR(tAPIDClassic)
FSTRINGVAR(tAPIDKp)
FSTRINGVAR(tAPIDKi)
FSTRINGVAR(tAPIDKd)
FSTRINGVAR(tAPIDFailedHigh)
FSTRINGVAR(tAPIDFailedTimeout)
FSTRINGVAR(tAPIDFinished)
FSTRINGVAR(tMTEMPColon)
FSTRINGVAR(tHeatedBed)
FSTRINGVAR(tExtruderSpace)
FSTRINGVAR(tTempSensorDefect)
FSTRINGVAR(tTempSensorWorking)
FSTRINGVAR(tDryModeUntilRestart)
#ifdef DEBUG_QUEUE_MOVE
FSTRINGVAR(tDBGId)
FSTRINGVAR(tDBGVStartEnd)
FSTRINGVAR(tDBAccelSteps)
FSTRINGVAR(tDBGStartEndSpeed)
FSTRINGVAR(tDBGFlags)
FSTRINGVAR(tDBGJoinFlags)
FSTRINGVAR(tDBGDelta)
FSTRINGVAR(tDBGDir)
FSTRINGVAR(tDBGFullSpeed)
FSTRINGVAR(tDBGVMax)
FSTRINGVAR(tDBGAcceleration)
FSTRINGVAR(tDBGAccelerationPrim)
FSTRINGVAR(tDBGRemainingSteps)
FSTRINGVAR(tDBGAdvanceFull)
FSTRINGVAR(tDBGAdvanceRate)
FSTRINGVAR(tDBGLimitInterval)
FSTRINGVAR(tDBGMoveDistance)
FSTRINGVAR(tDBGCommandedFeedrate)
FSTRINGVAR(tDBGConstFullSpeedMoveTime)
#endif // DEBUG_QUEUE_MOVEFSTRINGVALUE(Com::,"")
#ifdef DEBUG_DELTA_OVERFLOW
FSTRINGVAR(tDBGDeltaOverflow)
#endif // DEBUG_DELTA_OVERFLOW
#ifdef DEBUG_SPLIT
FSTRINGVAR(tDBGDeltaSeconds)
FSTRINGVAR(tDBGDeltaZDelta)
FSTRINGVAR(tDBGDeltaSegments)
FSTRINGVAR(tDBGDeltaNumLines)
FSTRINGVAR(tDBGDeltaSegmentsPerLine)
FSTRINGVAR(tDBGDeltaMaxDS)
FSTRINGVAR(tDBGDeltaStepsPerSegment)
FSTRINGVAR(tDBGDeltaVirtualAxisSteps)
#endif
#ifdef STEP_COUNTER
FSTRINGVAR(tDBGDeltaMeasurerDelta)
FSTRINGVAR(tDBGDeltaMeasurementReset)
FSTRINGVAR(tDBGDeltaMeasuredOriginSet)
#endif // STEP_COUNTER
#ifdef DEBUG_STEPCOUNT
FSTRINGVAR(tDBGMissedSteps)
#endif
#if FEATURE_Z_PROBE
FSTRINGVAR(tZProbe)
FSTRINGVAR(tZProbeState)
FSTRINGVAR(tZProbeFailed)
FSTRINGVAR(tZProbeStartScript)
FSTRINGVAR(tZProbeEndScript)
FSTRINGVAR(tHitZProbe)
FSTRINGVAR(tZProbeAverage)
FSTRINGVAR(tZProbeZReset)
FSTRINGVAR(tAutolevelReset)
#endif
FSTRINGVAR(tAutolevelEnabled)
FSTRINGVAR(tAutolevelDisabled)
#if MAX_HARDWARE_ENDSTOP_Z
FSTRINGVAR(tZProbeMax)
FSTRINGVAR(tZProbePrinterHeight)
#endif

#ifdef WAITING_IDENTIFIER
FSTRINGVAR(tWait)
#endif // WAITING_IDENTIFIER

#if EEPROM_MODE==0
FSTRINGVAR(tNoEEPROMSupport)
#else
#if FEATURE_Z_PROBE
FSTRINGVAR(tZProbeHeight)
FSTRINGVAR(tZProbeOffsetX)
FSTRINGVAR(tZProbeOffsetY)
FSTRINGVAR(tZProbeSpeed)
FSTRINGVAR(tZProbeSpeedXY)
FSTRINGVAR(tZProbeX1)
FSTRINGVAR(tZProbeY1)
FSTRINGVAR(tZProbeX2)
FSTRINGVAR(tZProbeY2)
FSTRINGVAR(tZProbeX3)
FSTRINGVAR(tZProbeY3)
#endif
#if FEATURE_AUTOLEVEL
FSTRINGVAR(tAutolevelActive)
#endif
FSTRINGVAR(tConfigStoredEEPROM)
FSTRINGVAR(tConfigLoadedEEPROM)
FSTRINGVAR(tEPRConfigResetDefaults)
FSTRINGVAR(tEPRProtocolChanged)
FSTRINGVAR(tExtrDot)
FSTRINGVAR(tEPR0)
FSTRINGVAR(tEPR1)
FSTRINGVAR(tEPR2)
FSTRINGVAR(tEPR3)
FSTRINGVAR(tEPRBaudrate)
FSTRINGVAR(tEPRFilamentPrinted)
FSTRINGVAR(tEPRPrinterActive)
FSTRINGVAR(tEPRMaxInactiveTime)
FSTRINGVAR(tEPRStopAfterInactivty)
FSTRINGVAR(tEPRXStepsPerMM)
FSTRINGVAR(tEPRYStepsPerMM)
FSTRINGVAR(tEPRZStepsPerMM)
FSTRINGVAR(tEPRXMaxFeedrate)
FSTRINGVAR(tEPRYMaxFeedrate)
FSTRINGVAR(tEPRZMaxFeedrate)
FSTRINGVAR(tEPRXHomingFeedrate)
FSTRINGVAR(tEPRYHomingFeedrate)
FSTRINGVAR(tEPRZHomingFeedrate)
FSTRINGVAR(tEPRMaxJerk)
FSTRINGVAR(tEPRMaxZJerk)
FSTRINGVAR(tEPRXHomePos)
FSTRINGVAR(tEPRYHomePos)
FSTRINGVAR(tEPRZHomePos)
FSTRINGVAR(tEPRXMaxLength)
FSTRINGVAR(tEPRYMaxLength)
FSTRINGVAR(tEPRZMaxLength)
FSTRINGVAR(tEPRXBacklash)
FSTRINGVAR(tEPRYBacklash)
FSTRINGVAR(tEPRZBacklash)
FSTRINGVAR(tEPRXAcceleration)
FSTRINGVAR(tEPRYAcceleration)
FSTRINGVAR(tEPRZAcceleration)
FSTRINGVAR(tEPRXTravelAcceleration)
FSTRINGVAR(tEPRYTravelAcceleration)
FSTRINGVAR(tEPRZTravelAcceleration)
FSTRINGVAR(tEPROPSMode)
FSTRINGVAR(tEPROPSMoveAfter)
FSTRINGVAR(tEPROPSMinDistance)
FSTRINGVAR(tEPROPSRetractionLength)
FSTRINGVAR(tEPROPSRetractionBacklash)
FSTRINGVAR(tEPRBedHeatManager)
FSTRINGVAR(tEPRBedPIDDriveMax)
FSTRINGVAR(tEPRBedPIDDriveMin)
FSTRINGVAR(tEPRBedPGain)
FSTRINGVAR(tEPRBedIGain)
FSTRINGVAR(tEPRBedDGain)
FSTRINGVAR(tEPRBedPISMaxValue)
FSTRINGVAR(tEPRStepsPerMM)
FSTRINGVAR(tEPRMaxFeedrate)
FSTRINGVAR(tEPRStartFeedrate)
FSTRINGVAR(tEPRAcceleration)
FSTRINGVAR(tEPRHeatManager)
FSTRINGVAR(tEPRDriveMax)
FSTRINGVAR(tEPRDriveMin)
FSTRINGVAR(tEPRPGain)
FSTRINGVAR(tEPRIGain)
FSTRINGVAR(tEPRDGain)
FSTRINGVAR(tEPRPIDMaxValue)
FSTRINGVAR(tEPRXOffset)
FSTRINGVAR(tEPRYOffset)
FSTRINGVAR(tEPRStabilizeTime)
FSTRINGVAR(tEPRRetractionWhenHeating)
FSTRINGVAR(tEPRDistanceRetractHeating)
FSTRINGVAR(tEPRExtruderCoolerSpeed)
FSTRINGVAR(tEPRAdvanceK)
FSTRINGVAR(tEPRAdvanceL)
#endif
#if SDSUPPORT
FSTRINGVAR(tSDRemoved)
FSTRINGVAR(tSDInserted)
FSTRINGVAR(tSDInitFail)
FSTRINGVAR(tErrorWritingToFile)
FSTRINGVAR(tBeginFileList)
FSTRINGVAR(tEndFileList)
FSTRINGVAR(tFileOpened)
FSTRINGVAR(tSpaceSizeColon)
FSTRINGVAR(tFileSelected)
FSTRINGVAR(tFileOpenFailed)
FSTRINGVAR(tSDPrintingByte)
FSTRINGVAR(tNotSDPrinting)
FSTRINGVAR(tOpenFailedFile)
FSTRINGVAR(tWritingToFile)
FSTRINGVAR(tDoneSavingFile)
FSTRINGVAR(tFileDeleted)
FSTRINGVAR(tDeletionFailed)
FSTRINGVAR(tDirectoryCreated)
FSTRINGVAR(tCreationFailed)
FSTRINGVAR(tSDErrorCode)
#endif // SDSUPPORT



static void printNumber(unsigned long n);
static void printWarningF(FSTRINGPARAM(text));
static void printInfoF(FSTRINGPARAM(text));
static void printErrorF(FSTRINGPARAM(text));
static void printWarningFLN(FSTRINGPARAM(text));
static void printInfoFLN(FSTRINGPARAM(text));
static void printErrorFLN(FSTRINGPARAM(text));
static void printFLN(FSTRINGPARAM(text));
static void printF(FSTRINGPARAM(text));
static void printF(FSTRINGPARAM(text),int value);
static void printF(FSTRINGPARAM(text),const char *msg);
static void printF(FSTRINGPARAM(text),long value);
static void printF(FSTRINGPARAM(text),unsigned long value);
static void printF(FSTRINGPARAM(text),float value,byte digits=2);
static void printFLN(FSTRINGPARAM(text),int value);
static void printFLN(FSTRINGPARAM(text),long value);
static void printFLN(FSTRINGPARAM(text),unsigned long value);
static void printFLN(FSTRINGPARAM(text),const char *msg);
static void printFLN(FSTRINGPARAM(text),float value,byte digits=2);
static void printArrayFLN(FSTRINGPARAM(text),float *arr,byte n=4,byte digits=2);
static void printArrayFLN(FSTRINGPARAM(text),long *arr,byte n=4);
static void print(long value);
static inline void print(unsigned long value) {printNumber(value);}
static inline void print(int value) {print((long)value);}
static void print(const char *text);
static inline void print(char c) {HAL::serialWriteByte(c);}
static void printFloat(float number, byte digits);
static inline void println() {HAL::serialWriteByte('\r');HAL::serialWriteByte('\n');}
    protected:
    private:
};

#endif // COMMUNICATION_H
