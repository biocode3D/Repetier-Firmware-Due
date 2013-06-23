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

// ================ Sanity checks ================
#ifndef STEP_DOUBLER_FREQUENCY
#error Please add new parameter STEP_DOUBLER_FREQUENCY to your configuration.
#else
#if STEP_DOUBLER_FREQUENCY<10000 || STEP_DOUBLER_FREQUENCY>20000
#error STEP_DOUBLER_FREQUENCY should be in range 10000-16000.
#endif
#endif
#ifdef EXTRUDER_SPEED
#error EXTRUDER_SPEED is not used any more. Values are now taken from extruder definition.
#endif
#if MAX_HALFSTEP_INTERVAL<=1900
#error MAX_HALFSTEP_INTERVAL must be greater then 1900
#endif
#ifdef ENDSTOPPULLUPS
#error ENDSTOPPULLUPS is now replaced by individual pullup configuration!
#endif
#ifdef EXT0_PID_PGAIN
#error The PID system has changed. Please use the new float number options!
#endif
// ####################################################################################
// #          No configuration below this line - just some errorchecking              #
// ####################################################################################
#ifdef SUPPORT_MAX6675
#if !defined SCK_PIN || !defined MOSI_PIN || !defined MISO_PIN
#error For MAX6675 support, you need to define SCK_PIN, MISO_PIN and MOSI_PIN in pins.h
#endif
#endif
#if X_STEP_PIN<0 || Y_STEP_PIN<0 || Z_STEP_PIN<0
#error One of the following pins is not assigned: X_STEP_PIN,Y_STEP_PIN,Z_STEP_PIN
#endif
#if EXT0_STEP_PIN<0 && NUM_EXTRUDER>0
#error EXT0_STEP_PIN not set to a pin number.
#endif
#if EXT0_DIR_PIN<0 && NUM_EXTRUDER>0
#error EXT0_DIR_PIN not set to a pin number.
#endif
#if MOVE_CACHE_SIZE<4
#error MOVE_CACHE_SIZE must be at least 5
#endif

#if DRIVE_SYSTEM==3
#define SIN_60 0.8660254037844386
#define COS_60 0.5
#define DELTA_DIAGONAL_ROD_STEPS (AXIS_STEPS_PER_MM * DELTA_DIAGONAL_ROD)
#define DELTA_DIAGONAL_ROD_STEPS_SQUARED (DELTA_DIAGONAL_ROD_STEPS * DELTA_DIAGONAL_ROD_STEPS)
#define DELTA_ZERO_OFFSET_STEPS (AXIS_STEPS_PER_MM * DELTA_ZERO_OFFSET)
#define DELTA_RADIUS_STEPS (AXIS_STEPS_PER_MM * DELTA_RADIUS)

#define DELTA_TOWER1_X_STEPS -SIN_60*DELTA_RADIUS_STEPS
#define DELTA_TOWER1_Y_STEPS -COS_60*DELTA_RADIUS_STEPS
#define DELTA_TOWER2_X_STEPS SIN_60*DELTA_RADIUS_STEPS
#define DELTA_TOWER2_Y_STEPS -COS_60*DELTA_RADIUS_STEPS
#define DELTA_TOWER3_X_STEPS 0.0
#define DELTA_TOWER3_Y_STEPS DELTA_RADIUS_STEPS

#define NUM_AXIS 4
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define E_AXIS 3

volatile unsigned int delta_segment_count = 0; // Number of delta moves cached 0 = nothing in cache

#endif

#define OVERFLOW_PERIODICAL  (int)(F_CPU/(TIMER0_PRESCALE*40))

//Inactivity shutdown variables
unsigned long previous_millis_cmd = 0;
unsigned long max_inactive_time = MAX_INACTIVE_TIME*1000L;
unsigned long stepper_inactive_time = STEPPER_INACTIVE_TIME*1000L;
long baudrate = BAUDRATE;         ///< Communication speed rate.
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
int maxadv=0;
#endif
int maxadv2=0;
float maxadvspeed=0;
#endif
byte pwm_pos[NUM_EXTRUDER+3]; // 0-NUM_EXTRUDER = Heater 0-NUM_EXTRUDER of extruder, NUM_EXTRUDER = Heated bed, NUM_EXTRUDER+1 Board fan, NUM_EXTRUDER+2 = Fan
volatile int waitRelax=0; // Delay filament relax at the end of print, could be a simple timeout

PrintLine PrintLine::lines[MOVE_CACHE_SIZE]; ///< Cache for print moves.
PrintLine *PrintLine::cur = 0;               ///< Current printing line
byte PrintLine::lines_write_pos=0;           ///< Position where we write the next cached line move.
volatile byte PrintLine::lines_count=0;      ///< Number of lines cached 0 = nothing to do.
byte PrintLine::lines_pos=0;                 ///< Position for executing line movement.

/**
Move printer the given number of steps. Puts the move into the queue. Used by e.g. homing commands.
*/
void PrintLine::moveRelativeDistanceInSteps(long x,long y,long z,long e,float feedrate,bool waitEnd,bool check_endstop)
{
    float saved_feedrate = Printer::feedrate;
    Printer::destinationSteps[0] = Printer::currentPositionSteps[0] + x;
    Printer::destinationSteps[1] = Printer::currentPositionSteps[1] + y;
    Printer::destinationSteps[2] = Printer::currentPositionSteps[2] + z;
    Printer::destinationSteps[3] = Printer::currentPositionSteps[3] + e;
    Printer::feedrate = feedrate;
#if DRIVE_SYSTEM==3
    split_delta_move(check_endstop,false,false);
#else
    queue_move(check_endstop,false);
#endif
    Printer::feedrate = saved_feedrate;
    Printer::updateCurrentPosition();
    if(waitEnd)
        Commands::waitUntilEndOfAllMoves();
}

#if DRIVE_SYSTEM != 3
/**
  Put a move to the current destination coordinates into the movement cache.
  If the cache is full, the method will wait, until a place gets free. During
  wait communication and temperature control is enabled.
  @param check_endstops Read endstop during move.
*/
void PrintLine::queue_move(byte check_endstops,byte pathOptimize)
{
    Printer::unsetAllSteppersDisabled();
    waitForXFreeLines(1);
    byte newPath=insertWaitMovesIfNeeded(pathOptimize, 0);
    PrintLine *p = getNextWriteLine();

    float axis_diff[4]; // Axis movement in mm
    if(check_endstops) p->flags = FLAG_CHECK_ENDSTOPS;
    else p->flags = 0;
    p->joinFlags = 0;
    if(!pathOptimize) p->setEndSpeedFixed(true);
    p->dir = 0;
    Printer::constrainDestinationCoords();
    //Find direction
    for(byte axis=0; axis < 4; axis++)
    {
        if((p->delta[axis]=Printer::destinationSteps[axis]-Printer::currentPositionSteps[axis])>=0)
            p->setPositiveDirectionForAxis(axis);
        else
            p->delta[axis] = -p->delta[axis];
        if(axis==3 && Printer::extrudeMultiply!=100)
            p->delta[3]=(long)((p->delta[3]*(float)Printer::extrudeMultiply)*0.01f);
        axis_diff[axis] = p->delta[axis]*Printer::invAxisStepsPerMM[axis];
        if(p->delta[axis]) p->setMoveOfAxis(axis);
        Printer::currentPositionSteps[axis] = Printer::destinationSteps[axis];
    }
    if(p->isNoMove())
    {
        if(newPath)   // need to delete dummy elements, otherwise commands can get locked.
            resetPathPlanner();
        return; // No steps included
    }
    Printer::filamentPrinted+=axis_diff[3];
    float xydist2;
#if ENABLE_BACKLASH_COMPENSATION
    if((p->isXYZMove()) && ((p->dir & 7)^(Printer::backlashDir & 7)) & (Printer::backlashDir >> 3))   // We need to compensate backlash, add a move
    {
        waitForXFreeLines(2);
        byte wpos2 = lines_write_pos+1;
        if(wpos2>=MOVE_CACHE_SIZE) wpos2 = 0;
        PrintLine *p2 = &lines[wpos2];
        memcpy(p2,p,sizeof(PrintLine)); // Move current data to p2
        byte changed = (p->dir & 7)^(Printer::backlashDir & 7);
        float back_diff[4]; // Axis movement in mm
        back_diff[3] = 0;
        back_diff[0] = (changed & 1 ? (p->isXPositiveMove() ? Printer::backlashX : -Printer::backlashX) : 0);
        back_diff[1] = (changed & 2 ? (p->isYPositiveMove() ? Printer::backlashY : -Printer::backlashY) : 0);
        back_diff[2] = (changed & 4 ? (p->isZPositiveMove() ? Printer::backlashZ : -Printer::backlashZ) : 0);
        p->dir &=7; // x,y and z are already correct
        for(byte i=0; i < 4; i++)
        {
            float f = back_diff[i]*Printer::axisStepsPerMM[i];
            p->delta[i] = abs((long)f);
            if(p->delta[i]) p->dir |= 16<<i;
        }
        //Define variables that are needed for the Bresenham algorithm. Please note that  Z is not currently included in the Bresenham algorithm.
        if(p->delta[1] > p->delta[0] && p->delta[1] > p->delta[2]) p->primaryAxis = 1;
        else if (p->delta[0] > p->delta[2] ) p->primaryAxis = 0;
        else p->primaryAxis = 2;
        p->stepsRemaining = p->delta[p->primaryAxis];
        //Feedrate calc based on XYZ travel distance
        xydist2 = back_diff[0] * back_diff[0] + back_diff[1] * back_diff[1];
        if(p->isZMove())
            p->distance = sqrt(xydist2 + back_diff[2] * back_diff[2]);
        else
            p->distance = sqrt(xydist2);
        Printer::backlashDir = (Printer::backlashDir & 56) | (p2->dir & 7);
        p->calculate_move(back_diff,pathOptimize);
        p = p2; // use saved instance for the real move
    }
#endif

    //Define variables that are needed for the Bresenham algorithm. Please note that  Z is not currently included in the Bresenham algorithm.
    if(p->delta[1] > p->delta[0] && p->delta[1] > p->delta[2] && p->delta[1] > p->delta[3]) p->primaryAxis = 1;
    else if (p->delta[0] > p->delta[2] && p->delta[0] > p->delta[3]) p->primaryAxis = 0;
    else if (p->delta[2] > p->delta[3]) p->primaryAxis = 2;
    else p->primaryAxis = 3;
    p->stepsRemaining = p->delta[p->primaryAxis];
    if(p->isXYZMove())
    {
        xydist2 = axis_diff[0] * axis_diff[0] + axis_diff[1] * axis_diff[1];
        if(p->isZMove())
            p->distance = sqrt(xydist2 + axis_diff[2] * axis_diff[2]);
        else
            p->distance = sqrt(xydist2);
    }
    else
        p->distance = fabs(axis_diff[3]);
    p->calculate_move(axis_diff,pathOptimize);
}
#endif
void PrintLine::calculate_move(float axis_diff[],byte pathOptimize)
{
#if DRIVE_SYSTEM==3
    long axis_interval[5];
#else
    long axis_interval[4];
#endif
    float time_for_move = (float)(F_CPU)*distance / Printer::feedrate; // time is in ticks
    bool critical=false;
    if(lines_count<MOVE_CACHE_LOW && time_for_move<LOW_TICKS_PER_MOVE)   // Limit speed to keep cache full.
    {
        //OUT_P_I("L:",lines_count);
        time_for_move += (3*(LOW_TICKS_PER_MOVE-time_for_move))/(lines_count+1); // Increase time if queue gets empty. Add more time if queue gets smaller.
        //OUT_P_F_LN("Slow ",time_for_move);
        critical=true;
    }
    timeInTicks = time_for_move;
    UI_MEDIUM; // do check encoder
    // Compute the solwest allowed interval (ticks/step), so maximum feedrate is not violated
    long limitInterval = time_for_move/stepsRemaining; // until not violated by other constraints it is your target speed
    axis_interval[0] = fabs(axis_diff[0])*F_CPU/(Printer::maxFeedrate[0]*stepsRemaining); // mm*ticks/s/(mm/s*steps) = ticks/step
    limitInterval = RMath::max(axis_interval[0],limitInterval);
    axis_interval[1] = fabs(axis_diff[1])*F_CPU/(Printer::maxFeedrate[1]*stepsRemaining);
    limitInterval = RMath::max(axis_interval[1],limitInterval);
    if(isZMove())   // normally no move in z direction
    {
        axis_interval[2] = fabs((float)axis_diff[2])*(float)F_CPU/(float)(Printer::maxFeedrate[2]*stepsRemaining); // must prevent overflow!
        limitInterval = RMath::max(axis_interval[2],limitInterval);
    }
    else axis_interval[2] = 0;
    axis_interval[3] = fabs(axis_diff[3])*F_CPU/(Printer::maxFeedrate[3]*stepsRemaining);
    limitInterval = RMath::max(axis_interval[3],limitInterval);
#if DRIVE_SYSTEM==3
    axis_interval[4] = fabs(axis_diff[4])*F_CPU/(Printer::maxFeedrate[0]*stepsRemaining);
#endif

    fullInterval = limitInterval>200 ? limitInterval : 200; // This is our target speed
    // new time at full speed = limitInterval*p->stepsRemaining [ticks]
    time_for_move = (float)limitInterval*(float)stepsRemaining; // for large z-distance this overflows with long computation
    float inv_time_s = (float)F_CPU/time_for_move;
    if(isXMove())
    {
        axis_interval[0] = time_for_move/delta[0];
        speedX = axis_diff[0]*inv_time_s;
        if(isXNegativeMove()) speedX = -speedX;
    }
    else speedX = 0;
    if(isYMove())
    {
        axis_interval[1] = time_for_move/delta[1];
        speedY = axis_diff[1]*inv_time_s;
        if(isYNegativeMove()) speedY = -speedY;
    }
    else speedY = 0;
    if(isZMove())
    {
        axis_interval[2] = time_for_move/delta[2];
        speedZ = axis_diff[2]*inv_time_s;
        if(isZNegativeMove()) speedZ = -speedZ;
    }
    else speedZ = 0;
    if(isEMove())
    {
        axis_interval[3] = time_for_move/delta[3];
        speedE = axis_diff[3]*inv_time_s;
        if(isENegativeMove()) speedE = -speedE;
    }
#if DRIVE_SYSTEM==3
    axis_interval[4] = time_for_move/stepsRemaining;
#endif
    fullSpeed = distance*inv_time_s;
    //long interval = axis_interval[primary_axis]; // time for every step in ticks with full speed
    byte is_print_move = isEPositiveMove(); // are we printing
    //If acceleration is enabled, do some Bresenham calculations depending on which axis will lead it.
#ifdef RAMP_ACCELERATION

    // slowest time to accelerate from v0 to limitInterval determines used acceleration
    // t = (v_end-v_start)/a
    float slowest_axis_plateau_time_repro = 1e20; // repro to reduce division Unit: 1/s
    for(byte i=0; i < 4 ; i++)
    {
        // Errors for delta move are initialized in timer
#if DRIVE_SYSTEM!=3
        error[i] = delta[primaryAxis] >> 1;
#endif
        if(isMoveOfAxis(i))
        {
            // v = a * t => t = v/a = F_CPU/(c*a) => 1/t = c*a/F_CPU
            slowest_axis_plateau_time_repro = RMath::min(slowest_axis_plateau_time_repro,
                                              (float)axis_interval[i] * (float)(is_print_move ?  Printer::maxPrintAccelerationStepsPerSquareSecond[i] : Printer::maxTravelAccelerationStepsPerSquareSecond[i])); //  steps/s^2 * step/tick  Ticks/s^2
        }
    }
    // Errors for delta move are initialized in timer (except extruder)
#if DRIVE_SYSTEM==3
    error[3] = stepsRemaining >> 1;
#endif
    invFullSpeed = 1.0/fullSpeed;
    accelerationPrim = slowest_axis_plateau_time_repro / axis_interval[primaryAxis]; // a = v/t = F_CPU/(c*t): Steps/s^2
    //Now we can calculate the new primary axis acceleration, so that the slowest axis max acceleration is not violated
    facceleration = 262144.0*(float)accelerationPrim/F_CPU; // will overflow without float!
    accelerationDistance2 = 2.0*distance*slowest_axis_plateau_time_repro*fullSpeed/((float)F_CPU); // mm^2/s^2
    startSpeed = endSpeed = safeSpeed();
    // Can accelerate to full speed within the line
    if (startSpeed*startSpeed+accelerationDistance2 >= fullSpeed*fullSpeed)
        setNominalMove();

    vMax = F_CPU / fullInterval; // maximum steps per second, we can reach
    // if(p->vMax>46000)  // gets overflow in N computation
    //   p->vMax = 46000;
    //p->plateauN = (p->vMax*p->vMax/p->accelerationPrim)>>1;
#ifdef USE_ADVANCE
    if(!isXYZMove() || !isEMove())
    {
#ifdef ENABLE_QUADRATIC_ADVANCE
        advanceRate = 0; // No head move or E move only or sucking filament back
        advanceFull = 0;
#endif
        advanceL = 0;
    }
    else
    {
        float advlin = fabs(speedE)*Extruder::current->advanceL*0.001*Printer::axisStepsPerMM[3];
        advanceL = (65536*advlin)/vMax; //advanceLscaled = (65536*vE*k2)/vMax
#ifdef ENABLE_QUADRATIC_ADVANCE;
        advanceFull = 65536*Extruder::current->advanceK*speedE*speedE; // Steps*65536 at full speed
        long steps = (HAL::U16SquaredToU32(vMax))/(accelerationPrim<<1); // v^2/(2*a) = steps needed to accelerate from 0-vMax
        advanceRate = advanceFull/steps;
        if((advanceFull>>16)>maxadv)
        {
            maxadv = (advanceFull>>16);
            maxadvspeed = fabs(speedE);
        }
#endif
        if(advlin>maxadv2)
        {
            maxadv2 = advlin;
            maxadvspeed = fabs(speedE);
        }
    }
#endif
    UI_MEDIUM; // do check encoder
    updateTrapezoids();
    // how much steps on primary axis do we need to reach target feedrate
    //p->plateauSteps = (long) (((float)p->acceleration *0.5f / slowest_axis_plateau_time_repro + p->vMin) *1.01f/slowest_axis_plateau_time_repro);
#else
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    advanceRate = 0; // No advance for constant speeds
    advanceFull = 0;
#endif
#endif
#endif

    // Correct integers for fixed point math used in bresenham_step
    if(fullInterval<MAX_HALFSTEP_INTERVAL || critical)
        halfstep = 4;
    else
    {
        halfstep = 1;
#if DRIVE_SYSTEM==3
        // Error 0-2 are used for the towers and set up in the timer
        error[3] = stepsRemaining;
#else
        error[0] = error[1] = error[2] = error[3] = delta[primaryAxis];
#endif
    }
#ifdef DEBUG_STEPCOUNT
// Set in delta move calculation
#if DRIVE_SYSTEM!=3
    totalStepsRemaining = delta[0]+delta[1]+delta[2];
#endif
#endif
#ifdef DEBUG_QUEUE_MOVE
    if(Printer::debugEcho())
    {
        logLine();
        Com::printFLN(Com::tDBGLimitInterval, limitInterval);
        Com::printFLN(Com::tDBGMoveDistance, distance);
        Com::printFLN(Com::tDBGCommandedFeedrate, Printer::feedrate);
        Com::printFLN(Com::tDBGConstFullSpeedMoveTime, time_for_move);
    }
#endif
    // Make result permanent
    if (pathOptimize) waitRelax = 70;
    pushLine();
    DEBUG_MEMORY;
}

/**
This is the path planner.

It goes from the last entry and tries to increase the end speed of previous moves in a fashion that the maximum jerk
is never exceeded. If a segment with reached maximum speed is met, the planner stops. Everything left from this
is already optimal from previous updates.
The first 2 entries in the queue are not checked. The first is the one that is already in print and the following will likely become active.

The method is called before lines_count is increased!
*/
void PrintLine::updateTrapezoids()
{
    byte first = lines_write_pos;
    PrintLine *firstLine;
    PrintLine *act = &lines[lines_write_pos];
    BEGIN_INTERRUPT_PROTECTED;
    byte maxfirst = lines_pos; // first non fixed segment
    if(maxfirst!=lines_write_pos)
        nextPlannerIndex(maxfirst); // don't touch the line printing
    // Now ignore enough segments to gain enough time for path planning
    long timeleft = 0;
    // Skip as many stored moves as needed to gain enough time for computation
    while(timeleft<4500*MOVE_CACHE_SIZE && maxfirst!=lines_write_pos)
    {
        timeleft+=lines[maxfirst].timeInTicks;
        nextPlannerIndex(maxfirst);
    }
    // Search last fixed element
    while(first!=maxfirst && !lines[first].isEndSpeedFixed())
        previousPlannerIndex(first);
    if(first!=lines_write_pos && lines[first].isEndSpeedFixed())
        nextPlannerIndex(first);
    if(first==lines_write_pos)   // Nothing to plan
    {
        ESCAPE_INTERRUPT_PROTECTED
        act->setStartSpeedFixed(true);
        act->updateStepsParameter();
        act->unblock();
        return;
    }
    // now we have at least one additional move for optimization
    // that is not a wait move
    // First is now the new element or the first element with non fixed end speed.
    // anyhow, the start speed of first is fixed
    firstLine = &lines[first];
    firstLine->block(); // don't let printer touch this or following segments during update
    END_INTERRUPT_PROTECTED;
    byte previousIndex = lines_write_pos;
    previousPlannerIndex(previousIndex);
    PrintLine *previous = &lines[previousIndex];
#if DRIVE_SYSTEM!=3
    if((previous->primaryAxis==2 && act->primaryAxis!=2) || (previous->primaryAxis!=2 && act->primaryAxis==2)) {
        previous->setEndSpeedFixed(true);
        act->setStartSpeedFixed(true);
        act->updateStepsParameter();
        firstLine->unblock();
        return;
    }
#endif // DRIVE_SYSTEM

    computeMaxJunctionSpeed(previous,act); // Set maximum junction speed if we have a real move before
    if(previous->isEOnlyMove() != act->isEOnlyMove())
    {
        previous->setEndSpeedFixed(true);
        act->setStartSpeedFixed(true);
        act->updateStepsParameter();
        firstLine->unblock();
        return;
    }
    backwardPlanner(lines_write_pos,first);
    // Reduce speed to reachable speeds
    forwardPlanner(first);

    // Update precomputed data
    do
    {
        lines[first].updateStepsParameter();
        lines[first].unblock();  // Flying block to release next used segment as early as possible
        nextPlannerIndex(first);
        lines[first].block();
    }
    while(first!=lines_write_pos);
    act->updateStepsParameter();
    act->unblock();
}

inline void PrintLine::computeMaxJunctionSpeed(PrintLine *previous,PrintLine *current)
{
#ifdef USE_ADVANCE
    if(Printer::isAdvanceActivated())
    {
        if(previous->isEMove()!=current->isEMove() && (previous->isXOrYMove() || current->isXOrYMove()))
        {
            previous->setEndSpeedFixed(true);
            current->setStartSpeedFixed(true);
            previous->endSpeed = current->startSpeed = previous->maxJunctionSpeed = RMath::min(previous->endSpeed,current->startSpeed);
            previous->invalidateParameter();
            current->invalidateParameter();
            return;
        }
    }
#endif // USE_ADVANCE
#if DRIVE_SYSTEM==3
    if (previous->moveID == current->moveID)   // Avoid computing junction speed for split delta lines
    {
        if(previous->fullSpeed>current->fullSpeed)
            previous->maxJunctionSpeed = current->fullSpeed;
        else
            previous->maxJunctionSpeed = previous->fullSpeed;
        return;
    }
#endif
    // First we compute the normalized jerk for speed 1
    float dx = current->speedX-previous->speedX;
    float dy = current->speedY-previous->speedY;
    float factor=1;
#if (DRIVE_SYSTEM == 3) // No point computing Z Jerk separately for delta moves
    float dz = current->speedZ-previous->speedZ;
    float jerk = sqrt(dx*dx+dy*dy+dz*dz);
#else
    float jerk = sqrt(dx*dx+dy*dy);
#endif
    if(jerk>Printer::maxJerk)
        factor = Printer::maxJerk/jerk;
#if DRIVE_SYSTEM!=3
    if((previous->dir & current->dir) & 64)
    {
        float dz = fabs(current->speedZ-previous->speedZ);
        if(dz>Printer::maxZJerk)
            factor = RMath::min(factor,Printer::maxZJerk/dz);
    }
#endif
    float eJerk = fabs(current->speedE-previous->speedE);
    if(eJerk>Extruder::current->maxStartFeedrate)
        factor = RMath::min(factor,Extruder::current->maxStartFeedrate/eJerk);
    previous->maxJunctionSpeed = RMath::min(previous->fullSpeed*factor,current->fullSpeed);
}

/** Update parameter used by updateTrapezoids

Computes the acceleration/decelleration steps and advanced parameter associated.
*/
void PrintLine::updateStepsParameter()
{
    if(areParameterUpToDate() || isWarmUp()) return;
    float startFactor = startSpeed * invFullSpeed;
    float endFactor   = endSpeed   * invFullSpeed;
    vStart = vMax*startFactor; //starting speed
    vEnd   = vMax*endFactor;
    unsigned long vmax2 = HAL::U16SquaredToU32(vMax);
    accelSteps = ((vmax2-HAL::U16SquaredToU32(vStart))/(accelerationPrim<<1))+1; // Always add 1 for missing precision
    decelSteps = ((vmax2-HAL::U16SquaredToU32(vEnd))  /(accelerationPrim<<1))+1;
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    advanceStart = (float)advanceFull*startFactor * startFactor;
    advanceEnd   = (float)advanceFull*endFactor   * endFactor;
#endif
#endif
    if(accelSteps+decelSteps>=stepsRemaining)   // can't reach limit speed
    {
        unsigned int red = (accelSteps+decelSteps+2-stepsRemaining)>>1;
        accelSteps = accelSteps-RMath::min(accelSteps,red);
        decelSteps = decelSteps-RMath::min(decelSteps,red);
    }
    setParameterUpToDate();
#ifdef DEBUG_QUEUE_MOVE
    if(Printer::debugEcho())
    {
        Com::printFLN(Com::tDBGId,(int)this);
        Com::printF(Com::tDBGVStartEnd,(long)vStart);
        Com::printFLN(Com::tSlash,(long)vEnd);
        Com::printF(Com::tDBAccelSteps,(long)accelSteps);
        Com::printF(Com::tSlash,(long)decelSteps);
        Com::printFLN(Com::tSlash,(long)stepsRemaining);
        Com::printF(Com::tDBGStartEndSpeed,(long)startSpeed);
        Com::printFLN(Com::tSlash,(long)endSpeed);
        Com::printFLN(Com::tDBGFlags,flags);
        Com::printFLN(Com::tDBGJoinFlags,joinFlags);
    }
#endif
}

/**
Compute the maximum speed from the last entered move.
The backwards planner traverses the moves from last to first looking at deceleration. The RHS of the accelerate/decelerate ramp.

start = last line inserted
last = last element until we check
*/
inline void PrintLine::backwardPlanner(byte start,byte last)
{
    PrintLine *act = &lines[start],*previous;
    float lastJunctionSpeed = act->endSpeed; // Start always with safe speed
    //PREVIOUS_PLANNER_INDEX(last); // Last element is already fixed in start speed
    while(start!=last)
    {
        previousPlannerIndex(start);
        previous = &lines[start];
        // Avoid speed calc once crusing in split delta move
#if DRIVE_SYSTEM==3
        if (previous->moveID==act->moveID && lastJunctionSpeed==previous->maxJunctionSpeed)
        {
            act->startSpeed = previous->endSpeed = lastJunctionSpeed;
            previous->invalidateParameter();
            act->invalidateParameter();
        }
#endif

        /* if(prev->isEndSpeedFixed())   // Nothing to update from here on, happens when path optimize disabled
         {
             act->setStartSpeedFixed(true);
             return;
         }*/

        // Avoid speed calcs if we know we can accelerate within the line
        if (act->isNominalMove())
            lastJunctionSpeed = act->fullSpeed;
        else
            // If you accelerate from end of move to start what speed do you reach?
            lastJunctionSpeed = sqrt(lastJunctionSpeed*lastJunctionSpeed+act->accelerationDistance2); // acceleration is acceleration*distance*2! What can be reached if we try?
        // If that speed is more that the maximum junction speed allowed then ...
        if(lastJunctionSpeed>=previous->maxJunctionSpeed)   // Limit is reached
        {
            // If the previous line's end speed has not been updated to maximum speed then do it now
            if(previous->endSpeed!=previous->maxJunctionSpeed)
            {
                previous->invalidateParameter(); // Needs recomputation
                previous->endSpeed = previous->maxJunctionSpeed; // possibly unneeded???
            }
            // If actual line start speed has not been updated to maximum speed then do it now
            if(act->startSpeed!=previous->maxJunctionSpeed)
            {
                act->startSpeed = previous->maxJunctionSpeed; // possibly unneeded???
                act->invalidateParameter();
            }
            lastJunctionSpeed = previous->maxJunctionSpeed;
        }
        else
        {
            // Block prev end and act start as calculated speed and recalculate plateau speeds (which could move the speed higher again)
            act->startSpeed = previous->endSpeed = lastJunctionSpeed;
            previous->invalidateParameter();
            act->invalidateParameter();
        }
        act = previous;
    } // while loop
}

void PrintLine::forwardPlanner(byte first)
{
    PrintLine *act,*next;
    next = &lines[first];
    float leftSpeed = next->startSpeed;
    while(first!=lines_write_pos)   // All except last segment, which has fixed end speed
    {
        act = next;
        nextPlannerIndex(first);
        next = &lines[first];
        /* if(act->isEndSpeedFixed())
         {
             leftSpeed = act->endSpeed;
             continue; // Nothing to do here
         }*/
        // Avoid speed calc once crusing in split delta move
#if DRIVE_SYSTEM==3
        if (act->moveID == next->moveID && act->endSpeed == act->maxJunctionSpeed)
        {
            act->startSpeed = leftSpeed;
            leftSpeed       = act->endSpeed;
            act->setEndSpeedFixed(true);
            next->setStartSpeedFixed(true);
            continue;
        }
#endif
        float vmaxRight;
        // Avoid speed calcs if we know we can accelerate within the line.
        if (act->isNominalMove())
            vmaxRight = act->fullSpeed;
        else
            vmaxRight = sqrt(leftSpeed*leftSpeed+act->accelerationDistance2);
        if(vmaxRight>act->endSpeed)   // Could be higher next run?
        {
            act->startSpeed = leftSpeed;
            leftSpeed       = act->endSpeed;
            if(act->endSpeed==act->maxJunctionSpeed)  // Full speed reached, don't compute again!
            {
                act->setEndSpeedFixed(true);
                next->setStartSpeedFixed(true);
            }
            act->invalidateParameter();
        }
        else     // We can accelerate full speed without reaching limit, which is as fast as possible. Fix it!
        {
            act->fixStartAndEndSpeed();
            act->invalidateParameter();
            act->startSpeed = leftSpeed;
            act->endSpeed = leftSpeed = vmaxRight;
            next->setStartSpeedFixed(true);
        }
    }
    next->startSpeed = leftSpeed; // This is the new segment, wgich is updated anyway, no extra flag needed.
}


inline float PrintLine::safeSpeed()
{
    float safe;
#ifdef USE_ADVANCE
    if(isEMove() && Printer::isAdvanceActivated())
    {
        safe = Printer::minimumSpeed;
    }
    else
#endif
        safe = RMath::max(Printer::minimumSpeed,Printer::maxJerk*0.5);
#if DRIVE_SYSTEM != 3
    if(isZMove())
    {
        if(primaryAxis==2)
            safe = Printer::maxZJerk*0.5*fullSpeed/fabs(speedZ);
        else if(fabs(speedZ)>Printer::maxZJerk*0.5)
            safe = RMath::min(safe,Printer::maxZJerk*0.5*fullSpeed/fabs(speedZ));
    }
#endif
    if(isEMove())
    {
        if(isXYZMove())
            safe = RMath::min(safe,0.5*Extruder::current->maxStartFeedrate*fullSpeed/fabs(speedE));
        else
            safe = 0.5*Extruder::current->maxStartFeedrate; // This is a retraction move
    }
    return RMath::min(safe,fullSpeed);
}


/** Check if move is new. If it is insert some dummy moves to allow the path optimizer to work since it does
not act on the first two moves in the queue. The stepper timer will spot these moves and leave some time for
processing.
*/
byte PrintLine::insertWaitMovesIfNeeded(byte pathOptimize, byte waitExtraLines)
{
    if(lines_count==0 && waitRelax==0 && pathOptimize)   // First line after some time - warmup needed
    {
        byte w = 3;
        while(w--)
        {
            PrintLine *p = getNextWriteLine();
            p->flags = FLAG_WARMUP;
            p->joinFlags = FLAG_JOIN_STEPPARAMS_COMPUTED | FLAG_JOIN_END_FIXED | FLAG_JOIN_START_FIXED;
            p->dir = 0;
            p->setWaitForXLinesFilled(w + waitExtraLines);
            p->setWaitTicks(10000*(unsigned int)w);
            pushLine();
        }
        return 1;
    }
    return 0;
}
void PrintLine::logLine()
{
#ifdef DEBUG_QUEUE_MOVE
    Com::printFLN(Com::tDBGId,(int)this);
    Com::printArrayFLN(Com::tDBGDelta,delta);
    Com::printFLN(Com::tDBGDir,dir);
    Com::printFLN(Com::tDBGFlags,flags);
    Com::printFLN(Com::tDBGFullSpeed,fullSpeed);
    Com::printFLN(Com::tDBGVMax,(long)vMax);
    Com::printFLN(Com::tDBGAcceleration,accelerationDistance2);
    Com::printFLN(Com::tDBGAccelerationPrim,(long)accelerationPrim);
    Com::printFLN(Com::tDBGRemainingSteps,stepsRemaining);
#ifdef USE_ADVANCE
#ifdef ENABLE_QUADRATIC_ADVANCE
    Com::printFLN(Com::tDBGAdvanceFull,advanceFull>>16);
    Com::printFLN(Com::tDBGAdvanceRate,advanceRate);
#endif
#endif
#endif // DEBUG_QUEUE_MOVE
}

void PrintLine::waitForXFreeLines(byte b)
{
    while(lines_count+b>MOVE_CACHE_SIZE)   // wait for a free entry in movement cache
    {
        GCode::readFromSerial();
        Commands::checkForPeriodicalActions();
    }
}


#if DRIVE_SYSTEM==3
/**
  Calculate and cache the delta robot positions of the cartesian move in a line.
  @return The largest delta axis move in a single segment
  @param p The line to examine.
*/
inline long PrintLine::calculate_delta_segments(byte softEndstop)
{

    long destination_steps[3], destination_delta_steps[3];

    for(byte i=0; i < NUM_AXIS - 1; i++)
    {
        // Save current position
        destination_steps[i] = Printer::currentPositionSteps[i];
    }

//	out.println_byte_P(PSTR("Calculate delta segments:"), p->numDeltaSegments);
    deltaSegmentReadPos = delta_segment_write_pos;
#ifdef DEBUG_STEPCOUNT
    totalStepsRemaining=0;
#endif

    long max_axis_move = 0;
    unsigned int produced_segments = 0;
    for (int s = numDeltaSegments; s > 0; s--)
    {
        for(byte i=0; i < NUM_AXIS - 1; i++)
            destination_steps[i] += (Printer::destinationSteps[i] - destination_steps[i]) / s;

        // Wait for buffer here
        while(delta_segment_count + produced_segments>=DELTA_CACHE_SIZE)   // wait for a free entry in movement cache
        {
            GCode::readFromSerial();
            Commands::checkForPeriodicalActions();
        }

        DeltaSegment *d = &segments[delta_segment_write_pos];

        // Verify that delta calc has a solution
        if (calculate_delta(destination_steps, destination_delta_steps))
        {
            d->dir = 0;
            for(byte i=0; i < NUM_AXIS - 1; i++)
            {
                if (softEndstop && destination_delta_steps[i] > Printer::maxDeltaPositionSteps)
                    destination_delta_steps[i] = Printer::maxDeltaPositionSteps;
                long delta = destination_delta_steps[i] - Printer::currentDeltaPositionSteps[i];
//#ifdef DEBUG_DELTA_CALC
//				out.println_long_P(PSTR("dest:"), destination_delta_steps[i]);
//				out.println_long_P(PSTR("cur:"), printer_state.currentDeltaPositionSteps[i]);
//#endif
                if (delta == 0)
                {
                    d->deltaSteps[i] = 0;
                }
                else if (delta > 0)
                {
                    d->dir |= 17<<i;
#ifdef DEBUG_DELTA_OVERFLOW
                    if (delta > 65535)
                        Com::printFLN(Com::tDBGDeltaOverflow, delta);
#endif
                    d->deltaSteps[i] = delta;
                }
                else
                {
                    d->dir |= 16<<i;
#ifdef DEBUG_DELTA_OVERFLOW
                    if (-delta > 65535)
                        Com::printFLN(Com::tDBGDeltaOverflow, delta);
#endif
                    d->deltaSteps[i] = -delta;
                }
#ifdef DEBUG_STEPCOUNT
                totalStepsRemaining += d->deltaSteps[i];
#endif

                if (max_axis_move < d->deltaSteps[i]) max_axis_move = d->deltaSteps[i];
                Printer::currentDeltaPositionSteps[i] = destination_delta_steps[i];
            }
        }
        else
        {
            // Illegal position - idnore move
            Com::printWarningFLN(Com::tInvalidDeltaCoordinate);
            d->dir = 0;
            for(byte i=0; i < NUM_AXIS - 1; i++)
            {
                d->deltaSteps[i]=0;
            }
        }
        // Move to the next segment
        delta_segment_write_pos++;
        if (delta_segment_write_pos >= DELTA_CACHE_SIZE) delta_segment_write_pos=0;
        produced_segments++;
    }
    BEGIN_INTERRUPT_PROTECTED
    delta_segment_count+=produced_segments;
    END_INTERRUPT_PROTECTED

#ifdef DEBUG_STEPCOUNT
//		out.println_long_P(PSTR("totalStepsRemaining:"), p->totalStepsRemaining);
#endif
    return max_axis_move;
}

/**
  Calculate the delta tower position from a cartesian position
  @param cartesianPosSteps Array containing cartesian coordinates.
  @param deltaPosSteps Result array with tower coordinates.
  @returns 1 if cartesian coordinates have a valid delta tower position 0 if not.
*/
byte calculate_delta(long cartesianPosSteps[], long deltaPosSteps[])
{
    long temp;
    long opt = DELTA_DIAGONAL_ROD_STEPS_SQUARED - sq(DELTA_TOWER1_Y_STEPS - cartesianPosSteps[Y_AXIS]);

    if ((temp = opt - sq(DELTA_TOWER1_X_STEPS - cartesianPosSteps[X_AXIS])) >= 0)
        deltaPosSteps[X_AXIS] = sqrt(temp) + cartesianPosSteps[Z_AXIS];
    else
        return 0;

    if ((temp = opt - sq(DELTA_TOWER2_X_STEPS - cartesianPosSteps[X_AXIS])) >= 0)
        deltaPosSteps[Y_AXIS] = sqrt(temp) + cartesianPosSteps[Z_AXIS];
    else
        return 0;

    if ((temp = DELTA_DIAGONAL_ROD_STEPS_SQUARED
                - sq(DELTA_TOWER3_X_STEPS - cartesianPosSteps[X_AXIS])
                - sq(DELTA_TOWER3_Y_STEPS - cartesianPosSteps[Y_AXIS])) >= 0)
        deltaPosSteps[Z_AXIS] = sqrt(temp) + cartesianPosSteps[Z_AXIS];
    else
        return 0;

    return 1;
}

void PrintLine::calculate_dir_delta(long difference[], byte *dir, long delta[])
{
    *dir = 0;
    //Find direction
    for(byte i=0; i < 4; i++)
    {
        if(difference[i]>=0)
        {
            delta[i] = difference[i];
            *dir |= 1<<i;
        }
        else
        {
            delta[i] = -difference[i];
        }
        if(delta[i]) *dir |= 16<<i;
    }
    if(Printer::extrudeMultiply!=100)
    {
        delta[3]=(long)((delta[3]*(float)Printer::extrudeMultiply)*0.01f);
    }
}

byte PrintLine::calculate_distance(float axis_diff[], byte dir, float *distance)
{
    // Calculate distance depending on direction
    if(dir & 112)
    {
        if(dir & 64)
        {
            *distance = sqrt(axis_diff[0] * axis_diff[0] + axis_diff[1] * axis_diff[1] + axis_diff[2] * axis_diff[2]);
        }
        else
        {
            *distance = sqrt(axis_diff[0] * axis_diff[0] + axis_diff[1] * axis_diff[1]);
        }
    }
    else if(dir & 128)
        *distance = fabs(axis_diff[3]);
    else
    {
        return 0; // no steps to take, we are finished
    }
    return 1;
}

#ifdef SOFTWARE_LEVELING
void PrintLine::calculate_plane(long factors[], long p1[], long p2[], long p3[])
{
    factors[0] = p1[1] * (p2[2] - p3[2]) + p2[1] * (p3[2] - p1[2]) + p3[1] * (p1[2] - p2[2]);
    factors[1] = p1[2] * (p2[0] - p3[0]) + p2[2] * (p3[0] - p1[0]) + p3[2] * (p1[0] - p2[0]);
    factors[2] = p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) + p3[0] * (p1[1] - p2[1]);
    factors[3] = p1[0] * ((p2[1] * p3[2]) - (p3[1] * p2[2])) + p2[0] * ((p3[1] * p1[2]) - (p1[1] * p3[2])) + p3[0] * ((p1[1] * p2[2]) - (p2[1] * p1[2]));
}

float PrintLine::calc_zoffset(long factors[], long pointX, long pointY)
{
    return (factors[3] - factors[0] * pointX - factors[1] * pointY) / (float) factors[2];
}
#endif

inline void PrintLine::queue_E_move(long e_diff,byte check_endstops,byte pathOptimize)
{
    Printer::flag0 &= ~PRINTER_FLAG0_STEPPER_DISABLED; // Motor is enabled now
    while(lines_count>=MOVE_CACHE_SIZE)   // wait for a free entry in movement cache
    {
        GCode::readFromSerial();
        Commands::checkForPeriodicalActions();
    }
    byte newPath=insertWaitMovesIfNeeded(pathOptimize, 0);
    PrintLine *p = getNextWriteLine();
    float axis_diff[4]; // Axis movement in mm
    if(check_endstops) p->checkEndstops();
    else p->flags = 0;
    p->joinFlags = 0;
    if(!pathOptimize) p->setEndSpeedFixed(true);
    p->dir = 0;
    //Find direction
    for(byte i=0; i< 3; i++)
    {
        p->delta[i] = 0;
        axis_diff[i] = 0;
    }
    axis_diff[3] = e_diff*Printer::invAxisStepsPerMM[3];
    if (e_diff >= 0)
    {
        p->delta[3] = e_diff;
        p->dir = 0x88;
    }
    else
    {
        p->delta[3] = -e_diff;
        p->dir = 0x80;
    }
    if(Printer::extrudeMultiply!=100)
    {
        //p->delta[3]=(p->delta[3]*printer_state.extrudeMultiply)/100;
        p->delta[3]=(long)((p->delta[3]*(float)Printer::extrudeMultiply)*0.01f);
    }
    Printer::currentPositionSteps[3] = Printer::destinationSteps[3];

    p->numDeltaSegments = 0;
    //Define variables that are needed for the Bresenham algorithm. Please note that  Z is not currently included in the Bresenham algorithm.
    p->primaryAxis = 3;
    p->stepsRemaining = p->delta[3];
    p->distance = fabs(axis_diff[3]);
    p->moveID = lastMoveID++;
    p->calculate_move(axis_diff,pathOptimize);
}

/**
  Split a line up into a series of lines with at most MAX_DELTA_SEGMENTS_PER_LINE delta segments.
  @param check_endstops Check endstops during the move.
  @param pathOptimize Run the path optimizer.
  @param delta_step_rate delta step rate in segments per second for the move.
*/
void PrintLine::split_delta_move(byte check_endstops,byte pathOptimize, byte softEndstop)
{
    if (softEndstop && Printer::destinationSteps[2] < 0) Printer::destinationSteps[2] = 0;
    long difference[NUM_AXIS];
    float axis_diff[5]; // Axis movement in mm. Virtual axis in 4;
    for(byte i=0; i < NUM_AXIS; i++)
    {
        difference[i] = Printer::destinationSteps[i] - Printer::currentPositionSteps[i];
        axis_diff[i] = fabs(difference[i] * Printer::invAxisStepsPerMM[i]);
    }
    Printer::filamentPrinted+=axis_diff[3];

#if max_software_endstop_r == true
// TODO - Implement radius checking
// I'm guessing I need the floats to prevent overflow. This is pretty horrible.
// The NaN checking in the delta calculation routine should be enough
//float a = difference[0] * difference[0] + difference[1] * difference[1];
//float b = 2 * (difference[0] * printer_state.currentPositionSteps[0] + difference[1] * printer_state.currentPositionSteps[1]);
//float c = printer_state.currentPositionSteps[0] * printer_state.currentPositionSteps[0] + printer_state.currentPositionSteps[1] * printer_state.currentPositionSteps[1] - r * r;
//float disc = b * b - 4 * a * c;
//if (disc >= 0) {
//    float t = (-b + (float)sqrt(disc)) / (2 * a);
//    printer_state.destinationSteps[0] = (long) printer_state.currentPositionSteps[0] + difference[0] * t;
//    printer_state.destinationSteps[1] = (long) printer_state.currentPositionSteps[1] + difference[1] * t;
// }
#endif

    float save_distance;
    byte save_dir;
    long save_delta[4];
    calculate_dir_delta(difference, &save_dir, save_delta);
    if (!calculate_distance(axis_diff, save_dir, &save_distance))
        return;

    if (!(save_dir & 112))
    {
        queue_E_move(difference[3],check_endstops,pathOptimize);
        return;
    }

    int segment_count;

    if (save_dir & 48)
    {
        // Compute number of seconds for move and hence number of segments needed
        //float seconds = 100 * save_distance / (Printer::feedrate * Printer::feedrateMultiply); multiply in feedrate included
        float seconds = save_distance / Printer::feedrate;
#ifdef DEBUG_SPLIT
        Com::printFLN(Com::tDBGDeltaSeconds, seconds);
#endif
        segment_count = RMath::max(1, int(((save_dir & 136)==136 ? DELTA_SEGMENTS_PER_SECOND_PRINT : DELTA_SEGMENTS_PER_SECOND_MOVE) * seconds));
    }
    else
    {
        // Optimize pure Z axis move. Since a pure Z axis move is linear all we have to watch out for is unsigned integer overuns in
        // the queued moves;
#ifdef DEBUG_SPLIT
        Com::printFLN(Com::tDBGDeltaZDelta, save_delta[2]);
#endif
        segment_count = (save_delta[2] + (unsigned long)65534) / (unsigned long)65535;
    }
    // Now compute the number of lines needed
    int num_lines = (segment_count + MAX_DELTA_SEGMENTS_PER_LINE - 1)/MAX_DELTA_SEGMENTS_PER_LINE;
    // There could be some error here but it doesn't matter since the number of segments will just be reduced slightly
    int segments_per_line = segment_count / num_lines;

    long start_position[4], fractional_steps[4];
    for (byte i = 0; i < 4; i++)
    {
        start_position[i] = Printer::currentPositionSteps[i];
    }

#ifdef DEBUG_SPLIT
    Com::printFLN(Com::tDBGDeltaSegments, segment_count);
    Com::printFLN(Com::tDBGDeltaNumLines, num_lines);
    Com::printFLN(Com::tDBGDeltaSegmentsPerLine, segments_per_line);
#endif

    Printer::unsetAllSteppersDisabled(); // Motor is enabled now
    waitForXFreeLines(1);

    // Insert dummy moves if necessary
    // Nead to leave at least one slot open for the first split move
    byte newPath=insertWaitMovesIfNeeded(pathOptimize, RMath::min(MOVE_CACHE_SIZE-4,num_lines-1));

    for (int line_number=1; line_number < num_lines + 1; line_number++)
    {
        waitForXFreeLines(1);
        PrintLine *p = &lines[lines_write_pos];
        // Downside a comparison per loop. Upside one less distance calculation and simpler code.
        if (num_lines == 1)
        {
            p->numDeltaSegments = segment_count;
            p->dir = save_dir;
            for (byte i=0; i < 4; i++)
            {
                p->delta[i] = save_delta[i];
                fractional_steps[i] = difference[i];
            }
            p->distance = save_distance;
        }
        else
        {
            for (byte i=0; i < 4; i++)
            {
                Printer::destinationSteps[i] = start_position[i] + (difference[i] * line_number / num_lines);
                fractional_steps[i] = Printer::destinationSteps[i] - Printer::currentPositionSteps[i];
                axis_diff[i] = fabs(fractional_steps[i]*Printer::invAxisStepsPerMM[i]);
            }
            calculate_dir_delta(fractional_steps, &p->dir, p->delta);
            calculate_distance(axis_diff, p->dir, &p->distance);
        }

        p->joinFlags = 0;
        p->moveID = lastMoveID;

        // Only set fixed on last segment
        if (line_number == num_lines && !pathOptimize)
            p->setEndSpeedFixed(true);

        if(check_endstops)
            p->flags = FLAG_CHECK_ENDSTOPS;
        else
            p->flags = 0;

        p->numDeltaSegments = segments_per_line;

        long max_delta_step = p->calculate_delta_segments(softEndstop);

#ifdef DEBUG_SPLIT
        Com::printFLN(Com::tDBGDeltaMaxDS, max_delta_step);
#endif
        long virtual_axis_move = max_delta_step * segments_per_line;
        if (virtual_axis_move == 0 && p->delta[3] == 0)
        {
            if (num_lines!=1)
                Com::printErrorFLN(Com::tDBGDeltaNoMoveinDSegment);
            return;  // Line too short in low precision area
        }
        p->primaryAxis = 4; // Virtual axis will lead bresenham step either way
        if (virtual_axis_move > p->delta[3])   // Is delta move or E axis leading
        {
            p->stepsRemaining = virtual_axis_move;
            axis_diff[4] = virtual_axis_move * Printer::invAxisStepsPerMM[0]; // Steps/unit same as all the towers
            // Virtual axis steps per segment
            p->numPrimaryStepPerSegment = max_delta_step;
        }
        else
        {
            // Round up the E move to get something divisible by segment count which is greater than E move
            p->numPrimaryStepPerSegment = (p->delta[3] + segments_per_line - 1) / segments_per_line;
            p->stepsRemaining = p->numPrimaryStepPerSegment * segments_per_line;
            axis_diff[4] = p->stepsRemaining * Printer::invAxisStepsPerMM[0];
        }
#ifdef DEBUG_SPLIT
        Com::printFLN(Com::tDBGDeltaStepsPerSegment, p->numPrimaryStepPerSegment);
        Com::printFLN(Com::tDBGDeltaVirtualAxisSteps, p->stepsRemaining);
#endif

        p->calculate_move(axis_diff,pathOptimize);
        for (byte i=0; i < 4; i++)
        {
            Printer::currentPositionSteps[i] += fractional_steps[i];
        }
    }
    lastMoveID++; // Will wrap at 255
}

#endif

#if ARC_SUPPORT
// Arc function taken from grbl
// The arc is approximated by generating a huge number of tiny, linear segments. The length of each
// segment is configured in settings.mm_per_arc_segment.
void PrintLine::arc(float *position, float *target, float *offset, float radius, uint8_t isclockwise)
{
    //   int acceleration_manager_was_enabled = plan_is_acceleration_manager_enabled();
    //   plan_set_acceleration_manager_enabled(false); // disable acceleration management for the duration of the arc
    float center_axis0 = position[0] + offset[0];
    float center_axis1 = position[1] + offset[1];
    float linear_travel = 0; //target[axis_linear] - position[axis_linear];
    float extruder_travel = (Printer::destinationSteps[3]-Printer::currentPositionSteps[3])*Printer::invAxisStepsPerMM[3];
    float r_axis0 = -offset[0];  // Radius vector from center to current location
    float r_axis1 = -offset[1];
    float rt_axis0 = target[0] - center_axis0;
    float rt_axis1 = target[1] - center_axis1;
    long xtarget = Printer::destinationSteps[0];
    long ytarget = Printer::destinationSteps[1];
    long ztarget = Printer::destinationSteps[2];
    long etarget = Printer::destinationSteps[3];

    // CCW angle between position and target from circle center. Only one atan2() trig computation required.
    float angular_travel = atan2(r_axis0*rt_axis1-r_axis1*rt_axis0, r_axis0*rt_axis0+r_axis1*rt_axis1);
    if (angular_travel < 0)
    {
        angular_travel += 2*M_PI;
    }
    if (isclockwise)
    {
        angular_travel -= 2*M_PI;
    }

    float millimeters_of_travel = fabs(angular_travel)*radius; //hypot(angular_travel*radius, fabs(linear_travel));
    if (millimeters_of_travel < 0.001)
    {
        return;
    }
    //uint16_t segments = (radius>=BIG_ARC_RADIUS ? floor(millimeters_of_travel/MM_PER_ARC_SEGMENT_BIG) : floor(millimeters_of_travel/MM_PER_ARC_SEGMENT));
    // Increase segment size if printing faster then computation speed allows
    uint16_t segments = (Printer::feedrate>60 ? floor(millimeters_of_travel/RMath::min(MM_PER_ARC_SEGMENT_BIG,Printer::feedrate*0.01666*MM_PER_ARC_SEGMENT)) : floor(millimeters_of_travel/MM_PER_ARC_SEGMENT));
    if(segments == 0) segments = 1;
    /*
      // Multiply inverse feed_rate to compensate for the fact that this movement is approximated
      // by a number of discrete segments. The inverse feed_rate should be correct for the sum of
      // all segments.
      if (invert_feed_rate) { feed_rate *= segments; }
    */
    float theta_per_segment = angular_travel/segments;
    float linear_per_segment = linear_travel/segments;
    float extruder_per_segment = extruder_travel/segments;

    /* Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
       and phi is the angle of rotation. Based on the solution approach by Jens Geisler.
           r_T = [cos(phi) -sin(phi);
                  sin(phi)  cos(phi] * r ;

       For arc generation, the center of the circle is the axis of rotation and the radius vector is
       defined from the circle center to the initial position. Each line segment is formed by successive
       vector rotations. This requires only two cos() and sin() computations to form the rotation
       matrix for the duration of the entire arc. Error may accumulate from numerical round-off, since
       all double numbers are single precision on the Arduino. (True double precision will not have
       round off issues for CNC applications.) Single precision error can accumulate to be greater than
       tool precision in some cases. Therefore, arc path correction is implemented.

       Small angle approximation may be used to reduce computation overhead further. This approximation
       holds for everything, but very small circles and large mm_per_arc_segment values. In other words,
       theta_per_segment would need to be greater than 0.1 rad and N_ARC_CORRECTION would need to be large
       to cause an appreciable drift error. N_ARC_CORRECTION~=25 is more than small enough to correct for
       numerical drift error. N_ARC_CORRECTION may be on the order a hundred(s) before error becomes an
       issue for CNC machines with the single precision Arduino calculations.

       This approximation also allows mc_arc to immediately insert a line segment into the planner
       without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
       a correction, the planner should have caught up to the lag caused by the initial mc_arc overhead.
       This is important when there are successive arc motions.
    */
    // Vector rotation matrix values
    float cos_T = 1-0.5*theta_per_segment*theta_per_segment; // Small angle approximation
    float sin_T = theta_per_segment;

    float arc_target[4];
    float sin_Ti;
    float cos_Ti;
    float r_axisi;
    uint16_t i;
    int8_t count = 0;

    // Initialize the linear axis
    //arc_target[axis_linear] = position[axis_linear];

    // Initialize the extruder axis
    arc_target[3] = Printer::currentPositionSteps[3]*Printer::invAxisStepsPerMM[3];

    for (i = 1; i<segments; i++)
    {
        // Increment (segments-1)

        if((count & 4) == 0)
        {
            GCode::readFromSerial();
            Commands::checkForPeriodicalActions();
            UI_MEDIUM; // do check encoder
        }

        if (count < N_ARC_CORRECTION)  //25 pieces
        {
            // Apply vector rotation matrix
            r_axisi = r_axis0*sin_T + r_axis1*cos_T;
            r_axis0 = r_axis0*cos_T - r_axis1*sin_T;
            r_axis1 = r_axisi;
            count++;
        }
        else
        {
            // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
            // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
            cos_Ti  = cos(i*theta_per_segment);
            sin_Ti  = sin(i*theta_per_segment);
            r_axis0 = -offset[0]*cos_Ti + offset[1]*sin_Ti;
            r_axis1 = -offset[0]*sin_Ti - offset[1]*cos_Ti;
            count = 0;
        }

        // Update arc_target location
        arc_target[0] = center_axis0 + r_axis0;
        arc_target[1] = center_axis1 + r_axis1;
        //arc_target[axis_linear] += linear_per_segment;
        arc_target[3] += extruder_per_segment;
        Printer::moveToReal(arc_target[0],arc_target[1],IGNORE_COORDINATE,arc_target[3],IGNORE_COORDINATE);
    }
    // Ensure last segment arrives at target location.
    Printer::moveToReal(target[0],target[1],IGNORE_COORDINATE,target[3],IGNORE_COORDINATE);
}
#endif



/**
  Moves the stepper motors one step. If the last step is reached, the next movement is started.
  The function must be called from a timer loop. It returns the time for the next call.
  This is a modified version that implements a bresenham 'multi-step' algorithm where the dominant
  cartesian axis steps may be less than the changing dominant delta axis.
*/
#if DRIVE_SYSTEM==3
int lastblk=-1;
long cur_errupd;
//#define DEBUG_DELTA_TIMER
// Current delta segment
DeltaSegment *curd;
// Current delta segment primary error increment
long curd_errupd, stepsPerSegRemaining;
long PrintLine::bresenhamStep() // Version for delta printer
{
    if(cur == 0)
    {
        HAL::allowInterrupts();
        setCurrentLine();
        if(cur->isBlocked())   // This step is in computation - shouldn't happen
        {
            if(lastblk!=(int)cur)
            {
                lastblk = (int)cur;
                Com::printFLN(Com::tBLK,lines_count);
            }
            cur = 0;
            return 2000;
        }
        lastblk = -1;
#ifdef INCLUDE_DEBUG_NO_MOVE
        if(DEBUG_NO_MOVES)   // simulate a move, but do nothing in reality
        {
            removeCurrentLineForbidInterrupt();
            return 1000;
        }
#endif
        if(cur->isWarmUp())
        {
            // This is a warmup move to initalize the path planner correctly. Just waste
            // a bit of time to get the planning up to date.
            if(lines_count<=cur->getWaitForXLinesFilled())
            {
                cur=0;
                return 2000;
            }
            long wait = cur->getWaitTicks();
            removeCurrentLineForbidInterrupt();
            return(wait); // waste some time for path optimization to fill up
        } // End if WARMUP
        if(cur->isEMove()) Extruder::enable();
        cur->fixStartAndEndSpeed();
        // Set up delta segments
        if (cur->numDeltaSegments)
        {
            // If there are delta segments point to them here
            curd = &segments[cur->deltaSegmentReadPos++];
            if (cur->deltaSegmentReadPos >= DELTA_CACHE_SIZE) cur->deltaSegmentReadPos=0;
            // Enable axis - All axis are enabled since they will most probably all be involved in a move
            // Since segments could involve different axis this reduces load when switching segments and
            // makes disabling easier.
            Printer::enableXStepper();
            Printer::enableYStepper();
            Printer::enableZStepper();

            // Copy across movement into main direction flags so that endstops function correctly
            cur->dir |= curd->dir;
            // Initialize bresenham for the first segment
            if (cur->isFullstepping())
            {
                cur->error[0] = cur->error[1] = cur->error[2] = cur->numPrimaryStepPerSegment>>1;
                curd_errupd = cur->numPrimaryStepPerSegment;
            }
            else
            {
                cur->error[0] = cur->error[1] = cur->error[2] = cur->numPrimaryStepPerSegment;
                curd_errupd = cur->numPrimaryStepPerSegment = cur->numPrimaryStepPerSegment<<1;
            }
            stepsPerSegRemaining = cur->numPrimaryStepPerSegment;
        }
        else curd=0;
        cur_errupd = (cur->isFullstepping() ? cur->stepsRemaining : cur->stepsRemaining << 1);

        if(!cur->areParameterUpToDate())  // should never happen, but with bad timings???
        {
            cur->updateStepsParameter();
        }
        Printer::vMaxReached = cur->vStart;
        Printer::stepNumber=0;
        Printer::timer = 0;
        HAL::forbidInterrupts();
        //Determine direction of movement
        if (curd)
        {
            Printer::setXDirection(curd->dir & 1);
            Printer::setYDirection(curd->dir & 2);
            Printer::setZDirection(curd->dir & 4);
        }
#if defined(USE_ADVANCE)
        if(!Printer::isAdvanceActivated()) // Set direction if no advance/OPS enabled
#endif
        Extruder::setDirection(cur->isEPositiveMove());
#ifdef USE_ADVANCE
        long h = HAL::mulu16xu16to32(cur->vStart,cur->advanceL);
        int tred = ((
#ifdef ENABLE_QUADRATIC_ADVANCE
                        (Printer::advance_executed = cur->advanceStart)+
#endif
                        h)>>16);
        Printer::extruderStepsNeeded+=tred-Printer::advance_steps_set;
        Printer::advance_steps_set = tred;
#endif
        if(Printer::waslasthalfstepping && cur->isFullstepping())   // Switch halfstepping -> full stepping
        {
            Printer::waslasthalfstepping = 0;
            return Printer::interval*3; // Wait an other 150% from last half step to make the 100% full
        }
        else if(!Printer::waslasthalfstepping && !cur->isFullstepping())     // Switch full to half stepping
        {
            Printer::waslasthalfstepping = 1;
        }
        else
            return Printer::interval; // Wait an other 50% from last step to make the 100% full
    } // End cur=0
    HAL::allowInterrupts();

    /* For halfstepping, we divide the actions into even and odd actions to split
       time used per loop. */
    byte do_even = cur->halfstep & 6;
    byte do_odd = cur->halfstep & 5;
    if(cur->halfstep!=4) cur->halfstep = 3-(cur->halfstep);
    HAL::forbidInterrupts();
    if(do_even)
    {
        if(cur->isCheckEndstops() && (curd != 0))
        {
#if X_MAX_PIN>-1 && MAX_HARDWARE_ENDSTOP_X
            if((curd->dir & 17)==17) if(READ(X_MAX_PIN) != ENDSTOP_X_MAX_INVERTING)
                {
                    curd->dir&=~16;
                    cur->setXMoveFinished();
                }
#endif
#if Y_MAX_PIN>-1 && MAX_HARDWARE_ENDSTOP_Y
            if((curd->dir & 34)==34) if(READ(Y_MAX_PIN) != ENDSTOP_Y_MAX_INVERTING)
                {
                    curd->dir&=~32;
                    cur->setYMoveFinished();
                }
#endif
#if Z_MAX_PIN>-1 && MAX_HARDWARE_ENDSTOP_Z
            if((curd->dir & 68)==68) if(READ(Z_MAX_PIN)!= ENDSTOP_Z_MAX_INVERTING)
                {
                    curd->dir&=~64;
                    cur->setZMoveFinished();
                }
#endif
        }
    }
    byte max_loops = (Printer::stepsPerTimerCall<=cur->stepsRemaining ? Printer::stepsPerTimerCall : cur->stepsRemaining);
    if(cur->stepsRemaining>0)
    {
        for(byte loop=0; loop<max_loops; loop++)
        {
            if(loop>0)
#if STEPPER_HIGH_DELAY>0
                HAL::delayMicroseconds(STEPPER_HIGH_DELAY+DOUBLE_STEP_DELAY);
#else
                HAL::delayMicroseconds(DOUBLE_STEP_DELAY);
#endif
            if(cur->isEMove())
            {
                if((cur->error[3] -= cur->delta[3]) < 0)
                {
#if defined(USE_ADVANCE)
                    if((Printer::flag0 & PRINTER_FLAG0_SEPERATE_EXTRUDER_INT))   // Use interrupt for movement
                    {
                        if(cur->isEPositiveMove())
                            Printer::extruderStepsNeeded++;
                        else
                            Printer::extruderStepsNeeded--;
                    }
                    else
#endif
                        Extruder::step();
                    cur->error[3] += cur_errupd;
                }
            }
            if (curd)
            {
                // Take delta steps
                if(curd->dir & 16)
                {
                    if((cur->error[0] -= curd->deltaSteps[0]) < 0)
                    {
                        cur->startXStep();
                        cur->error[0] += curd_errupd;
                    }
                }

                if(curd->dir & 32)
                {
                    if((cur->error[1] -= curd->deltaSteps[1]) < 0)
                    {
                        cur->startYStep();
                        cur->error[1] += curd_errupd;
                    }
                }

                if(curd->dir & 64)
                {
                    if((cur->error[2] -= curd->deltaSteps[2]) < 0)
                    {
                        cur->startZStep();
                        Printer::countZSteps += ( cur->dir & 4 ? 1 : -1 );
                        cur->error[2] += curd_errupd;
#ifdef DEBUG_STEPCOUNT
                        cur->totalStepsRemaining--;
#endif
                    }
                }

#if STEPPER_HIGH_DELAY>0
                HAL::delayMicroseconds(STEPPER_HIGH_DELAY);
#endif
                Printer::endXYZSteps();
                stepsPerSegRemaining--;
                if (!stepsPerSegRemaining)
                {
                    cur->numDeltaSegments--;
                    if (cur->numDeltaSegments)
                    {
                        // Get the next delta segment
                        curd = &segments[cur->deltaSegmentReadPos++];
                        if (cur->deltaSegmentReadPos >= DELTA_CACHE_SIZE) cur->deltaSegmentReadPos=0;

                        // Initialize bresenham for this segment (numPrimaryStepPerSegment is already correct for the half step setting)
                        cur->error[0] = cur->error[1] = cur->error[2] = cur->numPrimaryStepPerSegment>>1;

                        // Reset the counter of the primary steps. This is initialized in the line
                        // generation so don't have to do this the first time.
                        stepsPerSegRemaining = cur->numPrimaryStepPerSegment;

                        // Change direction if necessary
                        Printer::setXDirection(curd->dir & 1);
                        Printer::setYDirection(curd->dir & 2);
                        Printer::setZDirection(curd->dir & 4);
                    }
                    else
                        curd=0;// Release the last segment
                    delta_segment_count--;
                }
            }
#if defined(USE_ADVANCE)
            if(Printer::isAdvanceActivated()) // Use interrupt for movement
#endif
                Extruder::unstep();
        } // for loop
        if(do_odd)
        {
            HAL::allowInterrupts(); // Allow interrupts for other types, timer1 is still disabled
#ifdef RAMP_ACCELERATION
            //If acceleration is enabled on this move and we are in the acceleration segment, calculate the current interval
            if (cur->moveAccelerating())
            {
                Printer::vMaxReached = HAL::ComputeV(Printer::timer,cur->facceleration)+cur->vStart;
                if(Printer::vMaxReached>cur->vMax) Printer::vMaxReached = cur->vMax;
                unsigned int v = Printer::updateStepsPerTimerCall(Printer::vMaxReached);
                Printer::interval = HAL::CPUDivU2(v);
                Printer::timer+=Printer::interval;
                cur->updateAdvanceSteps(Printer::vMaxReached,max_loops,true);
            }
            else if (cur->moveDecelerating())     // time to slow down
            {
                unsigned int v = HAL::ComputeV(Printer::timer,cur->facceleration);
                if (v > Printer::vMaxReached)   // if deceleration goes too far it can become too large
                    v = cur->vEnd;
                else
                {
                    v=Printer::vMaxReached-v;
                    if (v<cur->vEnd) v = cur->vEnd; // extra steps at the end of desceleration due to rounding erros
                }
                cur->updateAdvanceSteps(v,max_loops,false);
                v = Printer::updateStepsPerTimerCall(v);
                Printer::interval = HAL::CPUDivU2(v);
                Printer::timer+=Printer::interval;
            }
            else
            {
                // If we had acceleration, we need to use the latest vMaxReached and interval
                // If we started full speed, we need to use cur->fullInterval and vMax
#ifdef USE_ADVANCE
                unsigned int v;
                if(!cur->accelSteps)
                {
                    v = cur->vMax;
                }
                else
                {
                    v = Printer::vMaxReached;
                }
#ifdef ENABLE_QUADRATIC_ADVANCE
                long h=HAL::mulu16xu16to32(cur->advanceL,v);
                int tred = ((Printer::advance_executed+h)>>16);
                HAL::forbidInterrupts();
                Printer::extruderStepsNeeded+=tred-Printer::advance_steps_set;
                Printer::advance_steps_set = tred;
                HAL::allowInterrupts();
#else
                int tred=mulu6xu16shift16(cur->advanceL,v);
                HAL::forbidInterrupts();
                Printer::extruderStepsNeeded+=tred-Printer::advance_steps_set;
                Printer::advance_steps_set = tred;
                HAL::allowInterrupts();
#endif
#endif
                if(!cur->accelSteps)
                {
                    if(cur->vMax>STEP_DOUBLER_FREQUENCY)
                    {
#if ALLOW_QUADSTEPPING
                        if(cur->vMax>STEP_DOUBLER_FREQUENCY*2)
                        {
                            Printer::stepsPerTimerCall = 4;
                            Printer::interval = cur->fullInterval>>2;
                        }
                        else
                        {
                            Printer::stepsPerTimerCall = 2;
                            Printer::interval = cur->fullInterval>>1;
                        }
#else
                        Printer::stepsPerTimerCall = 2;
                        Printer::interval = cur->fullInterval>>1;
#endif
                    }
                    else
                    {
                        Printer::stepsPerTimerCall = 1;
                        Printer::interval = cur->fullInterval;
                    }
                }
            }
#else
            Printer::interval = cur->fullInterval; // without RAMPS always use full speed
#endif
        } // do_odd
        if(do_even)
        {
            Printer::stepNumber+=max_loops;
            PrintLine::cur->stepsRemaining-=max_loops;
        }

    } // stepsRemaining
    long interval = (cur->isFullstepping() ? Printer::interval : Printer::interval>>1);
    if(do_even &&(cur->stepsRemaining<=0 || cur->isNoMove()))   // line finished
    {
//			out.println_int_P(PSTR("Line finished: "), (int) PrintLine::cur->numDeltaSegments);
//			out.println_int_P(PSTR("DSC: "), (int) delta_segment_count);
//			out.println_P(PSTR("F"));

        // Release remaining delta segments
#ifdef DEBUG_STEPCOUNT
        if(cur->totalStepsRemaining)
        {
            out.println_long_P(PSTR("Missed steps:"), cur->totalStepsRemaining);
            out.println_long_P(PSTR("Step/seg r:"), stepsPerSegRemaining);
            out.println_int_P(PSTR("NDS:"), (int) cur->numDeltaSegments);
            out.println_int_P(PSTR("HS:"), (int) cur->halfstep);
        }
#endif
        removeCurrentLineForbidInterrupt();
        delta_segment_count -= cur->numDeltaSegments; // should always be zero
        Printer::disableAllowedStepper();
        if(lines_count==0) UI_STATUS(UI_TEXT_IDLE);
        interval = Printer::interval = interval>>1; // 50% of time to next call to do cur=0
        DEBUG_MEMORY;
    } // Do even
    return interval;
}
#else
/**
  Moves the stepper motors one step. If the last step is reached, the next movement is started.
  The function must be called from a timer loop. It returns the time for the next call.

  Normal non delta algorithm
*/
int lastblk=-1;
long cur_errupd;
long PrintLine::bresenhamStep() // version for cartesian printer
{
    if(cur == 0) // Initalize new line
    {
        HAL::allowInterrupts();
        ANALYZER_ON(ANALYZER_CH0);
        setCurrentLine();
        if(cur->isBlocked())   // This step is in computation - shouldn't happen
        {
            /*if(lastblk!=(int)cur) // can cause output errors!
            {
                lastblk = (int)cur;
                Com::printFLN(Com::tBLK,lines_count);
            }*/
            cur = 0;
            return 2000;
        }
        lastblk = -1;
#ifdef INCLUDE_DEBUG_NO_MOVE
        if(DEBUG_NO_MOVES)   // simulate a move, but do nothing in reality
        {
            removeCurrentLineForbidInterrupt();
            return 1000;
        }
#endif
        ANALYZER_OFF(ANALYZER_CH0);
        if(cur->isWarmUp())
        {
            // This is a warmup move to initalize the path planner correctly. Just waste
            // a bit of time to get the planning up to date.
            if(lines_count<=cur->getWaitForXLinesFilled())
            {
                cur=0;
                return 2000;
            }
            long wait = cur->getWaitTicks();
            removeCurrentLineForbidInterrupt();
            return(wait); // waste some time for path optimization to fill up
        } // End if WARMUP
        //Only enable axis that are moving. If the axis doesn't need to move then it can stay disabled depending on configuration.
#ifdef XY_GANTRY
        if(cur->isXOrYMove())
        {
            Printer::enableXStepper();
            Printer::enableYStepper();
        }
#else
        if(cur->isXMove()) Printer::enableXStepper();
        if(cur->isYMove()) Printer::enableYStepper();
#endif
        if(cur->isZMove())
        {
            Printer::enableZStepper();
        }
        if(cur->isEMove()) Extruder::enable();
        cur->fixStartAndEndSpeed();
        HAL::allowInterrupts();
        cur_errupd = (cur->isFullstepping() ? cur->delta[cur->primaryAxis] : cur->delta[cur->primaryAxis]<<1);;
        if(!cur->areParameterUpToDate())  // should never happen, but with bad timings???
        {
            cur->updateStepsParameter();
        }
        Printer::vMaxReached = cur->vStart;
        Printer::stepNumber=0;
        Printer::timer = 0;
        HAL::forbidInterrupts();
        //Determine direction of movement,check if endstop was hit
#if !defined(XY_GANTRY)
        Printer::setXDirection(cur->isXPositiveMove());
        Printer::setYDirection(cur->isYPositiveMove());
#else
        long gdx = (cur->dir & 1 ? cur->delta[0] : -cur->delta[0]); // Compute signed difference in steps
        long gdy = (cur->dir & 2 ? cur->delta[1] : -cur->delta[1]);
        Printer::setXDirection(gdx+gdy>=0);
#if DRIVE_SYSTEM==1
        Printer::setYDirection(gdx>gdy);
#elif DRIVE_SYSTEM==2
        Printer::setYDirection(gdx<=gdy);
#endif
#endif
        Printer::setZDirection(cur->isZPositiveMove());
#if defined(USE_ADVANCE)
        if(!Printer::isAdvanceActivated()) // Set direction if no advance/OPS enabled
#endif
            Extruder::setDirection(cur->isEPositiveMove());
#ifdef USE_ADVANCE
        long h = HAL::mulu16xu16to32(cur->vStart,cur->advanceL);
        int tred = ((
#ifdef ENABLE_QUADRATIC_ADVANCE
                        (Printer::advance_executed = cur->advanceStart)+
#endif
                        h)>>16);
        Printer::extruderStepsNeeded+=tred-Printer::advance_steps_set;
        Printer::advance_steps_set = tred;
#endif
        if(Printer::waslasthalfstepping && cur->isFullstepping())   // Switch halfstepping -> full stepping
        {
            Printer::waslasthalfstepping = 0;
            return Printer::interval*3; // Wait an other 150% from last half step to make the 100% full
        }
        else if(!Printer::waslasthalfstepping && !cur->isFullstepping())     // Switch full to half stepping
        {
            Printer::waslasthalfstepping = 1;
        }
        else
            return Printer::interval; // Wait an other 50% from last step to make the 100% full
    } // End cur=0
    HAL::allowInterrupts();
    /* For halfstepping, we divide the actions into even and odd actions to split
       time used per loop. */
    byte do_even = cur->halfstep & 6;
    byte do_odd = cur->halfstep & 5;
    if(cur->halfstep!=4) cur->halfstep = 3-(cur->halfstep);
    HAL::forbidInterrupts();
    if(do_even) cur->checkEndstops();
    byte max_loops = RMath::min((long)Printer::stepsPerTimerCall,cur->stepsRemaining);
    if(cur->stepsRemaining>0)
    {
        for(byte loop=0; loop<max_loops; loop++)
        {
            ANALYZER_ON(ANALYZER_CH1);
            if(loop>0)
                HAL::delayMicroseconds(STEPPER_HIGH_DELAY+DOUBLE_STEP_DELAY);
            if(cur->isEMove())
            {
                if((cur->error[3] -= cur->delta[3]) < 0)
                {
#if defined(USE_ADVANCE)
                    if(Printer::isAdvanceActivated())   // Use interrupt for movement
                    {
                        if(cur->isEPositiveMove())
                            Printer::extruderStepsNeeded++;
                        else
                            Printer::extruderStepsNeeded--;
                    }
                    else
#endif
                        Extruder::step();
                    cur->error[3] += cur_errupd;
                }
            }
            if(cur->isXMove())
            {
                if((cur->error[0] -= cur->delta[0]) < 0)
                {
                    cur->startXStep();
                    cur->error[0] += cur_errupd;
                }
            }
            if(cur->isYMove())
            {
                if((cur->error[1] -= cur->delta[1]) < 0)
                {
                    cur->startYStep();
                    cur->error[1] += cur_errupd;
                }
            }
#if defined(XY_GANTRY)
            Printer::executeXYGantrySteps();
#endif

            if(cur->isZMove())
            {
                if((cur->error[2] -= cur->delta[2]) < 0)
                {
                    cur->startZStep();
                    cur->error[2] += cur_errupd;
#ifdef DEBUG_STEPCOUNT
                    cur->totalStepsRemaining--;
#endif
                }
            }
#if STEPPER_HIGH_DELAY>0
            HAL::delayMicroseconds(STEPPER_HIGH_DELAY);
#endif
#if defined(USE_ADVANCE)
            if(!Printer::isAdvanceActivated()) // Use interrupt for movement
#endif
                Extruder::unstep();
            Printer::endXYZSteps();
        } // for loop
        if(do_odd)  // Update timings
        {
            HAL::allowInterrupts(); // Allow interrupts for other types, timer1 is still disabled
#ifdef RAMP_ACCELERATION
            //If acceleration is enabled on this move and we are in the acceleration segment, calculate the current interval
            if (cur->moveAccelerating())   // we are accelerating
            {
                Printer::vMaxReached = HAL::ComputeV(Printer::timer,cur->facceleration)+cur->vStart;
                if(Printer::vMaxReached>cur->vMax) Printer::vMaxReached = cur->vMax;
                unsigned int v = Printer::updateStepsPerTimerCall(Printer::vMaxReached);
                Printer::interval = HAL::CPUDivU2(v);
                Printer::timer+=Printer::interval;
                cur->updateAdvanceSteps(Printer::vMaxReached,max_loops,true);
            }
            else if (cur->moveDecelerating())     // time to slow down
            {
                unsigned int v = HAL::ComputeV(Printer::timer,cur->facceleration);
                if (v > Printer::vMaxReached)   // if deceleration goes too far it can become too large
                    v = cur->vEnd;
                else
                {
                    v=Printer::vMaxReached-v;
                    if (v<cur->vEnd) v = cur->vEnd; // extra steps at the end of desceleration due to rounding erros
                }
                cur->updateAdvanceSteps(v,max_loops,false); // needs original v
                v = Printer::updateStepsPerTimerCall(v);
                Printer::interval = HAL::CPUDivU2(v);
                Printer::timer+=Printer::interval;
            }
            else // full speed reached
            {
                // constant speed reached
                if(cur->vMax>STEP_DOUBLER_FREQUENCY)
                {
#if ALLOW_QUADSTEPPING
                    if(cur->vMax>STEP_DOUBLER_FREQUENCY*2)
                    {
                        Printer::stepsPerTimerCall = 4;
                        Printer::interval = cur->fullInterval<<2;
                    }
                    else
                    {
                        Printer::stepsPerTimerCall = 2;
                        Printer::interval = cur->fullInterval<<1;
                    }
#else
                    Printer::stepsPerTimerCall = 2;
                    Printer::interval = cur->fullInterval<<1;
#endif
                }
                else
                {
                    Printer::stepsPerTimerCall = 1;
                    Printer::interval = cur->fullInterval;
                }
            }
#else
            Printer::stepsPerTimerCall = 1;
            Printer::interval = cur->fullInterval; // without RAMPS always use full speed
#endif // RAMP_ACCELERATION
        } // do_odd
        if(do_even)
        {
            Printer::stepNumber+=max_loops;
            cur->stepsRemaining-=max_loops;
        }

    } // stepsRemaining
    long interval;
    if(!cur->isFullstepping()) interval = (Printer::interval>>1); // time to come back
    else interval = Printer::interval;
    if(do_even && (cur->stepsRemaining<=0 || cur->isNoMove()))   // line finished
    {
#ifdef DEBUG_STEPCOUNT
        if(cur->totalStepsRemaining)
        {
            Com::printF(Com::tDBGMissedSteps,cur->totalStepsRemaining);
            Com::printFLN(Com::tComma,cur->stepsRemaining);
        }
#endif
        removeCurrentLineForbidInterrupt();
        Printer::disableAllowedStepper();
        if(lines_count==0) UI_STATUS(UI_TEXT_IDLE);
        interval = Printer::interval = interval>>1; // 50% of time to next call to do cur=0
        DEBUG_MEMORY;
    } // Do even
    return interval;
}
#endif
