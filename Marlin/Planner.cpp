#include "Planner.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>

/* TODO: replace with real functions from Marlin.h */
#define enable_x()
#define enable_y()
#define enable_z()
#define enable_e()

/* TODO: Seperate buffer from Planner and make only Buffer singleton */
Planner planner; /* Planner for now is a singleton and therefore will be created here. Need to find a better pattern later. */

Planner::Planner(void)
{

}

/**
 * Returns the index of the next block in the ring buffer
 * @param blockIndex current block index
 * @return Returns the index of next block
 * TODO: If possible replace blockIndex by private variable
 */
Planner_blockIndex_t Planner::getNextBlockIndex(Planner_blockIndex_t blockIndex)
{
  blockIndex++;
  if (blockIndex == BLOCK_BUFFER_SIZE) {
      blockIndex = 0;
  }
  return(blockIndex);
}

/**
 * Returns the index of the previous block in the ring buffer
 * @param blockIndex current block index
 * @return Returns the index of previous block
 * TODO: If possible replace blockIndex by private variable
 */
Planner_blockIndex_t Planner::getPreviousBlockIndex(Planner_blockIndex_t blockIndex)
{
  if (blockIndex == 0) {
      blockIndex = BLOCK_BUFFER_SIZE;
  }
  blockIndex--;
  return(blockIndex);
}


// Called when the current block is no longer needed. Discards the block and makes the memory
// availible for new blocks.
FORCE_INLINE void Planner::discardCurrentBlock()
{
  if (blockBufferHead != blockBufferTail) {
	  blockBufferTail = (blockBufferTail + 1) & (BLOCK_BUFFER_SIZE - 1);
  }
}

// Gets the current block. Returns NULL if buffer empty
FORCE_INLINE Planner_Block_t *Planner::getCurrentBlock( void )
{
	if (blockBufferHead != blockBufferTail) {
		Planner_Block_t *block = &(blockBuffer[blockBufferTail]);
		block->busy = true;
		return(block);
	}
	else {
		return(NULL);
	}
}

/**
 * Reads position from stepper and returns them
 * @return Current position of all axis
 */
Kinematics_AxisCoordinates_t Planner::getPosition(void)
{
  Kinematics_AxisCoordinates_t t = {0, 0, 0, 0};
  return t;
}


/**
 * Sets position of stepper to position given in variable position without moving them. Needed
 * for example during homing or calibration.
 * @param newPosition Actual position of the stepper in axis coordinate system
 */
void Planner::setPosition(Kinematics_AxisCoordinates_t newPosition)
{

}

/**
 * Adds movement to position to buffer if there is still space available in buffer. Position is
 * given in axis coordinate system.
 * @param [in] target Position in steps to move to in axis coordinate system
 * @param [in] feedRate Feedrate in steps/sec for this move
 * @param [in] activeExtruder Extruder used during this move
 * @return true if position was added to the buffer, false if not because buffer is full right now
 * @note Part of former plan_buffer_line from planner.cpp
 */
bool Planner::addMovement(Kinematics_AxisCoordinates_t target, uint32_t feedRate, uint8_t activeExtruder)
{
  static uint32_t previousNominalSpeed;
  // Calculate the buffer head after we push this byte
  Planner_blockIndex_t nextBufferHead = getNextBlockIndex(blockBufferHead);
  if (blockBufferTail != nextBufferHead)
    {
      // Prepare to set up new block
      Planner_Block_t *block = &(blockBuffer[blockBufferHead]);

      // Mark block as not busy (Not executed by the stepper interrupt)
      block->busy = false;

      block->steps.x = labs(target.x-lastGivenPosition.x);
      block->steps.y = labs(target.y-lastGivenPosition.y);
      block->steps.z = labs(target.z-lastGivenPosition.z);
      block->steps.e = labs(target.e-lastGivenPosition.e);
      block->step_event_count = max(block->steps.x, max(block->steps.y, max(block->steps.z, block->steps.e)));

      // Only continue if this is a non zero-length block, if not move will be merged with next move
      if (block->step_event_count >= dropsegments)
        {
          // Compute direction bits for this block
          block->direction.value = 0;
          if (target.x < lastGivenPosition.x)
          {
              block->direction.bits.x = 1;
          }
          if (target.y < lastGivenPosition.y)
          {
              block->direction.bits.y = 1;
          }
          if (target.z < lastGivenPosition.z)
          {
              block->direction.bits.z = 1;
          }
          if (target.e < lastGivenPosition.e)
          {
              block->direction.bits.e = 1;
          }

          block->active_extruder = activeExtruder;

          //enable active axes
          if(block->steps.x != 0) enable_x();
          if(block->steps.y != 0) enable_y();
          if(block->steps.z != 0) enable_z();
          if(block->steps.e != 0) enable_e();
          /* TODO: Add DISABLE_INACTIVE_EXTRUDER stuff here */

          if ( block->steps.x <= dropsegments && block->steps.y <= dropsegments && block->steps.z <= dropsegments )
            {
              block->totalTravelSteps = fabs(block->steps.e);
            }
          else
            {
              block->totalTravelSteps = sqrt(sq(block->steps.x) + sq(block->steps.y) + sq(block->steps.z));
            }
          float inverseSteps = 1.0/block->totalTravelSteps;  // Inverse total steps to remove multiple divides
          // Calculate speed in mm/second for each axis. No divide by zero due to previous checks.
          float inverse_second = feedRate * inverseSteps;

          int moves_queued = (blockBufferHead - blockBufferTail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1);

#ifdef SLOWDOWN
          //  segment time in micro seconds
          unsigned long segment_time = lround(1000000.0/inverse_second);
          if ((moves_queued > 1) && (moves_queued < (BLOCK_BUFFER_SIZE * 0.5)))
            {
              if (segment_time < minsegmenttime)
                { // buffer is draining, add extra time.  The amount of time added increases if the buffer is still emptied more.
                  inverse_second=1000000.0/(segment_time+lround(2*(minsegmenttime-segment_time)/moves_queued));
#ifdef XY_FREQUENCY_LIMIT
                  segment_time = lround(1000000.0/inverse_second);
#endif
                }
            }
#endif //  #ifdef SLOWDOWN

          block->nominal_speed = feedRate; // Set nominal speed to given, possibly corrected, feedRate
          block->nominal_rate = ceil(block->step_event_count * inverse_second); // (step/sec) Always > 0

          // Calculate and limit speed in steps/sec for each axis
          Kinematics_AxisCoordinates_t current_speed;
          float speedFactor = 1.0; //factor <=1 do decrease speed
          current_speed.x = block->steps.x * inverse_second;
          if (current_speed.x > configuration.maxFeedrate.x)
            speedFactor = min(speedFactor, configuration.maxFeedrate.x / current_speed.x);
          current_speed.y = block->steps.y * inverse_second;
          if (current_speed.y > configuration.maxFeedrate.y)
            speedFactor = min(speedFactor, configuration.maxFeedrate.y / current_speed.y);
          current_speed.z = block->steps.z * inverse_second;
          if (current_speed.z > configuration.maxFeedrate.z)
            speedFactor = min(speedFactor, configuration.maxFeedrate.z / current_speed.z);
          current_speed.e = block->steps.e * inverse_second;
          if (current_speed.e > configuration.maxFeedrate.e)
            speedFactor = min(speedFactor, configuration.maxFeedrate.e / current_speed.e);

          /* TODO: Add #ifdef XY_FREQUENCY_LIMIT from planner.cpp here */

          /* Correct all speeds by speedFactor */
          if( speedFactor < 1.0)
          {
              current_speed.x *= speedFactor;
              current_speed.y *= speedFactor;
              current_speed.z *= speedFactor;
              current_speed.e *= speedFactor;
              block->nominal_speed *= speedFactor;
              block->nominal_rate *= speedFactor;
          }

          /* TODO: Add acceleration limiter from planner.cpp here */
          block->acceleration_st = configuration.maxAcceleration;
          //block->acceleration = block->acceleration_st  / steps_per_mm;
          block->acceleration_rate = (long)((float)block->acceleration_st * (16777216.0 / (F_CPU / 8.0)));

          /* Jerk control will control the jerk between two moves. Value of maxEntrySpeed for this block
           * is set depending on the nominalSpeed (aka feedRate) or the last block
           */
          /* TODO: Add jerk control from planner.cpp here. For now we set maxEntrySpeed to half max allowed jerk */
          block->max_entry_speed = configuration.max_xy_jerk/2;

          /* Compute maximum allowed entry speed based on deceleration over complete travel distance down
           * to user-defined MINIMUM_PLANNER_SPEED */
          uint32_t maxAllowableSpeed = maxStartSpeed(-block->acceleration, MINIMUM_PLANNER_SPEED,block->totalTravelSteps);
          block->entry_speed = min(block->max_entry_speed, maxAllowableSpeed);

          // Initialize planner efficiency flags
          // Set flag if block will always reach maximum junction speed regardless of entry/exit speeds.
          // If a block can de/ac-celerate from nominal speed to zero within the length of the block, then
          // the current block and next block junction speeds are guaranteed to always be at their maximum
          // junction speeds in deceleration and acceleration, respectively. This is due to how the current
          // block nominal speed limits both the current and next maximum junction speeds. Hence, in both
          // the reverse and forward planners, the corresponding block junction speed will always be at the
          // the maximum junction speed and may always be ignored for any speed reduction checks.
          if (block->nominal_speed <= maxAllowableSpeed) {
            block->nominal_length_flag = true;
          }
          else {
            block->nominal_length_flag = false;
          }

          block->recalculate_flag = true; // Always calculate trapezoid for new block

          calculateTrapezoidForBlock(block);

          // Move buffer head
          blockBufferHead = nextBufferHead;

          /* update lastGivenPosition and previousNominalSpeed variable */
          memcpy(&lastGivenPosition, &target, sizeof(Kinematics_AxisCoordinates_t)); // lastGivenPosition[] = target[]
          previousNominalSpeed = block->nominal_speed;

          recalculate();

          st_wake_up();
        }
      return true;
    }
  else return false;
}



/**
 * Recalculates the motion plan according to the following algorithm:
 * 1. Go over every block in reverse order and calculate a junction speed reduction (i.e. block_t.entry_factor)
 *    so that:
 *     a. The junction jerk is within the set limit
 *     b. No speed reduction within one block requires faster deceleration than the one, true constant
 *        acceleration.
 *   2. Go over every block in chronological order and dial down junction speed reduction values if
 *     a. The speed increase within one block would require faster accelleration than the one, true
 *        constant acceleration.
 *
 * When these stages are complete all blocks have an entry_factor that will allow all speed changes to
 * be performed using only the one, true constant acceleration, and where no junction jerk is jerkier than
 * the set limit. Finally it will:
 *
 *   3. Recalculate trapezoids for all blocks.
 */
void Planner::recalculate(void)
{
  /*
  planner_reverse_pass();
  planner_forward_pass();
  planner_recalculate_trapezoids(); */
}

/**
 *  Calculates the distance (not time) it takes to accelerate from initial_rate to target_rate, both given in
 *  steps/sec using the given acceleration, given in steps/sec^2.
 *  Given v0 initial speed, v1 target speed, a acceleration, t time, s distance
 *  Acceleration: a = (v1-v0)/t -> t = (v1-v0)/a.
 *  Assuming constant acceleration (i.e. linear increasing velocity) s = 1/2 * (v1-v0) * t + V0 * t ->
 *  s = 1/2 * t * (v1+v0) -> t = 2 * s * (v1+v0)
 *  Together (v1-v0)/a = 2 * s * (v1 + v0) ->  s = ((v1-v0) * (v1+v0)) / (2 * a) or (v1^2 - v0^2)/(2 * a)
 *  @param [in] initialRate Start velocity in steps/sec
 *  @param [in] targetRate End velocity in steps/sec
 *  @param [in] acceleration Acceleration for this move in steps/sec^2
 *  @return Distance in steps
 *  TODO: Make make inline to save stack if compiler does not do automatically anyway.
 */
uint32_t Planner::estimateAccelerationDistance(uint32_t initialRate, uint32_t targetRate, uint32_t acceleration)
{
  if (acceleration != 0) {
      return (sq(targetRate) - sq(initialRate))/(2*acceleration);
  }
  else {
      return 0;
    }
}


/**
 *  Calculates the needed initial speed to reach target speed with given acceleration in steps/s^2 and
 *  distance in steps.
 *  Given v0 initial speed, v1 target speed, a acceleration, t time, s distance
 *  Acceleration: a = (v1-v0)/t -> t = (v1-v0)/a.
 *  Assuming constant acceleration (i.e. linear increasing velocity) s = 1/2 * (v1-v0) * t + V0 * t ->
 *  s = 1/2 * t * (v1+v0) -> t = 2 * s * (v1+v0)
 *  Together (v1-v0)/a = 2 * s * (v1 + v0) -> (v1-v0) * (v1+v0) = 2 * s * a > v1^2 - v0^2 = 2 * s * a
 *  -> v0^2 = v1^2 - 2 * s * a or v0 = sqrt (v1^2 - 2 * s * a)
 *  @param [in] acceleration Acceleration for this move in steps/sec^2
 *  @param [in] targetVelocity Desired velocity in steps/sec at end of move
 *  @param [in] distance Length of travel
 *  @return minimum start speed in steps/sec
 *  @note Function max_allowable_speed from planner.cpp
 *  TODO: Make make inline to save stack if compiler does not do automatically anyway.
 *
 */
uint32_t Planner::maxStartSpeed(uint32_t acceleration, uint32_t targetVelocity, uint32_t distance)
{
  return sqrt(sq(targetVelocity) - 2 * acceleration * distance);
}

/*
 * Calculates trapezoid parameters so that the entry- and exit-speed are compensated by the provided factors.
 * @param [in] block Pointer to block for which the calculation should be done
 */
void Planner::calculateTrapezoidForBlock(Planner_Block_t *block)
{
  uint32_t accelerate_steps = estimateAccelerationDistance(block->entry_speed, block->nominal_rate, block->acceleration_st);

}

/**
 * Blocks until all commands in buffer have been executed
 */
void Planner::synchronize(void)
{

}
