#ifndef Planner_h
#define Planner_h

#include<stdint.h>

#include "Kinematics.h"
#include "Configuration.h"
#include "Arduino.h"
#include "Stepper.h"

/**
 TODO: copy description from planner.c to here
 */

/**
 * Union for easy accessing and storing direction bits for the stepper.
 */
typedef union {
  struct {
    uint8_t x:1;        /*!< false: positive direction, count up, true: negative direction, count down */
    uint8_t y:1;        /*!< false: positive direction, count up, true: negative direction, count down */
    uint8_t z:1;        /*!< false: positive direction, count up, true: negative direction, count down */
    uint8_t e:1;
  } bits;
  uint8_t value;
} Planner_directionBits_t;

/**
 * This struct is used when buffering the setup for each linear movement "nominal"
 * values are as specified in the source g-code and may never actually be reached
 * if acceleration management is active.
 */
typedef struct {
  // Fields used by the bresenham algorithm for tracing the line
  Kinematics_AxisCoordinates_t steps;           /*!<  Step count along each axis */

  uint32_t step_event_count;                    /*!<  The number of step events required to complete this block */
  int32_t accelerate_until;                     /*!<  The index of the step event on which to stop acceleration */
  int32_t decelerate_after;                     /*!<  The index of the step event on which to start decelerating */
  int32_t acceleration_rate;                    /*!<  The acceleration rate used for acceleration calculation */
  Planner_directionBits_t direction;            /*!<  The direction bit set for this block (refers to *_DIRECTION_BIT in config.h) */
  uint8_t active_extruder;                      /*!<  Selects the active extruder */

  // Fields used by the motion planner to manage acceleration
  uint32_t nominal_speed;                       /*!<  Nominal speed for this block in steps/sec. Equals given feedrate limited by max allowed feedrate and corrected by speed factor.*/
  uint32_t entry_speed;                         /*!<  Entry speed at previous-current junction in steps/sec */
  uint32_t max_entry_speed;                     /*!<  Maximum allowable junction entry speed in steps/sec */
  uint32_t totalTravelSteps;                    /*!<  The total travel of this block in steps */
  uint32_t acceleration;                        /*!<  acceleration step/sec^2 */
  bool recalculate_flag;                        /*!<  Planner flag to recalculate trapezoids on entry junction */
  bool nominal_length_flag;                     /*!<  Planner flag for nominal speed always reached */

  // Settings for the trapezoid generator
  uint32_t nominal_rate;                        /*!<  The nominal step rate for this block in step_events/sec */
  uint32_t initial_rate;                        /*!<  The jerk-adjusted step rate at start of block */
  uint32_t final_rate;                          /*!<  The minimal rate at exit */
  uint32_t acceleration_st;                     /*!<  acceleration steps/sec^2 */
  volatile bool busy;
} Planner_Block_t;

#if BLOCK_BUFFER_SIZE > 255
#error BLOCK_BUFFER_SIZE must be below 256 to fit into uint8_t
#endif
typedef uint8_t Planner_blockIndex_t;

class Planner
{
public:
    Planner(void);
	void synchronize(void);
	Kinematics_AxisCoordinates_t getPosition( void );
	void setPosition(Kinematics_AxisCoordinates_t newPosition);
	bool addMovement(Kinematics_AxisCoordinates_t target, uint32_t feedRate, uint8_t activeExtruder);

	void discardCurrentBlock ( void );
	Planner_Block_t *getCurrentBlock( void );

private:
	Planner_blockIndex_t blockBufferHead = 0;
	Planner_blockIndex_t blockBufferTail = 0;
	Planner_Block_t blockBuffer[BLOCK_BUFFER_SIZE];
        /**
         * Last position the stepper were ordered to go to. While machine is moving this is *not* its actual stepper
         * position. Only after all moves have been finished their actual position matches this one (given no
         * end-stop was hit)
         */
	Kinematics_AxisCoordinates_t lastGivenPosition = {0, 0, 0, 0};

	static const uint8_t dropsegments = PLANNER_DROPSEGMENTS; /*!< everything with less than this number of steps will be ignored as move and joined with the next movement */

	Planner_blockIndex_t getNextBlockIndex(Planner_blockIndex_t blockIndex);
	Planner_blockIndex_t getPreviousBlockIndex(Planner_blockIndex_t blockIndex);
	void recalculate();
	uint32_t estimateAccelerationDistance (uint32_t initialRate, uint32_t targetRate, uint32_t acceleration);
	uint32_t maxStartSpeed(uint32_t acceleration, uint32_t targetVelocity, uint32_t distance);
	void calculateTrapezoidForBlock(Planner_Block_t *block);

};

extern Planner planner;

#endif // #ifndef Planner_h
