#ifndef Configuration_adv_h
#define Configuration_adv_h

//===========================================================================
//=============================Buffers           ============================
//===========================================================================

/**
 * The number of linear motions that can be in the plan at any give time.
 * @note THE BLOCK_BUFFER_SIZE NEEDS TO BE A POWER OF 2, i.g. 8,16,32 because shifts
 * and ORs are used to do the ringbuffering.
 */
#define BLOCK_BUFFER_SIZE 16   // SD,LCD,Buttons take more memory, block buffer needs to be smaller

/**
 * The ASCII buffer for receiving from the serial
 */
#define MAX_CMD_SIZE 96
#define BUFSIZE 4

/*
 * Everything with less than this number of steps will be ignored as move and joined with the next movement
 */
#define PLANNER_DROPSEGMENTS    5

/**
 * Minimum planner junction speed in steps/sec. Sets the default minimum speed the planner plans for at
 * the end of the buffer and all stops. This should not be much greater than zero and should only be
 * changed if unwanted behavior is observed on a user's machine when running at very slow speeds.
 */
#define MINIMUM_PLANNER_SPEED   2 // (steps/sec)

#endif  // #ifndef Configuration_adv_h
