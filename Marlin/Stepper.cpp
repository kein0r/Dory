#include "Stepper.h"

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse.
// It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately.
ISR(Stepper::TIMER1_COMPA_vect)
{
  static Kinematics_AxisCoordinates_t counter;
  static Kinematics_AxisCoordinates_t countDirection;  /* This seems overkill but we need to multiply anyway */
  static uint32_t stepEventsCompleted;
  static Planner_Block_t *currentBlock;

  // If there is no current block, attempt to pop one from the buffer
  if (currentBlock == NULL) {
      // Anything in the buffer?
      currentBlock = planner.getCurrentBlock();
      if (currentBlock != NULL) {
          currentBlock->busy = true;
          trapezoidGeneratorReset();    /* will also set OCR1A */
          counter.x = -(currentBlock->step_event_count >> 1);
          counter.y = counter.x;
          counter.z = counter.x;
          counter.e = counter.x;
          stepEventsCompleted = 0;
      }
      else {
          OCR1A=2000; // 1kHz.
      }
  }

  if (currentBlock != NULL) {
      // Set directions TODO This should be done once during init of trapezoid. Endstops -> interrupt
      Planner_directionBits_t out_bits = currentBlock->direction;

      /* Set the direction pins for x,y and z-axis and countDirection to +1/-1 */
      if(out_bits.bits.x) { /* - direction x-axis */
#ifdef DUAL_X_CARRIAGE
          if (extruder_duplication_enabled){
              WRITE(X_DIR_PIN, INVERT_X_DIR);
              WRITE(X2_DIR_PIN, INVERT_X_DIR);
          }
          else{
              if (currentBlock->active_extruder != 0)
                WRITE(X2_DIR_PIN, INVERT_X_DIR);
              else
                WRITE(X_DIR_PIN, INVERT_X_DIR);
          }
#else
          WRITE(X_DIR_PIN, INVERT_X_DIR);
#endif
          countDirection.x=-1;
      }
      else { /* + direction x-axis */
#ifdef DUAL_X_CARRIAGE
          if (extruder_duplication_enabled){
              WRITE(X_DIR_PIN, !INVERT_X_DIR);
              WRITE(X2_DIR_PIN, !INVERT_X_DIR);
          }
          else{
              if (currentBlock->active_extruder != 0)
                WRITE(X2_DIR_PIN, !INVERT_X_DIR);
              else
                WRITE(X_DIR_PIN, !INVERT_X_DIR);
          }
#else
          WRITE(X_DIR_PIN, !INVERT_X_DIR);
#endif
          countDirection.x=1;
      }
      if(out_bits.bits.y) { /* - direction y-axis */
          WRITE(Y_DIR_PIN, INVERT_Y_DIR);
#ifdef Y_DUAL_STEPPER_DRIVERS
          WRITE(Y2_DIR_PIN, !(INVERT_Y_DIR == INVERT_Y2_VS_Y_DIR));
#endif
          countDirection.y=-1;
      }
      else { /* + direction y-axis */
          WRITE(Y_DIR_PIN, !INVERT_Y_DIR);
#ifdef Y_DUAL_STEPPER_DRIVERS
          WRITE(Y2_DIR_PIN, (INVERT_Y_DIR == INVERT_Y2_VS_Y_DIR));
#endif
          countDirection.y=1;
      }

      if (out_bits.bits.z) {   // - direction z-Axis
          WRITE(Z_DIR_PIN,INVERT_Z_DIR);
#ifdef Z_DUAL_STEPPER_DRIVERS
          WRITE(Z2_DIR_PIN,INVERT_Z_DIR);
#endif
          countDirection.z=-1;
      }
      else { // +direction
          WRITE(Z_DIR_PIN,!INVERT_Z_DIR);
#ifdef Z_DUAL_STEPPER_DRIVERS
          WRITE(Z2_DIR_PIN,!INVERT_Z_DIR);
#endif
          countDirection.z=1;
      }

      /* We always check all existing end-stop regardless of the direction of travel.
       * If one is triggered something went wrong anyway. */
#if defined(X_MIN_PIN) && X_MIN_PIN > -1
      endstopStatus.bits.xMin = (READ(X_MIN_PIN) != X_MIN_ENDSTOP_INVERTING);
#endif
#if defined(X_MAX_PIN) && X_MAX_PIN > -1
      endstopStatus.bits.xMax =(READ(X_MAX_PIN) != X_MAX_ENDSTOP_INVERTING);
#endif
#if defined(Y_MIN_PIN) && Y_MIN_PIN > -1
      endstopStatus.bits.yMin = (READ(Y_MIN_PIN) != Y_MIN_ENDSTOP_INVERTING);
#endif
#if defined(Y_MAX_PIN) && Y_MAX_PIN > -1
      endstopStatus.bits.yMax =(READ(Y_MAX_PIN) != Y_MAX_ENDSTOP_INVERTING);
#endif
#if defined(Z_MIN_PIN) && Z_MIN_PIN > -1
      endstopStatus.bits.zMin = (READ(Z_MIN_PIN) != Z_MIN_ENDSTOP_INVERTING);
#endif
#if defined(Z_MAX_PIN) && Z_MAX_PIN > -1
      endstopStatus.bits.zMax =(READ(Z_MAX_PIN) != Z_MAX_ENDSTOP_INVERTING);
#endif

      /* TODO: Check why old_x_min_endstop etc. was used to delay reaction by one cycle in stepper.cpp */
      if (endstopStatus.value) {
          if (endstopStatus.bits.xMin || endstopStatus.bits.xMax) {
              endstopTriggerPosition.x = position.x;
          }
          if (endstopStatus.bits.yMin || endstopStatus.bits.yMax) {
              endstopTriggerPosition.y = position.y;
          }
          if (endstopStatus.bits.zMin || endstopStatus.bits.zMax) {
              endstopTriggerPosition.z = position.z;
          }
          stepEventsCompleted = currentBlock->step_event_count;
      }


      /* Now do steps on x,y and z-axis as needed. Take multiple steps per interrupt (For high speed moves) */
      for(int8_t i=0; i < stepLoops; i++) {
#ifndef AT90USB
          MSerial.checkRx(); // Check for serial chars.
#endif

          counter.x += currentBlock->steps.x;
          if (counter.x > 0) {
#ifdef DUAL_X_CARRIAGE
              if (extruder_duplication_enabled) {
                  WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN);
                  WRITE(X2_STEP_PIN, !INVERT_X_STEP_PIN);
              }
              else {
                  if (currentBlock->active_extruder != 0)
                    WRITE(X2_STEP_PIN, !INVERT_X_STEP_PIN);
                  else
                    WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN);
              }
#else
              WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN);
#endif
              counter.x -= currentBlock->step_event_count;
              position.x += countDirection.x;
#ifdef DUAL_X_CARRIAGE
              if (extruder_duplication_enabled){
                  WRITE(X_STEP_PIN, INVERT_X_STEP_PIN);
                  WRITE(X2_STEP_PIN, INVERT_X_STEP_PIN);
              }
              else {
                  if (currentBlock->active_extruder != 0)
                    WRITE(X2_STEP_PIN, INVERT_X_STEP_PIN);
                  else
                    WRITE(X_STEP_PIN, INVERT_X_STEP_PIN);
              }
#else
              WRITE(X_STEP_PIN, INVERT_X_STEP_PIN);
#endif
          }

          counter.y += currentBlock->steps.y;
          if (counter.y > 0) {
              WRITE(Y_STEP_PIN, !INVERT_Y_STEP_PIN);

#ifdef Y_DUAL_STEPPER_DRIVERS
              WRITE(Y2_STEP_PIN, !INVERT_Y_STEP_PIN);
#endif

              counter.y -= currentBlock->step_event_count;
              position.y+=countDirection.y;
              WRITE(Y_STEP_PIN, INVERT_Y_STEP_PIN);

#ifdef Y_DUAL_STEPPER_DRIVERS
              WRITE(Y2_STEP_PIN, INVERT_Y_STEP_PIN);
#endif
          }

          counter.z += currentBlock->steps.z;
          if (counter.z > 0) {
              WRITE(Z_STEP_PIN, !INVERT_Z_STEP_PIN);

#ifdef Z_DUAL_STEPPER_DRIVERS
              WRITE(Z2_STEP_PIN, !INVERT_Z_STEP_PIN);
#endif

              counter.z -= currentBlock->step_event_count;
              position.z+=countDirection.z;
              WRITE(Z_STEP_PIN, INVERT_Z_STEP_PIN);

#ifdef Z_DUAL_STEPPER_DRIVERS
              WRITE(Z2_STEP_PIN, INVERT_Z_STEP_PIN);
#endif
          }
          stepEventsCompleted += 1;
          if(stepEventsCompleted >= currentBlock->step_event_count) break;
      }

      // Calculare new timer value
      unsigned short timer;
      unsigned short step_rate;
      if (stepEventsCompleted <= (unsigned long int)currentBlock->accelerate_until) {

          MultiU24X24toH16(acc_step_rate, acceleration_time, currentBlock->acceleration_rate);
          acc_step_rate += currentBlock->initial_rate;

          // upper limit
          if(acc_step_rate > currentBlock->nominal_rate)
            acc_step_rate = currentBlock->nominal_rate;

          // step_rate to timer interval
          timer = calcTimer(acc_step_rate);
          OCR1A = timer;
          acceleration_time += timer;
#ifdef ADVANCE
          for(int8_t i=0; i < step_loops; i++) {
              advance += advance_rate;
          }
          //if(advance > currentBlock->advance) advance = currentBlock->advance;
          // Do E steps + advance steps
          e_steps[currentBlock->active_extruder] += ((advance >>8) - old_advance);
          old_advance = advance >>8;

#endif
      }
      else if (stepEventsCompleted > (unsigned long int)currentBlock->decelerate_after) {
          MultiU24X24toH16(step_rate, deceleration_time, currentBlock->acceleration_rate);

          if(step_rate > acc_step_rate) { // Check step_rate stays positive
              step_rate = currentBlock->final_rate;
          }
          else {
              step_rate = acc_step_rate - step_rate; // Decelerate from aceleration end point.
          }

          // lower limit
          if(step_rate < currentBlock->final_rate)
            step_rate = currentBlock->final_rate;

          // step_rate to timer interval
          timer = calc_timer(step_rate);
          OCR1A = timer;
          deceleration_time += timer;
#ifdef ADVANCE
          for(int8_t i=0; i < step_loops; i++) {
              advance -= advance_rate;
          }
          if(advance < final_advance) advance = final_advance;
          // Do E steps + advance steps
          e_steps[currentBlock->active_extruder] += ((advance >>8) - old_advance);
          old_advance = advance >>8;
#endif //ADVANCE
      }
      else {
          OCR1A = OCR1A_nominal;
          // ensure we're running at the correct step rate, even if we just came off an acceleration
          step_loops = step_loops_nominal;
      }

      // If current block is finished, reset pointer
      if (stepEventsCompleted >= currentBlock->step_event_count) {
          currentBlock = NULL;
          planner.discardCurrentBlock();
      }
  }
}

void Stepper::trapezoidGeneratorReset(void)
{

}

uint16_t Stepper::calcTimer(uint16_t stepRate)
{
  stepLoops = 1;
}
