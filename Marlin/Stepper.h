#ifndef Stepper_h
#define Stepper_h

#include "Configuration.h"
#include "Kinematics.h"
#include "Planner.h"
#include "Arduino.h"

#define CriticalSectionEnter()
#define CriticalSectionExit()

/* Workaround for now due to missing pins.h */
#define X_MIN_PIN 1
#define X_MAX_PIN 1
#define Y_MIN_PIN 1
#define Y_MAX_PIN 1
#define Z_MIN_PIN 1
#define Z_MAX_PIN 1


/**
 * Union for easy accessing endstops.
 */
typedef union {
  struct {
    uint8_t xMin:1;     /*!< true: end-stop was triggered, false: end-stop was not triggered. */
    uint8_t yMin:1;     /*!< true: end-stop was triggered, false: end-stop was not triggered. */
    uint8_t zMin:1;     /*!< true: end-stop was triggered, false: end-stop was not triggered. */
    uint8_t xMax:1;     /*!< true: end-stop was triggered, false: end-stop was not triggered. */
    uint8_t yMax:1;     /*!< true: end-stop was triggered, false: end-stop was not triggered. */
    uint8_t zMax:1;     /*!< true: end-stop was triggered, false: end-stop was not triggered. */
  } bits;
  uint8_t value;
} Stepper_EndstopStatus_t;

/**
 *
 */

class Stepper {
public:
  static void TIMER1_COMPA_vect(void);
private:
  static void trapezoidGeneratorReset(void);
  static uint8_t stepLoops = 1; /*!< Number of steps to be done in one call of ISR. Normally set to one but can be higher for high speed moves */
  Kinematics_AxisCoordinates_t position = {0, 0, 0, 0}; /*!< Current position of the stepper. Incremented/Decremented in TIMER1_COMPA_vect or updated after end-stop/probing move */
  static Stepper_EndstopStatus_t endstopStatus = 0;     /*!< Status of min/max end-stops */
  static Kinematics_AxisCoordinates_t endstopTriggerPosition = {0, 0, 0, 0}; /*!< Position at which end-stops got triggered. Only used for reporting */

  static uint16_t calcTimer(uint16_t stepRate);
};


#endif
