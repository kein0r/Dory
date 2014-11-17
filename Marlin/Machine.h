#ifndef Machine_h
#define Machine_h

#include "Configuration.h"
#include "Kinematics.h"
#include "Planner.h"
#include "Arduino.h"

#define MINUTES_TO_SECONDS      60

#define CriticalSectionEnter()
#define CriticalSectionExit()

/**
 *
 * TODO: Make feedmultiply a real factor and not given in percentage 100 -> 1
 */

class Machine {

public:
	void setPosition(Kinematics_WorldCoordinates_t position);
	Kinematics_WorldCoordinates_t getPosition(void);

	void addMovement(Kinematics_WorldCoordinates_t newPosition, float feedRate);
	void manageInactivity(void);

private:
	Planner planner;
	Kinematics kinematics;
	uint32_t previousCommand_millis;	/*!< Time stamp for the last command */
	uint8_t activeExtruder;
	/**
	 * Reciprocal of cross-sectional area of filament (in square millimeters), stored this way to reduce
	 * computational burden in planner
	 */
	float volumetricMultiplier[EXTRUDERS] = {1};
	/**
	 * Last position the machine was order to go to. While machine is moving this is *not* its actual position.
	 * Only after all moves have been finished its actual position matches this one (given no end-stop was hit)
	 */
	Kinematics_WorldCoordinates_t lastGivenPosition;

	Kinematics_WorldCoordinates_t minPosition;      /*!< Minimum position in world coordinates this machine can go. Used for software end-stops */
	Kinematics_WorldCoordinates_t maxPosition;      /*!< Maximum position in world coordinates this machine can go. Used for software end-stops */


	void clampToSoftwareEndstops(Kinematics_WorldCoordinates_t *newPosition);

};

#endif  // #ifndef Machine_h
