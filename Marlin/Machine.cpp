#include "Machine.h"

#include <math.h>

/**
 * Adds a new movement to planner. Checks if destination coordinates are within valid range (aka
 * software end-stops). Destination coordinates are given in world-coordinates and are therefore
 * translated to axis coordinates using kinematics.
 * Feedrate is adapted by feedRateFactor before its given to planner.
 * @param [in] newPosition Position the machine should move to
 * @param [in] feedRate Feedrate in mm/min for this move
 * @note Part of former prepare_move from Marlin_main.cpp and plan_buffer_line from planner.cpp
 */
void Machine::addMovement(Kinematics_WorldCoordinates_t newPosition, float feedRate)
{
	previousCommand_millis = millis();
	/* Check if new position is within valid bounds of this machine. Since end-stop position is given
	 * in world-coordinate system we can directly check them. */
	/* TODO: Add apply_rotation_xyz here */
	clampToSoftwareEndstops(&newPosition);
	/* TODO: Add PREVENT_DANGEROUS_EXTRUDE stuff from plan_buffer_line here */

	/* Apply factor for feedrate and extrusion */
	feedRate *= configuration.feedRateFactor / 100;
	/* TODO: Add check for mintravelfeedrate and minimumfeedrate here (see plan_buffer_line) */
	newPosition.e *= volumetricMultiplier[activeExtruder];
	newPosition.e *= configuration.extrudeFactor / 100;

	Kinematics_WorldCoordinates_t difference;
	difference.x = newPosition.x - lastGivenPosition.x;
	difference.y = newPosition.y - lastGivenPosition.y;
	difference.z = newPosition.z - lastGivenPosition.z;
	difference.e = newPosition.e - lastGivenPosition.e;
	float length = sqrt(sq(difference.x) + sq(difference.y) + sq(difference.z));
	/* TODO: Check if check for minimum distance to move is needed here as it was only implemented for Delta/Scara */
	float seconds = MINUTES_TO_SECONDS * length / feedRate;
	uint16_t steps = max(1, int(configuration.segments_per_second * seconds));
	for (uint16_t s = 1; s <= steps; s++) {
	    float fraction = float(s) / float(steps);
	    lastGivenPosition.x += difference.x * fraction;
	    lastGivenPosition.x += difference.y * fraction;
	    lastGivenPosition.y += difference.z * fraction;
	    /* convert to axis coordinate system */
	    Kinematics_AxisCoordinates_t axisPosition = kinematics.convertFromWorldToAxis(lastGivenPosition);
	    /* Add position to movement planner buffer. If the buffer is full: good! That means we are well
	     * ahead of the robot. Rest here until there is room in the buffer. */
	    /* TODO: Clarify comment: Do not use feedmultiply for E or Z only moves */
	    while(!planner.addMovement(axisPosition, feedRate, activeExtruder))
	      {
	        manageInactivity();
	      }
	}
	/* Those lines can be removed later. They are here only to cover rounding errors from above */
	lastGivenPosition.x = newPosition.x;
	lastGivenPosition.y = newPosition.y;
	lastGivenPosition.z = newPosition.z;
	lastGivenPosition.e = newPosition.e;
}

/* Set current position of the machine to values given in variable position. Position is given
 * in world coordinate system and translated to axis coordinate system (steps) using machine
 * kinematics. These coordinates are forwarded to Planner module
 * @param [in] position Current position of tool in world coordinate system.
 */
void Machine::setPosition(Kinematics_WorldCoordinates_t position)
{
	Kinematics_AxisCoordinates_t axisPosition = kinematics.convertFromWorldToAxis(position);
	planner.setPosition(axisPosition);
}

/* Returns current position from Planner in world coordinate system. Position is calculated from
 * axis coordinates using machine kinematics
 * @return current position in world coordinate system
 */
Kinematics_WorldCoordinates_t Machine::getPosition(void)
{
	Kinematics_AxisCoordinates_t axisPosition = planner.getPosition();
	return kinematics.convertAxisToWorld(axisPosition);
}

/*
 * Old void manage_inactivity()
 * Originally did the following
 * - get_command
 * - kill motor if idle
 * - read kill pin and react on it (KILL_PIN)
 * - Everything related to CONTROLLERFAN_PIN
 * - Everything related to EXTRUDER_RUNOUT_PREVENT. Called plan_buffer_line for this purpose
 * - TEMP_STAT_LEDS
 * - Something regarding DUAL_X_CARRIAGE
 * - check_axes_activity() (planner)
 */
void Machine::manageInactivity(void)
{
/* Check if the following should also be added to be able to call this function from addMovement
    manage_heater();
    manage_inactivity();
    lcd_update();
 */
}

/**
 * Checks given coordinates whether or not they are within valid bounds of the machine and limits thems
 * accordingly. This feature is usually referred as software end-stops.
 * @param [in,out] newPosition New position given in world coordinates to be checked
 */
void Machine::clampToSoftwareEndstops(Kinematics_WorldCoordinates_t *newPosition)
{
  if (min_software_endstops) {
      if (newPosition->x < minPosition.x) newPosition->x = minPosition.x;
      if (newPosition->y < minPosition.y) newPosition->y = minPosition.y;
      if (newPosition->z < minPosition.z) newPosition->z = minPosition.z;
  }

  if (max_software_endstops) {
      if (newPosition->x > maxPosition.x) newPosition->x = maxPosition.x;
      if (newPosition->y > maxPosition.y) newPosition->y = maxPosition.y;
      if (newPosition->z > maxPosition.z) newPosition->z = maxPosition.z;  }
}
