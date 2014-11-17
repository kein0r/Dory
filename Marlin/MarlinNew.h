#ifndef MarlinNew_h
#define MarlinNew_h

/** Mainly a refactoring of Marlin firmware
 * - Added description to functions
 * - Moved things where they belong to. It does not make sense to have machine related things in Marlin_main.cpp and always use #ifdef DELTA.
 * This should be done where its needed
 * - Prefixed some configuration items to make clear by which module they are used
 * - Removed as much as possible magic numbers from the code
 * - Removed all functionality regarding ADVANCE, DUAL_X_CARRIAGE, SCARA, COREXY, BARICUDA
 * - Added tests
 *
 * This software introduces two coordinate systems. The world coordinate system (x,y,z and e) and the motor/axis coordinate system (steps) for each
 * motor/axis. Translation between them is done by the machine kinematics. In simple case only gears and belts characteristics needs to be taken
 * into account for delta printer more complex kinematics needs to be calculated.
 * All g-code commands, as they are about the tool, are of course always given in world coordinate system.
 *
 * TODO: Add doxygen modules to each file
 * TODO: Add Watchdog and trigger it
 */

class Marlin
{
public:
	void loop(void);

};

#endif // #ifndef MarlinNew_h
