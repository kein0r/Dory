//============================================================================
// Name        : MarlinNew.cpp

//============================================================================

#include <iostream>

#include "MarlinNew.h"
#include "CommandProvider.h"
#include "Machine.h"
#include "Kinematics.h"
#include "Planner.h"

using namespace std;

CommandProvider commandProvider;
Machine machine;

void Marlin::loop(void) {
	CommandProvider_GCode_t gCode;
	if (commandProvider.readNextCommand(&gCode))
	{
		/* TODO: Add test if G is really present or if this is M-Code */
		switch(gCode.G) {
		case 0:
			//machine.addMovement()
			break;
		default:
			break;
		}
	}

	machine.manageInactivity();
}

int main() {
	Marlin marlin;
	Planner planner;

	Kinematics_AxisCoordinates_t testTarget;
	uint32_t testFeedRate = 1500;
	uint8_t testExtruder = 0;
	testTarget.x = 300;
	testTarget.y = 300;
	testTarget.z = 300;
	testTarget.e = 0;
	planner.addMovement(testTarget, testFeedRate, testExtruder);

	/* Add test for direction bits to match pattern and only to take 8 bits in total */
	/* Test movement buffer */
	/* test Stepper class */
	/* - Test long moves in order to reduce Kinematics_AxisCoordinates_t to uin16_t */
	/* - Test end-stop max/min move */

	while(1) marlin.loop();
	cout << "!!!Hello World!!!" << endl; // prints !!!Hello World!!!
	return 0;
}
