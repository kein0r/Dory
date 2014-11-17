#ifndef Kinematics_h
#define Kinematics_h

#include<stdint.h>

/**
 * World coordinates for tool in mm
 */
typedef struct {
	float x;
	float y;
	float z;
	float e;
} Kinematics_WorldCoordinates_t;

/**
 * Axis coordinates in steps
 */
typedef struct {
	uint32_t x;
	uint32_t y;
	uint32_t z;
	uint32_t e;
} Kinematics_AxisCoordinates_t;

class Kinematics {
public:
	Kinematics_AxisCoordinates_t convertFromWorldToAxis(Kinematics_WorldCoordinates_t world);
	Kinematics_WorldCoordinates_t convertAxisToWorld(Kinematics_AxisCoordinates_t axis);
};

#endif
