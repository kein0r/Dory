#ifndef Configuration_h
#define Configuration_h

#include "Kinematics.h"

/**
 * User-specified version info of this build to display in [Pronterface, etc] terminal window during
 * startup. Implementation of an idea by Prof Braino to inform user that any changes made to this
 * build by the user have been successfully uploaded into firmware.
 */
#define STRING_VERSION_CONFIG_H __DATE__ " " __TIME__ // build date and time
#define STRING_CONFIG_H_AUTHOR "(none, default config)" // Who made the changes.

/**
 * SERIAL_PORT selects which serial port should be used for communication with the host.
 * This allows the connection of wireless adapters (for instance) to non-default port pins.
 * Serial port 0 is still used by the Arduino bootloader regardless of this setting.
 */
#define SERIAL_PORT 0

/**
 * This determines the communication speed of the printer
 */
#define BAUDRATE 115200

/**
 *  This enables the serial port associated to the Bluetooth interface
 */
//#define BTENABLED              // Enable BT interface on AT90USB devices


/** The following define selects which electronics board you have. Please choose the one that matches your setup
 * 10 = Gen7 custom (Alfons3 Version) "https://github.com/Alfons3/Generation_7_Electronics"
 * 11 = Gen7 v1.1, v1.2 = 11
 * 12 = Gen7 v1.3
 * 13 = Gen7 v1.4
 * 2  = Cheaptronic v1.0
 * 20 = Sethi 3D_1
 * 3  = MEGA/RAMPS up to 1.2 = 3
 * 33 = RAMPS 1.3 / 1.4 (Power outputs: Extruder, Fan, Bed)
 * 34 = RAMPS 1.3 / 1.4 (Power outputs: Extruder0, Extruder1, Bed)
 * 35 = RAMPS 1.3 / 1.4 (Power outputs: Extruder, Fan, Fan)
 * 4  = Duemilanove w/ ATMega328P pin assignment
 * 5  = Gen6
 * 51 = Gen6 deluxe
 * 6  = Sanguinololu < 1.2
 * 62 = Sanguinololu 1.2 and above
 * 63 = Melzi
 * 64 = STB V1.1
 * 65 = Azteeg X1
 * 66 = Melzi with ATmega1284 (MaKr3d version)
 * 67 = Azteeg X3
 * 68 = Azteeg X3 Pro
 * 7  = Ultimaker
 * 71 = Ultimaker (Older electronics. Pre 1.5.4. This is rare)
 * 72 = Ultimainboard 2.x (Uses TEMP_SENSOR 20)
 * 77 = 3Drag Controller
 * 8  = Teensylu
 * 80 = Rumba
 * 81 = Printrboard (AT90USB1286)
 * 82 = Brainwave (AT90USB646)
 * 83 = SAV Mk-I (AT90USB1286)
 * 84 = Teensy++2.0 (AT90USB1286) CLI compile: DEFINES=AT90USBxx_TEENSYPP_ASSIGNMENTS HARDWARE_MOTHERBOARD=84  make
 * 9  = Gen3+
 * 70 = Megatronics
 * 701= Megatronics v2.0
 * 702= Minitronics v1.0
 * 90 = Alpha OMCA board
 * 91 = Final OMCA board
 * 301= Rambo
 * 21 = Elefu Ra Board (v3)
 * 88 = 5DPrint D8 Driver Board
 * 999 = Leapfrog
 */
#ifndef MOTHERBOARD
#define MOTHERBOARD 33
#endif

// Define this to set a custom name for your generic Mendel,
#define CUSTOM_MENDEL_NAME "Rostock Mini"

// Define this to set a unique identifier for this printer, (Used by some programs to differentiate between machines)
// You can use an online service to generate a random UUID. (eg http://www.uuidgenerator.net/version4)
// #define MACHINE_UUID "00000000-0000-0000-0000-000000000000"

// This defines the number of extruders
#define EXTRUDERS 1

//// The following define selects which power supply you have. Please choose the one that matches your setup
// 1 = ATX
// 2 = X-Box 360 203Watts (the blue wire connected to PS_ON and the red wire to VCC)

#define POWER_SUPPLY 1

// Define this to have the electronics keep the power supply off on startup. If you don't know what this is leave it.
// #define PS_DEFAULT_OFF

/**
 * Make delta curves from many straight lines (linear interpolation).
 * This is a trade-off between visible corners (not enough segments)
 * and processor overload (too many expensive sqrt calls).
 * @note: This is old DELTA_SEGMENTS_PER_SECOND but is now general to
 * all machines. Should be set to 1 for non-delta machines.
 */
#define SEGMENTS_PER_SECOND 200.0


//===========================================================================
//============================== Delta Settings =============================
//===========================================================================
// Enable DELTA kinematics and most of the default configuration for Deltas
#define DELTA

/* NOTE NB all values for DELTA_* values MOUST be floating point, so always have a decimal point in them */

/**
 * Center-to-center distance of the holes in the diagonal push rods.
 */
#define DELTA_DIAGONAL_ROD 184.0 // mm

/**
 * Horizontal offset from middle of printer to smooth rod center.
 */
#define DELTA_SMOOTH_ROD_OFFSET 139.5 // mm

/**
 * Horizontal offset of the universal joints on the end effector.
 */
#define DELTA_EFFECTOR_OFFSET 33.0 // mm

/**
 * Horizontal offset of the universal joints on the carriages.
 */
#define DELTA_CARRIAGE_OFFSET 22.0 // mm

/**
 * Effective horizontal distance bridged by diagonal push rods.
 */
#define DELTA_RADIUS (DELTA_SMOOTH_ROD_OFFSET-DELTA_EFFECTOR_OFFSET-DELTA_CARRIAGE_OFFSET)

/**
 * If true, axis won't move to coordinates less than HOME_POS.
 */
#define min_software_endstops true
/**
 * If true, axis won't move to coordinates greater than the defined lengths below.
 */
#define max_software_endstops true

// delta homing speeds must be the same on xyz
#define HOMING_FEEDRATE {200*10, 200*10, 200*10, 0}  // set the homing speeds (mm/min)

/*
 * Default settings for printer.
 * Note: Delta speeds must be the same on xyz axis
 */
#define XYZ_FULL_STEPS_PER_ROTATION 200
#define XYZ_MICROSTEPS 16
#define XYZ_BELT_PITCH 2
#define XYZ_PULLEY_TEETH 16
#define XYZ_STEPS (XYZ_FULL_STEPS_PER_ROTATION * XYZ_MICROSTEPS / double(XYZ_BELT_PITCH) / double(XYZ_PULLEY_TEETH))

/**
 * Default steps per mm
 * TODO: Rename to DEFAULT_AXIS_STEPS_PER_MM
 */
#define DEFAULT_AXIS_STEPS_PER_UNIT     {XYZ_STEPS, XYZ_STEPS, XYZ_STEPS, 460}
/**
 * Maximum allowed feedrate given to the stepper in steps/sec
 */
#define DEFAULT_MAX_FEEDRATE            {1200 * XYZ_STEPS, 1200 * XYZ_STEPS, 1200 * XYZ_STEPS, 300}
/**
 * Maximum allowed acceleration  in steps/sec^2
 */
#define DEFAULT_MAX_ACCELERATION        {3000,3000,3000,4000}    // X, Y, Z, E maximum start speed for accelerated moves. E default values are good for skeinforge 40+, for older versions raise them a lot.

/**
 *  X, Y, Z and E max acceleration in steps/s^2 for the steppers for printing moves
 */
#define DEFAULT_ACCELERATION          1500 * XYZ_STEPS
/**
 * X, Y, Z and E max acceleration in steps/s^2 for the stepper for retracts
 */
#define DEFAULT_RETRACT_ACCELERATION  1500 * 460

// Offset of the extruders (uncomment if using more than one and relying on firmware to position when changing).
// The offset has to be X=0, Y=0 for the extruder 0 hotend (default extruder).
// For the other hotends it is their distance from the extruder 0 hotend.
// #define EXTRUDER_OFFSET_X {0.0, 20.00} // (in mm) for each extruder, offset of the hotend on the X axis
// #define EXTRUDER_OFFSET_Y {0.0, 5.00}  // (in mm) for each extruder, offset of the hotend on the Y axis

/**
 * The speed change that does not require acceleration (i.e. the software might assume it can be done instantaneously)
 */
#define DEFAULT_XYJERK                20.0 * XYZ_STEPS  // (steps/sec)
#define DEFAULT_ZJERK                 20.0 * XYZ_STEPS  // (steps/sec) Must be same as XY for delta
#define DEFAULT_EJERK                 20.0 * XYZ_STEPS  // (steps/sec)


#include "Configuration_adv.h"

/**
 * Class to provide configuration data to all other classes.
 * Configuration variables are pre-filled with #define values given above which later will
 * get updated by the values read from EEPROM in the class constructor.
 */
class Configuration {

public:
    Configuration(void);
    static const float segments_per_second = SEGMENTS_PER_SECOND;
    static const float minimalMoveDistance = 0.000001;    /*!< Minimal distance tool can should be moved, if smaller move will be discarded */
    static const float feedRateFactor = 100;    /*!< Factor for feedrate given in percentage (100 mean no changes to feedrate). Applies to x,y and z . Can be changes using M220 command */
    static const float extrudeFactor = 100;     /*!< Factor for extrusion given in percentage (100 mean no changes to extrusion). Applies to E. Can be changes using M221 command */

    Kinematics_AxisCoordinates_t maxFeedrate;
    uint32_t maxAcceleration;
    uint32_t max_xy_jerk;


private:


};

/* Configuration is a singleton. */
extern Configuration configuration;

#endif  // #ifndef Configuration_h
