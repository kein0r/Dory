#ifndef CommandProvider_h
#define CommandProvider_h

#include<stdint.h>
#include "MarlinSerial.h"

/**
 * Will read commands from different sources (Serial, BT, SD-Card)
 */

/**
 * @note G-code structure partially copied from Repetier Firmware
 */
typedef struct {
   uint16_t params;
   uint16_t params2;
   uint16_t N; // Line number
   uint16_t M;
   uint16_t G;
   float X;
   float Y;
   float Z;
   float E;
   float F;
} CommandProvider_GCode_t;

class CommandProvider {

public:
	 bool readNextCommand(CommandProvider_GCode_t *code);
	 bool parseASCII(char *line, CommandProvider_GCode_t *code);

private:
	 float gCodeValueFloat(char *s);
	 int32_t gCodeValueLong(char *s);
};

#endif //#ifndef CommandProvider
