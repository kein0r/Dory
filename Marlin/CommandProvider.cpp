#include "CommandProvider.h"
#include "stdlib.h"     /* Needed for strtod, strtol */

/**
 *  read next command from provider. That is Serial, SD-Card, BT, ...
 */
bool CommandProvider::readNextCommand(CommandProvider_GCode_t *code)
{
	return true;
}

/**
 * Parses an ASCII GCode line and converts it into a GCode structure. Content of code variable
 * is not(!) cleared but just overwritten according to the information found in the ASCII
 * string. Whether or not one or the other value was part of the last command must be determined
 * by reading CommandProvider_GCode_t.param
 *
 * @param line ASCII string that should be parsed
 * @param code resulting G-code after parsing
 * @return true CRC was found and matches or if no CRC is found/used, else false
 * @note Code partially copied from Repetier firmware
 */
bool CommandProvider::parseASCII(char *line, CommandProvider_GCode_t *code)
{
	return true;
}

/**
 * Returns the next float in string where s is pointing to
 * @param s Pointer to string
 * @return Corresponding float value
 *
 * @note: Code partially copied from Repetier firmware
 */
float CommandProvider::gCodeValueFloat(char *s)
{
	return (strtod(s, NULL));
}

/**
 * Returns the next long (sint32_t) in string where s is pointing to
 * @param s Pointer to string
 * @return Corresponding long value
 *
 * @note: Code partially copied from Repetier firmware
 */
int32_t CommandProvider::gCodeValueLong(char *s)
{
	return (strtol(s, NULL, 10));
}

