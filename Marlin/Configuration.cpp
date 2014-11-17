#include "Configuration.h"
Configuration configuration; /* Configuration is a singleton. Therefore it will be created here */

Configuration::Configuration(void)
{
  /* load all default values for structs within constructor */
  maxFeedrate = DEFAULT_MAX_FEEDRATE;

  maxAcceleration = DEFAULT_ACCELERATION;
  max_xy_jerk = DEFAULT_XYJERK;
}
