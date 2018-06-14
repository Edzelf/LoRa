//***************************************************************************************************
//*                                   S E N S O R . C P P                                           *
//***************************************************************************************************
// Handling of data from sensor.                                                                    *
// In this case it is a dummy sensor, providing the string "Hello World!"                           *
//***************************************************************************************************
#include <string.h>
#include "sensor.h"                                   // Definitions for the sensor(s) connected


//***************************************************************************************************
//                                  S E N S O R _ I N I T                                           *
//***************************************************************************************************
// Initialize the sensor.                                                                           *
//***************************************************************************************************
void sensor_init()
{
  // No initialization necessary for this "sensor".
}


//***************************************************************************************************
//                                  S E N S O R _ G E T D A T A                                     *
//***************************************************************************************************
// Get data from the sensor.  Store the result in the area pointed to by the parameter.             *
// The length will be returned as the function result.                                              *
//***************************************************************************************************
int sensor_getdata ( void* data )
{
  strcpy ( (char*)data, "Hello World!" ) ;             // Provide the data
  return strlen ( (char*)data ) ;
}

