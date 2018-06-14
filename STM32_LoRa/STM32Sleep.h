//***************************************************************************************************
//*                               S T M 3 2 S L E E P . H                                           *
//***************************************************************************************************
// Copied from https://github.com/chacal/stm32sleep                                                 *
//***************************************************************************************************
#include <libmaple/gpio.h>
#include <RTClock.h>

typedef enum SleepMode
{
  STOP,
  STANDBY
} SleepMode ;

void sleepAndWakeUp ( SleepMode mode, RTClock *rt, uint8_t seconds ) ;

void goToSleep ( SleepMode mode ) ;

void disableAllPeripheralClocks() ;

void setGPIOModeToAllPins ( gpio_pin_mode mode ) ;

void switchToPLLwithHSE ( rcc_pll_multiplier pllMultiplier ) ;
