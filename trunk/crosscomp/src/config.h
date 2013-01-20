#ifndef config_h
#define config_h

//#define SERIAL_COM_SPEED 256000
//#define SERIAL_COM_SPEED 115200
#define SERIAL_COM_SPEED 57600

#define I2C_SPEED       400000L
//#define MWC_DEBUG

#define RC_MINTHROTTLE  1000
#define RC_MAXTHROTTLE  2000
#define RC_MINCHECK     1100
#define RC_MAXCHECK     1900
#define RC_MIDPOINT     1500

#define RC_DEADBAND_RP  2
#define RC_DEADBAND_YAW 5

#if (CONFIG == _TAROT450_)
  #include <conf/tarot450.h>
#endif
#if (CONFIG == _MINIX_)
  #include <conf/minix.h>
#endif
#if (CONFIG == _SHURICUS_)
  #include <conf/shuricus.h>
#endif
#if (CONFIG ==_STM32F3DISC_)
  #include <conf/stm32fdisc_conf.h>
#endif
#if (CONFIG == _NONE_)
  #include <conf/default.h>
#endif

#endif

