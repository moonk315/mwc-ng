#ifndef config_h
#define config_h

//#define SERIAL_COM_SPEED 256000
//#define SERIAL_COM_SPEED 115200
#define SERIAL_COM_SPEED 57600

#define I2C_SPEED 400000L
//#define MWC_DEBUG

#define RC_MINTHROTTLE 1000
#define RC_MAXTHROTTLE 2000
#define DEADBAND_RP  2
#define DEADBAND_YAW 5
#define MIDRC 1500

#if (CONFIG == _TAROT450_)
  #include <conf/tarot450.h>
#endif
#if (CONFIG == _MINIX_)
  #include <conf/minix.h>
#endif
#if (CONFIG == _SHURICUS_)
  #include <conf/shuricus.h>
#endif
#if (CONFIG == _NONE_)
  #include <conf/default.h>
#endif

#endif

