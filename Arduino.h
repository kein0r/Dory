#ifndef Arduino_h
#define Arduino_h

/**
 * Replacement file to be able to compile Arduino code on a PC using gcc.
 */

#define F_CPU 16000000

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
/*#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)*/
#define sq(x) ((x)*(x))

/* TODO: replace with something more usefull */
#define millis() 5
#define NULL (void*)0

#define FORCE_INLINE
#define ISR(a)  void a(void)

/* Stubs related to steppers */
#define st_wake_up()

/* Testfunctions */
#define WRITE(a,b)
#define READ(a) 1
#define MSerial.checkRx()

extern uint16_t OCR1A;

/* Implement two classes ArduinoRegister and ArduinoCountingRegsiter for testing purposes */
#endif
