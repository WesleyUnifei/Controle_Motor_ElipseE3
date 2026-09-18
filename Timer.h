#ifndef MODBUS_TIMER_H_
#define MODBUS_TIMER_H_

#include <lib/Modbus/Inc/DataTypes.h>
#include <lib/Modbus/Inc/Serial.h>
typedef struct Timer Timer;

typedef enum{
    TIMER_UNDEFINED = 0,
    TIMER_0,
    TIMER_1,
    TIMER_2
}timer_num_t;

struct Timer {
	Uint32 reloadTime;
	bool timerEnabled;



	void (*resetTimer)(Timer *self);
	bool (*expiredTimer)(Timer *self);
	void (*setTimerReloadPeriod)(Timer *self, Uint32 time);
	void (*init)(Timer *self, Uint32 time);
	void (*stop)(Timer *self);
	void (*start)(Timer *self);

	timer_num_t timer_num;
};

inline void timer_resetTimer(Timer *self);
inline bool timer_expiredTimer(Timer *self);
inline void timer_setTimerReloadPeriod(Timer *self, Uint32 time);
inline void timer_init(Timer *self, Uint32 time);
inline void timer_stop(Timer *self);
inline void timer_start(Timer *self);

Timer construct_Timer(timer_num_t timer_num);

#endif
