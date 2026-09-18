/*
 * Arquivo obtido de: https://github.com/brunoluiz/28335ModbusSlave
 *
 * modificado para o DSP F28379D
 *
 * */





//#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
//#include "DSP2833x_CpuTimers.h"

#include <F28x_Project.h>
#include <lib/Modbus/Inc/Timer.h>
#include <lib/Modbus/Inc/Log.h>
#include <lib/Modbus/Inc/ModbusSettings.h>

/* Timer 0 é usado pela CPU
 * Timer 1 é usado pelo Modbus da SCI-B
 * Timer 2 é usado pelo Modbus da SCI-D */
extern struct CPUTIMER_VARS CpuTimer1;
extern struct CPUTIMER_VARS CpuTimer2;

volatile struct CPUTIMER_VARS* TIMER_MODBUS_PTR[4] = {0x00, &CpuTimer0, &CpuTimer1, &CpuTimer2};
volatile struct CPUTIMER_REGS* TIMER_REG_MODBUS_PTR[4] = {0x00, &CpuTimer0Regs, &CpuTimer1Regs, &CpuTimer2Regs};

void timer_resetTimer(Timer *self){
//	TIMER_REG_MODBUS_PTR[self->timer_num]->TCR.bit.TRB = 1;
    TIMER_REG_MODBUS_PTR[self->timer_num]->TCR.bit.TRB = 1;
	TIMER_DEBUG();

}

bool timer_expiredTimer(Timer *self){
	Uint32 timerZeroed = TIMER_REG_MODBUS_PTR[self->timer_num]->TCR.bit.TIF;
//	TIMER_DEBUG();

	if (timerZeroed == true) {
		return true;
	}
	else {
		return false;
	}
}

void timer_setTimerReloadPeriod(Timer *self, Uint32 time){
//	TIMER_DEBUG();

	self->stop(self);
	self->reloadTime = time;

//	TIMER_MODBUS_PTR[self->timer_num]->CPUFreqInMHz = CPU_FREQ;
	TIMER_MODBUS_PTR[self->timer_num]->CPUFreqInMHz = CPU_FREQ;
	TIMER_MODBUS_PTR[self->timer_num]->PeriodInUSec = time;
	TIMER_MODBUS_PTR[self->timer_num]->RegsAddr->PRD.all = (long) time * CPU_FREQ;
}


void timer_init(Timer *self, Uint32 time){
	// START: GOT FROM TEXAS FILES //////////////////////////////////////
	// CPU Timer 0
    // Initialize address pointers to respective timer registers:
    TIMER_MODBUS_PTR[self->timer_num]->RegsAddr = TIMER_REG_MODBUS_PTR[self->timer_num];
    // Initialize pre-scale counter to divide by 1 (SYSCLKOUT):
    TIMER_REG_MODBUS_PTR[self->timer_num]->TPR.all  = 0;
    TIMER_REG_MODBUS_PTR[self->timer_num]->TPRH.all = 0;
    // Make sure timer is stopped:
    TIMER_REG_MODBUS_PTR[self->timer_num]->TCR.bit.TSS = 1;
    // Reload all counter register with period value:
    TIMER_REG_MODBUS_PTR[self->timer_num]->TCR.bit.TRB = 1;
    // Reset interrupt counters:
    TIMER_MODBUS_PTR[self->timer_num]->InterruptCount = 0;
    // END: GOT FROM TEXAS FILES ////////////////////////////////////////

	// Config the timer reload period
	self->reloadTime = time;
	TIMER_MODBUS_PTR[self->timer_num]->CPUFreqInMHz = CPU_FREQ;
	TIMER_MODBUS_PTR[self->timer_num]->PeriodInUSec = time;
	TIMER_REG_MODBUS_PTR[self->timer_num]->PRD.all = (long) time * CPU_FREQ;

	// Run mode settings
	TIMER_REG_MODBUS_PTR[self->timer_num]->TCR.bit.SOFT = 1;
	TIMER_REG_MODBUS_PTR[self->timer_num]->TCR.bit.FREE = 1;     // Timer Free Run

	// If needed, you can set interruptions and other things here
	//	TIMER_MODBUS_PTR[self->timer_num]->RegsAddr->TCR.bit.TIE = 1;      // 0 = Disable/ 1 = Enable Timer Interrupt

//	TIMER_DEBUG();
}

void timer_stop(Timer *self){
	TIMER_REG_MODBUS_PTR[self->timer_num]->TCR.bit.TSS = 1;
//	TIMER_DEBUG();
}

void timer_start(Timer *self){
	TIMER_REG_MODBUS_PTR[self->timer_num]->TCR.bit.TSS = 0;
//	TIMER_DEBUG();
}

Timer construct_Timer(timer_num_t timer_num){
	Timer timer;

	timer.timerEnabled = false;
	timer.reloadTime = 0;

	timer.resetTimer = timer_resetTimer;
	timer.expiredTimer = timer_expiredTimer;
	timer.setTimerReloadPeriod = timer_setTimerReloadPeriod;
	timer.init = timer_init;
	timer.stop = timer_stop;
	timer.start = timer_start;

	timer.timer_num = timer_num;

//	TIMER_DEBUG();

	return timer;
}
