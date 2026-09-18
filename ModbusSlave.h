#ifndef MODBUS_SLAVE_H_
#define MODBUS_SLAVE_H_

#include <lib/Modbus/Inc/ModbusDefinitions.h>
#include <lib/Modbus/Inc/ModbusData.h>
#include <lib/Modbus/Inc/ModbusDataHandler.h>
#include MB_DATA_MAP
#include <lib/Modbus/Inc/Serial.h>
#include <lib/Modbus/Inc/Timer.h>
#include <lib/Modbus/Inc/Crc.h>



// typedef struct ModbusSlave ModbusSlave;

struct ModbusSlave {
	ModbusState state;

	ModbusData dataRequest;
	ModbusData dataResponse;

#if MB_COILS_ENABLED
	ModbusCoilsMap coils;
#endif
#if MB_INPUTS_ENABLED
	ModbusInputsMap inputs;
#endif
#if MB_HOLDING_REGISTERS_ENABLED
	ModbusHoldingRegistersMap holdingRegisters;
#endif
#if MB_INPUT_REGISTERS_ENABLED
	ModbusInputRegistersMap inputRegisters;
#endif

	Serial serial;
	Timer timer;

	ModbusDataHandler dataHandler;

	void (*loopStates)(ModbusSlave *self);
	void (*create)(ModbusSlave *self);
	void (*start)(ModbusSlave *self);
	void (*timerT35Wait)(ModbusSlave *self);
	void (*idle)(ModbusSlave *self);
	void (*receive)(ModbusSlave *self);
	void (*process)(ModbusSlave *self);
	void (*transmit)(ModbusSlave *self);
	void (*destroy)(ModbusSlave *self);

	bool jumpProcessState;
};


void slave_loopStates(ModbusSlave *self);
inline void slave_create(ModbusSlave *self);
inline void slave_start(ModbusSlave *self);
inline void slave_timerT35Wait(ModbusSlave *self);
inline void slave_receive(ModbusSlave *self);
inline void slave_process(ModbusSlave *self);
inline void slave_transmit(ModbusSlave *self);
inline void slave_destroy(ModbusSlave *self);
//ModbusSlave construct_ModbusSlave();
ModbusSlave construct_ModbusSlave(modbus_com_port_t modbus_port, timer_num_t timer_num);

extern ModbusSlave mb;

#endif
