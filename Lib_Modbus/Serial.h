#ifndef MODBUS_SERIAL_H_
#define MODBUS_SERIAL_H_

#include <lib/Modbus/Inc/DataTypes.h>
typedef struct Serial Serial;

//typedef enum{
//    MODBUS_SCI_B = 0,
//    MODBUS_SCI_D
//}modbus_com_port_t;

typedef enum{
    MODBUS_UNDEFINED = 0,
    MODBUS_SCI_A,
    MODBUS_SCI_B,
    MODBUS_SCI_C,
    MODBUS_SCI_D
}modbus_com_port_t;


// Parity constants
typedef enum {
	SERIAL_PARITY_NONE,
	SERIAL_PARITY_EVEN,
	SERIAL_PARITY_ODD
} SerialParity;

struct Serial {
	Uint16 bitsNumber;
	Uint16 parityType;
	Uint32 baudrate;

	Uint16 fifoWaitBuffer;

	void (*clear)(Serial *self);
	Uint16 (*rxBufferStatus)(Serial *self);
	void (*setSerialRxEnabled)(bool status, Serial *self);
	void (*setSerialTxEnabled)(bool status, Serial *self);
	void (*init)(Serial *self);
	void (*transmitData)(Uint16 * data, Uint16 size, Serial *self);
	Uint16 (*getRxBufferedWord)(Serial *self);
	bool (*getRxError)(Serial *self);

	modbus_com_port_t modbus_port;
};

void serial_clear(Serial *self);
inline Uint16 serial_rxBufferStatus(Serial *self);
inline void serial_setSerialRxEnabled(bool status, Serial *self);
inline void serial_setSerialTxEnabled(bool status, Serial *self);
inline void serial_init(Serial *self);
inline void serial_transmitData(Uint16 * data, Uint16 size, Serial *self);
inline Uint16 serial_getRxBufferedWord(Serial *self);
inline bool serial_getRxError(Serial *self);
Serial construct_Serial(modbus_com_port_t modbus_port);

#endif
