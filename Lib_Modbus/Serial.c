/*
 * Arquivo obtido de: https://github.com/brunoluiz/28335ModbusSlave
 *
 *modificado para o DSP F28379D
 *
 * */





//#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File

#include <F28x_Project.h>
#include <lib/Modbus/Inc/Serial.h>
#include <lib/Modbus/Inc/Log.h>
#include <lib/Modbus/Inc/ModbusSettings.h>


//volatile struct SCI_REGS* SCI_MODBUS_PTR[2] = {&ScibRegs, &ScidRegs};
extern volatile struct SCI_REGS* SCI_PTR[5];

// Clear flags of overflow
void serial_clear(Serial *self){
	static unsigned short i, destroyFifo;

//	SERIAL_DEBUG();

	// Reset Serial in case of error
	if(SCI_PTR[self->modbus_port]->SCIRXST.bit.RXERROR == true){
		SCI_PTR[self->modbus_port]->SCICTL1.bit.SWRESET=0;
	}

	// Clears FIFO buffer (if there is any data)
	for (i = SCI_PTR[self->modbus_port]->SCIFFRX.bit.RXFFST; i > 0; i--)
	    destroyFifo = SCI_PTR[self->modbus_port]->SCIRXBUF.all;

	// Reset FIFO
	SCI_PTR[self->modbus_port]->SCIFFRX.bit.RXFIFORESET=1;
	SCI_PTR[self->modbus_port]->SCIFFTX.bit.TXFIFORESET=1;

	SCI_PTR[self->modbus_port]->SCICTL1.bit.SWRESET=1;

}

// Get how much data is at the RX FIFO Buffer
Uint16 serial_rxBufferStatus(Serial *self){
	return SCI_PTR[self->modbus_port]->SCIFFRX.bit.RXFFST;
}

// Enable or disable RX (receiver)
void serial_setSerialRxEnabled(bool status, Serial *self){
//	SERIAL_DEBUG();
	SCI_PTR[self->modbus_port]->SCICTL1.bit.RXENA = status;
}

// Enable or disable TX (trasmiter)
void serial_setSerialTxEnabled(bool status, Serial *self){
//	SERIAL_DEBUG();
	SCI_PTR[self->modbus_port]->SCICTL1.bit.TXENA = status;

}

// Initialize Serial (actually SCIA)
void serial_init(Serial *self){
	Uint32 baudrate;

//	// START: GOT FROM InitScia() FUNCTION (TEXAS FILES) ////////////////////////////////////////
//	EALLOW;
//
//	/* Enable internal pull-up for the selected pins */
//	// Pull-ups can be enabled or disabled disabled by the user.
//	// This will enable the pullups for the specified pins.
//	GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;    // Enable pull-up for GPIO28 (SCIRXDA)
//	GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;	   // Enable pull-up for GPIO29 (SCITXDA)
//
//	/* Set qualification for selected pins to asynch only */
//	// Inputs are synchronized to SYSCLKOUT by default.
//	// This will select asynch (no qualification) for the selected pins.
//	GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;  // Asynch input GPIO28 (SCIRXDA)
//
//	/* Configure SCI-A pins using GPIO regs*/
//	// This specifies which of the possible GPIO pins will be SCI functional pins.
//	GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;   // Configure GPIO28 for SCIRXDA operation
//	GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;   // Configure GPIO29 for SCITXDA operation
//
//	EDIS;
//	// END: GOT FROM InitScia() FUNCTION (TEXAS FILES) ////////////////////////////////////////
//	// Number of bytes


    // Configura os pinos da saída SCI-A
    /*  Esta função somento vai configurar as saídas no caso de ser usada a CPU1,
     * se for ser usada a CPU2 então a CPU1 deve fazer a configuração das saídas durante
     * a inicialização (antes da sincronização das CPUs).*/
#ifdef CPU1
//    GPIO_SetupPinMux(48, GPIO_MUX_CPU1, 6); // 6 = 0b0000 0110  gmux = 01b e mux = 10b -> SCITXDA
//    GPIO_SetupPinOptions(48, GPIO_OUTPUT, GPIO_ASYNC);
//    GPIO_SetupPinMux(49, GPIO_MUX_CPU1, 6); // 6 = 0b0000 0110  gmux = 01b e mux = 10b -> SCIRXDA
//    GPIO_SetupPinOptions(49, GPIO_INPUT, GPIO_PUSHPULL);
#endif

	switch(self->bitsNumber) {
		case 8:
		    SCI_PTR[self->modbus_port]->SCICCR.bit.SCICHAR = 0x7;
			break;
		case 7:
		    SCI_PTR[self->modbus_port]->SCICCR.bit.SCICHAR = 0x6;
			break;
		default:
		    SCI_PTR[self->modbus_port]->SCICCR.bit.SCICHAR = 0x7;
	}

	// Parity settings
	switch(self->parityType){
		case SERIAL_PARITY_EVEN:
		    SCI_PTR[self->modbus_port]->SCICCR.bit.PARITYENA = 1;
		    SCI_PTR[self->modbus_port]->SCICCR.bit.PARITY = 1;
			break;
		case SERIAL_PARITY_ODD:
		    SCI_PTR[self->modbus_port]->SCICCR.bit.PARITYENA = 1;
		    SCI_PTR[self->modbus_port]->SCICCR.bit.PARITY = 0;
			break;
		case SERIAL_PARITY_NONE:
		    SCI_PTR[self->modbus_port]->SCICCR.bit.PARITYENA = 0;
			break;
		default:
		    SCI_PTR[self->modbus_port]->SCICCR.bit.PARITYENA = 0;
	}

	// Baud rate settings - Automatic depending on self->baudrate
//	baudrate = (Uint32) (SysCtrlRegs.LOSPCP.bit.LSPCLK / (self->baudrate*8) - 1);
	baudrate = (Uint32) (LOW_SPEED_CLOCK / (self->baudrate*8) - 1);

	// Configure the High and Low baud rate registers
//	SCI_PTR[self->modbus_port]->SCIHBAUD = (baudrate & 0xFF00) >> 8;
//	SCI_PTR[self->modbus_port]->SCILBAUD = (baudrate & 0x00FF);
	SCI_PTR[self->modbus_port]->SCIHBAUD.all = (baudrate & 0xFF00) >> 8;
	SCI_PTR[self->modbus_port]->SCILBAUD.all = (baudrate & 0x00FF);

	// Enables TX and RX Interrupts
	SCI_PTR[self->modbus_port]->SCICTL2.bit.TXINTENA = 0;
	SCI_PTR[self->modbus_port]->SCIFFTX.bit.TXFFIENA = 0;
	SCI_PTR[self->modbus_port]->SCICTL2.bit.RXBKINTENA = 0;
	SCI_PTR[self->modbus_port]->SCIFFRX.bit.RXFFIENA = 0;

	// FIFO TX configurations
	SCI_PTR[self->modbus_port]->SCIFFTX.bit.TXFFIL = 1;	// Interrupt level
	SCI_PTR[self->modbus_port]->SCIFFTX.bit.SCIFFENA = 1;	// Enables FIFO
	SCI_PTR[self->modbus_port]->SCIFFTX.bit.TXFFINTCLR = 1;	// Clear interrupt flag

	// FIFO: RX configurations
	SCI_PTR[self->modbus_port]->SCIFFRX.bit.RXFFIL = 1;	// Interrupt level
	SCI_PTR[self->modbus_port]->SCIFFRX.bit.RXFFINTCLR = 1;	// Clear interrupt flag
	SCI_PTR[self->modbus_port]->SCIFFRX.bit.RXFFOVRCLR = 1;	// Clear overflow flag

	// FIFO: Control configurations
	SCI_PTR[self->modbus_port]->SCIFFCT.all=0x00;

	// Enable RX and TX and reset the serial
	SCI_PTR[self->modbus_port]->SCICTL1.bit.RXENA	 = 1;
	SCI_PTR[self->modbus_port]->SCICTL1.bit.TXENA	 = 1;
	SCI_PTR[self->modbus_port]->SCICTL1.bit.SWRESET = 1;

	// FIFO: Reset
	SCI_PTR[self->modbus_port]->SCIFFRX.bit.RXFIFORESET = 1;
	SCI_PTR[self->modbus_port]->SCIFFTX.bit.TXFIFORESET = 1;
	SCI_PTR[self->modbus_port]->SCIFFTX.bit.SCIRST = 0;
	SCI_PTR[self->modbus_port]->SCIFFTX.bit.SCIRST = 1;

//	SERIAL_DEBUG();
}

// Transmit variable data based on passed size
void serial_transmitData(Uint16 * data, Uint16 size, Serial *self){
	static unsigned short i = 0;
//	SERIAL_DEBUG();

	for (i = 0; i < size; i++){
//		SCI_PTR[self->modbus_port]->SCITXBUF= data[i];
	    SCI_PTR[self->modbus_port]->SCITXBUF.bit.TXDT = data[i];

		if(i%4 == 0){
			while (SCI_PTR[self->modbus_port]->SCICTL2.bit.TXEMPTY != true) ;
		}
	}

	// If you want to wait until the TX buffer is empty, uncomment line below
//	while (SCI_PTR[self->modbus_port]->SCICTL2.bit.TXEMPTY != true) ;
}

// Read data from buffer (byte per byte)
Uint16 serial_getRxBufferedWord(Serial *self){
//	SERIAL_DEBUG();

	// TODO: check if it is needed
	while (SCI_PTR[self->modbus_port]->SCIRXST.bit.RXRDY) ;

	return SCI_PTR[self->modbus_port]->SCIRXBUF.all;
}

bool serial_getRxError(Serial *self){
//	SERIAL_DEBUG();

	return SCI_PTR[self->modbus_port]->SCIRXST.bit.RXERROR;
}

// Construct the Serial Module
//Serial construct_Serial(){
Serial construct_Serial(modbus_com_port_t modbus_port){
	Serial serial;

	serial.clear = serial_clear;
	serial.rxBufferStatus = serial_rxBufferStatus;
	serial.setSerialRxEnabled = serial_setSerialRxEnabled;
	serial.setSerialTxEnabled = serial_setSerialTxEnabled;
	serial.init = serial_init;
	serial.transmitData = serial_transmitData;
	serial.getRxBufferedWord = serial_getRxBufferedWord;
	serial.getRxError = serial_getRxError;

	serial.fifoWaitBuffer = 0;

	serial.modbus_port = modbus_port;

//	SERIAL_DEBUG();

	return serial;
}
