/*
 * ModbusVarMap.h
 *
 *  Created on: 08/10/2014
 *      Author: bds
 */

#ifndef MODBUSVARMAP_H_
#define MODBUSVARMAP_H_

#include <lib/Modbus/Inc/ModbusSettings.h>

#if MB_COILS_ENABLED
typedef struct ModbusCoilsMap ModbusCoilsMap;
struct ModbusCoilsMap{      // 4*16 = 64 bits
    Uint8 Inverter_Sentido;         // 0
    Uint8 Conectar_Equipamento;     // 1
    Uint8 Desconectar_Equipamento;  // 2
    Uint8 Reset_Protecao;           // 3
};

ModbusCoilsMap construct_ModbusCoilsMap();
#endif

#if MB_INPUTS_ENABLED
typedef struct ModbusInputsMap ModbusInputsMap;
struct ModbusInputsMap{     // 7*16 = 112 bits
    int8 Sentido_giro;              // 0
    Uint8 Equipamento_Energizado;   // 1
    Uint8 Equipamento_Operando;     // 2
    Uint8 Alarme;                   // 3
    Uint8 Operacao_Local_Remoto;    // 4
    Uint8 Bot_Emergencia;           // 5
    Uint8 RESET_DSP_INFO;           // 6
};

ModbusInputsMap construct_ModbusInputsMap();
#endif

#if MB_HOLDING_REGISTERS_ENABLED
typedef struct ModbusHoldingRegistersMap ModbusHoldingRegistersMap;
struct ModbusHoldingRegistersMap { // 32 bits
    Uint16 Ref_Regulacao_Tensao;       // 0
    Uint16 Lim_Corrente_Deseq_neg;     // 1
    Uint16 Lim_Corrente_Deseq_zero;    // 2
    Uint16 Lim_Corrente_Reativo;       // 3
    Uint16 Lim_Corrente_Harmonico;     // 4
};

ModbusHoldingRegistersMap construct_ModbusHoldingRegistersMap();
#endif

#if MB_INPUT_REGISTERS_ENABLED
typedef struct ModbusInputRegistersMap ModbusInputRegistersMap;
struct ModbusInputRegistersMap { // 6*32 = 192 bits
    Uint16 Velocidade_RPM;         // 0
    Uint16 Corrente_fase_A;        // 1
    Uint16 Corrente_fase_B;        // 2
    Uint16 Corrente_fase_C;        // 3
    Uint16 Codigo_Falta;           // 4
    Uint16 Codigo_Alarme;          // 5
};


ModbusInputRegistersMap construct_ModbusInputRegistersMap();
#endif

#endif /* MODBUSVARMAP_H_ */
