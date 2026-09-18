/***************************************************************************************
 *  Firmware para DSP TMS320F28379D
 *
 *  Descrição: Controle de Motor de Indução Trifásico com Modbus RTU e Supervisório Elipse E3
 *             - Acionamento SPWM com modulação travada em ma_max = 0.89
 *             - Rampa de partida e parada suave com incrementos/decrementos de 0.01
 *             - Reversão de sentido de giro com trava de segurança de parada total (0 RPM)
 *             - Leitura e cálculo de correntes instantâneas e RMS com proteção de sobrecorrente
 *             - Leitura de velocidade por encoder em quadratura (eQEP)
 *             - Manter comentado as funções de TRIP de trava de segurança
 *_____________________________________________________________________________________
 *                                                                          J. Foster
 *                                                                         Agosto 2026
 *                                      Built with Code Composer Studio v12.8.1.00005
 *                                      Based on C2000Ware V5.04.00.00

                                        Atualizado em 11/09/2026 
                                            Wesley S. Marques
 **************************************************************************************/

/* Definição da versão do firmware e da data de atualização do firmware */
#define FIRMWARE_VERSION "5.3"                        /* Versão do firmware */
#define FIRMWARE_DATE "31/08/2026"                    /* Data dessa versão do firmware */
#define DSP_UPDATE_DATE "31/08/2026 09:00:00"         /* Data e hora de compilação */

#include "F28x_Project.h"
#include "Inc/Project/includes.h"
#include <F2837xD_Ipc_drivers.h>
#include <math.h>
#include "stdio.h"

#define PI 3.14159265358979f

/*====================================================================
 * PARÂMETROS E LIMITES DE CONTROLE E SEGURANÇA
 *====================================================================*/
#define MA_MAX                  0.89f   // Trava de modulação ótima para o motor (evita sobremodulação e distorção)
#define RAMP_STEP               0.02f   // Passo de incremento/decremento da rampa de modulação 0.02 para mais suave e não gerar ruído, leva cerca de 4.5s
#define RAMP_INTERVAL_COUNTS    2004    // Intervalo do passo da rampa: 2004 * 49.9us ~= 100ms (0.1s)
#define RPM_UPDATE_COUNTS       2004    // Intervalo de cálculo de RPM: ~100ms (10 leituras por segundo)
#define I_TRIP_MAX              2.00f   // Limite de sobrecorrente RMS por fase em Amperes (Motor In = 1.27A, SFA = 1.71A)
#define I_RESET_MAX (I_TRIP_MAX * 0.75f)    // Limite inferior para resetar o TRIP quando for seguro
#define RPM_ZERO_THRESHOLD      1.00f   // Limiar de rotação para considerar motor parado na reversão (em RPM)

/* Protótipos de funções */
interrupt void func_timer0(void);
void Setup_eQEP(void);
void Calc_RPM(void);
void Setup_Controle_PWM(void);
void Executar_Controle_Motor(void);

/* Variáveis do Sistema e Modbus */
#include "Inc/Project/Variaveis.h"

/*====================================================================
 * VARIÁVEIS DE VELOCIDADE (eQEP / ENCODER)
 *====================================================================*/
float rpm = 0.0f;
uint32_t new_pos = 0; 
static uint32_t old_pos = 0;
int32_t delta_pos = 0; 
float fator_encolder = 1.00f;     // Fator de calibração do encoder
uint32_t tempo_rpm = 0;

/*====================================================================
 * VARIÁVEIS DE MEDIÇÃO DE CORRENTE (ADC)
 *====================================================================*/
MEDIDA medidas;
SENSOR_OBJ SensorFase_A;
SENSOR_OBJ SensorFase_B;

float corrente_a = 0.0f;
float corrente_b = 0.0f;
float corrente_c = 0.0f;

float corrente_rms_a = 0.0f;
float corrente_rms_b = 0.0f;
float corrente_rms_c = 0.0f;

float acumulador_rms_a = 0.0f;
float acumulador_rms_b = 0.0f;
float acumulador_rms_c = 0.0f;

Uint16 k_amostras = 0;

/*====================================================================
 * VARIÁVEIS DE CONTROLE DO MOTOR E GERAÇÃO PWM (SPWM)
 *====================================================================*/
float sinetable_a[BUF_SIZE_SIGNAL];
float sinetable_b[BUF_SIZE_SIGNAL];
float sinetable_c[BUF_SIZE_SIGNAL];

float Duty_PWM1 = 0.0f;
float Duty_PWM2 = 0.0f;
float Duty_PWM4 = 0.0f;

float ma = 0.0f;                // Índice de modulação atual (0.0 a 0.89)
float ma_ref = 0.05f;           // Índice de modulação desejado (travado em no máximo 0.89)
float ma_alvo = 0.0f;           // Alvo dinâmico da rampa
Uint16 rampa_contador = 0;      // Contador de tempo para rampa (passo de 0.1 a cada ~100ms)

Uint16 motor_ligado = 0;        // 0 = Desligado / Parado, 1 = Ligado / Operando
Uint16 sentido_solicitado = 0;  // Sentido solicitado pelo Modbus (0 = Direto, 1 = Reverso)
Uint16 sentido_ativo = 0;       // Sentido de fase atualmente aplicado ao PWM
Uint16 em_reversao = 0;         // Flag que indica transição segura de reversão
Uint16 alarme_trip = 0;         // Flag de proteção/falha ativa


void main(void)
{
    Uint16 k;

    /* Inicializa GPIOs */
    InitGpio();

    EALLOW;
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0; // função GPIO
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;  // saída (LED indicador / Debug)
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;  // função GPIO
    EDIS;

    /* Desabilita e inicializa periféricos e interrupções */
    // 1. Inicializa o Sistema
    InitSysCtrl();
    DINT;
    InitPieCtrl();

    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();

    /*=========================================================
     * INICIALIZAÇÃO DE SENSORES E ADC (MEDIÇÃO DE CORRENTES)
     *=========================================================*/
    InitADC();               // Inicializa os módulos ADC
    InitMedidas(&medidas);   // Inicializa estrutura de medidas e offset DC

    // Configuração dos sensores de corrente das fases A e B
    // Sensor Fase A: ADC C, canal ADCIN3, ganho 1.43
    // Sensor Fase B: ADC B, canal ADCIN5, ganho 1.88
    SensorFase_A = SetupADC_Object(CONV_ADC_C, ADCIN3, RESULT0, TRIG_CPU1_TIMER0, ADC_INT_OFF, INT_OFF, Corrente_AC_Fase_A, CA, 1.43f, &medidas);
    SensorFase_B = SetupADC_Object(CONV_ADC_B, ADCIN5, RESULT0, TRIG_CPU1_TIMER0, ADC_INT1, INT_EOC0, Corrente_AC_Fase_B, CA, 1.88f, &medidas);

    /*=========================================================
     * TABELA SENOIDAL PARA SPWM (60 Hz, Fs = 20 kHz, 334 amostras)
     *=========================================================*/
    for(k = 0; k < BUF_SIZE_SIGNAL; k++){
        sinetable_a[k] = sinf(2.0f * PI * 60.0f * Ts * (float)k);
        sinetable_b[k] = sinf(2.0f * PI * 60.0f * Ts * (float)k - 2.0943951f); // -120°
        sinetable_c[k] = sinf(2.0f * PI * 60.0f * Ts * (float)k + 2.0943951f); // +120°
    }

    /*=========================================================
     * CONFIGURAÇÃO DOS MÓDULOS EPWM (EPWM1, EPWM2, EPWM4)
     *=========================================================*/
    Setup_Controle_PWM();

    /*=========================================================
     * CONFIGURAÇÃO DO HARDWARE DE ENCODER (eQEP)
     *=========================================================*/
    Setup_eQEP(); 

    /*=========================================================
     * CONFIGURAÇÃO DAS INTERRUPÇÕES DE TIMER
     *=========================================================*/
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.CPUTIMER0 = 1;      // Habilita clock do Timer 0
    PieVectTable.TIMER0_INT = &func_timer0;    // Redireciona interrupção do Timer 0
    EDIS;

    /*=========================================================
     * CONFIGURAÇÃO DA COMUNICAÇÃO SERIAL (MODBUS RTU - SCI-A)
     *=========================================================*/
    GPIO_SetupPinMux(43, GPIO_MUX_CPU2, 15);   // SCIRXDA
    GPIO_SetupPinOptions(43, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(42, GPIO_MUX_CPU2, 15);   // SCITXDA
    GPIO_SetupPinOptions(42, GPIO_OUTPUT, GPIO_ASYNC);
    
    InitCpuTimers();                           // Inicializa timers da CPU

    meu_escravo = construct_ModbusSlave(MODBUS_SCI_A, TIMER_1);
    SciaRegs.SCILBAUD.bit.BAUD = 53;           // Baudrate 115200 bps

    // Inicializa valores padrão no mapa Modbus
    meu_escravo.holdingRegisters.Ref_Regulacao_Tensao = 890; // Setpoint padrão = 500 (ma = 0.50)
    meu_escravo.inputs.Equipamento_Energizado = 1;

    meu_escravo.loopStates(&meu_escravo);      // Primeira leitura de estado

    /*=========================================================
     * INICIALIZAÇÃO FINAL DO SISTEMA
     *=========================================================*/
    DisableDog();                              // Desativa watchdog

    ConfigCpuTimer(&CpuTimer0, 100, Ts_us);    // Configura período da interrupção (49.9 us = 20040 Hz)
    ConfigInterrupt(vetorInterrupts);          // Habilita interrupções configuradas
    StartCpuTimer0();

    EINT;   // Habilita Interrupções Globais (INTM) 
    ERTM;   // Habilita Interrupções de Tempo Real (DBGM)

    while(1){
        /* Loop principal: processamento contínuo da pilha Modbus RTU */
        meu_escravo.loopStates(&meu_escravo);
    }
}


/*====================================================================
 * ROTINA DE INTERRUPÇÃO (TIMER0 - 20 kHz / 49.9 us)
 *====================================================================*/
__interrupt void func_timer0(void)
{
    /* 1. LEITURA DOS SENSORES ADC E CÁLCULO DAS CORRENTES */
    SampleADC_Object(&SensorFase_A);    // Amostra canal Fase A
    SampleADC_Object(&SensorFase_B);    // Amostra canal Fase B
    AjusteNivelDC(&medidas);            // Ajuste automático do nível DC (offset AC)
    Medida_Real(&SensorFase_A);         // Converte para valor real em Amperes
    Medida_Real(&SensorFase_B);         // Converte para valor real em Amperes

    corrente_a = *SensorFase_A.valorReal;
    corrente_b = *SensorFase_B.valorReal;
    corrente_c = -(corrente_a + corrente_b); // ia + ib + ic = 0 em sistema trifásico equilibrado

    // Acumulação para cálculo do RMS a cada ciclo da rede (334 amostras para 60 Hz)
    acumulador_rms_a += corrente_a * corrente_a;
    acumulador_rms_b += corrente_b * corrente_b;
    acumulador_rms_c += corrente_c * corrente_c;

    k_amostras++;
    if(k_amostras >= BUF_SIZE_SIGNAL){
        k_amostras = 0;

        corrente_rms_a = sqrtf(acumulador_rms_a * divAmostras);
        corrente_rms_b = sqrtf(acumulador_rms_b * divAmostras);
        corrente_rms_c = sqrtf(acumulador_rms_c * divAmostras);

        acumulador_rms_a = 0.0f;
        acumulador_rms_b = 0.0f;
        acumulador_rms_c = 0.0f;

    /*=========================================================
 * PROTEÇÃO DE SOBRECORRENTE
 *=========================================================*/
if(motor_ligado &&
   (corrente_rms_a > I_TRIP_MAX ||
    corrente_rms_b > I_TRIP_MAX ||
    corrente_rms_c > I_TRIP_MAX))
{
    alarme_trip = 1;

    motor_ligado = 0;

    ma = 0.0f;
    ma_alvo = 0.0f;

    meu_escravo.inputs.Alarme = 1;
    meu_escravo.inputRegisters.Codigo_Falta = 1;
}

/*=========================================================
 * TRATAMENTO DO TRIP
 *=========================================================*/
if(alarme_trip == 1)
{
    motor_ligado = 0;

    ma = 0.0f;
    ma_alvo = 0.0f;


     if(corrente_rms_a < I_RESET_MAX &&  // 1,5 [A]
       corrente_rms_b < I_RESET_MAX &&
       corrente_rms_c < I_RESET_MAX)
    {
        alarme_trip = 0;

        meu_escravo.inputs.Alarme = 0;
        meu_escravo.inputRegisters.Codigo_Falta = 0;

        motor_ligado = 0;
        ma = 0.0f;
        ma_alvo = 0.0f;
    }
} 


        // PONTE MODBUS: Disponibiliza as correntes RMS para o Elipse E3
        // Multiplicamos por 100 para manter 2 casas decimais no Supervisório (Ex: 1.27 A -> 127)
        meu_escravo.inputRegisters.Corrente_fase_A = (Uint16)(corrente_rms_a * 100);
        meu_escravo.inputRegisters.Corrente_fase_B = (Uint16)(corrente_rms_b * 100);
        meu_escravo.inputRegisters.Corrente_fase_C = (Uint16)(corrente_rms_c * 100);
    }

    /* 2. LEITURA DE VELOCIDADE (RPM) A CADA ~100 ms (2004 interrupções) */
    tempo_rpm++;
    if(tempo_rpm >= RPM_UPDATE_COUNTS) { 
        tempo_rpm = 0;
        Calc_RPM();
        
        // PONTE MODBUS: Atualiza RPM e Sentido medido para o Elipse E3
        meu_escravo.inputRegisters.Velocidade_RPM = (Uint16)fabsf(rpm);
        meu_escravo.inputs.Sentido_giro = (int8)EQep1Regs.QEPSTS.bit.QDF; 
    }

    /* 3. EXECUÇÃO DO CONTROLE DO MOTOR (PARTIDA, PARADA, REVERSÃO E SPWM) */
    Executar_Controle_Motor();

    /* 4. EXECUÇÃO DA MÁQUINA DE ESTADOS */
    State_Machine[State_Flags.estado_atual].function();

    /* Reconhecimento (ACK) da interrupção PIE */
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


/*====================================================================
 * LÓGICA DE CONTROLE DO MOTOR (PARTIDA, PARADA, REVERSÃO E SPWM)
 *====================================================================*/
void Executar_Controle_Motor(void){
    float half_prd = (float)EPwm1Regs.TBPRD * 0.5f;
    float sin_b, sin_c;

    /* A. TRATAMENTO DOS COMANDOS RECEBIDOS VIA MODBUS (ELIPSE E3) */
    
    // Comando de Reset de Proteção / Falha (Coil 2)
    if(meu_escravo.coils.Reset_Protecao == 1){
        alarme_trip = 0;
        meu_escravo.inputs.Alarme = 0;
        meu_escravo.inputRegisters.Codigo_Falta = 0;
        meu_escravo.coils.Reset_Protecao = 0; // Auto-reset do comando
    }

    // Comando de Partida (Coil 0: Conectar_Equipamento)
    if(meu_escravo.coils.Conectar_Equipamento == 1 && alarme_trip == 0){
        motor_ligado = 1;
        meu_escravo.coils.Conectar_Equipamento = 0; // Desarma comando de PARTIDA
    }

    // Comando de Parada (Coil 1: Desconectar_Equipamento)
    if(meu_escravo.coils.Desconectar_Equipamento == 1){
        motor_ligado = 0;
        meu_escravo.coils.Desconectar_Equipamento = 0;   // Desarma comando de DESLIGAMENTO
    }

    /* DESLIGAMENTO SEGURO VIA TRIP ZONE (Desativa A e B completamente) 
    if(motor_ligado == 0 || alarme_trip == 1)
    {
        EALLOW;
        // Força o Trip Zone (Desliga canal A e B de todos os PWMs)
        EPwm1Regs.TZFRC.bit.OST = 1;
        EPwm2Regs.TZFRC.bit.OST = 1;
        EPwm4Regs.TZFRC.bit.OST = 1;
        EDIS;
    }
    else
    {
        EALLOW;
        // Limpa o Trip Zone (Libera os PWMs para chavear)
        EPwm1Regs.TZCLR.bit.OST = 1;
        EPwm2Regs.TZCLR.bit.OST = 1;
        EPwm4Regs.TZCLR.bit.OST = 1;
        EDIS;
    }

    */

    // Leitura do comando de sentido solicitado (Coil 3: Inverter_Sentido)
    // 0 = Sentido Direto (Horário), 1 = Sentido Reverso (Anti-horário)
    sentido_solicitado = meu_escravo.coils.Inverter_Sentido;

    // Leitura do Setpoint de Modulação com TRAVA RIGOROSA em MA_MAX (0.89)
    if(meu_escravo.holdingRegisters.Ref_Regulacao_Tensao >= 0){
        ma_ref = (float)meu_escravo.holdingRegisters.Ref_Regulacao_Tensao / 1000.0f;
    } else {
        ma_ref = MA_MAX; // ma_ref = MA_MAX;   Valor padrão se 0
    }

    // Garante que o setpoint nunca ultrapasse MA_MAX (0.89)
    if(ma_ref > MA_MAX) 
    {
        ma_ref = MA_MAX;
    }

    /* B. TRAVA DE SEGURANÇA PARA REVERSÃO DE SENTIDO */
    // Se o sentido solicitado for diferente do sentido atualmente ativo:
    if(sentido_solicitado != sentido_ativo){
        em_reversao = 1; // Entra em modo de transição segura
    }

    if(em_reversao){
        // Durante a transição de reversão, desacelera o motor até parar completamente (ma = 0)
        ma_alvo = 0.0f;

        // TRAVA DE SEGURANÇA: Só comuta as fases quando ma estiver em 0.0 E o motor totalmente parado (RPM ~= 0)
        if(ma == 0.0f && fabsf(rpm) <= RPM_ZERO_THRESHOLD){
            sentido_ativo = sentido_solicitado; // Comuta a sequência de fases com segurança!
            em_reversao = 0;                    // Libera para voltar a acelerar no novo sentido
        }
    } else {
        // Operação normal: define o alvo de modulação com base no comando de ligar/desligar
        if(motor_ligado == 1 && alarme_trip == 0){
            ma_alvo = ma_ref; // Alvo = 0.89 (ou valor configurado no Holding Register)
        } else {
            ma_alvo = 0.0f;   // Alvo = 0.0 (Desligado)
        }
    }

    /* C. RAMPA SUAVE DE PARTIDA E DESLIGAMENTO (PASSO DE 0.1 A CADA ~100 ms) */
    rampa_contador++;
    if(rampa_contador >= RAMP_INTERVAL_COUNTS){
        rampa_contador = 0;

        // Incremento suave na partida (+0.1 por vez)
        if(ma < ma_alvo){
            ma += RAMP_STEP;
            if(ma > ma_alvo) {
                ma = ma_alvo;
            }
        }
        // Decremento suave no desligamento / desaceleração (-0.1 por vez)
        else if(ma > ma_alvo){
            ma -= RAMP_STEP;
            if(ma < ma_alvo) {
                ma = ma_alvo;
            }
        }
    }

    // Trava final de segurança de ma
    if(ma > MA_MAX) ma = MA_MAX;
    if(ma < 0.0f)   ma = 0.0f;

    // Atualização do status de operação para o Elipse E3
    meu_escravo.inputs.Equipamento_Operando = (ma > 0.0f || fabsf(rpm) > RPM_ZERO_THRESHOLD) ? 1 : 0;

    /* D. SELEÇÃO DA SEQUÊNCIA DE FASES (REVERSÃO DE SENTIDO) */
    if(sentido_ativo == 0){
        // Sequência Direta (A - B - C)
        sin_b = sinetable_b[k_amostras];
        sin_c = sinetable_c[k_amostras];
    } else {
        // Sequência Reversa (A - C - B): Inverte as fases B e C
        sin_b = sinetable_c[k_amostras];
        sin_c = sinetable_b[k_amostras];
    }

    /* E. CÁLCULO DO DUTY CYCLE SPWM */

        Duty_PWM1 = ma * sinetable_a[k_amostras] * half_prd + half_prd;
        Duty_PWM2 = ma * sin_b * half_prd + half_prd;
        Duty_PWM4 = ma * sin_c * half_prd + half_prd;
    

    // Saturação dos limites dos contadores de PWM (0 a TBPRD)
    if (Duty_PWM1 > (float)EPwm1Regs.TBPRD) { Duty_PWM1 = (float)EPwm1Regs.TBPRD; }
    else if (Duty_PWM1 < 0.0f) { Duty_PWM1 = 0.0f; }

    if (Duty_PWM2 > (float)EPwm1Regs.TBPRD) { Duty_PWM2 = (float)EPwm1Regs.TBPRD; }
    else if (Duty_PWM2 < 0.0f) { Duty_PWM2 = 0.0f; }

    if (Duty_PWM4 > (float)EPwm1Regs.TBPRD) { Duty_PWM4 = (float)EPwm1Regs.TBPRD; }
    else if (Duty_PWM4 < 0.0f) { Duty_PWM4 = 0.0f; }

    // Atualização dos registradores CMPA dos periféricos ePWM
    EPWM1_Modulante_CMPA = (Uint16)Duty_PWM1;
    EPWM2_Modulante_CMPA = (Uint16)Duty_PWM2;
    EPWM4_Modulante_CMPA = (Uint16)Duty_PWM4;
}


/*====================================================================
 * CONFIGURAÇÃO DOS PERIFÉRICOS EPWM
 *====================================================================*/
void Setup_Controle_PWM(void){
    // Configura GPIO 8 e 9 para saída 0 (evita acionamento indevido de drivers/travas)
    ConfigGPIO(8, SAIDA, NULL, NULL, NULL, NULL, NULL);
    ConfigGPIO(9, SAIDA, NULL, NULL, NULL, NULL, NULL);
    GpioDataRegs.GPACLEAR.bit.GPIO8 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;

    StartEPWMConfig();  // Pausa clock do ePWM para sincronização

    InitEPwmGpio_firmware();    // Configura os pinos de saída PWM

    // Configura ePWM 1, 2 e 4 (Frequência de comutação ~ 10 kHz)
    ConfigEPwm_REF(EPWM1, ePWM_HSPCLKDIV_2, ePWM_CLKDIV_1, 10020);
    ConfigEPwm_REF(EPWM2, ePWM_HSPCLKDIV_2, ePWM_CLKDIV_1, 10020);
    ConfigEPwm_REF(EPWM4, ePWM_HSPCLKDIV_2, ePWM_CLKDIV_1, 10020);

    // Deadband para evitar curto de braço (Tempo de subida e descida: 3us e 4us)
    // Os IGBTs IRGB15B60KD possuem tempo de desligamento td(off) + tf ~= 230ns.
    // 3us a 4us é mais que suficiente e garante proteção total contra shoot-through.
    ConfigDeadBandPWM(EPWM1, 3, 4);
    ConfigDeadBandPWM(EPWM2, 3, 4);
    ConfigDeadBandPWM(EPWM4, 3, 4);

/*
EALLOW;

         // Configura o Trip Zone para forçar nível BAIXO (0V) em A e B quando acionado

// PWM1
EPwm1Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
EPwm1Regs.TZCTL.bit.TZB = TZ_FORCE_LO;

// PWM2
EPwm2Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
EPwm2Regs.TZCTL.bit.TZB = TZ_FORCE_LO;

// PWM4
EPwm4Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
EPwm4Regs.TZCTL.bit.TZB = TZ_FORCE_LO;

EDIS;
*/
    ConfigSyncPWMs();           // Sincroniza fases dos módulos PWM
    //InitEPwmGpio_firmware();    // Configura os pinos de saída PWM , COLOCADA ACIMA P TESTE linha (486)
    EndEPWMConfig();            // 6. Libera o clock dos ePWMs para iniciarem a contagem junto
    
}


/*====================================================================
 * HARDWARE DE LEITURA DE VELOCIDADE (eQEP)
 *====================================================================*/
void Setup_eQEP(void){
    EALLOW;
    CpuSysRegs.PCLKCR4.bit.EQEP1 = 1; // Liga clock do eQEP1
    
    // Configuração dos pinos do encoder (GPIO10: EQEP1A, GPIO11: EQEP1B)
    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1;     // Disabilita pull-up interno (evita pino flutuante)
    GpioCtrlRegs.GPAQSEL1.bit.GPIO10 = 0;   // QUALIFICAÇÃO POR 6 AMOSTRAS (Filtro digital contra EMI)
    GpioCtrlRegs.GPAGMUX1.bit.GPIO10 = 1;   
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;    // Configura como EQEP1A

    
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1;     // Disabilita pull-up interno
    GpioCtrlRegs.GPAQSEL1.bit.GPIO11 = 0;   // QUALIFICAÇÃO POR 6 AMOSTRAS (Filtro digital contra EMI)
    GpioCtrlRegs.GPAGMUX1.bit.GPIO11 = 1;   
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;    // Configura como EQEP1B
    EDIS;

    EQep1Regs.QDECCTL.bit.QSRC = 0;         // Modo quadratura
    EQep1Regs.QDECCTL.bit.XCR = 0;          // Resolução 4x
    EQep1Regs.QEPCTL.bit.PCRM = 1;          // Reset no QPOSMAX
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2;     // Modo emulacao
    EQep1Regs.QPOSMAX = 0xFFFFFFFF;         // Contagem máxima
    EQep1Regs.QEPCTL.bit.QPEN = 1;          // Habilita módulo eQEP
    
} 



void Calc_RPM(void){
    
    // 2. Leitura da posição relativa
    new_pos = EQep1Regs.QPOSCNT;    
    delta_pos = (int32_t)(new_pos - old_pos); 
    old_pos = new_pos;

    // 3. Cálculo do RPM (janela de amostragem de ~100ms = 0,1s)
    rpm = ((float)(delta_pos)) / (360.0f * fator_encolder * 4.0f) * 600.0f;

}


/*====================================================================
 * FUNÇÕES DA MÁQUINA DE ESTADOS
 *====================================================================*/
void Idle(void){
}

void Inicializacao(void){
} 

void Operacao(void){
}

void Desligamento(void){
} 

void TratamentoFaltas(void){
}