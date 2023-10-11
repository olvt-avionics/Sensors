/*********************************************************************************

Copyright(c) 2017 Analog Devices, Inc. All Rights Reserved.

This software is proprietary and confidential.  By using this software you agree
to the terms of the associated Analog Devices License Agreement.
********************************************************************************/

#ifndef SENSOR_ADXL355_H
#define SENSOR_ADXL355_H
#define XL355_S_H

#include <cassert>
#include <string.h>
#include <stdio.h>
#include "adi_adxl355.h"
#include "board_adxl355.h"
#include "sensor_adxl355.h"
#include "aduc702x_uart.h" //February 2021 Ray: ?


/*! Acknowledgement sent to the slave when data ready signal is asserted by slave */ //February 2021 Ray: ?
#define DREADY_ACK              (0x0800u)
#define PAYLOAD_BUFF_MAX_SIZE   (32)             /*!< Payload max size */
#define PAYLOAD_SIZE_IDX        (1)              /*!< Payload size index */
#define HEADER_SIZE             (2)              /*!< Header index */
#define SEND_HEADER_SIZE        (3)              /*!< Header size index */
#ifdef BEACON_MODE
#define BTLE_PACKET_SIZE  (26u)                  /*!< Beacon packet size */
#else
#define BTLE_PACKET_SIZE  (20u)                  /*!< Paired packet size */
#endif
#define ODR_BLE_LIMIT     (4u)                   /*!< ODR limit for BLE communication */
#define BUFSIZE              1024
#define RESULT_SUCCESS  (0)
#define RESULT_ERROR    (1)
#define MAX_PAYLOAD_REGS (8)
#define RTC_ALARM_SECONDS (5)
#define MAX_REGS    (50)

#define PRINT_ERROR     "Unknown Error"
#define ADXL_ERROR      "ADXL355 Failed\n"
#define SERIAL_ERROR    "Serial Failed\n"
#define SPI_ERROR       "SPI Failed\n"
#define IRQ_IND         "Interrupt\n"
#define SYS_IND         "OK\n"

/*! Receive command enumeration. */
typedef enum
{
    ADI_RCV_CMD_NONE                       = 0x00,   /*!< Reserved */
    ADI_RCV_CMD_RESET                      = 0x01,   /*!< Reset command */
    ADI_RCV_CMD_SET_MEASURE_RANGE          = 0x02,   /*!< Set range command */
    ADI_RCV_CMD_TEMP_ENABLE                = 0x03,   /*!< Temperature enable command */
    ADI_RCV_CMD_MEASURE_ENABLE             = 0x04,   /*!< Measure enable command */
    ADI_RCV_CMD_SET_USER_OFFSET_ADJUST     = 0x05,   /*!< Offset setting command */
    ADI_RCV_CMD_SET_FILTER_SETTINGS        = 0x06,   /*!< Filter setting command */
    ADI_RCV_CMD_CONFIG_ACT_DETECT_MODE     = 0x07,   /*!< Activity detection command */
    ADI_RCV_CMD_SET_FIFO_SAMPLES           = 0x08,   /*!< FIFO settings command */
    ADI_RCV_CMD_SELF_TEST_ENABLE           = 0x09,   /*!< Self-test command */ 

    ADI_RCV_CMD_SET_REGISTER               = 0x20,   /*!< Set register command */
    ADI_RCV_CMD_GET_REGISTER               = 0x21,   /*!< Get register command */
    ADI_RCV_CMD_GET_ALL_REGISTERS          = 0x22,   /*!< Get all registers command */
    ADI_RCV_CMD_GET_TEMP               		 = 0x23,   /*!< Get register command */
    
    ADI_RCV_CMD_SERIAL                     = 0x30,   /*!< Get register command */
    ADI_RCV_CMD_SERIAL_DETAILS             = 0x31    /*!< Get all registers command */
} ADI_RCV_CMD;

/*! Transmit command enumeration. */
typedef enum
{
    ADI_XMIT_CMD_RESPONSE                  =  0x03,   /*!< Send response command */
    ADI_XMIT_CMD_ACCEL_DATA                =  0x04,   /*!< Send data command */
    ADI_XMIT_CMD_TEMP_DATA                 =  0x05,   /*!< Send temperature command */
    ADI_XMIT_CMD_REG_DATA                  =  0x06,   /*!< Send register command */
    ADI_XMIT_CMD_STATUS                    =  0x07,   /*!< Send status command */
    ADI_XMIT_CMD_SELF_TEST_DATA            =  0x08,   /*!< Send self-test data command */
    ADI_XMIT_CMD_DATA                 		 =  0x09   	/*!< Send command */
} ADI_XMIT_CMD;

struct PACKET {
        uint8_t sensorID;                                /*!< The sensor ID */
        uint8_t cmdID;                                   /*!< The command ID */
        uint8_t payload[PAYLOAD_BUFF_MAX_SIZE];          /*!< The payload buffer */
};


extern struct PACKET packet;                             /*!< Data packet */
extern ADI_ADXL355_HANDLE  hADXL355Device;

extern unsigned char    input[BUFSIZE];
extern unsigned char    output[BUFSIZE];

extern int16_t nSize;

extern uint8_t sensorID;
//extern uint8_t payloadLength;
extern bool_t bMeasureEnabled;
extern bool_t bTempEnabled;
extern bool_t eTimeOut;
extern int rangeG;

extern int16_t  temperature;
extern uint8_t  nSamples;
extern uint8_t  nCurrentSample;
extern uint16_t packetCount;

extern uint8_t SpRegAddr[50];
extern uint8_t SpRegData[50];

extern uint8_t  nRegs;
extern uint8_t  nCurrentReg;
   
extern uint8_t  status;

extern int32_t Xself;
extern int32_t Yself;
extern int32_t Zself;

extern bool_t bEnabled;
extern bool_t bNewRxData;                           /*!< New data flag */


static uint8_t regData[MAX_REGS * 2];      // address and data
static int32_t accelData[FIFO_DEPTH * 3];  // depth * 3-Axis

/* interrupt flags */
extern volatile unsigned char gXL355Int1;
extern volatile unsigned char gXL355Int2;

extern volatile bool_t bHibernateExitFlag;
extern volatile bool_t bRtcAlarmFlag;

unsigned char checkForSCommand(bool_t *SCmdReady);    
void Serial_initComm(void);
int Serial_enableComm(bool_t bEnable);
int Serial_writeData(void);
int Serial_checkForSCommand(bool_t *SCmdReady);
int Serial_getSCommand(struct PACKET *packet);
int Serial_sendIdentity(struct PACKET *packet);
int Serial_sendResponseOverSerial(int response);

//ADUC702x target
void System_Init(void);

//Serial board
void Board_ADXL355_Init(void);

//ADXL355 sensor
int ADXL355_config(void);
int ADXL355_Reset(void);
int ADXL355_readStatus(void);
int ADXL355_readAccel(void);
int ADXL355_readAccelXYZ(void);
int ADXL355_readTemp(void);
int ADXL355_setTempEnable(uint8_t enable);
int ADXL355_setRange(int range);
int ADXL355_setMeasureEnable(int enable);
int ADXL355_setSelfTest(void);
int ADXL355_ConfigActivityDetection(uint8_t ActivityEnable, uint16_t Threshold, uint8_t Count);
int ADXL355_OffsetAdjust(uint8_t *pOffsets);
int ADXL355_ConfigFilter(uint8_t hpf_corner, uint8_t odr_lpf);
int ADXL355_SetFifoSamples(uint8_t samples);
int ADXL355_setRegister(uint8_t address, uint8_t value);
int ADXL355_getRegister(uint8_t address);
int ADXL355_getAllRegisters(void);
void ADXL355_formatPacketStatus(struct PACKET *pPacket);
void ADXL355_formatPacketTemp(struct PACKET *pPacket);
void ADXL355_formatPacketReg(struct PACKET *pPacket);
void ADXL355_formatPacketSelfTestData(struct PACKET *pPacket);
void ADXL355_Format_Serial_AllRegisters(struct PACKET *pPacket);
void ADXL355_Format_Serial_Registers(struct PACKET *pPacket);
void ADXL355_Format_Serial_SelfTestData(struct PACKET *pPacket);
void ADXL355_Format_Serial_Data(struct PACKET *pPacket);
void ADXL355_Format_Serial_Temperature(struct PACKET *pPacket);
int ADXL355_readPCon(void);

#endif /* SENSOR_ADXL355_H */
