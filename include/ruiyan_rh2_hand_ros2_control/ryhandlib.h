#ifndef _RY_HAND_LIB_H_
#define _RY_HAND_LIB_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// 0 - Target platform is C language embedded/Linux platform (.lib .o .a .so ...)
// 1 - Target platform is Windows C++ DLL platform (.dll)
#ifndef Target_Platform
#define Target_Platform 0
#endif

// 0 - Compile library
// 1 - Reference library
#ifndef ExportType
#define ExportType 1
#endif

#if (ExportType == 0)  // Compile library

#if (Target_Platform == 0)

#define Exportmode

#elif (Target_Platform == 1)

#define Exportmode __declspec(dllexport)

#endif

#elif (ExportType == 1)  // Reference library

#if (Target_Platform == 0)

#define Exportmode

#elif (Target_Platform == 1)

#define Exportmode extern "C" __declspec(dllimport)

#endif

#endif

#define USE_ASYN_PRASE 1

#define C51  // reentrant

#ifndef NULL
#define NULL ((void *)0)
#endif

#define ULIB_ENABLE 1
#define ULIB_DISABLE 0

#define CMD_SUCCEED 1
#define CMD_FAIL 0

#define SERVO_BACK_ID(x) (x + 256u)
#define UPDATE_INFO 0xa0

typedef char s8_t;
typedef unsigned char u8_t;
typedef unsigned short u16_t;
typedef short s16_t;
typedef unsigned int u32_t;
typedef int s32_t;
typedef unsigned long long u64_t;
typedef long long s64_t;

typedef enum
{

  enServo_OK = 0,           // 0 Motor response function operation successful 1
  enServo_TempratureHighW,  // 1 Motor over-temperature warning
  enServo_TempratureHighE,  // 2 Motor over-temperature protection
  enServo_VoltageLowE,      // 3 Motor low-voltage protection
  enServo_VoltageHighE,     // 4 Motor over-voltage protection
  enServo_CurrentOverE,     // 5 Motor over-current protection
  enServo_TorqueOverE,      // 6 Motor torque protection
  enServo_FuseE,            // 7 Motor fuse misalignment protection
  enServo_PwmE,             // 8 Motor stall protection
  enServo_DriveE,           // 9 Driver anomaly protection
  enServo_HallE,            // 10 Motor Hall error protection

  enServoStatus_Bottom,

  enServo_Fail = 250,        //	Servo unresponsive function operation or response timeout
  enServo_ParamErr = 251,    //	Servo response data error
  enServo_LibInitErr = 252,  //	Servo group structure read/write function uninitialized or failed

  enCanFormatError = 253,      //  Asynchronous parsing: CAN data format mismatch
  enCanMsgSentFail = 254,      //  CAN message send failed
  enLibHookApplyFailed = 255,  //  Hook application failed  255
} enret_t;

typedef struct
{
  u32_t ulId;       // id
  u8_t ucLen;       // Data length
  u8_t pucDat[64];  // Data content
} CanMsg_t;

typedef s8_t (*BusWrite_t)(CanMsg_t stuMsg) C51;
typedef void (*Callback_t)(CanMsg_t stuMsg, void * para) C51;

#pragma pack(1)

typedef struct
{
  u64_t ucCmd : 8;     // Previous command (CMD)
  u64_t ub_year : 10;  // Years after 2000, 2021 corresponds to 21
  u64_t ub_month : 4;
  u64_t ub_day : 5;
  u64_t ub_hour : 5;
  u64_t ub_min : 6;
  u64_t ub_s : 6;
  u64_t ub_ms : 10;
  u64_t ub_us : 10;
} DevTimeCmd_t;

typedef struct
{
  u64_t : 8;           // Previous CMD
  u64_t ucStatus : 8;  // Fault status. 0 means no fault. For abnormal details, see enret_t.
  u64_t ub_P : 12;     // Current position: 0-4095 corresponds to 0 to full travel.
  u64_t ub_V : 12;     // Current speed, -2048~2047, unit 0.001 travel/s
  u64_t ub_I : 12;     // Current current, -2048~2047, unit 0.01A
  u64_t ub_F
    : 12;  // Current position: 0-4095 corresponds to the original ADC value of the finger pressure sensor.
} FingerServoInfo_t;

typedef struct
{
  u64_t : 8;        // Previous CMD
  u64_t usTp : 16;  // Target position
  u64_t usTv : 16;  // Target speed
  u64_t usTc : 16;  // Target current
  u64_t : 8;        // Reserved
} FingerServoCmd_t;

typedef union {
  FingerServoCmd_t stuCmd;
  FingerServoInfo_t stuInfo;
  u8_t pucDat[64];
} ServoData_t;

#pragma pack()

typedef struct
{
  volatile u8_t ucEn;     // Enable switch
  volatile u8_t ucAlive;  // Hook lifecycle value; after a new Hook is added, this value is 255.
  // Each time a CAN frame is received, a hook match is performed, and this value decreases by 1.
  // When it reaches 0, the hook automatically changes from an enabled to a disabled state.
  CanMsg_t * pstuMsg;
  Callback_t funCbk;
} MsgHook_t;

typedef struct
{
  MsgHook_t stuListen;
  ServoData_t stuRet;
  u8_t
    ucConfidence;  // Data credibility. A higher value indicates greater data credibility, with a maximum of 255.
  // A value of 0 means the data is unreliable, indicating many CAN messages were received but the data was not successfully updated.
} MsgListen_t;

typedef struct
{
  volatile u16_t *
    pusTicksMs;         // User program implemented ms counter address, period is 1000, i.e., 0~999.
  u16_t usTicksPeriod;  // ms counter period
  u16_t usHookNum;
  u16_t usListenNum;
  MsgHook_t * pstuHook;      // hook List head address
  MsgListen_t * pstuListen;  // listen List head address
  BusWrite_t pfunWrite;      // CAN device write interface address
} RyCanServoBus_t;

//*********************************************************************************************************************************
// Function Name: AddHook
// Functionality: Adds a message hook
// Parameters:
//   *pstuCan - [i] Address of the servo CAN bus object
//   *pstuMsg - [i] Message characteristics for adding the message hook (ID and cmd). Note: If the user operates the Hook asynchronously, set ID = actual servo ID + 256.
//   funCallback - [i] Callback function for the message hook
// Return Value: Index of the corresponding message hook on success, -1 on failure
//*********************************************************************************************************************************
Exportmode s16_t AddHook(RyCanServoBus_t * pstuCan, CanMsg_t * pstuMsg, Callback_t funCallback);

//*********************************************************************************************************************************
// Function Name: DeleteHook
// Functionality: Deletes a message hook
// Parameters:
//   *pstuCan - [i] Address of the servo CAN bus object
//   *pstuMsg - [i] Message characteristics for deleting the message hook (ID and cmd). Note: If the user operates the Hook asynchronously, set ID = actual servo ID + 256.
// Return Value: 0 on success, -1 on failure
//*********************************************************************************************************************************
Exportmode s16_t DeleteHook(RyCanServoBus_t * pstuCan, CanMsg_t * pstuMsg);

//*********************************************************************************************************************************
// Function Name: AddListen
// Functionality: Adds a message listener
// Parameters:
//   *pstuCan - [i] Address of the servo CAN bus object
//   *pstuMsg - [i] Message characteristics for adding the message listener (ID and cmd). Note: When operating a Listen, set ID = actual servo ID + 256.
//   funCallback - [i] Callback function for the message listener
// Return Value: Index of the corresponding message listener on success, -1 on failure
//*********************************************************************************************************************************
Exportmode s16_t AddListen(RyCanServoBus_t * pstuCan, CanMsg_t * pstuMsg, Callback_t funCallback);

//*********************************************************************************************************************************
// Function Name: DeleteListen
// Functionality: Deletes a message listener
// Parameters:
//   *pstuCan - [i] Address of the servo CAN bus object
//   *pstuMsg - [i] Message characteristics for deleting the message listener (ID and cmd). Note: When operating a Listen, set ID = actual servo ID + 256.
// Return Value: 0 on success, -1 on failure
//*********************************************************************************************************************************
Exportmode s16_t DeleteListen(RyCanServoBus_t * pstuCan, CanMsg_t * pstuMsg);

//*********************************************************************************************************************************
// Function Name: GetServoUpdateInfo
// Functionality: Retrieves servo reported information from a listened message
// Parameters:
//   *pstuCan - [i] Address of the servo CAN bus object
//   ucId - [i] ID of the servo corresponding to the information to be read
//   *pstuData - [o] Address of the read servo reported information (same as listened information)
// Return Value: 0 on success, 1 on failure, -1 if pstuCan parameter is null
//*********************************************************************************************************************************
Exportmode s8_t GetServoUpdateInfo(RyCanServoBus_t * pstuCan, u8_t ucId, MsgListen_t * pstuData);

//*********************************************************************************************************************************
// Function Name: RyCanServoLibRcvMsg
// Functionality: Protocol library message reception handling function
// Parameters:
//   *pstuCan - [i] Address of the servo CAN bus object
//   stuMsg - [i] CAN message to be processed
// Return Value: 0 on success, -1 if pstuCan parameter is null
//*********************************************************************************************************************************
Exportmode s8_t RyCanServoLibRcvMsg(RyCanServoBus_t * pstuCan, CanMsg_t stuMsg);

//*********************************************************************************************************************************
// Function Name: GetRyCanServoLibVersion
// Functionality: Retrieves the protocol library version number
// Parameters:
//   pucVer[30] - Character array for storing the protocol library version (ASCII) string
// Return Value: None
//*********************************************************************************************************************************
Exportmode void GetRyCanServoLibVersion(u8_t pucVer[30]);

//*********************************************************************************************************************************
// Function Name: RyCanServoBusInit
// Functionality: Initializes the CAN bus object
// Parameters:
//   *pstuCan - [i] Address of the servo CAN bus object
//   funWrite - [i] Address of the CAN bus message sending function
//   pusTime - [i] Address of the ms counter
//   pusPeriod - [i] Period of the ms counter
// Return Value: 0 on success, -1 on failure
//*********************************************************************************************************************************
Exportmode u8_t RyCanServoBusInit(
  RyCanServoBus_t * pstuCan, BusWrite_t funWrite, volatile u16_t * pusTime, u16_t usPeriod);

//*********************************************************************************************************************************
// Function Name: RyCanServoBusDeInit
// Functionality: Destroys the CAN bus object, frees occupied memory, and clears member content
// Parameters:
//   *pstuCan - [i] Address of the servo CAN bus object
// Return Value: None
//*********************************************************************************************************************************
Exportmode void RyCanServoBusDeInit(RyCanServoBus_t * pstuCan);

//*********************************************************************************************************************************
// Function Name: Transmit
// Functionality: Data transmission
// Parameters:
//   *pstuCan - [i] Address of the servo CAN bus object
//   pstuTxmsg - [i] Address of the transmit message object
//   pstuRxmsg - [i/o] Address of the receive message object
//   *psHook - [o] Address of the message receive hook index
//   *pusT - [o] Address of the start transmission time variable
//   ucCmd - [i] Transmit data command code
//   usTimeout - [i] Receive timeout duration
// Return Value: Refer to enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
u8_t Transmit(
  RyCanServoBus_t * pstuCan, CanMsg_t * pstuTxmsg, CanMsg_t * pstuRxmsg, s16_t * psHook,
  u16_t * pusT, u8_t ucCmd, u16_t usTimeout);

//*********************************************************************************************************************************
// Function Name: TransmitNone
// Functionality: Transmits dummy data, solely to receive desired data from the bus
// Parameters:
//   *pstuCan - [i] Address of the servo CAN bus object
//   pstuTxmsg - [i] Address of the transmit message object
//   pstuRxmsg - [i/o] Address of the receive message object
//   *psHook - [o] Address of the message receive hook index
//   *pusT - [o] Address of the start transmission time variable
//   ucCmd - [i] Transmit data command code
//   usTimeout - [i] Receive timeout duration
// Return Value: Refer to enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
u8_t TransmitNone(
  RyCanServoBus_t * pstuCan, CanMsg_t * pstuTxmsg, CanMsg_t * pstuRxmsg, s16_t * psHook,
  u16_t * pusT, u8_t ucCmd, u16_t usTimeout);

//*********************************************************************************************************************************
// Function Name: Receive
// Functionality: Receives CAN bus data
// Parameters:
//   *pstuCan - [i] Address of the servo CAN bus object
//   pstuRxmsg - [i] Address of the receive message object
//   sHook - [i] Message receive hook index
//   usT - [i] Start transmission time variable
//   usTimeout - [i] Receive timeout duration
// Return Value: Refer to enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
u8_t Receive(
  RyCanServoBus_t * pstuCan, CanMsg_t * pstuRxmsg, s16_t sHook, u16_t usT, u16_t usTimeout);

//*********************************************************************************************************************************
// Function Name: RyFunc_GetServoInfo
// Functionality: Retrieves servo information
// Parameters:
//   *pstuCan - [i] Address of the servo CAN bus object
//   *psutServoData - [o] Starting address for receiving servo information, refer to ServoData_t data description
//   usTimeout - [i] Receive timeout duration in ms, 0-65535. 0 means asynchronous parsing or not caring about the return value, the function does not wait and returns immediately after sending. Other values indicate the maximum waiting time for a return.
// Return Value: Refer to enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t RyFunc_GetServoInfo(
  RyCanServoBus_t * pstuCan, u8_t ucId, ServoData_t * psutServoData, u16_t usTimeout);

//*********************************************************************************************************************************
// Function Name: RyFunc_GetVersion
// Functionality: Reads version information
// Parameters:
//   *pstuCan - [i] Address of the servo CAN bus object
//   ucId - [i] Specified servo ID, 0-254. 0 means broadcast, 255 is reserved and not used.
//   *pucVer - [o] Address for storing version information
//   usTimeout - [i] Receive timeout duration in ms, 0-65535. 0 means asynchronous parsing or not caring about the return value, the function does not wait and returns immediately after sending. Other values indicate the maximum waiting time for a return.
// Return Value: Refer to enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t
RyFunc_GetVersion(RyCanServoBus_t * pstuCan, u8_t ucId, u8_t pucVer[64], u16_t usTimeout);

//*********************************************************************************************************************************
// Function Name: RyFunc_StartUpgrade
// Functionality: Initiates an upgrade request
// Parameters:
//   *pstuCan - [i] Address of the servo CAN bus object
//   ucId - [i] Specified servo ID, 0-254. 0 means broadcast, 255 is reserved and not used.
//   ulDataLength - [i] Upgrade data length in bytes
//   *usPsw - [i/o] Upgrade password. To prevent accidental entry into upgrade mode, password verification is required during upgrade.
//                      When the host uses this command for the first time, any value can be given for this parameter. The device's response frame
//                      will provide the correct upgrade password. After the host obtains the correct upgrade password,
//                      it must send the correct upgrade password via this command for the device to enter upgrade mode.
//   *pucFrameLen - [i/o] Maximum single frame length supported by the host. Returns the minimum of the host's and slave's supported maximum frame lengths. Subsequent upgrade operations must adhere to the returned frame length.
//   *pucStatus - [o] Device command execution status. The content means:
//                      0: Password incorrect. The correct password will be returned in *usPsw.
//                      1: Entered upgrade mode from app.
//                      2: Entered upgrade mode from boot.
//                      3: Insufficient memory for writing; upgrade mode entry failed.
//                      4: Data erase started.
//                      5: Data erase OK; upgrade data transfer can proceed.
//   usTimeout - [in] Receive timeout duration in ms, 0-65535. 0 means asynchronous parsing or not caring about the return value; the function does not wait and returns immediately after sending. Other values indicate the maximum waiting time for a return.
// Return Value: Refer to enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t RyFunc_StartUpgrade(
  RyCanServoBus_t * pstuCan, u8_t ucId, u32_t ulDataLength, u16_t * usPsw, u8_t * pucFrameLen,
  u8_t * pucStatus, u16_t usTimeout);

//*********************************************************************************************************************************
//* Function Name - RyFunc_WriteUpgradeData
//* Function Purpose - Write upgrade data
//* Parameters:
//*     *pstuCan     - [i] Address of the servo CAN bus object
//*     ucId         - [i] Specified servo ID, range 0–254; 0 means broadcast, 255 is reserved and unused
//*     ulDataAddr   - [i] Upgrade data address, relative address range: 0~0xFFFFFF
//*     *pucData     - [i] Start address of the upgrade data
//*     ucDataLen    - [i] Length of the current data frame, must be less than or equal to the value returned by RyFunc_StartUpgrade ( *pucFrameLen - 4 )
//*     *ulNextAddr  - [o] Address of the next data to be sent
//*     *ulPreData   - [o] The most recently received upgrade data content
//*     usTimeout    - [i] Receive timeout in milliseconds, range: 0~65535. 0 indicates asynchronous parsing or ignoring the return value-the function returns immediately after sending. Other values specify the maximum wait time for a response.
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t RyFunc_WriteUpgradeData(
  RyCanServoBus_t * pstuCan, u8_t ucId, u32_t ulDataAddr, u8_t * pucData, u8_t ucDataLen,
  u32_t * ulNextAddr, u16_t usTimeout);

//*********************************************************************************************************************************
//* Function Name - RyFunc_FinishUpgrade
//* Function Purpose - Finish upgrade
//* Parameters:
//*     *pstuCan      - [i] Address of the servo CAN bus object
//*     ucId          - [i] Specified servo ID, 0–254; 0 means broadcast, 255 is reserved and unused
//*     *pulCRCValue  - [io] CRC32 checksum of the uploaded data. The CRC32 polynomial is 0x4C11DB7. If the check fails, the result calculated by the servo/device will be returned here.
//*     *pucStatus    - [o] Upgrade status:
//*                           0: Upgrade successful, node rebooted
//*                           1: CRC32 verification failed
//*                           2: New firmware hardware version mismatch
//*     usTimeout     - [i] Receive timeout in milliseconds, 0–65535. 0 means asynchronous parsing or ignoring return value-the function returns immediately after sending. Other values specify the maximum wait time.
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t RyFunc_FinishUpgrade(
  RyCanServoBus_t * pstuCan, u16_t ucId, u32_t * pulCRCValue, u32_t * pucStatus, u16_t usTimeout);

//*********************************************************************************************************************************
//* Function Name - RyFunc_SetSNCode
//* Function Purpose - Set servo SN code
//* Parameters:
//*     *pstuCan    - [i] Address of the servo CAN bus object
//*     ucId        - [i] Specified servo ID, 0–254; 0 means broadcast, 255 is reserved and unused
//*     pucBuff     - [i] Address of the SN code to be written to the servo (standard content is ASCII code, length less than 36 bytes)
//*     usTimeout   - [i] Receive timeout in milliseconds, 0–65535. 0 means asynchronous parsing or ignoring return value-the function returns immediately after sending. Other values specify the maximum wait time.
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t
RyFunc_SetSNCode(RyCanServoBus_t * pstuCan, u8_t ucId, u8_t pucBuff[40], u16_t usTimeout);

//*********************************************************************************************************************************
//* Function Name - RyFunc_GetSNCode
//* Function Purpose - Read servo SN code
//* Parameters:
//*     *pstuCan    - [i] Address of the servo CAN bus object
//*     ucId        - [i] Specified servo ID, 0–254; 0 means broadcast, 255 is reserved and unused
//*     pucBuff     - [o] Start address of the read servo SN code (standard content is ASCII code, length less than 36 bytes)
//*     usTimeout   - [i] Receive timeout in milliseconds, 0–65535. 0 means asynchronous parsing or ignoring return value-the function returns immediately after sending. Other values specify the maximum wait time.
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t
RyFunc_GetSNCode(RyCanServoBus_t * pstuCan, u8_t ucId, u8_t pucBuff[40], u16_t usTimeout);

//*********************************************************************************************************************************
//* Function Name - RyParam_SetTime
//* Function Purpose - Set device time
//* Parameters:
//*     *pstuCan    - [i] Address of the servo CAN bus object
//*     ucId        - [i] Specified servo ID, 0–254; 0 means broadcast, 255 is reserved and unused
//*     ubTime      - [i] Time object to be set, refer to the definition of DevTimeCmd_t
//*     usTimeout   - [i] Receive timeout in milliseconds, 0–65535. 0 means asynchronous parsing or ignoring return value-the function returns immediately after sending. Other values specify the maximum wait time.
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t
RyParam_SetTime(RyCanServoBus_t * pstuCan, u8_t ucId, DevTimeCmd_t ubTime, u16_t usTimeout);

//*********************************************************************************************************************************
//* Function Name - RyParam_SetID
//* Function Purpose - Set servo ID
//* Parameters:
//*     *pstuCan       - [i] Address of the servo CAN bus object
//*     ucId           - [i] Specified servo ID, 0–254; 0 means broadcast, 255 is reserved and unused
//*     ucNewId        - [i] New servo ID to be set, range 1–254
//*     *pulUniqueCode - [io] Address of the variable identifying the servo's unique code. If unknown, any value can be given (e.g., 0). The servo will report its own unique code in the response.
//*     usTimeout      - [i] Receive timeout in milliseconds, 0–65535. 0 means asynchronous parsing or ignoring return value-the function returns immediately after sending. Other values specify the maximum wait time.
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t RyParam_SetID(
  RyCanServoBus_t * pstuCan, u8_t ucId, u8_t ucNewId, u32_t * pulUniqueCode, u16_t usTimeout);

//*********************************************************************************************************************************
//* Function Name - RyParam_Recover
//* Function Purpose - Restore factory settings
//* Parameters:
//*     *pstuCan       - [i] Address of the servo CAN bus object
//*     ucId           - [i] Specified servo ID, 0–254; 0 means broadcast, 255 is reserved and unused
//*     *pulUniqueCode - [io] Address of the variable identifying the servo's unique code. If unknown, any value can be given (e.g., 0). The servo will report its own unique code in the response.
//*     usTimeout      - [i] Receive timeout in milliseconds, 0–65535. 0 means asynchronous parsing or ignoring return value-the function returns immediately after sending. Other values specify the maximum wait time.
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t
RyParam_Recover(RyCanServoBus_t * pstuCan, u8_t ucId, u32_t * pulUniqueCode, u16_t usTimeout);

//*********************************************************************************************************************************
//* Function Name - RyFunc_Reset
//* Function Purpose - Device reboot
//* Parameters:
//*     *pstuCan    - [i] Address of the servo CAN bus object
//*     ucId        - [i] Specified servo ID, 0–254; 0 means broadcast, 255 is reserved and unused
//*     usTimeout   - [i] Receive timeout in milliseconds, 0–65535. 0 means asynchronous parsing or ignoring return value-the function returns immediately after sending. Other values specify the maximum wait time.
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t RyFunc_Reset(RyCanServoBus_t * pstuCan, u8_t ucId, u16_t usTimeout);

//*********************************************************************************************************************************
//* Function Name - RyParam_SetCanFDBaudRate
//* Function Purpose - Set CAN/CANFD bus baud rate
//* Parameters:
//*     *pstuCan       - [i] Address of the servo CAN bus object
//*     ucId           - [i] Specified servo ID, 0–254; 0 means broadcast, 255 is reserved and unused
//*     ulCtrlBuadRate - [i] CANFD bus arbitration phase baud rate, or CAN bus baud rate. Supported baud rates (* is default):
//*                           5000,
//*                           10000,
//*                           20000,
//*                           25000,
//*                           40000,
//*                           50000,
//*                           62500,
//*                           80000,
//*                           100000,
//*                           125000,
//*                           200000,
//*                           250000,
//*                           400000,
//*                           500000,
//*                           800000,
//*                           1000000, *
//*     ulDataBaudRate - [i] CANFD bus data phase baud rate. For CAN bus, keep default or same as control phase. Supported baud rates (* is default):
//*                           5000,
//*                           10000,
//*                           20000,
//*                           25000,
//*                           40000,
//*                           50000,
//*                           62500,
//*                           80000,
//*                           100000,
//*                           125000,
//*                           200000,
//*                           250000,
//*                           400000,
//*                           500000,
//*                           800000,
//*                           1000000,
//*                           2000000,
//*                           4000000,
//*                           5000000, *
//*     usTimeout      - [i] Receive timeout in milliseconds, 0–65535. 0 means asynchronous parsing or ignoring return value-the function returns immediately after sending. Other values specify the maximum wait time.
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t RyParam_SetCanFDBaudRate(
  RyCanServoBus_t * pstuCan, u8_t ucId, u32_t ulCtrlBuadRate, u32_t ulDataBaudRate,
  u16_t usTimeout);

//*********************************************************************************************************************************
//* Function Name - RyParam_SetRS485BaudRate
//* Function Purpose - Set RS485 bus baud rate
//* Parameters:
//*     *pstuCan     - [i] Address of the servo CAN bus object
//*     ucId         - [i] Specified servo ID, 0–254; 0 means broadcast, 255 is reserved and unused
//*     ulBuadRate   - [i] RS485 bus baud rate, common baud rates (* is default):
//*                      1200
//*                      2400
//*                      4800
//*                      9600
//*                      19200
//*                      38400
//*                      57600
//*                      115200
//*                      230400
//*                      460800
//*                      921600
//*                      1000000
//*                      2000000
//*                      2500000
//*                      3000000
//*                      4000000
//*                      5000000 *
//*                      6000000
//*                      8000000
//*                      10000000
//*     usTimeout    - [i] Receive timeout in milliseconds, 0–65535. 0 means asynchronous parsing or ignoring return value-the function returns immediately after sending. Other values specify the maximum wait time.
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t
RyParam_SetRS485BaudRate(RyCanServoBus_t * pstuCan, u8_t ucId, u32_t ulBuadRate, u16_t usTimeout);

//*********************************************************************************************************************************
//* Function Name - RyParam_SetUpateRate
//* Function Purpose - Set the active reporting frequency of core data, not saved on power off
//* Parameters:
//*     *pstuCan       - [i] Address of the servo CAN bus object
//*     ucId           - [i] Specified servo ID, 0–254; 0 means broadcast, 255 is reserved and unused
//*     usTspan        - [i] Active reporting time interval in milliseconds, range 0–1000; 0 means no reporting, values above 1000 are treated as 1000
//*     *psutServoData - [o] Start address for receiving servo information
//*     usTimeout      - [i] Receive timeout in milliseconds, 0–65535. 0 means asynchronous parsing or ignoring return value-the function returns immediately after sending. Other values specify the maximum wait time.
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t RyParam_SetUpateRate(
  RyCanServoBus_t * pstuCan, u8_t ucId, u16_t usTspan, ServoData_t * psutServoData,
  u16_t usTimeout);

//*********************************************************************************************************************************
//* Function Name - RyParam_SetMotionMute
//* Function Purpose - Set motion command mute, i.e., enable or disable motion command response to save bus bandwidth; not saved on power off
//* Parameters:
//*     *pstuCan    - [i] Address of the servo CAN bus object
//*     ucId        - [i] Specified servo ID, 0–254; 0 means broadcast, 255 is reserved and unused
//*     ucMute      - [i] Motion command mute usage, 0 - motion command responds normally, 1 - motion command does not respond
//*     usTimeout   - [i] Receive timeout in milliseconds, 0–65535. 0 means asynchronous parsing or ignoring return value-the function returns immediately after sending. Other values specify the maximum wait time.
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t
RyParam_SetMotionMute(RyCanServoBus_t * pstuCan, u8_t ucId, u8_t ucMute, u16_t usTimeout);

//*********************************************************************************************************************************
//* Function Name - RyParam_GetRigidity
//* Function Purpose - Read motor control rigidity value
//* Parameters:
//*     *pstuCan      - [i] Address of the servo CAN bus object
//*     ucId          - [i] Specified servo ID, 0–254; 0 means broadcast, 255 is reserved and unused
//*     *pucRigidity  - [i] Used to receive servo position control rigidity information
//*     usTimeout     - [i] Receive timeout in milliseconds, 0–65535. 0 means asynchronous parsing or ignoring return value-the function returns immediately after sending. Other values specify the maximum wait time.
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t
RyParam_GetRigidity(RyCanServoBus_t * pstuCan, u8_t ucId, u8_t * pucRigidity, u16_t usTimeout);

//*********************************************************************************************************************************
//* Function Name - RyParam_SetRigidity
//* Function Purpose - Set motor control rigidity
//* Parameters:
//*     *pstuCan    - [i] Address of the servo CAN bus object
//*     ucId        - [i] Specified servo ID, 0–254; 0 means broadcast, 255 is reserved and unused
//*     ucRigidity  - [i] Servo position control rigidity value
//*     usTimeout   - [i] Receive timeout in milliseconds, 0–65535. 0 means asynchronous parsing or ignoring return value-the function returns immediately after sending. Other values specify the maximum wait time.
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t
RyParam_SetRigidity(RyCanServoBus_t * pstuCan, u8_t ucId, u8_t ucRigidity, u16_t usTimeout);

//*********************************************************************************************************************************
//* Function Name - RyParam_SetPosition
//* Function Purpose - Set servo position
//* Parameters:
//*     *pstuCan     - [i] Address of the servo CAN bus object
//*     ucId         - [i] Specified servo ID, 0–254; 0 means broadcast, 255 is reserved and unused
//*     usPosition   - [i] Target position value, 0 to 4095 corresponds to 0 to full stroke
//*     *pusPosoff   - [o] Received internal position compensation value, 0 to 4095 corresponds to 0 to full stroke
//*     usTimeout    - [i] Receive timeout in milliseconds, 0–65535. 0 means asynchronous parsing or ignoring return value-the function returns immediately after sending. Other values specify the maximum wait time.
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t RyParam_SetPosition(
  RyCanServoBus_t * pstuCan, u8_t ucId, u16_t usPosition, u16_t * pusPosoff, u16_t usTimeout);

//*********************************************************************************************************************************
//* Function Name - RyParam_ClearFault
//* Function Purpose - Clear servo fault
//* Parameters:
//*     *pstuCan    - [i] Address of the servo CAN bus object
//*     ucId        - [i] Specified servo ID, 0–254; 0 means broadcast, 255 is reserved and unused
//*     usTimeout   - [i] Receive timeout in milliseconds, 0–65535. 0 means asynchronous parsing or ignoring return value-the function returns immediately after sending. Other values specify the maximum wait time.
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t RyParam_ClearFault(RyCanServoBus_t * pstuCan, u8_t ucId, u16_t usTimeout);

//*********************************************************************************************************************************
//* Function Name - RyParam_GetProtectionCfg
//* Function Purpose - Read protection configuration
//* Parameters:
//*     *pstuCan           - [i] Address of the servo CAN bus object
//*     ucId               - [i] Specified servo ID, 0–254; 0 means broadcast, 255 is reserved and unused
//*     *pusProtectionCfg  - [o] Start address to receive protection configuration. Each bit corresponds to a protection type:
//*                            Enable status: 0 - disabled, 1 - enabled
//*                            bit 0  - Motor temperature protection
//*                            bit 1  - Motor voltage protection
//*                            bit 2  - Motor overcurrent protection
//*                            bit 3  - Motor torque protection
//*                            bit 4  - Motor fuse bit error protection
//*                            bit 5  - Motor stall protection
//*     usTimeout          - [i] Receive timeout in milliseconds, 0–65535. 0 means asynchronous parsing or ignoring return value-the function
//*                              returns immediately after sending. Other values specify the maximum wait time.
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t RyParam_GetProtectionCfg(
  RyCanServoBus_t * pstuCan, u8_t ucId, u16_t * pusProtectionCfg, u16_t usTimeout);

//*********************************************************************************************************************************
//* Function Name - RyParam_SetProtectionCfg
//* Function Purpose - Set protection configuration
//* Parameters:
//*     *pstuCan           - [i] Address of the servo CAN bus object
//*     ucId               - [i] Specified servo ID, 0–254; 0 means broadcast, 255 is reserved and unused
//*     *pusProtectionCfg  - [io] Start address of the protection configuration values. Each bit corresponds to a protection type:
//*                            Enable status: 0 - disabled, 1 - enabled
//*                            bit 0  - Motor temperature protection
//*                            bit 1  - Motor voltage protection
//*                            bit 2  - Motor overcurrent protection
//*                            bit 3  - Motor torque protection
//*                            bit 4  - Motor fuse bit error protection
//*                            bit 5  - Motor stall protection
//*     usTimeout          - [i] Receive timeout in milliseconds, 0–65535. 0 means asynchronous parsing or ignoring return
//*                              value-the function returns immediately after sending. Other values specify the maximum wait time.
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t RyParam_SetProtectionCfg(
  RyCanServoBus_t * pstuCan, u8_t ucId, u16_t * pusProtectionCfg, u16_t usTimeout);

//*********************************************************************************************************************************
//* Function Name - RyParam_GetStroke
//* Function Purpose - Read servo stroke
//* Parameters:
//*     *pstuCan     - [i] Address of the servo CAN bus object
//*     ucId         - [i] Specified servo ID, 0–254; 0 means broadcast, 255 is reserved and unused
//*     *pulStroke   - [o] Start address to return stroke information, unit is motor original position sensor/encoder stroke value
//*     usTimeout    - [i] Receive timeout in milliseconds, 0–65535. 0 means asynchronous parsing or ignoring return value-the
//*                        function returns immediately after sending. Other values specify the maximum wait time.
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t
RyParam_GetStroke(RyCanServoBus_t * pstuCan, u8_t ucId, u32_t * pulStroke, u16_t usTimeout);

//*********************************************************************************************************************************
//* Function Name - RyParam_GetStroke
//* Function Purpose - Set servo stroke
//* Parameters:
//*     *pstuCan    - [i] Address of the servo CAN bus object
//*     ucId        - [i] Specified servo ID, 0–254; 0 means broadcast, 255 is reserved and unused
//*     ulStroke    - [i] Stroke value, unit is motor original position sensor/encoder stroke value
//*     usTimeout   - [i] Receive timeout in milliseconds, 0–65535. 0 means asynchronous parsing or ignoring return value
//*                       the function returns immediately after sending. Other values specify the maximum wait time.
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t
RyParam_SetStroke(RyCanServoBus_t * pstuCan, u8_t ucId, u32_t ulStroke, u16_t usTimeout);

//*********************************************************************************************************************************
//* Function Name - RyParam_GetStroke_H
//* Function Purpose - Read servo stroke upper limit
//* Parameters:
//*   *pstuCan     - [i] Address of the servo CAN bus object
//*   ucId         - [i] Specified servo ID, 0-254; 0 means broadcast, 255 is reserved and unused
//*   *pulStrokeH  - [o] Start address to return stroke upper limit value, unit is motor raw position sensor/encoder stroke value
//*   usTimeout    - [i] Receive timeout in milliseconds, 0-65535. 0 means asynchronous parsing or ignoring return value,
//*                      the function returns immediately after sending. Other values specify the maximum wait time.
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t
RyParam_GetStroke_H(RyCanServoBus_t * pstuCan, u8_t ucId, u32_t * pulStrokeH, u16_t usTimeout);

//*********************************************************************************************************************************
//* Function Name - RyParam_GetStroke_H
//* Function Purpose - Read servo stroke upper limit
//* Parameters:
//*     *pstuCan     - [i] Address of the servo CAN bus object
//*     ucId         - [i] Specified servo ID, 0–254; 0 means broadcast, 255 is reserved and unused
//*     *pulStrokeH  - [o] Start address to return stroke upper limit value, unit is motor original position sensor/encoder stroke value
//*     usTimeout    - [i] Receive timeout in milliseconds, 0–65535. 0 means asynchronous parsing or ignoring return value
//*                        the function returns immediately after sending. Other values specify the maximum wait time.
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t
RyParam_SetStroke_H(RyCanServoBus_t * pstuCan, u8_t ucId, u32_t ulStrokeH, u16_t usTimeout);

//*********************************************************************************************************************************
//* Function Name - RyParam_GetStroke_L
//* Function Purpose - Read servo stroke lower limit value
//* Parameters:
//*     *pstuCan     - [i] Address of the servo CAN bus object
//*     ucId         - [i] Specified servo ID, 0–254; 0 means broadcast, 255 is reserved and unused
//*     *pulStrokeL  - [o] Start address to return stroke lower limit value, unit is motor original position sensor/encoder stroke value
//*     usTimeout    - [i] Receive timeout in milliseconds, 0–65535. 0 means asynchronous parsing or ignoring return value
//*                        the function returns immediately after sending. Other values specify the maximum wait time.
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t
RyParam_GetStroke_L(RyCanServoBus_t * pstuCan, u8_t ucId, u32_t * pulStrokeL, u16_t usTimeout);

//*********************************************************************************************************************************
//* Function Name - RyParam_SetStroke_L
//* Function Purpose - Set servo stroke lower limit value
//* Parameters:
//*     *pstuCan    - [i] Address of the servo CAN bus object
//*     ucId        - [i] Specified servo ID, 0–254; 0 means broadcast, 255 is reserved and unused
//*     ulStrokeL   - [i] Stroke lower limit value, unit is motor original position sensor/encoder stroke value
//*     usTimeout   - [i] Receive timeout in milliseconds, 0–65535. 0 means asynchronous parsing or ignoring return value
//*                       the function returns immediately after sending. Other values specify the maximum wait time.
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t
RyParam_SetStroke_L(RyCanServoBus_t * pstuCan, u8_t ucId, u32_t ulStrokeL, u16_t usTimeout);

//*********************************************************************************************************************************
//* Function Name - RyMotion_ServoMove_Speed
//* Function Purpose - Move at specified speed
//* Parameters:
//*     *pstuCan       - [i] Address of the servo CAN bus object
//*     ucId           - [i] Specified servo ID, 0–254; 0 means broadcast, 255 is reserved and unused
//*     sTargetAngle   - [i] Servo target angle, 0 to 4095 corresponds to 0 to full stroke
//*     usRunSpeed     - [i] Servo running speed, 0–65535, unit 0.001 stroke/s; actual value around 0–5000 depending on motor speed
//*     *psutServoData - [o] Start address to receive servo information
//*     usTimeout      - [i] Receive timeout in milliseconds, 0–65535. 0 means asynchronous parsing or ignoring return value
//*                          the function returns immediately after sending. Other values specify the maximum wait time.
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t RyMotion_ServoMove_Speed(
  RyCanServoBus_t * pstuCan, u8_t ucId, s16_t sTargetAngle, u16_t usRunSpeed,
  ServoData_t * psutServoData, u16_t usTimeout);

//*********************************************************************************************************************************
//* Function Name - RyMotion_ServoMove_Pwm
//* Function Purpose - Move with specified PWM
//* Parameters:
//*     *pstuCan       - [i] Address of the servo CAN bus object
//*     ucId           - [i] Specified servo ID, 0–254; 0 means broadcast, 255 is reserved and unused
//*     sPwm           - [i] Motor running PWM, -1000 to 1000 corresponds to full negative to full positive PWM
//*     *psutServoData - [o] Start address to receive servo information, data content defined in ServoData_t
//*     usTimeout      - [i] Receive timeout in milliseconds, 0–65535. 0 means asynchronous parsing or ignoring return value
//*                          the function returns immediately after sending. Other values specify the maximum wait time.
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t RyMotion_ServoMove_Pwm(
  RyCanServoBus_t * pstuCan, u8_t ucId, s16_t sPwm, ServoData_t * psutServoData, u16_t usTimeout);

//*********************************************************************************************************************************
//* Function Name - RyMotion_CurrentMode
//* Function Purpose - Current mode motion
//* Parameters:
//*     *pstuCan        - [i] Address of the servo CAN bus object
//*     ucId            - [i] Specified servo ID, 0–254; 0 means broadcast, 255 is reserved and unused
//*     sTargetCurrent  - [i] Target current, -32768 to 32767 corresponds to -32.768A to 32.767A; actual usable range is -3000 to 3000
//*     *psutServoData  - [o] Start address to receive servo information
//*     usTimeout       - [i] Receive timeout in milliseconds, 0–65535. 0 means asynchronous parsing or ignoring return value
//*                           the function returns immediately after sending. Other values specify the maximum wait time.
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t RyMotion_CurrentMode(
  RyCanServoBus_t * pstuCan, u8_t ucId, s16_t sTargetCurrent, ServoData_t * psutServoData,
  u16_t usTimeout);

//*********************************************************************************************************************************
//* Function Name - RyMotion_ServoMove_Mix
//* Function Purpose - Force-position mixed control
//* Parameters:
//*     *pstuCan       - [i] Address of the servo CAN bus object
//*     ucId           - [i] Specified servo ID, 0–254; 0 means broadcast, 255 is reserved and unused
//*     sTargetAngle   - [i] Servo target angle, 0 to 4095 corresponds to 0 to full stroke
//*     usRunSpeed     - [i] Servo running speed, 0–65535, unit 0.001 stroke/s; actual value around 0–5000 depending on motor speed
//*     sMaxCurrent    - [i] Maximum allowed current during operation, -32768 to 32767 corresponds to -32.768A to 32.767A; actual usable range -1000 to 1000
//*     *psutServoData - [o] Start address to receive servo information
//*     usTimeout      - [i] Receive timeout in milliseconds, 0–65535. 0 means asynchronous parsing or ignoring return value
//*                          the function returns immediately after sending. Other values specify the maximum wait time.
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t RyMotion_ServoMove_Mix(
  RyCanServoBus_t * pstuCan, u8_t ucId, s16_t sTargetAngle, u16_t usRunSpeed, u16_t sMaxCurrent,
  ServoData_t * psutServoData, u16_t usTimeout);

#if USE_ASYN_PRASE

//*********************************************************************************************************************************
//* Function Name - AsynParse_RyFunc_GetServoInfo
//* Function Purpose - Asynchronous parsing to obtain servo information
//* Parameters:
//*     stuRxmsg       - [i] CAN received message
//*     *pusId         - [o] Node ID corresponding to the message
//*     *psutServoData - [o] Returned servo data
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t
AsynParse_RyFunc_GetServoInfo(CanMsg_t stuRxmsg, u16_t * pusId, ServoData_t * psutServoData);

//*********************************************************************************************************************************
//* Function Name - AsynParse_RyFunc_GetVersion
//* Function Purpose - Asynchronous parsing to read version information.
//*                    In asynchronous frequency mode, this interface may parse incompletely; please use synchronous mode.
//* Parameters:
//*     stuRxmsg      - [i] CAN received message
//*     *pusId        - [o] Node ID corresponding to the message
//*     *pucVer       - [o] Memory address for receiving version information
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t AsynParse_RyFunc_GetVersion(CanMsg_t stuRxmsg, u16_t * pusId, u8_t pucVer[64]);

//*********************************************************************************************************************************
//* Function Name - AsynParse_RyFunc_StartUpgrade
//* Function Purpose - Asynchronous parsing for start upgrade request
//* Parameters:
//*     stuRxmsg      - [i] CAN received message
//*     *pusId        - [o] Node ID corresponding to the message
//*     *usPsw        - [io] Upgrade password; to prevent accidental entry into upgrade mode, password verification is required during upgrade!
//*                      When the host uses this command for the first time, this value can be arbitrary. The device reply frame
//*                      will provide the correct upgrade password. After the host obtains the correct password,
//*                      it must send the correct upgrade password with this command to allow the device to enter upgrade mode.
//*     *pucFrameLen  - [io] Maximum single frame length supported by the host; returns the minimum of host and device maximum frame length, subsequent upgrade operations must follow this frame length.
//*     *pucStatus    - [o] Device execution status of the command, meanings:
//*                          0: Password verification failed; the correct password will be returned in *usPsw
//*                          1: Enter upgrade mode from app
//*                          2: Enter upgrade mode from boot
//*                          3: Insufficient writable memory, failed to enter upgrade mode
//*                          4: Data erase started
//*                          5: Data erase successful; ready for upgrade data transfer
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t AsynParse_RyFunc_StartUpgrade(
  CanMsg_t stuRxmsg, u16_t * pusId, u16_t * usPsw, u8_t * pucFrameLen, u8_t * pucStatus);

//*********************************************************************************************************************************
//* Function Name - AsynParse_RyFunc_WriteUpgradeData
//* Function Purpose - Asynchronous parsing for writing upgrade data
//* Parameters:
//*     stuRxmsg      - [i] CAN received message
//*     *pusId        - [o] Node ID corresponding to the message
//*     *ulNextAddr   - [o] Address of the next data to be written
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t
AsynParse_RyFunc_WriteUpgradeData(CanMsg_t stuRxmsg, u16_t * pusId, u32_t * ulNextAddr);

//*********************************************************************************************************************************
//* Function Name - AsynParse_RyFunc_FinishUpgrade
//* Function Purpose - Asynchronous parsing for upgrade completion
//* Parameters:
//*     stuRxmsg      - [i] CAN received message
//*     *pusId        - [o] Node ID corresponding to the message
//*     *pulCRCValue  - [io] CRC32 checksum of the upgrade data, with CRC32 polynomial 0x4C11DB7.
//*                          If the check fails, this will return the CRC32 result calculated by the servo/device.
//*     *pucStatus    - [o] Upgrade status:
//*                       0: Upgrade successful, node restarts
//*                       1: CRC32 checksum failed
//*                       2: New firmware hardware version mismatch
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t AsynParse_RyFunc_FinishUpgrade(
  CanMsg_t stuRxmsg, u16_t * pusId, u32_t * pulCRCValue, u8_t * pucStatus);

//*********************************************************************************************************************************
//* Function Name - AsynParse_RyFunc_SetSNCode
//* Function Purpose - Asynchronous parsing for setting servo SN code
//* Parameters:
//*     stuRxmsg      - [i] CAN received message
//*     *pusId        - [o] Node ID corresponding to the message
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t AsynParse_RyFunc_SetSNCode(CanMsg_t stuRxmsg, u16_t * pusId);

//*********************************************************************************************************************************
//* Function Name - AsynParse_RyFunc_GetSNCode
//* Function Purpose - Asynchronous parsing to read servo SN code; this interface requires multiple calls in asynchronous mode
//* Parameters:
//*     stuRxmsg      - [i] CAN received message
//*     *pusId        - [o] Node ID corresponding to the message
//*     *pucSN        - [o] Memory address for receiving SN information
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t AsynParse_RyFunc_GetSNCode(CanMsg_t stuRxmsg, u16_t * pusId, u8_t pucSN[64]);

//*********************************************************************************************************************************
//* Function Name - AsynParse_RyParam_SetTime
//* Function Purpose - Asynchronous parsing for setting servo SN code
//* Parameters:
//*     stuRxmsg      - [i] CAN received message
//*     *pusId        - [o] Node ID corresponding to the message
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t AsynParse_RyParam_SetTime(CanMsg_t stuRxmsg, u16_t * pusId);

//*********************************************************************************************************************************
//*函数名称 - AsynParse_RyParam_SetID
//*函数作用 - 异步解析 设置伺服ID
//*  参数 ：
//*     stuRxmsg      - [i] CAN接收消息
//*     *pusId        - [o] 消息对应的节点ID
//*     *ulPsw      	- [o] 用于存储设备返回的密码首地址，
//* 返回值 - 参考枚举类型 enret_t 及对应解释
//*********************************************************************************************************************************
Exportmode u8_t AsynParse_RyParam_SetID(CanMsg_t stuRxmsg, u16_t * pusId, u32_t * ulPsw);

//*********************************************************************************************************************************
//* Function Name - AsynParse_RyParam_SetID
//* Function Purpose - Asynchronous parsing for setting servo ID
//* Parameters:
//*     stuRxmsg      - [i] CAN received message
//*     *pusId        - [o] Node ID corresponding to the message
//*     *ulPsw        - [o] Memory address to store the password returned by the device
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t AsynParse_RyParam_Recover(CanMsg_t stuRxmsg, u16_t * pusId, u32_t * ulPsw);

//*********************************************************************************************************************************
//* Function Name - AsynParse_RyFunc_Reset
//* Function Purpose - Asynchronous parsing for device reboot
//* Parameters:
//*     stuRxmsg      - [i] CAN received message
//*     *pusId        - [o] Node ID corresponding to the message
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t AsynParse_RyFunc_Reset(CanMsg_t stuRxmsg, u16_t * pusId);

//*********************************************************************************************************************************
//* Function Name - AsynParse_RyParam_SetCanFDBaudRate
//* Function Purpose - Asynchronous parsing for setting CAN/CANFD bus baud rate
//* Parameters:
//*     stuRxmsg      - [i] CAN received message
//*     *pusId        - [o] Node ID corresponding to the message
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t AsynParse_RyParam_SetCanFDBaudRate(CanMsg_t stuRxmsg, u16_t * pusId);

//*********************************************************************************************************************************
//* Function Name - AsynParse_RyParam_SetRS485BaudRate
//* Function Purpose - Asynchronous parsing for setting RS485 bus baud rate
//* Parameters:
//*     stuRxmsg      - [i] CAN received message
//*     *pusId        - [o] Node ID corresponding to the message
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t AsynParse_RyParam_SetRS485BaudRate(CanMsg_t stuRxmsg, u16_t * pusId);

//*********************************************************************************************************************************
//* Function Name - AsynParse_RyParam_SetUpateRate
//* Function Purpose - Asynchronous parsing for setting core data active reporting frequency; not saved on power-off
//* Parameters:
//*     stuRxmsg      - [i] CAN received message
//*     *pusId        - [o] Node ID corresponding to the message
//*     *psutServoData - [o] Returned servo data
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t
AsynParse_RyParam_SetUpateRate(CanMsg_t stuRxmsg, u16_t * pusId, ServoData_t * psutServoData);

//*********************************************************************************************************************************
//* Function Name - AsynParse_RyParam_SetMotionMute
//* Function Purpose - Asynchronous parsing for setting motion command mute, i.e., enabling or disabling motion command responses to save bus bandwidth; not saved on power-off
//* Parameters:
//*     stuRxmsg      - [i] CAN received message
//*     *pusId        - [o] Node ID corresponding to the message
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t AsynParse_RyParam_SetMotionMute(CanMsg_t stuRxmsg, u16_t * pusId);

//*********************************************************************************************************************************
//* Function Name - AsynParse_RyParam_GetRigidity
//* Function Purpose - Asynchronous parsing to read motor control rigidity value
//* Parameters:
//*     stuRxmsg      - [i] CAN received message
//*     *pusId        - [o] Node ID corresponding to the message
//*     *ucRigidity   - [o] Used to store the device servo control rigidity
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t AsynParse_RyParam_GetRigidity(CanMsg_t stuRxmsg, u16_t * pusId, u8_t * ucRigidity);

//*********************************************************************************************************************************
//* Function Name - AsynParse_RyParam_SetRigidity
//* Function Purpose - Asynchronous parsing for setting motor control rigidity
//* Parameters:
//*     stuRxmsg      - [i] CAN received message
//*     *pusId        - [o] Node ID corresponding to the message
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t AsynParse_RyParam_SetRigidity(CanMsg_t stuRxmsg, u16_t * pusId);

//*********************************************************************************************************************************
//* Function Name - AsynParse_RyParam_SetPosition
//* Function Purpose - Asynchronous parsing for setting servo position
//* Parameters:
//*     stuRxmsg      - [i] CAN received message
//*     *pusId        - [o] Node ID corresponding to the message
//*     *pusPosoff    - [o] Received internal position compensation value, 0 to 4095 corresponds to 0~full travel
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t AsynParse_RyParam_SetPosition(CanMsg_t stuRxmsg, u16_t * pusId, u16_t * pusPosoff);

//*********************************************************************************************************************************
//* Function Name - AsynParse_RyParam_ClearFault
//* Function Purpose - Asynchronous parsing to clear servo faults
//* Parameters:
//*     stuRxmsg      - [i] CAN received message
//*     *pusId        - [o] Node ID corresponding to the message
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t AsynParse_RyParam_ClearFault(CanMsg_t stuRxmsg, u16_t * pusId);

//*********************************************************************************************************************************
//* Function Name - AsynParse_RyParam_GetProtectionCfg
//* Function Purpose - Asynchronous parsing to read protection configuration
//* Parameters:
//*     stuRxmsg          - [i] CAN received message
//*     *pusId            - [o] Node ID corresponding to the message
//*     *pusProtectionCfg - [o] Address to receive protection configuration, each bit corresponds to one type of protection, each bit represents a protection:
//*                            Enable status: 0 - Disabled, 1 - Enabled
//*                            bit 0  - Motor temperature protection
//*                            bit 1  - Motor voltage protection
//*                            bit 2  - Motor overcurrent protection
//*                            bit 3  - Motor torque protection
//*                            bit 4  - Motor fuse misalignment protection
//*                            bit 5  - Motor stall protection
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t
AsynParse_RyParam_GetProtectionCfg(CanMsg_t stuRxmsg, u16_t * pusId, u16_t * pusProtectionCfg);

//*********************************************************************************************************************************
//* Function Name - AsynParse_RyParam_SetProtectionCfg
//* Function Purpose - Asynchronous parsing to set protection configuration
//* Parameters:
//*     stuRxmsg          - [i] CAN received message
//*     *pusId            - [o] Node ID corresponding to the message
//*     *pusProtectionCfg - [o] Address to receive protection configuration, each bit corresponds to one type of protection, each bit represents a protection:
//*                            Enable status: 0 - Disabled, 1 - Enabled
//*                            bit 0  - Motor temperature protection
//*                            bit 1  - Motor voltage protection
//*                            bit 2  - Motor overcurrent protection
//*                            bit 3  - Motor torque protection
//*                            bit 4  - Motor fuse misalignment protection
//*                            bit 5  - Motor stall protection
//* Return Value - Refer to the enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t
AsynParse_RyParam_SetProtectionCfg(CanMsg_t stuRxmsg, u16_t * pusId, u16_t * pusProtectionCfg);

//*********************************************************************************************************************************
//* Function Name - AsynParse_RyParam_GetStroke
//* Function Purpose - Asynchronous parsing to read servo stroke
//* Parameters:
//*     stuRxmsg      - [i] CAN received message
//*     *pusId        - [o] Node ID corresponding to the message
//*     *pulStroke    - [o] Address to return stroke information, dimension is the motor's original position sensor/encoder stroke value
//* Return Value - Refer to enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t AsynParse_RyParam_GetStroke(CanMsg_t stuRxmsg, u16_t * pusId, u32_t * pulStroke);

//*********************************************************************************************************************************
//* Function Name - AsynParse_RyParam_SetStroke
//* Function Purpose - Asynchronous parsing to set servo stroke
//* Parameters:
//*     stuRxmsg      - [i] CAN received message
//*     *pusId        - [o] Node ID corresponding to the message
//* Return Value - Refer to enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t AsynParse_RyParam_SetStroke(CanMsg_t stuRxmsg, u16_t * pusId);

//*********************************************************************************************************************************
//* Function Name - AsynParse_RyParam_GetStroke_H
//* Function Purpose - Asynchronous parsing to read servo stroke upper limit
//* Parameters:
//*     stuRxmsg      - [i] CAN received message
//*     *pusId        - [o] Node ID corresponding to the message
//*     *pulStrokeH   - [o] Pointer to store the returned stroke upper limit value (unit: raw motor position sensor/encoder count)
//* Return Value - Refer to enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t AsynParse_RyParam_GetStroke_H(CanMsg_t stuRxmsg, u16_t * pusId, u32_t * pulStrokeH);

//*********************************************************************************************************************************
//* Function Name - AsynParse_RyParam_SetStroke_H
//* Function Purpose - Asynchronous parsing to set servo stroke upper limit
//* Parameters:
//*     stuRxmsg      - [i] CAN received message
//*     *pusId        - [o] Node ID corresponding to the message
//* Return Value - Refer to enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t AsynParse_RyParam_SetStroke_H(CanMsg_t stuRxmsg, u16_t * pusId);

//*********************************************************************************************************************************
//* Function Name - AsynParse_RyParam_GetStroke_L
//* Function Purpose - Asynchronous parsing to read servo stroke lower limit value
//* Parameters:
//*     stuRxmsg      - [i] CAN received message
//*     *pusId        - [o] Node ID corresponding to the message
//*     *pulStrokeL   - [o] Pointer to store the returned stroke lower limit value, unit: raw motor position sensor/encoder stroke count
//* Return Value - Refer to enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t AsynParse_RyParam_GetStroke_L(CanMsg_t stuRxmsg, u16_t * pusId, u32_t * pulStrokeL);

//*********************************************************************************************************************************
//* Function Name - AsynParse_RyParam_SetStroke_L
//* Function Purpose - Asynchronous parsing to set servo stroke lower limit value
//* Parameters:
//*     stuRxmsg      - [i] CAN received message
//*     *pusId        - [o] Node ID corresponding to the message
//* Return Value - Refer to enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t AsynParse_RyParam_SetStroke_L(CanMsg_t stuRxmsg, u16_t * pusId);

//*********************************************************************************************************************************
//* Function Name - AsynParse_RyMotion_ServoMove_Speed
//* Function Purpose - Asynchronous parsing for specified speed movement
//* Parameters:
//*     stuRxmsg      - [i] CAN received message
//*     *pusId        - [o] Node ID corresponding to the message
//*     *psutServoData - [o] Returned servo data
//* Return Value - Refer to enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t
AsynParse_RyMotion_ServoMove_Speed(CanMsg_t stuRxmsg, u16_t * pusId, ServoData_t * psutServoData);

//*********************************************************************************************************************************
//* Function Name - AsynParse_RyMotion_ServoMove_Pwm
//* Function Purpose - Asynchronous parsing for specified PWM movement
//* Parameters:
//*     stuRxmsg      - [i] CAN received message
//*     *pusId        - [o] Node ID corresponding to the message
//*     *psutServoData - [o] Returned servo data
//* Return Value - Refer to enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t
AsynParse_RyMotion_ServoMove_Pwm(CanMsg_t stuRxmsg, u16_t * pusId, ServoData_t * psutServoData);

//*********************************************************************************************************************************
//* Function Name - AsynParse_RyMotion_CurrentMode
//* Function Purpose - Asynchronous parsing for current mode motion
//* Parameters:
//*     stuRxmsg      - [i] CAN received message
//*     *pusId        - [o] Node ID corresponding to the message
//*     *psutServoData - [o] Returned servo data
//* Return Value - Refer to enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t
AsynParse_RyMotion_CurrentMode(CanMsg_t stuRxmsg, u16_t * pusId, ServoData_t * psutServoData);

//*********************************************************************************************************************************
//* Function Name - AsynParse_RyMotion_ServoMove_Mix
//* Function Purpose - Asynchronous parsing for force-position mixed control
//* Parameters:
//*     stuRxmsg      - [i] CAN received message
//*     *pusId        - [o] Node ID corresponding to the message
//*     *psutServoData - [o] Returned servo data
//* Return Value - Refer to enumeration type enret_t and corresponding explanations
//*********************************************************************************************************************************
Exportmode u8_t
AsynParse_RyMotion_ServoMove_Mix(CanMsg_t stuRxmsg, u16_t * pusId, ServoData_t * psutServoData);

#endif

#ifdef __cplusplus
}
#endif

#endif
