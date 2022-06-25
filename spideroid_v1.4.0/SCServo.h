/* SCServo.h modified: Taner AKKAN 4.4.2017*/
#ifndef _SCSERVO_h_
#define _SCSERVO_h_

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define s8 char
#define u8 unsigned char
#define u16 unsigned short
#define s16 short
#define u32 unsigned long
#define s32 long

class SCServo{
public:
SCServo();
u8   EnableTorque(u8 ID, u8 Enable, u8 ReturnLevel=2);
u8   WritePos(u8 ID, s16 pozition, s16 velocity, u8 ReturnLevel=2);
u8   RegWritePos(u8 ID, s16 pozition, s16 velocity, u8 ReturnLevel=2);
s16  ReadPos(u8 ID);
s16  ReadReg(u8 ID,u8 kayitcino);
s16  ReadReg16(u8 ID,u8 kayitcino);
u8   WheelDir(u8 ID, u8 Enable, u8 ReturnLevel=2);
void RegWriteAction();
void SyncWritePos(u8 ID[], u8 IDN, s16 pozition, s16 velocity);
s16  WriteID(u8 oldID, u8 newID, u8 ReturnLevel=2);
u8   WriteLimitAngle(u8 ID, u16 MinAngel, u16 MaxAngle, u8 ReturnLevel=2);
u8   WriteLimitTorque(u8 ID, u16 MaxTorque, u8 ReturnLevel=2);
u8   WritePunch(u8 ID, u16 Punch, u8 ReturnLevel=2);
u8   WriteBaud(u8 ID, u8 Baud, u8 ReturnLevel=2);
u8   WriteComplianceMargin(u8 ID, u8 CCW, u8 CW, u8 ReturnLevel=2);
u8   WritePID(u8 ID, u8 P, u8 I, u8 D, u8 ReturnLevel=2);
u8   WriteSpeed(u8 ID, s16 velocity, u8 ReturnLevel=2);
u8   LockEeprom(u8 ID, u8 Enable, u8 ReturnLevel=2);
private:
u8   ReadBuf(u8 len, u8 *buf=NULL);

#define startByte 0xFF
#define TIMEOUT 2000 //TIMEOUT 2000

#define B_1M                     0
#define B_0_5M                   1
#define B_250K                   2
#define B_128K                   3
#define B_115200                 4
#define B_76800                  5
#define B_57600                  6
#define B_38400                  7

//register Address
#define P_MODEL_NUMBER_L         0
#define P_MODEL_NUMBER_H         1
#define P_VERSION_L              3
#define P_VERSION_H              4
#define P_ID                     5
#define P_BAUD_RATE              6
#define P_RETURN_DELAY_TIME      7
#define P_RETURN_LEVEL           8
#define P_MIN_ANGLE_LIMIT_L      9
#define P_MIN_ANGLE_LIMIT_H     10
#define P_MAX_ANGLE_LIMIT_L     11
#define P_MAX_ANGLE_LIMIT_H     12
#define P_LIMIT_TEMPERATURE     13
#define P_MAX_LIMIT_VOLTAGE     14
#define P_MIN_LIMIT_VOLTAGE     15
#define P_MAX_TORQUE_L          16
#define P_MAX_TORQUE_H          17
#define P_ALARM_LED             18
#define P_ALARM_SHUTDOWN        19
#define P_COMPLIANCE_P          21
#define P_COMPLIANCE_D          22
#define P_COMPLIANCE_I          23
#define P_PUNCH_L               24
#define P_PUNCH_H               25
#define P_CW_COMPLIANCE_MARGIN  26
#define P_CCW_COMPLIANCE_MARGIN 27
#define P_IMAX_L                28
#define P_IMAX_H                29
#define P_OFFSET_L              30
#define P_OFFSET_H              31

#define P_TORQUE_ENABLE         40
#define P_LED                   41
#define P_GOAL_POSITION_L       42
#define P_GOAL_POSITION_H       43
#define P_GOAL_TIME_L           44
#define P_GOAL_TIME_H           45
#define P_GOAL_SPEED_L          46
#define P_GOAL_SPEED_H          47
#define P_LOCK                  48

#define P_PRESENT_POSITION_L    56
#define P_PRESENT_POSITION_H    57
#define P_PRESENT_SPEED_L       58
#define P_PRESENT_SPEED_H       59
#define P_PRESENT_LOAD_L        60
#define P_PRESENT_LOAD_H        61
#define P_PRESENT_VOLTAGE       62
#define P_PRESENT_TEMPERATURE   63
#define P_REGISTERED_INSTRUCTION 64
#define P_ERROR                 65
#define P_MOVING                66


//16bits register address
#define P_MODEL_NUMBER         0
#define P_VERSION              3
#define P_MIN_ANGLE_LIMIT      9
#define P_MAX_ANGLE_LIMIT      11
#define P_MAX_TORQUE           16
#define P_PUNCH                24
#define P_IMAX                 28
#define P_OFFSET               30
#define P_GOAL_POSITION        42
#define P_GOAL_TIME            44
#define P_GOAL_SPEED           46
#define P_PRESENT_POSITION     56
#define P_PRESENT_SPEED        58
#define P_PRESENT_LOAD         60
#define P_VIR_POSITION         67
#define P_CURRENT              69

//Instruction:
#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03
#define INST_REG_WRITE 0x04
#define INST_ACTION 0x05
#define INST_RESET 0x06
#define INST_SYNC_WRITE 0x83
};

#endif
