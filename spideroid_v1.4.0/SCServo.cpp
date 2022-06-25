/* SCServo.h modified: Taner AKKAN 4.4.2017*/
/* Data send Format is the same with AX12.
  0XFF 0XFF ID LENGTH INSTR PAR1 PAR2 ... PARN CHECKSUM
  ID: 0-253 , 254:ALL
  INSTR -- () means number of parameters
  0x01: PING          No execution.  (0)
  0x02: READ_DATA     Read Data       (2)
  0x03: WRITE_DATA    Write Data to one motor or more  (2 or more)
  0x04: REG WRITE     Similar to WRITE_DATA, but remains in the standby state (2 or more)
                      without being executed until the ACTION command arrives.
  0x05: ACTION        initiates motions registered with REG WRITE              (0)
  0x06: RESET         restores the state of motor to the factory default setting. (0)
  0x83: SYNC WRITE    used to control several motors simultaneously at a time. (4 or more)

  return byte is the error byte .. 1: means error 0: no error
  Bit 7 : 0                   , -
  Bit 6 : Instruction Error   , undefined instr or action command without REG WRITE command
  Bit 5 : Overload Error      , current overload
  Bit 4 : Checksum Error      , Checksum of the transmitted Instruction Packet is incorrect
  Bit 3 : Range Error         , command is out of the range 
  Bit 2 : Overheating  Error  , internal temp is out of limits
  Bit 1 : Angle Limit Error   , Goal Position is  out of the range from CW to CCW Angle Limit 
  Bit 0 : Input Voltage Error , applied voltage is out of the range of operating voltage 

  Compliance PID explanation :  Combined concept of margine and slope
  The less Pgain, the larger backlash & the weaker amount of output near goal position  
  Kp = Pgain / 8 ,   Ki = Igain *1000 / 2048 ,   Kd = Dgain * 4 / 1000
  Compliance slope =   8 --> Pgain = 128  -- 
  Compliance slope =  16 --> Pgain =  64  -- 
  Compliance slope =  32 --> Pgain =  32      
  Compliance slope =  64 --> Pgain =  16
  Compliance slope = 128 --> Pgain =   8
 */
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#define printf(args) (Serial2.write(args))
#else
#include "WProgram.h"
#define printf(args) (Serial2.print(args,BYTE))
#endif
#include "SCServo.h"

SCServo::SCServo () { }
//--------------------------------------------------------------------
u8 SCServo::EnableTorque(u8 ID, u8 Enable, u8 ReturnLevel)
{ int msgLen = 4;
printf(startByte);  printf(startByte); printf(ID); printf(msgLen);
printf(INST_WRITE); printf(P_TORQUE_ENABLE); printf(Enable);
printf((~(ID + msgLen + INST_WRITE + Enable + P_TORQUE_ENABLE))&0xFF);
if(ID != 0xfe && ReturnLevel==2) return ReadBuf(6); else return 0;
}
//--------------------------------------------------------------------
u8 SCServo:: WritePos(u8 ID, s16 pozition, s16 velocity, u8 ReturnLevel)
{ int msgLen = 7;
byte posL = pozition>>8; byte posH = pozition&0xff;
byte velL = velocity>>8; byte velH = velocity&0xff;

printf(startByte); printf(startByte); printf(ID); printf(msgLen);
printf(INST_WRITE); printf(P_GOAL_POSITION_L);
printf(posL); printf(posH);
printf(velL); printf(velH);
printf((~(ID+msgLen+INST_WRITE+P_GOAL_POSITION_L+posL+posH+velL+velH))&0xFF);
if(ID != 0xfe && ReturnLevel==2) return ReadBuf(6); else return 0;
}
//--------------------------------------------------------------------
u8 SCServo:: RegWritePos(u8 ID, s16 pozition, s16 velocity, u8 ReturnLevel)
{ int msgLen = 7;
byte posL = pozition>>8; byte posH = pozition&0xff;
byte velL = velocity>>8; byte velH = velocity&0xff;

printf(startByte); printf(startByte); printf(ID);printf(msgLen);
printf(INST_REG_WRITE); printf(P_GOAL_POSITION_L);
printf(posL);printf(posH); printf(velL);printf(velH);
printf((~(ID+msgLen+INST_REG_WRITE+P_GOAL_POSITION_L+posL+posH+velL+velH))&0xFF);
if(ID != 0xfe && ReturnLevel==2) return ReadBuf(6); else return 0;
}
//--------------------------------------------------------------------
void SCServo:: RegWriteAction()
{ int msgLen = 2; byte ID = 0xFE;
printf(startByte); printf(startByte); printf(ID); printf(msgLen);
printf(INST_ACTION);
printf((~(ID + msgLen + INST_ACTION))&0xFF);
}
//--------------------------------------------------------------------
u8 SCServo:: ReadBuf(u8 len, u8 *buf)
{ u16 n = 0; u8 sayz = 0; u8 ComData;
while(n<TIMEOUT)
{
if(Serial2.available())
{
if(buf) buf[sayz] = Serial2.read();
else ComData = Serial2.read();
sayz++;
if(sayz>=len) break;
n = 0;
}
n++;
}
return sayz;
}
//--------------------------------------------------------------------
s16 SCServo:: ReadPos(u8 ID)
{ int msgLen = 4; u8 buf[8]; u8 sayz; u16 pos; int numbyte=2;
memset(buf,0,sizeof(buf));
printf(startByte); printf(startByte); printf(ID); printf(msgLen);
printf(INST_READ); printf(P_PRESENT_POSITION_L); printf(numbyte);
printf((~(ID + msgLen + INST_READ + P_PRESENT_POSITION_L + numbyte))&0xFF);
sayz = ReadBuf(8, buf);
if(sayz<8) return -1;
pos = buf[5]; pos <<= 8; pos |= buf[6]; 
return (s16)pos;
}
//--------------------------------------------------------------------
void SCServo:: SyncWritePos(u8 ID[], u8 IDN, s16 pozition, s16 velocity)
{ int msgLen=5*IDN+4; int numbyte=4;
u8 Sum = 0;
byte posL = pozition>>8; byte posH = pozition&0xff;
byte velL = velocity>>8; byte velH = velocity&0xff;

printf(startByte); printf(startByte); printf(0xfe);printf(msgLen);
printf(INST_SYNC_WRITE);printf(P_GOAL_POSITION_L); printf(numbyte);
Sum = 0xfe + msgLen + INST_SYNC_WRITE + P_GOAL_POSITION_L + numbyte;
int i;
for(i=0; i<IDN; i++)
{ printf(ID[i]);
printf(posL);printf(posH);
printf(velL);printf(velH);
Sum += ID[i] + posL + posH + velL + velH;
}
printf((~Sum)&0xFF);
}
//--------------------------------------------------------------------
s16 SCServo:: WriteID(u8 oldID, u8 newID, u8 ReturnLevel)
{ int msgLen = 4; 
u8 buf[7]; u8 sayz; s16 kayitci;
memset(buf,0,sizeof(buf));

printf(startByte); printf(startByte); printf(oldID);printf(msgLen);
printf(INST_WRITE); printf(P_ID); printf(newID);
printf((~(oldID + msgLen + INST_WRITE + P_ID + newID ))&0xFF);
if(oldID != 0xfe && ReturnLevel==2) 
{
sayz = ReadBuf(7, buf);
if(sayz<7) return -1;
kayitci = buf[4];
return kayitci;
}
else return 0;
}
//--------------------------------------------------------------------
u8 SCServo:: WriteLimitAngle(u8 ID, u16 MinAngel, u16 MaxAngle, u8 ReturnLevel)
{ int msgLen = 7;
byte MinAL = MinAngel>>8; byte MinAH = MinAngel&0xff;
byte MaxAL = MaxAngle>>8; byte MaxAH = MaxAngle&0xff;

printf(startByte); printf(startByte); printf(ID);printf(msgLen);
printf(INST_WRITE);printf(P_MIN_ANGLE_LIMIT_L);
printf(MinAL);printf(MinAH);
printf(MaxAL);printf(MaxAH);
printf((~(ID+msgLen+INST_WRITE+P_MIN_ANGLE_LIMIT_L+MinAL+MinAH+MaxAL+MaxAH))&0xFF);
if(ID != 0xfe && ReturnLevel==2) return ReadBuf(6);
else return 0;
}
//--------------------------------------------------------------------
u8 SCServo:: WriteLimitTorque(u8 ID, u16 MaxTorque, u8 ReturnLevel)
{ int msgLen = 5;
byte MaxTL = MaxTorque>>8;
byte MaxTH = MaxTorque&0xff;

printf(startByte); printf(startByte); printf(ID);printf(msgLen);
printf(INST_WRITE); printf(P_MAX_TORQUE_L); printf(MaxTL); printf(MaxTH);
printf((~(ID + msgLen + INST_WRITE + P_MAX_TORQUE_L + MaxTL + MaxTH))&0xFF);
if(ID != 0xfe && ReturnLevel==2) return ReadBuf(6);
else return 0;
}
//--------------------------------------------------------------------
u8 SCServo:: WritePunch(u8 ID, u16 Punch, u8 ReturnLevel)
{ int msgLen = 5;
byte PunchL = Punch>>8;
byte PunchH = Punch&0xff;

printf(startByte); printf(startByte); printf(ID);printf(msgLen);
printf(INST_WRITE); printf(P_PUNCH_L); printf(PunchL); printf(PunchH);
printf((~(ID + msgLen + INST_WRITE + P_PUNCH_L + PunchL + PunchH))&0xFF);
if(ID != 0xfe && ReturnLevel==2) return ReadBuf(6);
else return 0;
}
//--------------------------------------------------------------------
u8 SCServo:: WriteBaud(u8 ID, u8 Baud, u8 ReturnLevel)
{ int msgLen = 4;
printf(startByte); printf(startByte); printf(ID);printf(msgLen);
printf(INST_WRITE); printf(P_BAUD_RATE); printf(Baud);
printf((~(ID + msgLen + INST_WRITE + P_BAUD_RATE + Baud))&0xFF);
if(ID != 0xfe && ReturnLevel==2) return ReadBuf(6);
else return 0;
}
//--------------------------------------------------------------------
u8 SCServo:: WriteComplianceMargin(u8 ID, u8 CCW, u8 CW, u8 ReturnLevel)
{ int msgLen = 5;

printf(startByte); printf(startByte); printf(ID);printf(msgLen);
printf(INST_WRITE); printf(P_CW_COMPLIANCE_MARGIN); printf(CW); printf(CCW);
printf((~(ID+msgLen+INST_WRITE+P_CW_COMPLIANCE_MARGIN+CW+CCW))&0xFF);
if(ID != 0xfe && ReturnLevel==2) return ReadBuf(6);
else return 0;
}
//--------------------------------------------------------------------
u8 SCServo:: LockEeprom(u8 ID, u8 Enable, u8 ReturnLevel)
{ int msgLen = 4;

printf(startByte); printf(startByte); printf(ID);printf(msgLen);
printf(INST_WRITE); printf(P_LOCK); printf(Enable);
printf((~(ID + msgLen + INST_WRITE + P_LOCK + Enable))&0xFF);
if(ID != 0xfe && ReturnLevel==2) return ReadBuf(6);
else return 0;
}
//--------------------------------------------------------------------
u8 SCServo:: WritePID(u8 ID, u8 P, u8 I, u8 D, u8 ReturnLevel)
{ int msgLen = 6;

printf(startByte); printf(startByte); printf(ID);printf(msgLen);
printf(INST_WRITE); printf(P_COMPLIANCE_P); printf(P); printf(D); printf(I);
printf((~(ID + msgLen + INST_WRITE + P_COMPLIANCE_P + P + D + I))&0xFF);
if(ID != 0xfe && ReturnLevel==2) return ReadBuf(6);
else return 0;
}
//--------------------------------------------------------------------
u8 SCServo:: WriteSpeed(u8 ID, s16 velocity, u8 ReturnLevel)
{ int msgLen = 5;
u16 vel = velocity;
if(velocity<0) {vel = -velocity;vel |= (1<<10);}
byte velL = vel>>8;
byte velH = vel&0xff;

printf(startByte); printf(startByte); printf(ID);printf(msgLen);
printf(INST_WRITE); printf(P_GOAL_SPEED_L); printf(velL); printf(velH);
printf((~(ID + msgLen + INST_WRITE + P_GOAL_SPEED_L + velL + velH))&0xFF);
if(ID != 0xfe && ReturnLevel==2) return ReadBuf(6);
else return 0;
}
//--------------------------------------------------------------------
s16 SCServo:: ReadReg(u8 ID,u8 kayitcino)
{ int msgLen = 4; u8 buf[7]; u8 sayz; s16 kayitci; int numbyte=1;
memset(buf,0,sizeof(buf));
printf(startByte); printf(startByte); printf(ID);printf(msgLen);
printf(INST_READ); printf(kayitcino); printf(numbyte);
printf((~(ID + msgLen+ INST_READ + kayitcino + numbyte))&0xFF);

sayz = ReadBuf(7, buf);
if(sayz<7)
return -1;
kayitci = buf[5];
return kayitci;
}
//--------------------------------------------------------------------
s16 SCServo:: ReadReg16(u8 ID,u8 kayitcino)
{ int msgLen = 4; u8 buf[8]; u8 sayz; u16 kayitci; int numbyte=2;
memset(buf,0,sizeof(buf));
printf(startByte); printf(startByte); printf(ID);printf(msgLen);
printf(INST_READ); printf(kayitcino); printf(numbyte);
printf((~(ID + msgLen + INST_READ + kayitcino + numbyte))&0xFF);

sayz = ReadBuf(8, buf);
if(sayz<8) return -1;
kayitci = buf[5]; kayitci <<= 8; kayitci |= buf[6];
return (s16)kayitci;
}
//--------------------------------------------------------------------
u8 SCServo:: WheelDir(u8 ID, u8 Enable, u8 ReturnLevel)
{ int msgLen = 4;
printf(startByte);  printf(startByte); printf(ID); printf(msgLen);
printf(INST_WRITE); printf(P_GOAL_SPEED_H); printf(Enable);
printf((~(ID + msgLen + INST_WRITE + Enable + P_TORQUE_ENABLE))&0xFF);
if(ID != 0xfe && ReturnLevel==2) return ReadBuf(6); else return 0;
}
