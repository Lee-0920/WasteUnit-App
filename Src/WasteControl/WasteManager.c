/*
 * WasteManagerControl.c
 *
 *  Created on: 2021年10月11日
 *      Author: liwenqin
 */

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "SystemConfig.h"
#include "Common/Types.h"
#include "WasteManager.h"
#include "Tracer/Trace.h"
#include "DNCP/App/DscpSysDefine.h"
#include "SystemConfig.h"
#include "DNCP/Lai/LaiRS485Handler.h"
#include "DncpStack/DncpStack.h"
#include "Peripheral/DetectSensorManager.h"
#include "Peripheral/OutputManager.h"
#include "PeristalticPump/PumpManager.h"
#include "System.h"
#include "SolenoidValve/ValveManager.h"

#define PUMP_COLLECT_CELL 0  //收集池泵索引
#define PUMP_CYCLE_CELL   1  //循环池索引
#define PUMP_MAX_VALUME  100000
#define PUMP_SPEED  0.25
#define PUMP_ACC  0.2

#define CMD_SYSTEM_STOP 		1 //全部停止
#define CMD_COLLECT_CELL_STOP   2 //停止收集池抽液
#define CMD_CYCLE_CELL_STOP		3 //停止循环池排液
#define CMD_CYCLE_HANDLE_STOP	4 //停止循环池废水处理
#define CMD_SYS_RESET 			5 //系统复位
#define CMD_COLLECT_PUMP_DRAIN	6 //收集池泵排液
#define CMD_COLLECT_PUMP_SUCK	7 //收集池泵抽液
#define CMD_CYCLE_PUMP_DRAIN	8 //循环池泵排液
#define CMD_CYCLE_PUMP_SUCK	    9 //循环池泵抽液

#define VALVE_OPEN				03
#define VALVE_CLOSE				01

// LaiRS485指令 定义
#define CMD_HOLD                3
#define CMD_READ                4
#define CMD_WRITE               6
// LaiRS485寄存器地址 定义
#define REGISTER_STATUS         0
#define REGISTER_CONTROL        1

// 寄存器定义
typedef enum
{
    SystemStatus = 0x00,		//读系统状态
    ErrorCode = 0x01,			//读错误代码
    CollectPumpParam = 0x02,	//读收集池泵参数
	CyclePumpParam = 0x03,		//读循环池泵参数
	CycleDeviceParam = 0x04,	//读净化设备参数
	ControlCode = 0x10,		    //写控制指令
	SetCollectPumpParam = 0x11, //写收集池泵参数
	SetCyclePumpParam = 0x12,   //写循环池泵参数
	SetCycleDeviceParam = 0x13  //写净化设备参数
}RegisterMap;

// 系统状态定义
typedef enum
{
    CollectCell_Start = 1,		//收集池启动
	CollectCell_Working = 2,    //收集池工作
	CollectCell_Delay = 3,      //收集池过抽
	CycleCell_Start = 4,		//循环池启动
	CycleCell_Working = 5,	    //循环池工作
	CycleCell_Emptying = 6,		//循环池排空
	CollectCell_WorkingOnly = 7,//仅收集池工作
	CycleCell_WorkingOnly = 8,  //仅循环池工作
}SysSatus;

// 错误码定义
typedef enum
{
    CollectCell_Normal = 0,			//收集池正常
	CollectCell_Error_Full = 1,    //收集池液位告警
	CollectCell_Error_Dangerous = 2,//收集池溢出告警
	CycleCell_Normal = 10,			//循环池正常
	CycleCell_Error_Full = 11,		//循环池液位告警
	CollectCell_Pump_Normal = 20,	//收集池泵正常
	CollectCell_Pump_Error = 21,	//收集池泵异常告警
	CycleCell_Pump_Normal = 22,		//循环池泵正常
	CycleCell_Pump_Error = 23,		//循环池泵异常告警
	CycleCell_Device_Error = 30,    //循环池设备异常
}ErrCode;

//废水系统
typedef struct
{
	SysSatus sysStatus;			//系统状态
	ErrCode	 collectCell;		//收集池状态
	ErrCode	 collectPump;		//收集池泵状态
	ErrCode  cycleCell;			//循环池状态
	ErrCode  cyclePump;			//循环池泵状态
	int collectPumpSuckTime;    //收集池泵抽液时长
	Uint16 collectPumpDelayTime;//收集池泵过抽时长
	int collectPumpTimeout;		//收集池泵超时时长
	int collectPumpTimecnt;	    //收集池泵计时
	int cyclePumpDrainTime;		//循环池泵排液时长
	int cyclePumpTimeout;		//循环池泵超时时长
	int cyclePumpTimecnt;	    //循环池泵计时
	int cycleHandleTime;	    //循环池设备工作时间
	int cycleWaterInTime;		//循环设备进水时间
	int cyclePowerOnDrainTime;  //循环设备进水完成后反排时间
	int cyclePowerOnSuckTime;   //循环设备进水完成后抽水时间
}WasteUnit;

// 收集池传感器定义
typedef enum
{
    Collect_High = 2,
    Collect_Middle = 1,
    Collecl_Low = 0,
}CollectCellSensor;

// 循环池传感器定义
typedef enum
{
    Cycle_High = 4,
    Cycle_Low = 3,
	Cycle_DeviceOutput = 6,//设备完成信号
}CycleCellSensor;

// 泵方向定义 0-逆时针, 1-顺时针
typedef enum
{
	Drain_Collect = 1,
	Suck_Collect = 0,
	Drain_Cycle = 1,
	Suck_Cycle = 0,
}DrainDir;

//上电状态
typedef enum
{
	PowerOn_DrainStart = 0,
	PowerOn_WaitEmpty = 1,
	PowerOn_SuckStart = 2,
	PowerOn_Idle = 3,
}PowerStatus;

static void WasteManager_CollectCellTaskHandle(void *pvParameters);
static void WasteManager_CycleCellTaskHandle(void *pvParameters);
static void WasteManager_InputDetectTaskHandle(void *pvParameters);

static xTaskHandle s_xCollectCellHandle;
static xTaskHandle s_xCycleCellHandle;
static xTaskHandle s_xInputHandle;
static int s_WasteManagerReportPeriod = 1000;
static WasteUnit s_wasteUnit;
static Bool s_powerOn = TRUE;
static PowerStatus s_pStatus = PowerOn_Idle;

Byte s_sendBuf[8] = {0x01,0x04,0x00,0x01,0x00,0x00,0x00,0x00};

#define MODE_DEBUG 	0

void WasteManager_Init(void)
{
	memset(&s_wasteUnit, 0 , sizeof(s_wasteUnit));
	s_wasteUnit.sysStatus = CollectCell_Start;
	s_wasteUnit.collectCell = CollectCell_Normal;
	s_wasteUnit.collectPump = CollectCell_Pump_Normal;
	s_wasteUnit.cycleCell = CycleCell_Normal;
	s_wasteUnit.cyclePump = CycleCell_Pump_Normal;
	if(MODE_DEBUG)
	{
		s_wasteUnit.collectPumpSuckTime = 5*60; //收集池泵抽时间  未使用
		s_wasteUnit.collectPumpDelayTime = 10; //收集池泵过抽时间
		s_wasteUnit.collectPumpTimeout = 5*60;  //收集池泵超时时间
		s_wasteUnit.cyclePumpDrainTime = 0.2*60;  //循环池泵排液时间
		s_wasteUnit.cyclePumpTimeout = 0.2*60;    //循环池泵超时时间
		s_wasteUnit.cycleHandleTime = 30;     //循环设备工作时间
		s_wasteUnit.cycleWaterInTime = 5; //循环设备进水时间
		s_wasteUnit.collectPumpTimecnt = 0;   //收集池计时
		s_wasteUnit.cyclePumpTimecnt = 0;     //循环池计时
		s_wasteUnit.cyclePowerOnDrainTime = 5;//循环设备首次进水完成后反排时间
		s_wasteUnit.cyclePowerOnSuckTime = 10; //循环设备首次进水完成后抽水时间
	}
	else
	{
		s_wasteUnit.collectPumpSuckTime = 15*60; //收集池泵抽时间  未使用
		s_wasteUnit.collectPumpDelayTime = 5; //收集池泵过抽时间
		s_wasteUnit.collectPumpTimeout = 15*60;  //收集池泵超时时间
		s_wasteUnit.cyclePumpDrainTime = 8*60;  //循环池泵排液时间
		s_wasteUnit.cyclePumpTimeout = 8*60;    //循环池泵超时时间
		s_wasteUnit.cycleHandleTime = 12*60*60;     //循环设备工作时间
		s_wasteUnit.cycleWaterInTime = 6*60; //循环设备进水时间
		s_wasteUnit.collectPumpTimecnt = 0;   //收集池计时
		s_wasteUnit.cyclePumpTimecnt = 0;     //循环池计时
		s_wasteUnit.cyclePowerOnDrainTime = 3*60;//循环设备首次进水完成后反排时间
		s_wasteUnit.cyclePowerOnSuckTime = 1*60; //循环设备首次进水完成后抽水时间
	}


    xTaskCreate(WasteManager_CollectCellTaskHandle, "CollectCellTaskHandle", LEAK_MONITOR_STK_SIZE, NULL,
       		     LEAK_MONITOR_TASK_PRIO, &s_xCollectCellHandle);
    xTaskCreate(WasteManager_CycleCellTaskHandle, "CycleCellTaskHandle", LEAK_MONITOR_STK_SIZE, NULL,
          		     LEAK_MONITOR_TASK_PRIO, &s_xCycleCellHandle);
    xTaskCreate(WasteManager_InputDetectTaskHandle, "InputDetectTaskHandle", LEAK_MONITOR_STK_SIZE, NULL,
              		     LEAK_MONITOR_TASK_PRIO, &s_xInputHandle);
    System_NonOSDelay(1000);
    ValveManager_SetValvesMap(VALVE_CLOSE);

}

void WasteManager_StopAll(void)
{
	OutputManager_SetOutputsMap(0x00);
	ValveManager_SetValvesMap(VALVE_CLOSE);
	PumpManager_Stop(PUMP_COLLECT_CELL);
	PumpManager_Stop(PUMP_CYCLE_CELL);
	vTaskSuspend(s_xCycleCellHandle);
	vTaskSuspend(s_xCollectCellHandle);
	Printf("\n Stop All Task");
}

void WasteManager_StopCollectTask(void)
{
	PumpManager_Stop(PUMP_COLLECT_CELL);
	vTaskSuspend(s_xCollectCellHandle);
	Printf("\n Stop Collect Cell Task");
}

void WasteManager_StopCycleTask(void)
{
	OutputManager_SetOutputsMap(0x00);
	ValveManager_SetValvesMap(VALVE_CLOSE);
	PumpManager_Stop(PUMP_CYCLE_CELL);
	vTaskSuspend(s_xCycleCellHandle);
	Printf("\n Stop Cycle Cell Task");
}

void WasteManager_ModbusPump(Uint8 index, DrainDir dir, float vol)
{
	PumpManager_SetMotionParam(index, PUMP_ACC, PUMP_SPEED, NoWriteFLASH);
	PumpManager_Start(index, dir, vol, NoReadFLASH);
	Printf("\n Modbus Pump[%d] dir %d, vol %f",index, dir, vol);
}

void WasteManager_ErrorEvent(Uint16 code)
{
	s_sendBuf[1] = CMD_READ;
	s_sendBuf[3] = ErrorCode;
	s_sendBuf[5] = code;
	LaiRS485_ModbuSend(s_sendBuf, sizeof(s_sendBuf));
}

void WasteManager_Cmd(Uint8 cmd, Uint16 reg, Uint16 param)
{
	switch(cmd)
	{
		case CMD_READ: //读指令
			switch(reg)
			{
				case SystemStatus:
					Printf("\n [Modbus] SystemStatus");
					s_sendBuf[1] = CMD_READ;
					s_sendBuf[3] = SystemStatus;
					s_sendBuf[5] = s_wasteUnit.sysStatus;
					LaiRS485_ModbuSend(s_sendBuf, sizeof(s_sendBuf));
					break;
				case ErrorCode:
					Printf("\n [Modbus] ErrorCode");
					s_sendBuf[1] = CMD_READ;
					s_sendBuf[3] = ErrorCode;
					Uint8 err = 0;
					if(s_wasteUnit.collectCell == CollectCell_Error_Full)
					{
						err = CollectCell_Error_Full;
					}
					else if(s_wasteUnit.cycleCell == CycleCell_Error_Full)
					{
						err = CycleCell_Error_Full;
					}
					else if(s_wasteUnit.collectPump == CollectCell_Pump_Error)
					{
						err = CollectCell_Pump_Error;
					}
					else if(s_wasteUnit.cyclePump == CycleCell_Pump_Error)
					{
						err = CycleCell_Pump_Error;
					}
					s_sendBuf[5] = err;
					LaiRS485_ModbuSend(s_sendBuf, sizeof(s_sendBuf));
					break;
				case CollectPumpParam:
					Printf("\n [Modbus] CollectPumpParam");
					s_sendBuf[1] = CMD_READ;
					s_sendBuf[2] = 2;
					s_sendBuf[3] = (s_wasteUnit.collectPumpTimeout >> 8) & 0xFF;
					s_sendBuf[4] = s_wasteUnit.collectPumpTimeout & 0xFF;
//					memcpy(&s_sendBuf[3], &s_wasteUnit.collectPumpTimeout,sizeof(Uint16));
					LaiRS485_ModbuSend(s_sendBuf, sizeof(s_sendBuf)-1);
					break;
				case CyclePumpParam:
					Printf("\n [Modbus] CyclePumpParam");
					s_sendBuf[1] = CMD_READ;
					s_sendBuf[3] = 2;
					s_sendBuf[3] = (s_wasteUnit.cyclePumpTimeout >> 8) & 0xFF;
					s_sendBuf[4] = s_wasteUnit.cyclePumpTimeout & 0xFF;
//					memcpy(&s_sendBuf[4], &s_wasteUnit.cyclePumpTimeout,sizeof(Uint16));
					LaiRS485_ModbuSend(s_sendBuf, sizeof(s_sendBuf));
					break;
				case CycleDeviceParam:
					Printf("\n [Modbus] CycleDeviceParam");
					s_sendBuf[1] = CMD_READ;
					s_sendBuf[3] = 2;
					s_sendBuf[3] = (s_wasteUnit.cycleHandleTime >> 8) & 0xFF;
					s_sendBuf[4] = s_wasteUnit.cycleHandleTime & 0xFF;
//					memcpy(&s_sendBuf[4], &s_wasteUnit.cycleHandleTime,sizeof(Uint16));
					LaiRS485_ModbuSend(s_sendBuf, sizeof(s_sendBuf));
					break;
				default :
					Printf("\nInvalid Read Reg %d",reg);
					break;
			}
			break;
		case CMD_WRITE:
			switch(reg)
			{
				case ControlCode://输入寄存器写入参数
					Printf("\n ControlCode");
					switch(param)
					{
						case CMD_SYSTEM_STOP:
							Printf("\n [Modbus] CMD_SYSTEM_STOP");
							WasteManager_StopAll();
							break;
						case CMD_COLLECT_CELL_STOP:
							Printf("\n [Modbus] CMD_COLLECT_CELL_STOP");
							WasteManager_StopCollectTask();
							break;
						case CMD_CYCLE_CELL_STOP:
							Printf("\n [Modbus] CMD_CYCLE_CELL_STOP");
							WasteManager_StopCycleTask();
							break;
						case CMD_CYCLE_HANDLE_STOP:
							Printf("\n [Modbus] CMD_CYCLE_HANDLE_STOP");
							break;
						case CMD_SYS_RESET:
							Printf("\n [Modbus] CMD_SYS_RESET");
							param = CMD_SYS_RESET;
							s_sendBuf[1] = CMD_WRITE;
							s_sendBuf[2] = 0;
							s_sendBuf[3] = reg;
							s_sendBuf[4] = (param >> 8) & 0xFF;
							s_sendBuf[5] = param & 0xFF;
							LaiRS485_ModbuSend(s_sendBuf, sizeof(s_sendBuf));
							System_Delay(500);
							System_Reset();
							break;
						case CMD_COLLECT_PUMP_DRAIN:
							Printf("\n [Modbus] CMD_COLLECT_PUMP_DRAIN");
							WasteManager_ModbusPump(PUMP_COLLECT_CELL, Drain_Collect, 10);
							break;
						case CMD_COLLECT_PUMP_SUCK:
							Printf("\n [Modbus] CMD_COLLECT_PUMP_DRAIN");
							WasteManager_ModbusPump(PUMP_COLLECT_CELL, Suck_Collect, 10);
							break;
						case CMD_CYCLE_PUMP_DRAIN:
							Printf("\n [Modbus] CMD_CYCLE_PUMP_DRAIN");
							WasteManager_ModbusPump(PUMP_CYCLE_CELL, Drain_Cycle, 10);
							break;
						case CMD_CYCLE_PUMP_SUCK:
							Printf("\n [Modbus] CMD_CYCLE_PUMP_SUCK");
							WasteManager_ModbusPump(PUMP_CYCLE_CELL, Suck_Cycle, 10);
							break;
						default :
							Printf("\nInvalid Write Reg %d",reg);
							break;
					}
					break;
				case SetCollectPumpParam://输入寄存器写入参数
					if(param > 60)
					{
						Printf("\n SetCollectPumpParam %d", param);
						s_wasteUnit.collectPumpTimeout = param;
					}
					else
					{
						Printf("\nInvalid Param %d, can not less than 60s",param);
						param = s_wasteUnit.collectPumpTimeout;
					}
					break;
				case SetCyclePumpParam://输入寄存器写入参数
					if(param > 60)
					{
						Printf("\n SetCyclePumpParam %d", param);
						s_wasteUnit.cyclePumpTimeout = param;
					}
					else
					{
						Printf("\nInvalid Param %d, can not less than 60s",param);
						param = s_wasteUnit.cyclePumpTimeout;
					}
					break;
				case SetCycleDeviceParam://输入寄存器写入参数
					if(param > 300)
					{
						Printf("\n SetCycleDeviceParam %d", param);
						s_wasteUnit.cycleHandleTime = param;
					}
					else
					{
						Printf("\nInvalid Param %d, can not less than 300s",param);
						param = s_wasteUnit.cycleHandleTime;
					}
					break;
				case CollectPumpParam: //保持寄存器写入参数
					if(param > 60)
					{
						Printf("\n SetCollectPumpParam %d", param);
						s_wasteUnit.collectPumpTimeout = param;
					}
					else
					{
						Printf("\nInvalid Param %d, can not less than 60s",param);
						param = s_wasteUnit.collectPumpTimeout;
					}
					break;
				case CyclePumpParam: //保持寄存器写入参数
					if(param > 60)
					{
						Printf("\n SetCyclePumpParam %d", param);
						s_wasteUnit.cyclePumpTimeout = param;
					}
					else
					{
						Printf("\nInvalid Param %d, can not less than 60s",param);
						param = s_wasteUnit.cyclePumpTimeout;
					}
					break;
				case CycleDeviceParam://保持寄存器写入参数
					if(param > 300)
					{
						Printf("\n SetCycleDeviceParam %d", param);
						s_wasteUnit.cycleHandleTime = param;
					}
					else
					{
						Printf("\nInvalid Param %d, can not less than 300s",param);
						param = s_wasteUnit.cycleHandleTime;
					}
					break;
				default :
					Printf("\nInvalid Write Reg %d",reg);
					break;
			}
			s_sendBuf[1] = CMD_WRITE;
			s_sendBuf[2] = 0;
			s_sendBuf[3] = reg;
			s_sendBuf[4] = (param >> 8) & 0xFF;
			s_sendBuf[5] = param & 0xFF;
			LaiRS485_ModbuSend(s_sendBuf, sizeof(s_sendBuf));
			break;
		case CMD_HOLD:
			switch(reg)
			{
				case SystemStatus:
					Printf("\n [Modbus] SystemStatus");
					param = s_wasteUnit.sysStatus;
					break;
				case ErrorCode:
					Printf("\n [Modbus] ErrorCode");
					Uint8 err = 0;
					if(s_wasteUnit.collectCell == CollectCell_Error_Full)
					{
						err = CollectCell_Error_Full;
					}
					else if(s_wasteUnit.cycleCell == CycleCell_Error_Full)
					{
						err = CycleCell_Error_Full;
					}
					else if(s_wasteUnit.collectPump == CollectCell_Pump_Error)
					{
						err = CollectCell_Pump_Error;
					}
					else if(s_wasteUnit.cyclePump == CycleCell_Pump_Error)
					{
						err = CycleCell_Pump_Error;
					}
					param = err;
					break;
				case CollectPumpParam:
					Printf("\n [Modbus] CollectPumpParam");
					param = s_wasteUnit.collectPumpTimeout;
					break;
				case CyclePumpParam:
					Printf("\n [Modbus] CyclePumpParam");
					param = s_wasteUnit.cyclePumpTimeout;
					break;
				case CycleDeviceParam:
					Printf("\n [Modbus] CycleDeviceParam");
					param = s_wasteUnit.cycleHandleTime;
					break;
				case ControlCode:
					Printf("\n ControlCode");
					switch(param)
					{
						case CMD_SYSTEM_STOP:
							Printf("\n [Modbus] CMD_SYSTEM_STOP");
							WasteManager_StopAll();
							param = CMD_SYSTEM_STOP;
							break;
						case CMD_COLLECT_CELL_STOP:
							Printf("\n [Modbus] CMD_COLLECT_CELL_STOP");
							WasteManager_StopCollectTask();
							param = CMD_COLLECT_CELL_STOP;
							break;
						case CMD_CYCLE_CELL_STOP:
							Printf("\n [Modbus] CMD_CYCLE_CELL_STOP");
							WasteManager_StopCycleTask();
							param = CMD_CYCLE_CELL_STOP;
							break;
						case CMD_CYCLE_HANDLE_STOP:
							Printf("\n [Modbus] CMD_CYCLE_HANDLE_STOP");
							param = CMD_CYCLE_HANDLE_STOP;
							break;
						case CMD_SYS_RESET:
							Printf("\n [Modbus] CMD_SYS_RESET");
							param = CMD_SYS_RESET;
							s_sendBuf[1] = CMD_HOLD;
							s_sendBuf[2] = 0;
							s_sendBuf[3] = reg;
							s_sendBuf[4] = (param >> 8) & 0xFF;
							s_sendBuf[5] = param & 0xFF;
							LaiRS485_ModbuSend(s_sendBuf, sizeof(s_sendBuf));
							System_Delay(500);
							System_Reset();
							break;
						case CMD_COLLECT_PUMP_DRAIN:
							Printf("\n [Modbus] CMD_COLLECT_PUMP_DRAIN");
							WasteManager_ModbusPump(PUMP_COLLECT_CELL, Drain_Collect, 10);
							param = CMD_COLLECT_PUMP_DRAIN;
							break;
						case CMD_COLLECT_PUMP_SUCK:
							Printf("\n [Modbus] CMD_COLLECT_PUMP_DRAIN");
							WasteManager_ModbusPump(PUMP_COLLECT_CELL, Suck_Collect, 10);
							param = CMD_COLLECT_PUMP_SUCK;
							break;
						case CMD_CYCLE_PUMP_DRAIN:
							Printf("\n [Modbus] CMD_CYCLE_PUMP_DRAIN");
							WasteManager_ModbusPump(PUMP_CYCLE_CELL, Drain_Cycle, 10);
							param = CMD_CYCLE_PUMP_DRAIN;
							break;
						case CMD_CYCLE_PUMP_SUCK:
							Printf("\n [Modbus] CMD_CYCLE_PUMP_SUCK");
							WasteManager_ModbusPump(PUMP_CYCLE_CELL, Suck_Cycle, 10);
							param = CMD_CYCLE_PUMP_SUCK;
							break;
						default :
							Printf("\nInvalid Write Reg %d",reg);
							break;
					}
					break;
				case SetCollectPumpParam:
					if(param > 60)
					{
						Printf("\n SetCollectPumpParam %d", param);
						s_wasteUnit.collectPumpTimeout = param;
					}
					else
					{
						Printf("\nInvalid Param %d, can not less than 60s",param);
						param = s_wasteUnit.collectPumpTimeout;
					}
					break;
				case SetCyclePumpParam:
					if(param > 60)
					{
						Printf("\n SetCyclePumpParam %d", param);
						s_wasteUnit.cyclePumpTimeout = param;
					}
					else
					{
						Printf("\nInvalid Param %d, can not less than 60s",param);
						param = s_wasteUnit.cyclePumpTimeout;
					}
					break;
				case SetCycleDeviceParam:
					if(param > 300)
					{
						Printf("\n SetCycleDeviceParam %d", param);
						s_wasteUnit.cycleHandleTime = param;
					}
					else
					{
						Printf("\nInvalid Param %d, can not less than 300s",param);
						param = s_wasteUnit.cycleHandleTime;
					}
					break;
				default :
					Printf("\nInvalid Write Reg %d",reg);
					break;
			}
			s_sendBuf[1] = CMD_HOLD;
			s_sendBuf[2] = 2;
			s_sendBuf[3] = (param >> 8) & 0xFF;
			s_sendBuf[4] = param & 0xFF;
			LaiRS485_ModbuSend(s_sendBuf, sizeof(s_sendBuf)-1);
			break;
		default :
			Printf("\nInvalid Cmd %d",cmd);
	}
}

/*
 * 收集池处理任务，负责监测收集池液位状态、泵状态，并上报异常给到中控系统
 * */
static void WasteManager_CollectCellTaskHandle(void *pvParameters)
{
	(void) pvParameters;
	static Uint16 temp = 0;
//	vTaskSuspend(NULL);
	while(1)
	{
		switch(s_wasteUnit.sysStatus)
		{
			case CollectCell_Start:
				//初始循环池液位有水,收集池泵执行反抽
				if(TRUE == DetectSensorManager_GetSensor(Cycle_Low) && FALSE == DetectSensorManager_GetSensor(Collect_Middle))
				{
					if(s_wasteUnit.collectPumpTimecnt == 0)
					{
						Printf("\n CycleCell Already Reached");
						Printf("\n CollectCell Suck From CycleCell");
						PumpManager_SetMotionParam(PUMP_COLLECT_CELL, PUMP_ACC, PUMP_SPEED, NoWriteFLASH);
						PumpManager_Start(PUMP_COLLECT_CELL, Suck_Collect, PUMP_MAX_VALUME, NoReadFLASH);
					}
					else if(s_wasteUnit.collectPumpTimecnt> s_wasteUnit.collectPumpTimeout)
					{
						Printf("\n CollectCell Suck From CycleCell Failed,Pls Check Pump");
						WasteManager_ErrorEvent(CollectCell_Pump_Error);
						PumpManager_Stop(PUMP_COLLECT_CELL);
						vTaskSuspend(NULL);
					}
					s_wasteUnit.collectPumpTimecnt++;
				}
				//收集池启动回抽过程中,收集池上液位异常-停止
				else if(s_wasteUnit.collectPumpTimecnt > 0
						&& TRUE == DetectSensorManager_GetSensor(Collect_Middle))
				{
					Printf("\n CollectCell Suck From CycleCell Over Volume");
					WasteManager_ErrorEvent(CollectCell_Pump_Error);
					PumpManager_Stop(PUMP_COLLECT_CELL);
					s_wasteUnit.collectPumpTimecnt = 0;
					s_wasteUnit.collectCell = CollectCell_Error_Full;
				}
				//收集池启动回抽,等待液位传感器为无水状态-停止
				else if(FALSE == DetectSensorManager_GetSensor(Cycle_Low) && s_wasteUnit.collectPumpTimecnt > 0)
				{
					PumpManager_Stop(PUMP_COLLECT_CELL);
					Printf("\n CycleCell Drain From CycleCell Finish");
					System_Delay(2000);
					PumpManager_SetMotionParam(PUMP_COLLECT_CELL, PUMP_ACC, PUMP_SPEED, NoWriteFLASH);
					PumpManager_Start(PUMP_COLLECT_CELL, Drain_Collect, PUMP_MAX_VALUME, NoReadFLASH);
					s_wasteUnit.sysStatus = CollectCell_Working;
					s_wasteUnit.collectCell = CollectCell_Normal;
					s_wasteUnit.collectPumpTimecnt = 0;
				}
				else if(TRUE == DetectSensorManager_GetSensor(Collecl_Low))
				{
					Printf("\n CollectCell_Start Drain Timeout %d s",s_wasteUnit.collectPumpTimeout);
					PumpManager_SetMotionParam(PUMP_COLLECT_CELL, PUMP_ACC, PUMP_SPEED, NoWriteFLASH);
					PumpManager_Start(PUMP_COLLECT_CELL, Drain_Collect, PUMP_MAX_VALUME, NoReadFLASH);
					s_wasteUnit.sysStatus = CollectCell_Working;
					s_wasteUnit.collectCell = CollectCell_Normal;
					s_wasteUnit.collectPumpTimecnt = 0;
				}
				break;
			case CollectCell_Working:
				s_wasteUnit.collectPumpTimecnt++;
				if(TRUE == DetectSensorManager_GetSensor(Cycle_Low))
				{
					Printf("\n CollectCell_Delay %d s",s_wasteUnit.collectPumpDelayTime);
					s_wasteUnit.sysStatus = CollectCell_Delay;
					s_wasteUnit.collectPumpTimecnt = 0;
				}
				break;
			case CollectCell_Delay:
				s_wasteUnit.collectPumpTimecnt++;
				if(s_wasteUnit.collectPumpTimecnt > s_wasteUnit.collectPumpDelayTime)
				{
					Printf("\n CollectCell_Delay Finish");
					PumpManager_Stop(PUMP_COLLECT_CELL);
					s_wasteUnit.sysStatus = CycleCell_Start;
					s_wasteUnit.collectPumpTimecnt = 0;
				}
				break;
		}

		//收集池满液
		if(TRUE == DetectSensorManager_GetSensor(Collect_Middle))
		{
			s_wasteUnit.collectCell = CollectCell_Error_Full;
			Printf("\n CollectCell_Error_Full");
			WasteManager_ErrorEvent(CollectCell_Error_Full);
		}
//		//收集池溢出
//		if(TRUE == DetectSensorManager_GetSensor(Collect_High))
//		{
//			s_wasteUnit.collectCell = CollectCell_Error_Dangerous;
//			Printf("\n CollectCell_Error_Dangerous");
//			WasteManager_ErrorEvent(CollectCell_Error_Dangerous);
//		}
		//收集池泵异常
		if(s_wasteUnit.collectPumpTimecnt > s_wasteUnit.collectPumpTimeout && FALSE == DetectSensorManager_GetSensor(Cycle_Low))
		{
			s_wasteUnit.collectPump = CollectCell_Pump_Error;
			Printf("\n CollectCell_Pump_Error, Run Time %d s, Timeout %d s",s_wasteUnit.collectPumpTimecnt, s_wasteUnit.collectPumpTimeout);
			PumpManager_Stop(PUMP_COLLECT_CELL);
			WasteManager_ErrorEvent(CollectCell_Pump_Error);
			s_wasteUnit.collectPump = CollectCell_Pump_Error;
			s_wasteUnit.sysStatus = CycleCell_WorkingOnly;
			s_wasteUnit.collectPumpTimecnt = 0;
//			vTaskSuspend(NULL);//不挂起，保持液位监测功能
		}
		vTaskDelay(s_WasteManagerReportPeriod / portTICK_RATE_MS);
	}
}

/*
 * 循环池处理任务，负责监测循环池液位状态、泵状态，并上报异常给到中控系统
 * */
static void WasteManager_CycleCellTaskHandle(void *pvParameters)
{
	(void) pvParameters;
	static Uint16 temp = 0;
//	vTaskSuspend(NULL);
	while(1)
	{
		switch(s_wasteUnit.sysStatus)
		{
			case CycleCell_Start:
				if(TRUE == DetectSensorManager_GetSensor(Cycle_Low))
				{
					Printf("\n CycleCell_Start");
					s_wasteUnit.sysStatus = CycleCell_Working;
					s_wasteUnit.cyclePumpTimecnt = 0;
					OutputManager_SetOutputsMap(0x03);
				}
				break;
			case CycleCell_Working:
				if(s_wasteUnit.cyclePumpTimecnt == 0)
				{
					Printf("\n CycleCell_Working Start %d s", s_wasteUnit.cycleHandleTime);
					ValveManager_SetValvesMap(VALVE_OPEN);
				}
				else if(s_wasteUnit.cyclePumpTimecnt == 3)
				{
					OutputManager_SetOutputsMap(0x00);
				}
				s_wasteUnit.cyclePumpTimecnt++;
				if(s_wasteUnit.cyclePumpTimecnt > s_wasteUnit.cycleHandleTime)
				{
					Printf("\n CycleCell_Working Error, Device Output Timeout");
					s_wasteUnit.sysStatus = CycleCell_Working;
					s_wasteUnit.cyclePumpTimecnt = 0;
					s_wasteUnit.cyclePump = CycleCell_Device_Error;
					WasteManager_ErrorEvent(CycleCell_Device_Error);
				}
				if(s_wasteUnit.cyclePumpTimecnt > s_wasteUnit.cycleWaterInTime && s_powerOn == TRUE)
				{
					Printf("\n CycleCell Finish Water Input");
					Printf("\n CycleCell Drain Empty");
					PumpManager_SetMotionParam(PUMP_CYCLE_CELL, PUMP_ACC, 0.4, NoWriteFLASH);
					PumpManager_Start(PUMP_CYCLE_CELL, Drain_Cycle, 0.4 * s_wasteUnit.cyclePowerOnDrainTime, NoReadFLASH);
					s_powerOn = FALSE;
					s_pStatus = PowerOn_DrainStart;
				}

				if(PUMP_BUSY == PumpManager_GetStatus(PUMP_CYCLE_CELL) && PowerOn_DrainStart == s_pStatus)
				{
					s_pStatus = PowerOn_WaitEmpty;
				}
				else if(PUMP_IDLE == PumpManager_GetStatus(PUMP_CYCLE_CELL) && PowerOn_WaitEmpty == s_pStatus)
				{
					Printf("\n CycleCell Finish Drain Empty");
					Printf("\n CollectCell Drain To CycleCell");
					PumpManager_SetMotionParam(PUMP_COLLECT_CELL, PUMP_ACC, 0.4, NoWriteFLASH);
					PumpManager_Start(PUMP_COLLECT_CELL, Drain_Collect, 0.4 * s_wasteUnit.cyclePowerOnSuckTime, NoReadFLASH);
					s_pStatus = PowerOn_SuckStart;
				}
				else if(PUMP_IDLE == PumpManager_GetStatus(PUMP_COLLECT_CELL) && PowerOn_SuckStart == s_pStatus)
				{
					s_pStatus = PowerOn_Idle;
				}

				break;
			case CycleCell_Emptying:
				if(s_wasteUnit.cyclePumpTimecnt == 0)
				{
					Printf("\n CycleCell_Emptying Drain %d s", s_wasteUnit.cyclePumpDrainTime);
					ValveManager_SetValvesMap(VALVE_CLOSE);
				}
				s_wasteUnit.cyclePumpTimecnt++;
				//循环池泵正常，进入下一次循环
				if(s_wasteUnit.cyclePumpTimecnt > s_wasteUnit.cyclePumpDrainTime && FALSE == DetectSensorManager_GetSensor(Cycle_Low))
				{
					Printf("\n CycleCell_Emptying Finish, Enter Next Loop");
					PumpManager_Stop(PUMP_CYCLE_CELL);
					s_wasteUnit.sysStatus = CollectCell_Start;
					s_wasteUnit.cycleCell = CycleCell_Normal;
					OutputManager_SetOutputsMap(0x00);
				}
				//循环池泵排液异常
				if(s_wasteUnit.cyclePumpTimecnt > s_wasteUnit.cyclePumpTimeout && TRUE == DetectSensorManager_GetSensor(Cycle_Low))
				{
					s_wasteUnit.cyclePump = CycleCell_Pump_Error;
					Printf("\n CycleCell_Pump_Error Drain Time %d s, Timeout %d s",s_wasteUnit.cyclePumpTimecnt, s_wasteUnit.cyclePumpTimeout);
					PumpManager_Stop(PUMP_CYCLE_CELL);
					OutputManager_SetOutputsMap(0x00);
					ValveManager_SetValvesMap(VALVE_CLOSE);
					WasteManager_ErrorEvent(CycleCell_Pump_Error);
					s_wasteUnit.sysStatus = CollectCell_WorkingOnly;
					s_wasteUnit.cyclePump = CycleCell_Pump_Error;
					vTaskSuspend(NULL);
				}
				break;
		}

		//循环池满液,停止循环设备、循环池泵、收集池泵, 收集池继续收集废液，直至废液装满
		if(TRUE == DetectSensorManager_GetSensor(Cycle_High))
		{
			s_wasteUnit.cycleCell = CycleCell_Error_Full;
			OutputManager_SetOutputsMap(0x00);
			ValveManager_SetValvesMap(VALVE_CLOSE);
			PumpManager_Stop(PUMP_COLLECT_CELL);
			PumpManager_Stop(PUMP_CYCLE_CELL);
			Printf("\n CycleCell_Error_Full, Stop Cycle Cell Task");
			WasteManager_ErrorEvent(CycleCell_Error_Full);
			s_wasteUnit.sysStatus = CollectCell_WorkingOnly;
			vTaskSuspend(NULL);
		}
		if(s_wasteUnit.sysStatus == CycleCell_Working && (s_wasteUnit.cyclePumpTimecnt % 300 == 0))
		{
			Printf("\n CycleCell_Working, Send Trigger Output");
			OutputManager_SetOutputsMap(0x03);
			System_Delay(1000);
			OutputManager_SetOutputsMap(0x00);
		}
		vTaskDelay(s_WasteManagerReportPeriod / portTICK_RATE_MS);
	}

}

/*
 * 循环池信号检测任务，负责检测净化设备催化完成信号
 *
 *  * */
static void WasteManager_InputDetectTaskHandle(void *pvParameters)
{
	(void) pvParameters;
	static Uint32 cnt = 0;
//	vTaskSuspend(NULL);
	while(1)
	{
		if(s_wasteUnit.sysStatus == CycleCell_Working && FALSE == DetectSensorManager_GetSensor(Cycle_DeviceOutput))
		{
			Printf("\n CycleCell_Working Finish");
			s_wasteUnit.sysStatus = CycleCell_Emptying;
			s_wasteUnit.cyclePumpTimecnt = 0;
			PumpManager_SetMotionParam(PUMP_CYCLE_CELL, PUMP_ACC, PUMP_SPEED, NoWriteFLASH);
			PumpManager_Start(PUMP_CYCLE_CELL, Drain_Cycle, PUMP_MAX_VALUME, NoReadFLASH);
		}
		vTaskDelay(50 / portTICK_RATE_MS);
	}
}

