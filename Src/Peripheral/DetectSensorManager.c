/*
 * DetectSensorManager.c
 *
 *  Created on: 2016年6月7日
 *      Author: Administrator
 */
#include <LiquidDriver/DetectSensorDriver.h>
#include <LiquidDriver/DetectSensorMap.h>
#include "Tracer/Trace.h"
#include "string.h"
#include "DetectSensorManager.h"
//系统中使用的继电器总数目

static Sensor s_sensors[DETECTSENSORCONF_TOTAL];

void DetectSensorManager_Init(void)
{
    memset(s_sensors, 0, sizeof(Sensor) * DETECTSENSORCONF_TOTAL);

    DetectSensorMap_Init(s_sensors);
}

Uint16 DetectSensorManager_GetTotalSensors(void)
{
    return (DETECTSENSORCONF_TOTAL);
}

Bool DetectSensorManager_GetSensor(Uint8 index)
{
    Uint8 i;
    Uint32 mask = 1;
    if (index <= DETECTSENSORCONF_TOTAL)
    {
        if (SENSOR_HIGH == DetectSensorDriver_ReadStatus(&s_sensors[index]))
		{
        	TRACE_MARK("\nSensor %d: High", index);
        	return TRUE;
		}
        else
        {
        	TRACE_MARK("\nSensor %d: Low", index);
        	return FALSE;
        }
    }
    else
    {
        TRACE_ERROR("\n The map must be 0 to 0x%x.", DETECTSENSOR_MAX_MAP);
        return FALSE;
    }
}

Uint32 DetectSensorManager_GetSensorsMap(void)
{
    Uint8 i;
    Uint32 mask = 0;
    Uint32 Temp = 1;

    for (i = 0; i < DETECTSENSORCONF_TOTAL; i++)
    {
        if (SENSOR_HIGH == DetectSensorDriver_ReadStatus(&s_sensors[i]))
        {
            mask |= Temp;
            Printf("\nSensor %d: High", i);
        }
        else
        {
        	Printf("\nSensor %d: Low", i);
        }
        Temp = Temp << 1;
    }
    return mask;
}

