/*
 * OutputManager.c
 *
 *  Created on: 2016年6月7日
 *      Author: Administrator
 */
#include <LiquidDriver/OutputDriver.h>
#include <LiquidDriver/OutputMap.h>
#include "Tracer/Trace.h"
#include "string.h"
#include "OutputManager.h"
//系统中使用的继电器总数目

static Output s_outputs[OUTPUTCONF_TOTAL];

void OutputManager_Init(void)
{
    memset(s_outputs, 0, sizeof(Output) * OUTPUTCONF_TOTAL);

    OutputMap_Init(s_outputs);
}

Uint16 OutputManager_GetTotalValves(void)
{
    return (OUTPUTCONF_TOTAL);
}

Bool OutputManager_SetOutputsMap(Uint32 map)
{
    Uint8 i;
    Uint32 mask = 1;
    if (map <= OUTPUT_MAX_MAP)
    {
        TRACE_INFO("\nOutput SetMap: 0x%x", map);
        for (i = 0; i < OUTPUTCONF_TOTAL; i++)
        {
            if (map & mask)
            {
                OutputDriver_Control(&s_outputs[i], OUTPUT_OPEN);
            }
            else
            {
                OutputDriver_Control(&s_outputs[i], OUTPUT_CLOSE);
            }
            mask = mask << 1;
        }
        return TRUE;
    }
    else
    {
        TRACE_ERROR("\n The map must be 0 to 0x%x.", OUTPUT_MAX_MAP);
        return FALSE;
    }
}

Uint32 OutputManager_GetOutputsMap(void)
{
    Uint8 i;
    Uint32 mask = 0;
    Uint32 Temp = 1;

    for (i = 0; i < OUTPUTCONF_TOTAL; i++)
    {
        if (OUTPUT_OPEN == OutputDriver_ReadStatus(&s_outputs[i]))
        {
            mask |= Temp;
        }
        Temp = Temp << 1;
    }
    return mask;
}

