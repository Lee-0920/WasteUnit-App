/**
 * @file
 * @brief 光学采集控制接口实现头文件
 * @details
 * @version 1.0.0
 * @author lemon.xiaoxun
 * @date 2016-5-27
 */

#ifndef SRC_DNCPSTACK_OPTICALCONTROL_H_
#define SRC_DNCPSTACK_OPTICALCONTROL_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "LuipApi/OpticalAcquireInterface.h"
#include "Common/Types.h"

typedef struct
{
    Uint32 reference;        // 参考端AD。
    Uint32 measure;          // 测量端AD。
}OptacalSignalAD;

void OpticalControl_Init();
void OpticalControl_Restore(void);
void OpticalControl_TurnOnLed(Bool);
OptacalSignalAD OpticalControl_GetNowSignalAD(void);
Bool OpticalControl_StartAcquirer(float adacquiretime);
Bool OpticalControl_StopAcquirer();
void OpticalControl_SendEventOpen(void);
void OpticalControl_SendEventClose(void);
void OpticalControl_SetSignalADNotifyPeriod(float period);
void OpticalControl_CollectTestFun(Bool isOpenTestMode, Uint8 channel);
void OpticalControl_PrintfInfo(void);

#ifdef __cplusplus
}
#endif

#endif /* SRC_DNCPSTACK_OPTICALCONTROL_H_ */
