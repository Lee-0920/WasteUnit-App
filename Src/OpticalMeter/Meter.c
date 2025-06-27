/*
 * Meter.c
 *
 *  Created on: 2016年6月14日
 *      Author: Administrator
 */

#include <math.h>
#include <McuFlash.h>
#include "string.h"
#include "SystemConfig.h"
#include "MeterScheduler.h"
#include "FreeRTOS.h"
#include "task.h"
#include "LuipApi/OpticalMeterInterface.h"
#include "DncpStack/DncpStack.h"
#include "SolenoidValve/ValveManager.h"
#include "System.h"
#include "DNCP/App/DscpSysDefine.h"
#include "DNCP/Lai/LaiRS485Handler.h"
#include "Meter.h"

#define METER_VOlUME_MAX             60                  //最大定量体积，单位为毫升的10倍

#define PUMP_HIGH_SPEED              150                 //高速，单位步/秒
#define PUMP_MEDIUM_SPEED            50                  //中速，单位步/秒
#define PUMP_LOW_SPEED               20                  //低速，单位步/秒

#define PUMP_HIGH_ACC                6000                //高加速，单位步/平方秒
#define PUMP_MEDIUM_ACC              800                 //中加速，单位步/平方秒
#define PUMP_LOW_ACC                 200                 //低加速，单位步/平方秒
//加速度和最大速度的单位转换，步/平方秒和步/秒分别转化为ml/平方秒,ml/秒
#define PUMP_ACC_SPEED(X)            (PumpManager_GetFactor(METER_PUMP_NUM) * X)

#define HARDWARE_RIGHT_MIN_AD        1000                // 判断硬件好坏的下限值
#define HARDWARE_RIGHT_MAX_AD        4095                // 判断硬件好坏的上限值

#define READ_AIR_AD_N                32                  // 读空气AD值个数
#define READ_AIR_AD_FILTER_LOW       10                  // 最小空气AD值数据过滤个数
#define READ_AIR_AD_FILTER_HIGH      10                  // 最大空气AD值数据过滤个数
#define READ_CUR_AD_N                1                   // 读当前AD值个数
#define LED_ON_DELAY_MS              2000                // LED打开后读AD值得延时
#define METER_MAX_OVER_VALVE         150
#define METER_DEF_OVER_VALVE         100

//积分定量参数定义
#define INTERGRAL_INTERVAL           200                 // 积分区间长度
#define DIFFERENTIAL_INTERVAL        5                   // 一次微分包含的AD值个数
#define DIFFERENTIAL_NUM             40                  // (INTERGRAL_INTERVAL/DIFFERENTIAL_INTERVAL)   规定的微分执行次数
/**
 * @brief 定量操作结果。
 */
typedef enum
{
    METER_RESULT_FINISHED = 0,                           //定量正常完成。
    METER_RESULT_FAILED = 1,                             //定量中途出现故障，未能完成。
    METER_RESULT_STOPPED = 2,                            //定量被停止。
    METER_RESULT_OVERFLOW = 3,                           //定量溢出。
    METER_RESULT_UNFINISHED = 4,                         //定量目标未达成。
    METER_RESULT_AIRBUBBLE = 5                           //定量遇到气泡
} MeterResult;

//光学检测模块状态
typedef enum
{
    OPTICAL_STATUS_AD_AIR_STABLE,                        // 空气的AD值稳定，用于抽取
    OPTICAL_STATUS_AD_LIQUID_STABLE,                     // 液体的AD值稳定，用于排空
    OPTICAL_STATUS_AD_INCREASE,                          // AD值递增，用于排空
    OPTICAL_STATUS_AD_DECREASE,                          // AD值递减，用于抽取
    OPTICAL_STATUS_AD_COMPLETE,                          // 完成
    OPTICAL_STATUS_AD_WRONG,                             // 出错，出现气泡
    MAX_OPTICAL_STATUS
} enum_OpticalState;

static enum_OpticalState s_opticalState;                 //检查定量点状态机的状态

static MeterStatus s_status;                             //定量状态
static Bool s_isStatusSwitchStart;                       //定量切换标志量
static MeterMode s_mode;                                 //定量模式
static MeterDir s_dir;                                   //定量方向，向上：从定量点下方定量到定量点或者上方。
static Bool s_isWaitPumpStop;                            //等待电机停止标志量
static Bool s_isSendEvent;                               //是否发送定量事件，只有在接收到DNCP的启动定量的命令才为true，定量结束后和其他时候为false
static MeterResult s_result;                             //定量结果

static float s_smartPumpVolume;                          //智能定量需要抽取体积
//系统定量点体积，保存的有效的设置定量点体积是必须从小到大的，设置定量点体积为0表示无效，
//例如 定量点0 设置体积 1 真实体积 0.8 ； 定量点1 设置体积 0 真实体积 0； 定量点2 设置体积 2 真实体积 2.5
static MeterPoint s_meterPoint;
static Uint8 s_minMeterPoint;                            //系统最小定量点
static Uint8 s_maxMeterPoint;                            //系统最大定量点

static Uint8 s_setMeterPoint;                            //设置定量点号
static Bool s_excessJudgeOpen;                           //过量判断打开标志
static Uint8 s_excessMeterPoint;                         //过量定量点号

static float s_lastStatusVolume;                         //总共抽取的液体体积
static float s_limitVolume;                              //限制体积
static float s_drainLimitVolume   =  2.0;            //下压限制体积
static Bool s_limitJudgeOpen;                            //限制体积是否打开

static Uint16 s_setMeterAirAD;                           //定量点空气AD值
static Uint16 s_excessMeterAirAD;                        //过量定量点空气AD值
static Uint16 s_MeterAirAD[METER_POINT_NUM] =            //各个定量点空气的AD值，用于向下定量
{ 4095, 4095};

static Bool s_isStartCheackMeter;                        //开启检查是否到达定量点
static float s_airBubbleRatio;                           //空气AD值与积分平均值 有气泡的比率
static Bool s_isAutoCloseValve = TRUE;                   //是否每次定量结束自动关阀

static float s_setPumpSpeed;                             //设定泵的定量速度，此速度只能用于第一次抽取
static float s_pumpExtractSpeed;                         //定量时泵第一次抽取和智能抽取的速度，单位步/秒 //初次定量速度
static float s_pumpDrainSpeed;                           //定量时泵排空的速度，单位步/秒  //过量反压速度
//各个状态机启动的最大速度和加速度
#define FIRST_PUMPEXTRACT_ACC        PUMP_HIGH_ACC       //第一抽取加速度
#define FIRST_PUMPEXTRACT_MAX_SPEED  s_pumpExtractSpeed  //第一抽取最大速度，此参数由Meter_SetPumpParam设置

#define TWO_PUMPEXTRACT_ACC          PUMP_MEDIUM_ACC     //第二抽取加速度
#define TWO_PUMPEXTRACT_MAX_SPEED    PUMP_LOW_SPEED      //第二抽取最大速度

#define DRAIN_ACC                    PUMP_MEDIUM_ACC     //往下排空的加速度
#define DRAIN_MAX_SPEED              s_pumpDrainSpeed    //往下排空的最大速度，此参数由Meter_SetPumpParam设置

#define SMARTEXTRACT_ACC             PUMP_MEDIUM_ACC     //智能抽取的加速度
#define SMARTEXTRACT_MAX_SPEED       s_pumpExtractSpeed  //智能抽取的最大速度，此参数由Meter_SetPumpParam设置

static Uint32 s_setStandardCnt;                          //用于计数，是否到达标准值，
static Bool s_isDrianOverSwong;                          //向下排空是否过冲，过冲则采用微分方式,保证液体在定量点之下，与检查气泡s_isDrianCheckAirBubble互斥
static Bool s_isExtractOverSwong;                        //向上抽取是否过冲，过冲则采用微分方式,保证液体在定量点之上  向上定量为TRUE
static Bool s_isDrianCheckAirBubble;                     //向下排空是否检气泡，检则采用积分方式。与过冲s_isDrianOverSwong互斥   向上定量为TRUE

static Uint32 s_meterFinishValveMap = 0;
static Uint16 s_ropinessOverValue = 400;        //粘稠模式过冲参数
static Uint16 s_accurateOverValue = 100;        //精确模式过冲参数

static float s_stableDiff = 100.0;

static Uint16 s_meterThreshold = 5;                 //精确定量结果判定门限

static Uint16 s_meterInterval = 10;                     //精确定量AD抽值间隔数目

static void Meter_Handler(void);
static void Meter_CheackMeterHandle(void);
static void Meter_MeterADPeriodHandle(void);
static void Meter_FindMeterVolume(MeterPoint *points);
static void Meter_SetPumpParam(float extraceSpeed, float drainSpeed);

void Meter_Init(void)
{
    Uint8 buffer[METER_FACTORY_SIGN_FLASH_LEN] =
    { 0 };
    Uint8 meterBuffer[METER_OVER_VALVE_FLASH_LEN] =
    { 0 };

    Uint32 flashFactorySign = 0;
    MeterLED_Init();
    MerterADC_Init();
    MeterADC_Start();

    s_status = METER_STATUS_IDLE;
    s_isStatusSwitchStart = FALSE;

    McuFlash_Read(METER_FACTORY_SIGN_FLASH_BASE_ADDR,
    METER_FACTORY_SIGN_FLASH_LEN, buffer);             //读取出厂标志位
    memcpy(&flashFactorySign, buffer, METER_FACTORY_SIGN_FLASH_LEN);

    if (FLASH_FACTORY_SIGN == flashFactorySign)             //表示已经过出厂设置
    {
        s_meterPoint = Meter_GetMeterPoints();
        if (s_meterPoint.num > 0 && s_meterPoint.num <= METER_POINT_NUM)
        {
            Meter_FindMeterVolume(&s_meterPoint);
        }
        else
        {
            memset(&s_meterPoint, 0, sizeof(MeterPoint));
        }
    }
    else             //未设置,使用默认值，并写入出厂标志
    {
        flashFactorySign = FLASH_FACTORY_SIGN;

        memset(&s_meterPoint, 0, sizeof(MeterPoint));
        s_meterPoint.num = 2;
        s_meterPoint.volume[0][SETVIDX] = 1;
        s_meterPoint.volume[0][REALIDX] = 1;
        s_meterPoint.volume[1][SETVIDX] = 2;
        s_meterPoint.volume[1][REALIDX] = 2;
        Meter_SetMeterPoints(&s_meterPoint);

        memcpy(buffer, &flashFactorySign, METER_FACTORY_SIGN_FLASH_LEN);

        McuFlash_Write(
        METER_FACTORY_SIGN_FLASH_BASE_ADDR,
        METER_FACTORY_SIGN_FLASH_LEN, buffer);
    }

    s_isSendEvent = FALSE;
    s_isDrianOverSwong = FALSE;
    s_isExtractOverSwong = FALSE;
    s_isDrianCheckAirBubble = FALSE;

    MeterScheduler_RegisterTask(Meter_Handler); // 定量过程管控、定量整体状态处理回调函数注册
    MeterScheduler_RegisterTimer(Meter_CheackMeterHandle,
            Meter_MeterADPeriodHandle); //定量点识别、AD值上报定时器回调函数注册
    MeterScheduler_Init();
    s_setPumpSpeed = 150; //刚启动时使用默认的设置速度
    Meter_SetPumpParam(s_setPumpSpeed, PUMP_LOW_SPEED);
}

/**
 * @brief 定量状态恢复初始化
 */
void Meter_Restore(void)
{
    s_isSendEvent = FALSE;//DNCP可能调用此函数，此时不能给上位机上报定量事件
    Meter_RequestStop();
}

/**
 * @brief 由DNCP或者控制台设置定量速度，单位ml/s
 * @param speed定量速度，单位ml/s
 */
void Meter_SetMeterSpeed(float speed)
{
    s_setPumpSpeed = speed / PumpManager_GetFactor(METER_PUMP_NUM);
    TRACE_INFO("Meter set speed: ");
    System_PrintfFloat(TRACE_LEVEL_INFO, speed, 6);
    TRACE_INFO(" ml/s, ");
    System_PrintfFloat(TRACE_LEVEL_INFO, s_setPumpSpeed, 3);
    TRACE_INFO(" step/s");
    if (s_setPumpSpeed < PUMP_LOW_SPEED)
    {
        s_setPumpSpeed = PUMP_LOW_SPEED;
    }
    //定量过程中不能改变速度，定量结束会自动重新设置定量使用的运动参数组
    if (METER_STATUS_IDLE == Meter_GetMeterStatus())
    {
        Meter_SetPumpParam(s_setPumpSpeed, PUMP_LOW_SPEED);
    }
}

/**
 * @brief 由DNCP或者控制台设置定量速度，单位ml/s
 * @return 定量速度，单位ml/s
 */
float Meter_GetMeterSpeed(void)
{
    return (s_setPumpSpeed * PumpManager_GetFactor(METER_PUMP_NUM));
}
/**
 * 设置定量过程使用的运动参数组
 * @param extraceSpeed 抽取的速度，改变FIRST_PUMPEXTRACT_MAX_SPEED(第一次抽取)， SMARTEXTRACT_MAX_SPEED(智能抽取)的速度
 * @param drainSpeed 往下排的速度，改变DRAIN_MAX_SPEED(往下排)的速度
 * @return 改变的值s_pumpExtractSpeed  s_pumpDrainSpeed
 */
static void Meter_SetPumpParam(float extraceSpeed, float drainSpeed)
{
    s_pumpExtractSpeed = extraceSpeed;
    s_pumpDrainSpeed = drainSpeed;
    if (s_pumpExtractSpeed < PUMP_LOW_SPEED)
    {
        s_pumpExtractSpeed = PUMP_LOW_SPEED;
    }
    if (s_pumpDrainSpeed < PUMP_LOW_SPEED)
    {
        s_pumpDrainSpeed = PUMP_LOW_SPEED;
    }
}

/**
 * @brief 获取定量状态
 * @return
 */
MeterStatus Meter_GetMeterStatus(void)
{
    return (s_status);
}

/**
 * @brief 查找最小定量点和最大定量点。
 * @note points里保存的有效定量点体积是必须从小到大的，定量点体积为0表示无效
 * @param points 本系统的定量体系
 * @return 改变的值 s_minMeterPoint s_maxMeterPoint
 */
static void Meter_FindMeterVolume(MeterPoint *points)
{
    for (Uint8 i = 0; i < points->num; i++)
    {
        if (points->volume[i][SETVIDX] > 0)
        {
            s_minMeterPoint = i;
            break;
        }
    }
    for (int i = points->num - 1; i >= 0; i--)
    {
        if (points->volume[i][SETVIDX] > 0)
        {
            s_maxMeterPoint = i;
            break;
        }
    }
    TRACE_MARK("\n minMeterPoint %d maxMeterPoint %d", s_minMeterPoint + 1,
            s_maxMeterPoint + 1);
}

/**
 * @brief 设置系统定量点,无效定量点为0
 * @param dot
 * @return 是否设置成功 改变的值 s_meterPoint s_minMeterPoint s_maxMeterPoint
 */
Bool Meter_SetMeterPoints(MeterPoint *dot)
{
    if (METER_STATUS_IDLE == s_status)
    {
        Uint8 writeData[METER_POINTS_FLASH_LEN] =
        { 0 };
        s_meterPoint = *dot;
        Meter_FindMeterVolume(&s_meterPoint);
        memcpy(writeData, &s_meterPoint, sizeof(MeterPoint));
        McuFlash_Write(
        METER_POINTS_FLASH_BASE_ADDR,
        METER_POINTS_FLASH_LEN, writeData);
        TRACE_MARK("\n num :%d ", s_meterPoint.num);
        for (Uint8 i = 0; i < s_meterPoint.num; i++)
        {
            TRACE_MARK("\n %d point ", i + 1);
            TRACE_MARK("SetV:");
            System_PrintfFloat(TRACE_LEVEL_MARK,
                    s_meterPoint.volume[i][SETVIDX], 2);
            TRACE_MARK(" ml RealV:");
            System_PrintfFloat(TRACE_LEVEL_MARK,
                    s_meterPoint.volume[i][REALIDX], 2);
            TRACE_MARK(" ml");
        }
        return TRUE;
    }
    else
    {
        TRACE_ERROR(
                "\n Set the volume of the meter point to fail because the meter is running");
    }
    return FALSE;
}

/**
 * @brief 获取系统定量点体积设置
 * @return
 */
MeterPoint Meter_GetMeterPoints(void)
{
    Uint8 readData[METER_POINTS_FLASH_LEN] =
    { 0 };
    MeterPoint dot;
    McuFlash_Read(METER_POINTS_FLASH_BASE_ADDR, METER_POINTS_FLASH_LEN,
            readData);
    memcpy(&dot, readData, sizeof(MeterPoint));
    return (dot);
}

/**
 * @brief 切换定量状态
 * @param status
 */
static void Meter_StatusSwitch(MeterStatus status)
{
    // 状态切换
    s_status = status;
    s_isStatusSwitchStart = TRUE;
}

/**
 * @note 此函数只能由Meter_StartMeter调用。非智能定量模式，寻找出等于要定量体积的某个定量点体积。
 *      智能模式的话，则寻找出定量体积靠近某个定量点的，并且比这个定量点体积大。还有一个特殊情况，
 *      就是定量体积比最小定量点小，这会使用最小定量点定量，和比最小定量点大一个点的定量点做过冲定量点。
 * @param mode 定量模式
 * @param volume 定量体积
 * @return true 定量体积解析成功， false 失败 。 改变的量 s_setMeterPoint  s_excessMeterPoint s_excessJudgeOpen s_smartPumpVolume
 */
static Bool Meter_ConfirmMeterPoint(MeterMode mode, float volume)
{
    //如果定量体积比设置最大定量点的体积大或者等于
    if (volume >= s_meterPoint.volume[s_maxMeterPoint][SETVIDX])
    {
        s_setMeterPoint = s_maxMeterPoint;
        s_excessMeterPoint = 0;
        s_excessJudgeOpen = FALSE;
    }
    else //如果定量体积比设置最大定量点的体积小
    {
        float tempVolume[METER_POINT_NUM];
        Uint8 tempPoints[METER_POINT_NUM];
        Uint8 i, j, index = 1;
        s_excessJudgeOpen = TRUE;
        //把定量点体积有效数据(不等于0)按原顺序存放在tempVolume数组
        for (i = 0, j = 0; i < s_meterPoint.num; i++)
        {
            if (s_meterPoint.volume[i][SETVIDX] > 0)
            {
                tempVolume[j] = s_meterPoint.volume[i][SETVIDX];
                tempPoints[j] = i;
                j++;
            }
        }

        if (volume < tempVolume[0]) //判断定量体积是否比最小定量点小，此定量体积只能用于智能模式
        {
            s_setMeterPoint = tempPoints[0];
            s_excessMeterPoint = tempPoints[1];
        }
        else
        {
            //寻找过量定量点，从次小的定量体积开始判断，当要定量的volume小于时，此体积便是过量体积
            while (1)
            {
                if (volume < tempVolume[index])
                {
                    s_setMeterPoint = tempPoints[index - 1];
                    s_excessMeterPoint = tempPoints[index];
                    break;
                }
                else
                {
                    index++;
                    if (index >= j)
                    {
                        return FALSE;
                    }
                }
            }
        }
    }
    s_smartPumpVolume = 0;
    // s_accuratePumpVolume = 0;

    if (METER_MODE_SMART == mode)
    {
        s_smartPumpVolume = volume
                - s_meterPoint.volume[s_setMeterPoint][SETVIDX];
    }
    else if (volume != s_meterPoint.volume[s_setMeterPoint][SETVIDX]) //其他模式要定量的体积必须等于定量点体积，不然启动定量失败
    {
        return FALSE;
    }
    TRACE_INFO("\n setMeterPoint %d excessMeterPoint %d smartPumpVolume",
            s_setMeterPoint + 1, s_excessMeterPoint + 1);
    System_PrintfFloat(TRACE_LEVEL_INFO, s_smartPumpVolume, 2);
    TRACE_INFO(" ml");
    return TRUE;
}
/**
 * @brief 开始定量。
 * @param dir 定量方向，0为正向转动向上定量，1为向下定量。
 * @param mode 定量模式
 * @param volume 定量体积，单位为 ml。
 * @param limitVolume 限量体积，在定量过程中总的泵取体积不能超过该值，0表示不进行限量判定。单位为 ml。
 * @return 状态回应 TRUE 操作成功; FALSE 操作失败(参数错误)
 */
Uint16 Meter_Start(MeterDir dir, MeterMode mode, float volume,
        float limitVolume)
{
    //以下情况启动失败：
    //方向 和 模式不正确，或者排空定量采用不是快速和慢速
    //系统定量点体系小于1或者大于最大定量点数目
    //定量体积大于 最大定量体积 或者 在非智能模式下小于最小定量体积
    //如果有限制体积，限制体积小于等于需要定量体积
    if (METER_STATUS_IDLE == s_status)
    {
        if (dir < 2 && mode < METER_MODE_MAX && s_meterPoint.num > 0
                && s_meterPoint.num <= METER_POINT_NUM
                && volume < (METER_VOlUME_MAX / 10.0))
        {
            if (volume < s_meterPoint.volume[s_minMeterPoint][SETVIDX]
                    && mode != METER_MODE_SMART)
            {
                TRACE_ERROR(
                        "\n Meter startup failed because the meter volume  less than min Meter volume and  mode %d is not METER_MODE_SMART",
                        mode);
                return ((Uint16) DSCP_ERROR_PARAM);
            }

            if (limitVolume != 0)    //需要限量判定
            {
                if (limitVolume <= volume)
                {
                    TRACE_ERROR(
                            "\n Meter startup failed because the limit volume error. ");
                    return ((Uint16) DSCP_ERROR_PARAM);
                }
                else    //配置限制功能
                {
                    s_limitJudgeOpen = TRUE;
                    s_limitVolume = limitVolume;
                    s_lastStatusVolume = 0;//用于限制体积判断，清零保存的上一个状态结束后抽取的体积
                }
            }
            else
            {
                s_limitJudgeOpen = FALSE;
                s_limitVolume = 0;
                s_lastStatusVolume = 0;//用于限制体积判断，清零保存上一个状态结束后抽取的体积
            }

            s_mode = mode;
            s_dir = dir;

            if (FALSE == Meter_ConfirmMeterPoint(s_mode, volume))//解析定量体积，根据模式找到定量点号，过量定量点号和泵取的智能体积
            {
                TRACE_ERROR(
                        "\n Meter startup failed because the meter volume error. ");
                return ((Uint16) DSCP_ERROR_PARAM);
            }

//            s_airBubbleRatio = 0.3;

            if (METER_DIR_DRAIN == s_dir)    //向下排空定量常用于小体系模式
            {
                //用PUMP_LOW_SPEED 的速度参数，降低第一次抽取，向下排空，智能抽取的速度
                //避免向下排空把液体排光，或者第一次抽取的准确性不高
                Meter_SetPumpParam(s_setPumpSpeed, PUMP_LOW_SPEED);
                s_excessJudgeOpen = FALSE; //关闭过冲判断。排空定量不支持
                s_limitJudgeOpen = FALSE;  //关闭限制体积功能。排空定量不支持
                s_limitVolume = 0;
                MeterLED_TurnOn(s_setMeterPoint + 1);
                s_isDrianOverSwong = TRUE;
                s_isDrianCheckAirBubble = FALSE;
                if(s_mode == METER_MODE_SMART)
                {
                    s_isExtractOverSwong = TRUE;
                }
                else
                {
                    s_isExtractOverSwong = FALSE;
                }
            }
            else                //向上抽取定量
            {
                s_isDrianCheckAirBubble = TRUE;
                s_isDrianOverSwong = FALSE;
                if(s_mode == METER_MODE_DIRECT)
                {
                    s_isExtractOverSwong = FALSE;
                }
				else
				{
					s_isExtractOverSwong = TRUE;
				}
                for (int i = 0; i < METER_POINT_NUM; i++)    //打开所有的灯，为了保存空气的AD值
                {
                    MeterLED_TurnOn(i + 1);
                }
            }
            DncpStack_ClearBufferedEvent();

            MerterADC_SetBufferSize(READ_AIR_AD_N);
            MeterADC_Start();

            TRACE_INFO("\n Meter_Start");
            TRACE_INFO("\n dir:%d MeterMode: %d", dir, mode);
            TRACE_INFO(" volume:");
            System_PrintfFloat(TRACE_LEVEL_INFO, volume, 2);
            TRACE_INFO("ml limitVolume:");
            System_PrintfFloat(TRACE_LEVEL_INFO, limitVolume, 2);
            TRACE_INFO("ml");
            TRACE_INFO("\n To START");
            Meter_StatusSwitch(METER_STATUS_START);
            MeterScheduler_MeterResume();
            return DSCP_OK;
        }
        else
        {
            TRACE_ERROR(
                    "\n Meter startup failed because the parameter error. ");
            return ((Uint16) DSCP_ERROR_PARAM);
        }
    }
    else
    {
        TRACE_ERROR("\n Meter startup failed because the meter is runing. ");
    }
    return DSCP_ERROR;
}

/**
 * @brief 设置定量泵运动参数和启动定量泵函数
 * @param acceleration
 * @param maxSpeed
 * @param dir
 * @param volume
 * @return
 */
static Uint16 Meter_PumpStart(float acceleration, float maxSpeed, Direction dir,
        float volume)
{
    PumpManager_SetMotionParam(METER_PUMP_NUM, acceleration, maxSpeed,
            NoWriteFLASH);
    return (PumpManager_Start(METER_PUMP_NUM, dir, volume, NoReadFLASH));
}

/**
 * @brief 请求停止定量泵
 * @return
 */
static void Meter_RequestPumpStop(void)
{
    s_isWaitPumpStop = TRUE;
    PumpManager_Stop(METER_PUMP_NUM);
}

/**
 * @brief 请求停止定量
 * @return
 */
Bool Meter_RequestStop(void)
{
    if (METER_STATUS_IDLE != s_status)
    {
        TRACE_INFO("\n Meter request stop");
        s_result = METER_RESULT_STOPPED;
        Meter_RequestPumpStop();
        return TRUE;
    }
    else
    {
        TRACE_ERROR(
                "\n Because the meter is not running, the request to stop failure. ");
    }
    return FALSE;
}
/**
 * @brief 停止定量
 * @note
 */
static void Meter_Stop(MeterResult result)
{
    if (TRUE == s_isAutoCloseValve)
    {
        ValveManager_SetValvesMap(s_meterFinishValveMap);
    }

    if (METER_DIR_DRAIN == s_dir)
    {
        MeterLED_TurnOff(s_setMeterPoint + 1);
    }
    else
    {
        for (int i = 0; i < METER_POINT_NUM; i++)
        {
            MeterLED_TurnOff(i + 1);
        }
    }

    s_status = METER_STATUS_IDLE;
    s_isStatusSwitchStart = FALSE;

    TRACE_INFO("\n Meter stop");
    if (TRUE == s_isSendEvent)
    {
        DncpStack_SendEvent(DSCP_EVENT_OMI_METER_RESULT, &result,
                sizeof(MeterResult));
        DncpStack_BufferEvent(DSCP_EVENT_OMI_METER_RESULT, &result,
                sizeof(MeterResult));
    }
    s_isSendEvent = FALSE;
    Meter_SetPumpParam(s_setPumpSpeed, PUMP_LOW_SPEED);
}
/**
 * @brief 打开定量停止发送事件功能
 */
void Meter_SendMeterEventOpen(void)
{
    s_isSendEvent = TRUE;
}

/**
 * @brief 定量错误判断，通过过量定量点的AD值和限量体积来判断
 */
static void Meter_ErrorJudge(void)
{
    if ((METER_STATUS_DRAIN != s_status) && FALSE == s_isWaitPumpStop
            && PUMP_IDLE != PumpManager_GetStatus(METER_PUMP_NUM))//必须判断泵启动了，不然获取的体积可能会是上一次泵取的体积
    {
        float curVolume = s_lastStatusVolume
                + PumpManager_GetVolume(METER_PUMP_NUM);
        Uint16 excessMeterAD = MeterADC_GetCurAD(s_excessMeterPoint);

        if (TRUE == s_excessJudgeOpen)
        {
            static Uint16 s_curSum = 0, s_sumCnt = 0;
            static float differential = 0.0;
            float curADAverage;
            static float s_lastADAverage = 0;
            static Uint16 s_standardCnt = 0;

            if (excessMeterAD < (s_excessMeterAirAD * 0.5))    //AD值正常，可以进行积分
            {
                if (s_sumCnt >= 5)    //合并5个值求一次微分，加大定量速度慢时微分幅度
                {
                    curADAverage = s_curSum / 5.0;    //求平均
                    differential = curADAverage - s_lastADAverage;    //求微分
                    differential = fabs(differential);
                    if (s_lastADAverage != 0)
                    {
                        TRACE_ERROR("e differential:"); //打印比率
                        System_PrintfFloat(TRACE_LEVEL_ERROR,
                                   differential, 3);
                    }
                    s_lastADAverage = curADAverage;
                    s_curSum = 0;
                    s_sumCnt = 0;

                    if (differential < s_stableDiff) //微分处于这个范围表示当前AD值稳定 //15.0
                    {
                        if (++s_standardCnt > 20)
                        {
                            s_standardCnt = 0;
                            s_curSum = 0;
                            s_sumCnt = 0;
                            s_result = METER_RESULT_OVERFLOW;
                            Meter_RequestPumpStop();
                            TRACE_ERROR("\n Meter excess Stop pump");
                            TRACE_ERROR("\n excessMeterAirAD = %d excessMeterAD = %d ",
                                    s_excessMeterAirAD, excessMeterAD);
                            System_PrintfFloat(TRACE_LEVEL_ERROR,
                                    excessMeterAD * 1.0 / s_excessMeterAirAD * 100, 3);
                            TRACE_ERROR("%%");

                            s_excessJudgeOpen = FALSE;//避免第二次进来
                            s_limitJudgeOpen = FALSE;
                        }
                    }
                    else
                    {
                        s_standardCnt = 0;
                    }
                }
                else
                {
                    s_curSum += excessMeterAD;
                    s_sumCnt++;
                }
            }
            else
            {
                s_standardCnt = 0;
                s_curSum = 0;
                s_sumCnt = 0;
                s_lastADAverage = excessMeterAD;
            }
        }
        if (TRUE == s_limitJudgeOpen && curVolume >= s_limitVolume)
        {
            s_result = METER_RESULT_UNFINISHED;
            Meter_RequestPumpStop();

            TRACE_ERROR("\n Meter limit Stop pump");
            TRACE_ERROR("\n limitVolume =");
            System_PrintfFloat(TRACE_LEVEL_ERROR, s_limitVolume, 2);
            TRACE_ERROR(" ml curVolume =");
            System_PrintfFloat(TRACE_LEVEL_ERROR, curVolume, 2);
            TRACE_ERROR(" ml");
            s_excessJudgeOpen = FALSE;
            s_limitJudgeOpen = FALSE;
        }
    }

    //向下压的体积限制
    if ((METER_STATUS_DRAIN == s_status) && (METER_DIR_EXTRACT == s_dir) && FALSE == s_isWaitPumpStop
            && PUMP_IDLE != PumpManager_GetStatus(METER_PUMP_NUM))//必须判断泵启动了，不然获取的体积可能会是上一次泵取的体积
    {
        float drainVolume = PumpManager_GetVolume(METER_PUMP_NUM);

        if (TRUE == s_limitJudgeOpen && drainVolume >= s_drainLimitVolume)
        {
            s_result = METER_RESULT_FAILED;
            Meter_RequestPumpStop();

            TRACE_ERROR("\n Meter drain vol out limit Stop pump");
            TRACE_ERROR("\n drainLimitVolume =");
            System_PrintfFloat(TRACE_LEVEL_ERROR, s_drainLimitVolume, 2);
            TRACE_ERROR(" ml drainVolume =");
            System_PrintfFloat(TRACE_LEVEL_ERROR, drainVolume, 2);
            TRACE_ERROR(" ml");
            s_excessJudgeOpen = FALSE;
            s_limitJudgeOpen = FALSE;
        }
    }

    if (FALSE == s_isWaitPumpStop
            && PUMP_IDLE == PumpManager_GetStatus(METER_PUMP_NUM))
    {
        s_isWaitPumpStop = TRUE;
        s_result = METER_RESULT_UNFINISHED;
    }
}

/**
 * @brief 定量点识别处理函数
 */
static void Meter_CheackMeterHandle(void)
{
    static Uint16 s_standardCnt = 0;
    // static Uint16 s_ADNumber;
    // static Int32 s_sumADIntegral;
    //float s_AverageInterg;  //积分均值
    //float s_ratio;        //记录空气AD值与积分平均值的比率
    static Uint16 s_curSum, s_sumCnt;
    static float differential = 65535;
    // static float s_differential[DIFFERENTIAL_NUM];   // 存放微分结果的数组
    static Uint16 s_differentialCnt; // 记录微分执行的次数
    float curADAverage;
    static float s_lastADAverage;
    // static Uint16 s_ADErrorCnt = 0;
    // TRACE_FATAL("\n # 1 # \n");  // 用来寻找上抽再下压定量方式的程序执行路径
    if (TRUE == s_isStartCheackMeter)
    {
        Uint16 curAD = MeterADC_GetCurAD(s_setMeterPoint);
        TRACE_CODE("\n A: %d ", curAD);

        switch (s_opticalState)
        {
        //定量抽取过程===============================================================================================
        case OPTICAL_STATUS_AD_AIR_STABLE:
            if (TRUE == s_isExtractOverSwong)  //抽取液体过冲的话需要提前减速，尽量使不同的定量速度过冲体积一样
            {
                if (curAD < (s_setMeterAirAD * 0.8))
                {
                    s_opticalState = OPTICAL_STATUS_AD_DECREASE;
                    s_curSum = 0;
                    s_sumCnt = 0;
                    s_lastADAverage = curAD;
                    // s_ADErrorCnt = 0;
                    TRACE_CODE("\n # 1 # \n");
                }

            }
            else
            {
                if (curAD < (s_setMeterAirAD * 0.6))
                {
                    s_opticalState = OPTICAL_STATUS_AD_DECREASE;
                    TRACE_CODE("\n # 2 # \n");
                }

            }
            break;
        case OPTICAL_STATUS_AD_DECREASE:
            // 上抽平台检测 ：1 AD值判断； 2 求微分；3 微分结果判断； 4 期望的结果次数累加； 5 是否触发状态转换判断。
            // 注：不同模式下s_setStandardCnt（决定定量什么时候完成/向上过冲的大小）的值不一样：e 150; d 25; s 50.
            if (TRUE == s_isExtractOverSwong)
            {
                if (curAD < (s_setMeterAirAD * 0.3) || curAD < 500) // * 1 AD值判断；  //低于空气的0.3或者小于500就认为有液体了
                {
                    if(s_sumCnt < 5)
                    {
                        s_curSum += curAD;
                        s_sumCnt++;
                    }
                    else
                    {
                        // * 2 求微分；
                        curADAverage = s_curSum / 5.0;
                        differential = curADAverage - s_lastADAverage;
                        differential = fabs(differential);
                        s_lastADAverage = curADAverage;
                        if (s_lastADAverage != 0)
                        {
                            TRACE_CODE("\n d:");
                               System_PrintfFloat(TRACE_LEVEL_CODE,
                                       differential, 1);
                        }
                        if(differential < s_stableDiff) // * 3 微分结果判断； //15.0
                        {
                            s_differentialCnt++;   // * 4 期望的结果次数累加
//                            TRACE_CODE("\n C: %d ", s_differentialCnt);
                            if (s_differentialCnt >= s_setStandardCnt)   // * 5 触发状态转换判断  这个判断影响过冲量
                            {
                                s_isExtractOverSwong = FALSE;
                                s_opticalState = OPTICAL_STATUS_AD_COMPLETE;
//                                TRACE_INFO("\n Extract Over Swong Finish");
                                s_differentialCnt = 0;
                                TRACE_CODE("\n # 3 # \n");
                            }else{ ;}
                        }
                        else
                        {
                            s_differentialCnt = 0;
                        }
                        s_curSum = 0;
                        s_sumCnt = 0;
                    }
                }
                else
                {
                    s_curSum = 0;
                    s_sumCnt = 0;
                    s_differentialCnt = 0;
                }
            }
            else
            {
                if (curAD < (s_setMeterAirAD * 0.4)) //小于值则表示液体曾经过定量点，计数一定值然后停止，定量体积相对准，但有风险
                {
                    s_standardCnt++;
                    if (s_standardCnt >= 25)
                    {
                        s_opticalState = OPTICAL_STATUS_AD_COMPLETE;
                        TRACE_INFO("\n Extract Finish");
                        TRACE_CODE("\n # 4 # \n");
                    }
                }
                else
                {
                    s_standardCnt = 0;
                }
            }
            break;

            //定量排空过程===============================================================================================
        case OPTICAL_STATUS_AD_LIQUID_STABLE:
            if (TRUE == s_isDrianOverSwong)
            {
                if (curAD > (s_setMeterAirAD * 0.4))
                {
                    s_opticalState = OPTICAL_STATUS_AD_INCREASE;
                    s_lastADAverage = curAD;
                    s_curSum = 0;
                    s_sumCnt = 0;
                    TRACE_CODE("\n # 5 # \n");
                }
            }
            else    // 精确定量 方向为e时执行
            {
                // 微分定量
                s_sumCnt = 0;
                s_opticalState = OPTICAL_STATUS_AD_INCREASE; // 直接跳转至AD值增加状态
                TRACE_FATAL("\n # 6 # \n");
                if (curAD > (s_setMeterAirAD * 0.5))
                {
                    s_opticalState = OPTICAL_STATUS_AD_INCREASE;
                    // s_ADNumber = 0;
                    // s_sumADIntegral = 0;
                }
            }
            break;

        case OPTICAL_STATUS_AD_INCREASE:
            if (TRUE == s_isDrianOverSwong)
            {
                if (curAD > (s_setMeterAirAD * 0.4))
                {
                    if (s_sumCnt >= 10)
                    {
                        curADAverage = s_curSum / 10.0; //求平均
                        differential = curADAverage - s_lastADAverage;
                        if (s_lastADAverage != 0)
                        {
                            TRACE_DEBUG("differential:"); //打印比率
                               System_PrintfFloat(TRACE_LEVEL_DEBUG,
                                       differential, 3);
                        }
                        s_lastADAverage = curADAverage;
                        s_curSum = 0;
                        s_sumCnt = 0;
                    }
                    else
                    {
                        s_curSum += curAD;
                        s_sumCnt++;
                    }
                    if (differential < 2 && differential > -1)
                    {
                        s_standardCnt++;
                        if (s_standardCnt >= 25)
                        {
                            TRACE_CODE("\n # 7 # \n");
                            s_isDrianOverSwong = FALSE;
                            s_opticalState = OPTICAL_STATUS_AD_COMPLETE;
                            TRACE_INFO("\n Drian Over Swong Finish");
                            TRACE_DEBUG("\n curAD: -6000, ");
                            TRACE_DEBUG("d differential: -6000,");
                        }
                    }
                }
                else
                {
                    s_standardCnt = 0;
                    s_curSum = 0;
                    s_sumCnt = 0;
                    s_lastADAverage = curAD;
                }
            }
            else //向上定量（抽1 压1）时，下压执行，精确定量最后一个动作节点
            {
                if (TRUE == s_isDrianCheckAirBubble)
                {
                        // 微分定量  1 计算微分； 2 判断微分； 3 累计判断结果；4 触发定量完成。
                        // 1 计算微分；
                        if (s_sumCnt >=(s_meterInterval - 1))
                        {
                            curADAverage = s_curSum / (float)s_meterInterval; //求平均
                            if(0 != s_lastADAverage)
                            {
                                differential = curADAverage - s_lastADAverage;

                                //  2 判断微分；
                                if(differential > 1)
                                {
                                    // 3 累计判断结果；
                                    s_differentialCnt++;

                                    // 4 触发定量完成。
                                    if(s_meterThreshold <= s_differentialCnt)
                                    {
                                        if(s_mode == METER_MODE_LAYERED) //分层定量模式需要AD值高于0.2倍空气AD才判定成功
                                        {
                                            if(curADAverage > 0.2 * s_setMeterAirAD)
                                            {
                                                s_differentialCnt = 0;
                                                s_lastADAverage = 0;
                                                s_curSum = 0;
                                                s_sumCnt = 0;
                                                s_opticalState = OPTICAL_STATUS_AD_COMPLETE;    // 仅返回定量成功，
                                                TRACE_CODE("\n # 9 # \n");
                                            }
                                        }
                                        else
                                        {
                                            s_differentialCnt = 0;
                                            s_lastADAverage = 0;
                                            s_curSum = 0;
                                            s_sumCnt = 0;
                                            s_opticalState = OPTICAL_STATUS_AD_COMPLETE;    // 仅返回定量成功，
                                            TRACE_CODE("\n # 8 # \n");
                                        }
                                    }
                                }
                                else
                                {
                                    s_differentialCnt = 0;
                                }
                            }
//                            if (s_lastADAverage != 0)
//                            {
//                                TRACE_FATAL("\n D:"); // 打印微分值
//                                   System_PrintfFloat(TRACE_LEVEL_FATAL,
//                                           differential, 0);
//                            }else{ ;}
                            s_lastADAverage = curADAverage;
                            s_curSum = 0;
                            s_sumCnt = 0;
                        }
                        else
                        {
                            s_curSum += curAD;
                            s_sumCnt++;
                        }
                }
            }
            break;

            //定量结束处理===============================================================================================
        case OPTICAL_STATUS_AD_COMPLETE:

            TRACE_INFO("\n s_standardCnt %d", s_standardCnt);
            TRACE_INFO("\n curAD = %d ", curAD);
            System_PrintfFloat(TRACE_LEVEL_INFO,
                    curAD * 1.0 / s_setMeterAirAD * 100, 3);
            TRACE_INFO("%%");
            TRACE_INFO("\n s_airBubbleRatio ="); //打印比率
            System_PrintfFloat(TRACE_LEVEL_INFO, s_airBubbleRatio, 3);
            s_standardCnt = 0;
            s_isStartCheackMeter = FALSE;
            MeterScheduler_StopMeterCheckTimer();
            break;

        case OPTICAL_STATUS_AD_WRONG:

            TRACE_ERROR("\n curAD = %d ", curAD);
            System_PrintfFloat(TRACE_LEVEL_ERROR,
                    curAD * 1.0 / s_setMeterAirAD * 100, 3);
            TRACE_ERROR("%%");
            TRACE_ERROR("\n s_airBubbleRatio ="); //打印比率
            System_PrintfFloat(TRACE_LEVEL_ERROR, s_airBubbleRatio, 3);

            s_standardCnt = 0;
            s_isStartCheackMeter = FALSE;
            MeterScheduler_StopMeterCheckTimer();
            break;

        default:
            break;
        }
    }
    else
    {
        MeterScheduler_StopMeterCheckTimer();
    }
}




static void Meter_Handler(void)
{
    static Bool s_isSpeedDown;
    vTaskDelay(2 / portTICK_RATE_MS);

    switch (s_status)
    {
    case METER_STATUS_IDLE:
        MeterScheduler_MeterSuspend();          //切换到空闲状态才挂起本任务，那么下次恢复会在这里恢复。
        break;

    case METER_STATUS_START:
        if (TRUE == s_isStatusSwitchStart)
        {
            s_isStatusSwitchStart = FALSE;
            s_isWaitPumpStop = FALSE;

            vTaskDelay(LED_ON_DELAY_MS / portTICK_RATE_MS);       //等待定量AD稳定

            if (METER_DIR_DRAIN == s_dir) //读取之前保存的空气AD值；读取当前AD值，判断是否存在液体，不存在则定量失败
            {
                s_setMeterAirAD = s_MeterAirAD[s_setMeterPoint];
                Uint16 ad = MeterADC_AirADFilter(s_setMeterPoint,
                READ_AIR_AD_FILTER_LOW, READ_AIR_AD_FILTER_HIGH);
                TRACE_INFO("\n curad = %d", ad);
                if (ad > HARDWARE_RIGHT_MIN_AD)
                {
                    TRACE_ERROR(
                            "\n Hardware AD error or Liquid does not exist.");
                    s_result = METER_RESULT_FAILED;
                    Meter_Stop(METER_RESULT_FAILED);
                    break;
                }
            }
            else
            {
                for (int i = 0; i < METER_POINT_NUM; i++)       //读取所有通道的AD值
                {
                    Uint16 ad = MeterADC_AirADFilter(i,
                    READ_AIR_AD_FILTER_LOW, READ_AIR_AD_FILTER_HIGH);
                    if (s_setMeterPoint == i)
                    {
                        s_setMeterAirAD = ad;
                    }
                    if (s_excessMeterPoint == i)
                    {
                        s_excessMeterAirAD = ad;
                    }
                    if (ad >= HARDWARE_RIGHT_MIN_AD
                            && ad <= HARDWARE_RIGHT_MAX_AD) //判断AD值是否在范围内，在则认为是空气
                    {
                        s_MeterAirAD[i] = ad;
                    }
                }

                if (s_setMeterAirAD < HARDWARE_RIGHT_MIN_AD
                        || s_setMeterAirAD > HARDWARE_RIGHT_MAX_AD
                        || (TRUE == s_excessJudgeOpen
                                && (s_excessMeterAirAD < HARDWARE_RIGHT_MIN_AD
                                        || s_excessMeterAirAD
                                                > HARDWARE_RIGHT_MAX_AD)))
                {

                    TRACE_ERROR("\n Hardware AD error");
                    TRACE_ERROR("\n s_setMeterAirAD = %d", s_setMeterAirAD);
                    TRACE_ERROR("\n excessMeterAirAD = %d", s_excessMeterAirAD);
                    s_result = METER_RESULT_FAILED;
                    Meter_Stop(METER_RESULT_FAILED);
                    break;

                }
            }

            TRACE_INFO("\n s_setMeterAirAD = %d ", s_setMeterAirAD);   //打印空气AD值
            MerterADC_SetBufferSize(READ_CUR_AD_N); //设置读取当前AD值的个数，用于快速反应，一般个数会较小
            MeterADC_Start();       //由于设置时需要停止AD，所有设置后启动AD

            if (FALSE == s_isWaitPumpStop)
            {
                if (METER_DIR_EXTRACT == s_dir) //向上抽取的所有定量模式都是从METER_STATUS_FIRST_PUMPEXTRACT开始
                {
//                    s_calibrateStep = 0;
                    TRACE_INFO("\n To FIRST_PUMPEXTRACT");
                    Meter_StatusSwitch(METER_STATUS_FIRST_PUMPEXTRACT);
                }
                else       //向下排空的所有定量模式都是从METER_STATUS_DRAIN开始
                {
                    TRACE_INFO("\n To DRAIN");
                    Meter_StatusSwitch(METER_STATUS_DRAIN);
                }
           }
            else       //定量处于启动状态时被请求停止。
            {
                Meter_Stop(s_result);
            }
        }
        break;

    case METER_STATUS_FIRST_PUMPEXTRACT:
        if (TRUE == s_isStatusSwitchStart)       //设置状态
        {
            s_isStatusSwitchStart = FALSE;

            if (METER_DIR_DRAIN == s_dir) //向下定量的反抽阶段使用反抽速度
            {
                if (DSCP_OK
                        != Meter_PumpStart(PUMP_ACC_SPEED(FIRST_PUMPEXTRACT_ACC),
                                PUMP_ACC_SPEED(DRAIN_MAX_SPEED),
                                COUNTERCLOCKWISE, (METER_VOlUME_MAX / 10.0) * 3))
                {
                    TRACE_ERROR("\n Meter Pump Start error");
                    s_result = METER_RESULT_FAILED;
                    Meter_Stop(METER_RESULT_FAILED);
                    break;
                }
            }
            else
            {
                if (DSCP_OK
                        != Meter_PumpStart(PUMP_ACC_SPEED(FIRST_PUMPEXTRACT_ACC),
                                PUMP_ACC_SPEED(FIRST_PUMPEXTRACT_MAX_SPEED),
                                COUNTERCLOCKWISE, (METER_VOlUME_MAX / 10.0) * 3))
                {
                    TRACE_ERROR("\n Meter Pump Start error");
                    s_result = METER_RESULT_FAILED;
                    Meter_Stop(METER_RESULT_FAILED);
                    break;
                }
            }
            //启动确认定量点的定时器
            s_isStartCheackMeter = TRUE;
            MeterScheduler_StartMeterCheckTimer();

            if(s_isExtractOverSwong == TRUE)
            {
                if(METER_DIR_DRAIN == s_dir)
                {
                     s_setStandardCnt = 25;
                }
                else
                {
                    if(s_mode == METER_MODE_SMART)
                    {
                         s_setStandardCnt = 25;
                    }
                    else if(s_mode == METER_MODE_ACCURATE)
                    {
                         s_setStandardCnt = s_accurateOverValue;
                    }
                    else if(s_mode == METER_MODE_ROPINESS || s_mode == METER_MODE_LAYERED)
                    {
                         s_setStandardCnt = s_ropinessOverValue;
                    }
                }
            }
            s_isSpeedDown = FALSE;
            s_isWaitPumpStop = FALSE;
            s_opticalState = OPTICAL_STATUS_AD_AIR_STABLE;
        }

        if (FALSE == s_isWaitPumpStop)
        {
            if (OPTICAL_STATUS_AD_DECREASE == s_opticalState
            && (FALSE == s_isSpeedDown))   //接近凹液面
            {
//               if(METER_DIR_EXTRACT != s_dir ) // 如果是精确向上定量  第一次抽液不减速
//                {
//                    s_isSpeedDown = TRUE;
//                    PumpManager_SetMotionParam(METER_PUMP_NUM,
//                            PUMP_ACC_SPEED(PUMP_HIGH_ACC),
//                            PUMP_ACC_SPEED(PUMP_LOW_SPEED), NoWriteFLASH);
//                }
            }
            else if (OPTICAL_STATUS_AD_AIR_STABLE == s_opticalState
            && (TRUE == s_isSpeedDown))   //接近凹液面，减速,精确定量不减速
            {
                s_isSpeedDown = FALSE;
                PumpManager_SetMotionParam(METER_PUMP_NUM,
                        PUMP_ACC_SPEED(FIRST_PUMPEXTRACT_ACC),
                        PUMP_ACC_SPEED(FIRST_PUMPEXTRACT_MAX_SPEED), NoWriteFLASH);
            }
            else if (OPTICAL_STATUS_AD_COMPLETE == s_opticalState)
            {
                if (METER_MODE_SMART == s_mode) //向上或向下智能模式的第一次抽取完成(忽视快速定量的过冲液体部分)
                {
                    TRACE_INFO("\n PUMPEXTRACT To SMART");
                    Meter_StatusSwitch(METER_STATUS_SMART); //直接切换到智能状态，而不停泵

                    TRACE_DEBUG("\n s_smartPumpVolume:");
                    System_PrintfFloat(TRACE_LEVEL_DEBUG, s_smartPumpVolume, 4);
                    TRACE_DEBUG(" ml");
                    //由于没有停泵，泵取的体积不会清零，不需计算此状态已泵取的体积
                }
                else
                {
//                     s_accuratePumpVolume += PumpManager_GetVolume(METER_PUMP_NUM); //获取已经泵取得体积再加上还需要泵取得体积。
//                     PumpManager_ChangeVolume(METER_PUMP_NUM, s_accuratePumpVolume + 0.04); // 由于浓硫酸过冲太少，精确定量第一次上抽完成后再向上抽0.2mL
                     Meter_RequestPumpStop();
                }
            }
        }
        //请求泵停止的情况：
        //液体达到过量定量点，泵取体积到达限量体积，向上定量精准模式第一次抽取完成，向下定量精准模式完成
        else if (PUMP_IDLE == PumpManager_GetStatus(METER_PUMP_NUM))
        {
            if (OPTICAL_STATUS_AD_COMPLETE == s_opticalState)
            {
                if (METER_DIR_EXTRACT == s_dir)
                {
					if(METER_MODE_DIRECT == s_mode)
	                {
	                    TRACE_INFO("\n direct mode meter succeed");
	                    s_result = METER_RESULT_FINISHED;
	                    Meter_Stop(METER_RESULT_FINISHED);
	                }
					else
					{
						TRACE_INFO("\n PUMPEXTRACT To DRAIN");
                   		Meter_StatusSwitch(METER_STATUS_DRAIN);
					}
				
                    
//                    s_calibrateStep += PumpManager_GetAlreadyStep(
//                    METER_PUMP_NUM);
                    s_lastStatusVolume += PumpManager_GetVolume(
                    METER_PUMP_NUM);

//                    TRACE_CODE("\n s_calibrateStep: %d", s_calibrateStep);
                    TRACE_DEBUG("\n s_lastStatusVolume:");
                    System_PrintfFloat(TRACE_LEVEL_DEBUG,
                        s_lastStatusVolume, 2);
                    TRACE_DEBUG(" ml");
                }
                else
                {
                    TRACE_INFO("\n METER_DIR_DRAIN accurate meter succeed");
                    s_result = METER_RESULT_FINISHED;//定量结果保存在s_result，提高给其他地方判断
                    Meter_Stop(METER_RESULT_FINISHED);
                }
            }
            else
            {
                s_isStartCheackMeter = FALSE;
                Meter_Stop(s_result);
            }

        }
        break;

    case METER_STATUS_SMART:
        if (TRUE == s_isStatusSwitchStart)
        {
            s_isStatusSwitchStart = FALSE;

            if (s_smartPumpVolume >= 0) //s_smartPumpVolume大于零，表示整体定量体积大于s_setMeterPoint定量点体积，泵继续抽液体
            {
                s_smartPumpVolume += PumpManager_GetVolume(METER_PUMP_NUM); //获取已经泵取得体积再加上还需要泵取得体积。
                PumpManager_ChangeVolume(METER_PUMP_NUM, s_smartPumpVolume); //这是配置整个泵取过程所泵取得体积
            }
            else   //s_smartPumpVolume小于零，表示整体定量体积小于s_minMeterPoint定量点体积，泵停止后往下压
            {
                PumpManager_Stop(METER_PUMP_NUM);
                while (PUMP_IDLE != PumpManager_GetStatus(METER_PUMP_NUM))
                {
                    vTaskDelay(2 / portTICK_RATE_MS);
                }
                PumpManager_Start(METER_PUMP_NUM, CLOCKWISE,
                        -1 * s_smartPumpVolume, NoReadFLASH);
            }
            PumpManager_SetMotionParam(METER_PUMP_NUM,
                    PUMP_ACC_SPEED(SMARTEXTRACT_ACC),
                    PUMP_ACC_SPEED(SMARTEXTRACT_MAX_SPEED), NoWriteFLASH);

            s_isSpeedDown = FALSE;
            s_result = METER_RESULT_FINISHED;
            s_isWaitPumpStop = TRUE;
        }

        if (PUMP_IDLE == PumpManager_GetStatus(METER_PUMP_NUM))
        {
            TRACE_INFO("\n smart extract meter succeed");
            Meter_Stop(s_result);
        }
        break;

    case METER_STATUS_DRAIN:
        if (TRUE == s_isStatusSwitchStart)
        {
            s_isStatusSwitchStart = FALSE;
            if(METER_DIR_DRAIN == s_dir)
            {
                if (DSCP_OK
                        != Meter_PumpStart(PUMP_ACC_SPEED(DRAIN_ACC),
                                PUMP_ACC_SPEED(FIRST_PUMPEXTRACT_MAX_SPEED), CLOCKWISE,
                                (METER_VOlUME_MAX / 10.0) * 2))
                {
                    TRACE_ERROR("\n Meter Pump Start error");
                    s_result = METER_RESULT_FAILED;
                    Meter_Stop(METER_RESULT_FAILED);
                    break;
                }
            }
            else
            {
                if (DSCP_OK
                        != Meter_PumpStart(PUMP_ACC_SPEED(DRAIN_ACC),
                                PUMP_ACC_SPEED(DRAIN_MAX_SPEED), CLOCKWISE,
                                (METER_VOlUME_MAX / 10.0) * 2))
                {
                    TRACE_ERROR("\n Meter Pump Start error");
                    s_result = METER_RESULT_FAILED;
                    Meter_Stop(METER_RESULT_FAILED);
                    break;
                }
            }

            s_isStartCheackMeter = TRUE;
            s_opticalState = OPTICAL_STATUS_AD_LIQUID_STABLE;
            MeterScheduler_StartMeterCheckTimer();

            s_isWaitPumpStop = FALSE;
            s_isSpeedDown = FALSE;
        }

        if (FALSE == s_isWaitPumpStop)
        {
            if (OPTICAL_STATUS_AD_INCREASE == s_opticalState
                    && (FALSE == s_isSpeedDown))
            {
                s_isSpeedDown = TRUE;
                PumpManager_SetMotionParam(METER_PUMP_NUM,
                        PUMP_ACC_SPEED(PUMP_HIGH_ACC),
                        PUMP_ACC_SPEED(PUMP_LOW_SPEED), NoWriteFLASH);
            }
            else if (OPTICAL_STATUS_AD_COMPLETE == s_opticalState)
            {
                Meter_RequestPumpStop();
            }
            else if (OPTICAL_STATUS_AD_WRONG == s_opticalState)
            {
                s_result = METER_RESULT_AIRBUBBLE;
                Meter_RequestPumpStop();
                TRACE_ERROR("\n AirBubble error stop");
            }
        }
        else if (PUMP_IDLE == PumpManager_GetStatus(METER_PUMP_NUM))
        {
            if (OPTICAL_STATUS_AD_COMPLETE == s_opticalState)
            {
                if (METER_DIR_EXTRACT == s_dir)
                {
//                    s_calibrateStep -= PumpManager_GetAlreadyStep(
//                    METER_PUMP_NUM);

                    s_lastStatusVolume -= PumpManager_GetVolume(
                    METER_PUMP_NUM);

                    TRACE_ERROR("\n METER_DIR_EXTRACT accurate meter succeed");
                    s_result = METER_RESULT_FINISHED;
                    Meter_Stop(METER_RESULT_FINISHED);
                }
                else
                {
                    TRACE_INFO("\n DRAIN To FIRST_PUMPEXTRACT");
                    Meter_StatusSwitch(METER_STATUS_FIRST_PUMPEXTRACT);
                }
            }
            else
            {
                s_isStartCheackMeter = FALSE;
                Meter_Stop(s_result);
            }
        }
        break;

    default:
        break;
    }

     Meter_ErrorJudge();
}

OpticalMeterAD Meter_GetOpticalAD(void)
{
    OpticalMeterAD pointsAD;
    pointsAD.num = s_meterPoint.num;
    for (Uint8 i = 0; i < pointsAD.num; i++)
    {
        pointsAD.adValue[i] = MeterADC_GetCurAD(i);
    }
    return (pointsAD);
}

static void Meter_MeterADPeriodHandle(void)
{
    static OpticalMeterAD s_pointsAD;
    static Uint32 s_tempAD[METER_POINT_NUM] =
    { 0 };
    static Uint8 s_data[METER_POINT_NUM * sizeof(float) + 1] =
    { 0 };
    static Uint16 s_len;

    if (TRUE == LaiRS485_GetHostStatus())
    {
        s_pointsAD = Meter_GetOpticalAD();
        for (Uint8 i = 0; i < s_pointsAD.num; i++)
        {
            if (0 != s_meterPoint.volume[i][SETVIDX]) //如果此定量点体积为0表示不存在，则此定量点的AD值赋值为0
            {
                s_tempAD[i] = (Uint32) s_pointsAD.adValue[i];
            }
            TRACE_MARK("\n %d point adValue = %d , %d", i,
                    s_pointsAD.adValue[i], s_tempAD[i]);
        }

        s_len = sizeof(Uint8) + sizeof(Uint32) * (s_pointsAD.num);

        s_data[0] = s_pointsAD.num;
        memcpy(s_data + 1, s_tempAD, s_len - 1);

        DncpStack_SendEvent(DSCP_EVENT_OMI_OPTICAL_AD_NOTICE, s_data, s_len);
    }
}

void Meter_SetMeterADReportPeriod(float period)
{
    TRACE_MARK("\n period:");
    System_PrintfFloat(TRACE_LEVEL_MARK, period, 3);
    TRACE_MARK(" s");
    MeterScheduler_SetMeterADReportPeriod(period);
}

/**
 * 打开定量使用的LED灯，仅供上层命令控制LED
 * @param num 定量点1到定量点METER_POINT_NUM
 * @return TRUE 操作成功，FALSE操作失败
 */
Bool Meter_TurnOnLED(Uint8 num)
{
    return MeterLED_TurnOn(num);
}

/**
 * 关闭定量使用的LED灯，仅供上层命令控制LED
 * @param Num 定量点1到定量点METER_POINT_NUM
 * @return TRUE 操作成功，FALSE操作失败
 */
Bool Meter_TurnOffLED(Uint8 num)
{
    return MeterLED_TurnOff(num);
}

Bool Meter_AutoCloseValve(Bool isAutoCloseValve)
{
    s_isAutoCloseValve = isAutoCloseValve;
    return TRUE;
}

Bool Meter_SetMeterFinishValveMap(Uint32 map)
{
    if (map <= SOLENOIDVALVE_MAX_MAP)
    {
        TRACE_INFO("\nSetMeterFinishValveMap: 0x%x", map);
        s_meterFinishValveMap = map;
        return TRUE;
    }
    else
    {
        TRACE_ERROR("\n The map must be 0 to 0x%x.", SOLENOIDVALVE_MAX_MAP);
        return FALSE;
    }
}

Uint32 Meter_GetSingleOpticalAD(Uint8 num)
{
    Uint32 value = 0;
    if (num <= s_meterPoint.num && num > 0)
    {
        value = MeterADC_GetCurAD(num - 1);
    }
    return value;
}

Bool Meter_SetRopinessOverValue(Uint16 value)
{
    if(value >= 50 && value <= 1000)
    {
        s_ropinessOverValue = value;
        TRACE_INFO("\n Set ropiness meter over value = %d success", value);
        return TRUE;
    }
    else
    {
        TRACE_INFO("\n Set ropiness meter over value %d fail: out of limit[50, 1000]", value);
        return FALSE;
    }
}

Uint16 Meter_GetRopinessOverValue()
{
    TRACE_INFO("\n Get ropiness meter over value = %d", s_ropinessOverValue);
    return s_ropinessOverValue;
}

Uint16 Meter_GetMeterThreshold(void)
{
    TRACE_INFO("\n Get meter Threshold = %d", s_meterThreshold);
    return s_meterThreshold;
}

void Meter_SetMeterThreshold(Uint16 value)
{
    TRACE_INFO("\n Set meter Threshold = %d", value);
    s_meterThreshold = value;
}

Uint16 Meter_GetMeterInterval(void)
{
    TRACE_INFO("\n Get meter Interval = %d", s_meterInterval);
    return s_meterInterval;
}

void Meter_SetMeterInterval(Uint16 value)
{
    TRACE_INFO("\n Set meter Interval = %d", value);
    s_meterInterval = value;
}

Bool Meter_WriteAccurateModeOverValve(Uint16 valve)
{
	if (valve>0 && valve<=METER_MAX_OVER_VALVE)
	{
		s_accurateOverValue = valve;
		return TRUE;
	}
	else
	{
		return FALSE;
	}

}

Uint16 Meter_ReadAccurateModeOverValve()
{
    return s_accurateOverValue;
}
