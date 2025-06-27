/**
 * @addtogroup module_DNCP
 * @{
 */

/**
 * @file
 * @brief 链路适配接口Socket-LaiRS485的实现。
 * @details 在Socket上，通过LaiRS485实现一个测试用的Lai层实体，作为上层协议的测试桩。
 * @version 1.0.0
 * @author kim.xiejinqiang
 * @date 2016-05-09
 */
#include <string.h>
#include "Driver/System.h"
#include "LaiRS485Handler.h"
#include "dncp/lai/Driver/LaiRS485Adapter.h"
#include "dncp/lai/LaiTokenManager.h"
#include "Console/Console.h"
#include "Common/MessageDigest.h"
#include "Tracer/Trace.h"
#include "Common/Types.h"
#include "DNCP/Base/Frame.h"
#include "Common/Utils.h"
#include "DNCP/lai/OS/LaiRS485Scheduler.h"
#include "FreeRTOS.h"
#include "task.h"
#include "WasteControl/WasteManager.h"

#define FPS_FRAME_TRY_SYNC      1   // 帧等待同步中（未同步上）
#define FPS_FRAME_RECEIVING     2   // 帧接收中（已同步上）
#define RES_ESC_NORMAL          1   // 正常状态，无转义
#define RES_ESC_ACTION          2   // 转义中
#define RES_ESC_NONE            1
#define RES_ESC_SYNC            2
#define RES_ESC_ESC             3

static LaiRS485* s_LaiRS485 = NULL; //全局操作
static Byte TxPollingFrame[4] ={ 0, 0, 0, 0 };
static Byte RxPollingFrame[4] ={ 0, 0, 0, 0 };

static void LaiRS485_Reset(LaiRS485* lai, Uint16 action);
static Bool LaiRS485_IsSendable(LaiRS485* lai);
static void LaiRS485_Send(LaiRS485* lai, Uint8 destAddr, Byte* data, int len);

static void LaiRS485_CheckCRC(Byte* data, Uint16 len, Uint8 sourceAddr);
static void LaiRS485_Handle(Byte* data, Uint16 len, Uint8 cmd);

const ILaiComm g_kCommInterfaceLaiBleUart =
{ (IfLaiReset) LaiRS485_Reset, (IfLaiIsSendable) LaiRS485_IsSendable,
        (IfLaiSend) LaiRS485_Send };

void LaiRS485_Reset(LaiRS485* lai, Uint16 action)
{
    if (action & (LAI_TX_ABORT | LAI_TX_CLEAR))
    {
        LaiRS485Adapter_INT_TC_DISABLE();
        lai->idxTx = 0;
        lai->sendBytes = 0;
        lai->isSending = FALSE;
        lai->sendEscStatus = RES_ESC_NONE;
        lai->isSendSync = lai->isSendEnd = FALSE;
        lai->sendBytes = 0;
        lai->idxTx = 0;
        lai->RS485CurrentStatus = Idle;
        LaiRS485Adapter_INT_TC_ENABLE(); //开启发送中断
    }

    if (action & (LAI_RX_ABORT | LAI_RX_CLEAR))
    {
        // 状态初始化
        LaiRS485Adapter_INT_RX_DISABLE(); //拷贝数据前要关接收中断
        lai->recvBytes = 0;
        lai->isFrameDetected = FALSE;
        lai->recvStatus = FPS_FRAME_TRY_SYNC;
        lai->recvEscStatus = RES_ESC_NORMAL;
        lai->isParseTaskActive = FALSE;
        lai->RS485CurrentStatus = Idle;
        LaiRS485Adapter_INT_RX_ENABLE(); //开启接收中断
    }
}
/**
 * @brief 查询接口是否可发送数据。
 * @note 此函数在上层需要发送业务帧时调用，由于RS485是只能在
 *       得到令牌后才能发送的，上层实际上只能缓存数据部能发送，
 *       所以这里返回值一直为0
 * @param lai UDP的Lai层实体对象
 * @return 0
 */
Bool LaiRS485_IsSendable(LaiRS485* lai)
{
    if (lai->RS485CurrentStatus == Idle)//当上层请求数据时,由空闲状态转化为请求状态
    {
        lai->RS485CurrentStatus = Requesting;
//        TRACE_CODE("\nhas no Polling");
        return FALSE;
//        return TRUE;
    }
//    当从机得到令牌处于发送状态，
//    并当前不处于帧发送中，
//    并是上层已经请求发送数据， 而不是得到令牌后请求，
//    并连续发送的帧数没超过设置值
    else if (lai->RS485CurrentStatus == Sending)
    {
        if ((lai->isSending != TRUE) && (lai->isOnSendRequesting == TRUE)
                && (lai->txByteCount < lai->base.maxTransmitUnit))
        {
//           TRACE_CODE("\nhas Polling");
            return TRUE;
        }
    }
//    TRACE_CODE("\ncan't send");
    return FALSE;

//    lai->isOnSendRequesting = TRUE;
//    return !(lai->isSending);
}
/**
 * @brief 发送数据，并进行转义处理
 * @param lai UDP的Lai层实体对象
 */
static inline void LaiRS485_EscapingSendChar(LaiRS485* lai)
{
    Byte data;
    data = lai->sendBuffer[lai->idxTx]; // 从缓冲中检出一个数据，但没有真正读出
    switch (lai->sendEscStatus)
    // 状态：正常态，上一次转义没有因为FIFO空间不足而被打断
    {
    case RES_ESC_NONE: // 状态转换：判断当前字符是否需要转义
        if (data == FRAME_CHAR_SYNC || data == FRAME_CHAR_ESC) // 转换动作：发送一个同步转义符
        {
            lai->sendEscStatus =
                    (data == FRAME_CHAR_SYNC) ? RES_ESC_SYNC : RES_ESC_ESC;
            data = FRAME_CHAR_ESC;
            LaiRS485Adapter_DATA_WRITE(data);
        }
        else // 无需转义
        {
            (lai->idxTx)++;
            LaiRS485Adapter_DATA_WRITE(data);
        }
        break;
    case RES_ESC_SYNC: // 状态：转义态，上一次已经发送了转义符
        (lai->idxTx)++;
        data = FRAME_CHAR_RAW_SYNC;
        lai->sendEscStatus = RES_ESC_NONE;
        LaiRS485Adapter_DATA_WRITE(data);
        break;
    case RES_ESC_ESC: // 状态：转义态，上一次已经发送了转义符
        (lai->idxTx)++;
        data = FRAME_CHAR_RAW_ESC;
        lai->sendEscStatus = RES_ESC_NONE;
        LaiRS485Adapter_DATA_WRITE(data);
        break;
    }
}


/**
 * @brief 16进制Modbus CRC校验值计算
 * @param buffer 需要数据
 * @param len 数据长度
 */
unsigned short LaiRS485_GetCRCData(unsigned char *buffer,unsigned len)
{
    unsigned short wcrc = 0xFFFF;                   //16位CRC寄存器预置
    unsigned char temp;
    unsigned i = 0, j = 0;                      //计数
    for (i = 0; i < len; i++)                        //循环计算每个数据
    {
        temp = *buffer & 0x00FF;                    //将八位数据与crc寄存器亦或
        buffer++;                                   //指针地址增加，指向下个数据
        wcrc ^= temp;                               //将数据存入crc寄存器
        for (j = 0; j < 8; j++)                      //循环计算数据的
        {
            if (wcrc & 0x0001)                      //判断右移出的是不是1，如果是1则与多项式进行异或。
            {
                wcrc >>= 1;                           //先将数据右移一位
                wcrc ^= 0xA001;                     //与上面的多项式进行异或
            }
            else                                    //如果不是1，则直接移除
            {
                wcrc >>= 1;                           //直接移除
            }
        }
    }
    unsigned char CRC_L,CRC_H;                      //定义数据高低位
    CRC_L = wcrc & 0xFF;                            //CRC的低8位
    CRC_H = wcrc >> 8;                                //CRC的高8位

    return(( CRC_H << 8) | CRC_L);
}

/**
 * @brief 做发送前准备，把发送数据存入发送缓冲区，发送同步字节
 * @param lai lai UDP的Lai层实体对象
 * @param destAddr 没有用到
 * @param data 要发送的数据
 * @param len 数据长度
 */
void LaiRS485_Send(LaiRS485* lai, Uint8 destAddr, Byte* data, int len)
{
    Byte byteToSend;
    ILaiHandle* handle = s_LaiRS485->base.handle;
    LaiRS485Adapter_RS485_SwitchToTx();
    lai->isSending = TRUE;
    ++lai->txByteCount;
    TRACE_INFO("\nLaiRS485_Send: %d", len);

    //发送队列已空，即当前帧为最后一帧数据
    //此帧已是发送最大帧数
    //同时还得处于请求发送业务帧，避免发送令牌帧的时候进行处理
//    if ((handle->OnSendQuery(handle) == 0
//            || (lai->txByteCount == lai->base.maxTransmitUnit))
//            && lai->isOnSendRequesting == TRUE)
    {
        FrameCtrlWord fcw;
        Byte* p;
        Uint16 chksum;

        //标志控制字为最后一帧数据
//        fcw.data = data[1];
//        fcw.info.pf = 1;
//        data[1] = fcw.data;
        //更新CRC校验
        chksum = LaiRS485_GetCRCData(data, len - 2);
        p = data + len - 2;
        *p++ = (Uint8) (chksum & 0x00FF);
        *p = (Uint8) ((chksum & 0xFF00) >> 8);

        lai->isSendLastFrame = TRUE;    //标志最后一帧
        lai->isOnSendRequesting = FALSE;    //请求发送业务帧结束
        TRACE_DEBUG("\n %x %x", chksum >> 8, chksum & 0x00FF);
    }

    lai->sendEscStatus = RES_ESC_NONE;
    lai->idxTx = 0;
    lai->isSendSync = TRUE;
    lai->isSendEnd = FALSE;

    lai->sendBytes = len;
    memcpy(lai->sendBuffer, data, len);
    byteToSend = FRAME_CHAR_SYNC;

    for(Uint8 i = 0;i<len;i++)
    {
    	 TRACE_DEBUG("\t %x", lai->sendBuffer[i]);
    }
    // System_Delay(2);

    for (int i = 0; i < 1000; i++)
    {

    }

    LaiRS485Adapter_INT_TXE_ENABLE();
}

void LaiRS485_Init(LaiRS485* lai)
{
    s_LaiRS485 = lai;
    lai->idxTx = 0;
    lai->sendBytes = 0;
    lai->base.comm = (ILaiComm*) &g_kCommInterfaceLaiBleUart; //本层通信接口，具体实现时必须初始化它
}

void LaiRS485_Start(LaiRS485* lai)
{
    Uint16 chksum;
    FrameCtrlWord fcw;

    // 状态初始化
    lai->ctrl.data = 0;
    lai->isSending = FALSE;
    lai->isPolling = FALSE;
    lai->pollSendNum = 0;

    lai->isParseTaskActive = FALSE;
    lai->recvStatus = FPS_FRAME_TRY_SYNC;
    lai->recvEscStatus = RES_ESC_NONE;
    lai->sendEscStatus = RES_ESC_NONE;
    lai->isSendSync = FALSE;
    lai->isSendEnd = FALSE;
    lai->recvBytes = 0;
    lai->recvFrameBytes = 0;
    lai->sendBytes = 0;
    lai->idxTx = 0;
    lai->isFrameDetected = FALSE;
    lai->checkSlave = 0;
    lai->idxTx = 0;
    lai->sendBytes = 0;

    lai->RS485CurrentStatus = Idle;
    lai->isSendAllow = FALSE;

    //配置用于发送给主机的令牌帧。根据现行CRC算法计算令牌帧校验结果并保存在令牌帧里
    fcw.data = 0;
    fcw.man.pf = 1;
    fcw.man.flag = 1;
    TxPollingFrame[1] = fcw.data;
    chksum = MessageDigest_Crc16Ccitt(0, TxPollingFrame, 2);
    TxPollingFrame[2] = (Uint8) (chksum & 0x00FF);
    TxPollingFrame[3] = (Uint8) ((chksum & 0xFF00) >> 8);

    //配置用于校验接收的令牌帧正确与否的接收令牌帧
    RxPollingFrame[0] = lai->base.address;
    RxPollingFrame[1] = fcw.data;
    chksum = MessageDigest_Crc16Ccitt(0, RxPollingFrame, 2);
    RxPollingFrame[2] = (Uint8) (chksum & 0x00FF);
    RxPollingFrame[3] = (Uint8) ((chksum & 0xFF00) >> 8);

    TRACE_INFO("RS485Slaveaddr:%02d",lai->base.address);
}

void LaiRS485_Stop(LaiRS485* lai)
{
    LaiRS485Adapter_INT_TC_DISABLE();
}
/**
 * @brief RS485使用的串口中断服务程序,有发送中断(发送完成中断或者发送寄存器空中断)
 *      还有接收中断
 * @note 此函数在startup_stm32f10x_hd.s文件里被调用。发送中断主要处理的是发送同步字，
 *      进行字符转义，发送完一帧数据后进行相关标志，485转变为接收状态，然后激活请求发送任务
 *      接收中断主要处理，识别并接收一帧完整数据，然后做本机数据帧判断，
 *      是则接着判断是令牌帧还是业务帧，并进行对应处理。
 */
void LaiRS485_IntHandle(void)
{
    LaiRS485* lai = s_LaiRS485;
    unsigned char data;

//    LaiRS485Adapter_INT_HANDLE_INIT();
    if (LaiRS485Adapter_INT_STATUS_IS_RX())
    {
        data = LaiRS485Adapter_DATA_READ();
        TRACE_INFO(" %02x", data);
        if (data == lai->base.address && lai->recvStatus == FPS_FRAME_TRY_SYNC) // lai->base.address FRAME_CHAR_SYNC
        {
            {
                lai->recvStatus = FPS_FRAME_RECEIVING;
				lai->txByteCount = 0;
				lai->isSendLastFrame = FALSE;
				lai->isSendAllow = TRUE;
				lai->tokenTimeoutCnt = 0;
				lai->isTokenTimeout = FALSE;
				lai->recvBuf[(lai->recvBytes)++] = data;
            }
        }
        else
        {
            if (lai->recvStatus == FPS_FRAME_TRY_SYNC)
            {
                TRACE_ERROR(" OOD:%02x", data);
                // 帧外数据统计
                (lai->base.commStat.outOfFrameBytes)++;
            }
            else // FPS_FRAME_RECEIVING  帧接收中（已同步上）
            {
                // 转义状态机
                if (lai->recvEscStatus == RES_ESC_ACTION)
                {
                    if (data == FRAME_CHAR_RAW_SYNC) // 原始字符为同步符
                    {
                        data = FRAME_CHAR_SYNC;
                    }
                    else if (data == FRAME_CHAR_RAW_ESC) // 原始字符为转义符
                    {
                        data = FRAME_CHAR_ESC;
                    }
                    lai->recvBuf[(lai->recvBytes)++] = data;
                    lai->recvEscStatus = RES_ESC_NORMAL;
                }
                else
                {
                    // 未转义
                    if (data == FRAME_CHAR_ESC)
                    {
                        lai->recvEscStatus = RES_ESC_ACTION;
                    }
                    else
                    {
                        // 正常字符，提交
                        lai->recvBuf[lai->recvBytes++] = data;
                    }
                }
            }
        }
    }

	if (LaiRS485Adapter_INT_STATUS_IS_TX())
	{
		if(lai->idxTx < lai->sendBytes)
		{
			data = lai->sendBuffer[lai->idxTx++];
			LaiRS485Adapter_DATA_WRITE(data);
//			TRACE_INFO(" #%x", data);
		}
		else
		{
//			LaiRS485Adapter_RS485_SwitchToRx();  //注意：放在空闲任务内进行切换接收状态，否则发送最后一个字节将会FF异常
			lai->isSendAllow = FALSE;
			lai->RS485CurrentStatus = Idle;
			// 已发完一帧
			lai->isSending = FALSE;
			lai->isSendEnd = TRUE;
			lai->isSendSync = FALSE;
			LaiRS485Adapter_INT_TXE_DISABLE();
			LaiRS485Adapter_INT_TC_DISABLE();
		}
	}

}
/**
 * @brief 有业务帧需要发送则请求上层发送业务帧，无则回应令牌帧。
 *      发送完还查看上层是否有数据请求发送
 * @note 此函数由后台请求发送事件处理任务调用
 */
void LaiRS485_SendRequest(void)
{

    LaiRS485* lai = s_LaiRS485;
    ILaiHandle* handle = lai->base.handle;

    //有令牌允许发送
    if(lai->isSendAllow == TRUE)
    {
        //有业务帧请求发送，回应业务帧
        if (lai->RS485CurrentStatus == Requesting || lai->isOnSendRequesting == TRUE)
        {
//            TRACE_CODE("\nSend Data Frame");
            //进入发送状态，查看队列是否有数据，有则上层请求数据发送
            lai->RS485CurrentStatus = Sending;
            if (handle->OnSendQuery(handle))
            {
                static unsigned long int TxCnt=0;
                lai->isOnSendRequesting = TRUE;
                TxCnt++;
//                TRACE_INFO("\r\n################SendCount%d\r\n",TxCnt);
                handle->OnSendRequest(handle);
            }
        }
        else //无业务帧请求发送,回应令牌帧
        {
//            TRACE_CODE("\nSend Polling");
            //进入发送状态，标志此为最后一帧，发送令牌帧
            lai->RS485CurrentStatus = Sending;
            lai->isSendLastFrame = TRUE;
            LaiRS485_Send(lai, TxPollingFrame[0], TxPollingFrame, 4);
        }
    }
    else//无令牌请求数据
    {
        //请求上层发送数据,实际上没有令牌数据是发送不成功的
        if (handle->OnSendQuery(handle))
        {
            handle->OnSendRequest(handle);
        }
    }

}
/**
 * @brief 使用上层接口，把数据提交给上层
 * @note 此函数由后台接收事件处理任务调用
 */
void LaiRS485_CommitToUpper(void)
{
    LaiRS485* lai = s_LaiRS485;
//    lai->base.handle->OnReceived(lai->base.handle, lai->recvFrameData,
//            lai->recvFrameBytes, 1);

    LaiRS485_CheckCRC(lai->recvFrameData,lai->recvFrameBytes, 1);
}

void LaiRS485_RxComplete(void)
{
	LaiRS485* lai = s_LaiRS485;
//	TRACE_INFO("%c [%d]",'A', lai->recvBuf[0]);
	if (lai->isFrameDetected)
	{
		//判断是否是本机地址
		if (lai->recvBuf[0] == lai->base.address)
		{
			++(lai->base.commStat.recvFrames);
			{
				//拷贝数据到二级缓存区
				memcpy(lai->recvFrameData, lai->recvBuf, lai->recvBytes);
				lai->recvFrameBytes = lai->recvBytes;

				memcpy(lai->sendBuffer, lai->recvBuf, lai->recvBytes);
				lai->sendBytes = lai->recvBytes;

				//激活后台任务，提交接收到的业务帧给上层处理
				LaiRS485CommitToUpperTask_Active();
			}
		}
		//清除接收状态
		lai->recvBytes = 0;
		lai->isFrameDetected = FALSE;
		lai->recvStatus = FPS_FRAME_TRY_SYNC;
	}
}

void LaiRS485_PollingTimeOut(void)
{
    s_LaiRS485->tokenTimeoutCnt++;
    if(s_LaiRS485->tokenTimeoutCnt > 3 && s_LaiRS485->recvBytes > 0)
    {
    	s_LaiRS485->isFrameDetected = TRUE;
    	LaiRS485_RxComplete();
    	Printf("#");
    }
    else if(s_LaiRS485->isSending == FALSE)
    {
    	LaiRS485Adapter_RS485_SwitchToRx();
    }
    System_Delay(50);
}

Bool LaiRS485_GetHostStatus(void)
{
    LaiRS485* lai = s_LaiRS485;
    if(TRUE == lai->isTokenTimeout || lai->tokenTimeoutCnt > 5)
    {
        lai->isTokenTimeout = TRUE;
        TRACE_CODE("\n Failed to communicate with host.");
        return FALSE;
    }
    return TRUE;
}

void LaiRS485_CheckCRC(Byte* data, Uint16 len, Uint8 sourceAddr)
{
	Uint16 chksum = 0;
	Byte* p = data + len - 1;

	// 检验帧的完整性，不完整的帧将被丢弃
	if (len >= 4)
	{
		chksum = *p--;
		chksum <<= 8;
		chksum |= *p;
		if (chksum == LaiRS485_GetCRCData(data, len-2))
		{
			Printf("CRC ok");
			LaiRS485_Handle(data+2, len, data[1]);
		}
		else
		{
			TRACE_WARN("\n Lai CRC Error! size=%d", len);
		}
		Printf("\n Rec %x , CRC %x", chksum, LaiRS485_GetCRCData(data, len-2));
	}
	else
	{
		TRACE_WARN("\n DSLP Frame Size Error! size=%d", len);
	}
}

void LaiRS485_ModbuSend(Byte* data, int len)
{
	LaiRS485_Send(s_LaiRS485, s_LaiRS485->destAddr, data, len);
}

void LaiRS485_Handle(Byte* data, Uint16 len, Uint8 cmd)
{
	Uint16 reg = data[0]<<8 | data[1];
	Uint16 param = data[2]<<8 | data[3];
	Printf("\n %x %x %x %x",data[0], data[1], data[2], data[3]);

	Printf("\nCmd %d Reg %d Param %d",cmd, reg, param);
	WasteManager_Cmd(cmd, reg, param);
//	switch(cmd)
//	{
//		case CMD_READ: //读指令
//			switch(reg)
//			{
//				case REGISTER_STATUS:
//					Printf("\n REGISTER_STATUS");
//					break;
//				default :
//					Printf("\nInvalid Read Reg %d",reg);
//					break;
//			}
////			LaiRS485_Send(s_LaiRS485, s_LaiRS485->destAddr, s_LaiRS485->sendBuffer, s_LaiRS485->sendBytes);
//			break;
//		case CMD_WRITE:
//			switch(reg)
//			{
//				case REGISTER_CONTROL:
//					Printf("\n REGISTER_CONTROL");
//
//					break;
//				default :
//					Printf("\nInvalid Write Reg %d",reg);
//					break;
//			}
//			WasteManager_Cmd(CMD_WRITE, reg, param);
////			LaiRS485_Send(s_LaiRS485, s_LaiRS485->destAddr, s_LaiRS485->sendBuffer, s_LaiRS485->sendBytes);
//			break;
//		case CMD_RW_HOLD:
//			break;
//		default :
//			Printf("\nInvalid Cmd %d",cmd);
//	}
}
