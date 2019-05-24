/**
************************************************************
* @file         gizwits_product.c
* @brief        Gizwits control protocol processing, and platform-related       hardware initialization 
* @author       Gizwits
* @date         2017-07-19
* @version      V03030000
* @copyright    Gizwits
* 
* @note         机智云.只为智能硬件而生
*               Gizwits Smart Cloud  for Smart Products
*               链接|增值ֵ|开放|中立|安全|自有|自由|生态
*               www.gizwits.com
*
***********************************************************/

#include <stdio.h>
#include <string.h>
#include "Vending_Machine.h"
#include "hal_key.h"
#include "gizwits_product.h"
#include "common.h"

static uint32_t timerMsCount;
uint8_t aRxBuffer;

/** User area the current device state structure*/
dataPoint_t currentDataPoint;
extern keysTypedef_t keys;

extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

/**@} */
/**@name Gizwits User Interface
* @{
*/

/**
* @brief Event handling interface

* Description:

* 1. Users can customize the changes in WiFi module status

* 2. Users can add data points in the function of event processing logic, such as calling the relevant hardware peripherals operating interface

* @param [in] info: event queue
* @param [in] data: protocol data
* @param [in] len: protocol data length
* @return NULL
* @ref gizwits_protocol.h
*/
int8_t gizwitsEventProcess(eventInfo_t *info, uint8_t *gizdata, uint32_t len)
{
  uint8_t i = 0;
  dataPoint_t *dataPointPtr = (dataPoint_t *)gizdata;
  moduleStatusInfo_t *wifiData = (moduleStatusInfo_t *)gizdata;
  protocolTime_t *ptime = (protocolTime_t *)gizdata;
  
#if MODULE_TYPE
//  gprsInfo_t *gprsInfoData = (gprsInfo_t *)gizdata;//2018无引用修改注释
#else
  moduleInfo_t *ptModuleInfo = (moduleInfo_t *)gizdata;
#endif

  if((NULL == info) || (NULL == gizdata))
  {
    return -1;
  }

  for(i=0; i<info->num; i++)
  {
    switch(info->event[i])
    {
      case EVENT_Msg_Control:
        GIZWITS_LOG("Evt: EVENT_Msg_Control\n");
        memcpy((uint8_t *)&currentDataPoint.valueMsg_Control,(uint8_t *)&dataPointPtr->valueMsg_Control,sizeof(currentDataPoint.valueMsg_Control));
        //user handle
        //首先对信号解码，而后再对信号进行处理
        switch(currentDataPoint.valueMsg_Control[0]/16){
          case 1:VM_Control.ControlState=selfcheck;break;
          case 2:VM_Control.ControlState=shipment;break;
          case 3:VM_Control.ControlState=opendoor;break;
          case 4:VM_Control.ControlState=shutdown;break;
          case 5:VM_Control.ControlState=clearallstate;break;
          default:VM_Control.ControlState=stand_by;break;
        }
        
        switch(currentDataPoint.valueMsg_Control[0]%16){
          case 1:VM_Control.Aisle=Aisle1;break;
          case 2:VM_Control.Aisle=Aisle2;break;
          case 3:VM_Control.Aisle=Aisle3;break;
          default:VM_Control.Aisle=NoAisle;break;
        }
//        VM_Control.Aisle=currentDataPoint.valueMsg_Control[0]%16;
        for(int i=0;i<9;i++)
          VM_Control.Ordernumber[i]=currentDataPoint.valueMsg_Control[1+i];
        VM_Control.CkeckSum=currentDataPoint.valueMsg_Control[10];//获得了校验位但是先不做什么.
       //获取信息完成
       //处理信息开始
        if(VM_InnerCtrl.InnerState==stand_by||VM_InnerCtrl.InnerState==stand_by1){//如果在待机状态下则更新控制信息
//        if(1){//如果在待机状态下则更新控制信息
          VM_InnerCtrl.InnerState=VM_Control.ControlState;
          VM_InnerCtrl.Aisle=VM_Control.Aisle;
          for(int i=0;i<9;i++)
            VM_InnerCtrl.Ordernumber[i]=VM_Control.Ordernumber[i];
          switch(VM_InnerCtrl.InnerState){//每次遍历
            case selfcheck:VM_InnerCtrl.DoSelfCheck=ON;break;
            case shipment:VM_InnerCtrl.DoShipment=ON;break;
            case opendoor:VM_InnerCtrl.DoOpenDoor=ON;break;
            case shutdown:VM_InnerCtrl.DoShutdown=ON;break;
            case clearallstate:VM_Control.ControlState=clearallstate;break;
          }
        }
        if(VM_Control.ControlState==opendoor){
          VM_InnerCtrl.DoOpenDoor=ON;
        }
        if(VM_Control.ControlState==clearallstate){
          VM_InnerCtrl.InnerState=stand_by;
          VM_InnerCtrl.Aisle=NoAisle;
          VM_InnerCtrl.ErrorCode=NoError;
      
          VM_InnerCtrl.StartCheckState=OFF;
          VM_InnerCtrl.StartCheckFinish=OFF;
          VM_InnerCtrl.OpenStepMotoState=OFF;
          VM_InnerCtrl.OpenStepMotoFinish=OFF;
          VM_InnerCtrl.MotoState=OFF;
          VM_InnerCtrl.MotoFinish=OFF;
          VM_InnerCtrl.CloseStepMotoState=OFF;
          VM_InnerCtrl.CloseStepMotoFinish=OFF;
          VM_InnerCtrl.DoorLazerCheckState=OFF;
          VM_InnerCtrl.DoorLazerCheckFinish=OFF;
          VM_InnerCtrl.ShipmentDoorState=OFF;
          VM_InnerCtrl.ShipmentDoorFinish=OFF;
          Speakaword(selfcheck_complete);
        }
        //user handle
        break;

      case WIFI_SOFTAP:
        break;
      case WIFI_AIRLINK:
        break;
      case WIFI_STATION:
        break;
      case WIFI_CON_ROUTER:
 
        break;
      case WIFI_DISCON_ROUTER:
 
        break;
      case WIFI_CON_M2M:
 
        break;
      case WIFI_DISCON_M2M:
        break;
      case WIFI_RSSI:
        GIZWITS_LOG("RSSI %d\n", wifiData->rssi);
        break;
      case TRANSPARENT_DATA:
        GIZWITS_LOG("TRANSPARENT_DATA \n");
        //user handle , Fetch data from [data] , size is [len]
        break;
      case WIFI_NTP:
        GIZWITS_LOG("WIFI_NTP : [%d-%d-%d %02d:%02d:%02d][%d] \n",ptime->year,ptime->month,ptime->day,ptime->hour,ptime->minute,ptime->second,ptime->ntp);
        break;
      case MODULE_INFO:
            GIZWITS_LOG("MODULE INFO ...\n");
      #if MODULE_TYPE
            GIZWITS_LOG("GPRS MODULE ...\n");
            //Format By gprsInfo_t
      #else
            GIZWITS_LOG("WIF MODULE ...\n");
            //Format By moduleInfo_t
            GIZWITS_LOG("moduleType : [%d] \n",ptModuleInfo->moduleType);
      #endif
    break;
      default:
        break;
    }
  }

  return 0;
}

/**
* User data acquisition

* Here users need to achieve in addition to data points other than the collection of data collection, can be self-defined acquisition frequency and design data filtering algorithm

* @param none
* @return none
*/
void userHandle(void)
{
 /*
  //XXX is Extend Datapoint Address ,User defined
  memcpy((uint8_t *)currentDataPoint.valuemsg_heart,XXX,sizeof(currentDataPoint.valuemsg_heart));
  */
if(VM_InnerCtrl.InnerState!=stand_by||VM_InnerCtrl.InnerState!=stand_by1){//不在stand_by状态则开始执行函数
  switch(VM_InnerCtrl.InnerState){
    case selfcheck:do_selfcheck();break;
    case shipment:do_shipment();break;
    case opendoor:do_opendoor();break;
    //case opendoor:Speakaword(Sorry);break;
    case shutdown:do_shutdown();break;
    case clearallstate:do_zhuce();break;
  }
}

if(VM_Count.OpPasscount>100){//自检过后两秒
  VM_Count.OpPassSwitch=OFF;
  VM_Count.OpPasscount=0;
  VM_InnerCtrl.InnerState=stand_by;
  VM_Count.HeartSwitch=ON;//自检完毕开启心跳。
}

//下面是回复的代码
if(VM_InnerCtrl.ReplySwitch){//开了回复代码后
  if(VM_Count.HeartSwitch==ON){//如果心跳开关打开则发送心跳信息
    if(VM_Count.Heartcount>8000){//每隔五分钟发送一次//0121修改测试
      VM_Count.Heartcount=0;
      if(VM_Reply.ReplyState==stand_by)
        VM_Reply.ReplyState=stand_by1;
      else if(VM_Reply.ReplyState==stand_by1)
        VM_Reply.ReplyState=stand_by;
      else
        VM_Reply.ReplyState=VM_InnerCtrl.InnerState;
        VM_Reply.ReplyOPState=VM_InnerCtrl.InnerOPState;
        if(VM_InnerCtrl.ErrorCode==NoGoods)
          VM_Reply.Aisle=VM_InnerCtrl.NoGoodsAisle;
        else
          VM_Reply.Aisle=VM_InnerCtrl.Aisle;
        VM_Reply.ErrorCode=VM_InnerCtrl.ErrorCode;
        VM_Reply.BatteryCapacity=VM_InnerCtrl.BatteryCapacity;//提交电池电量
      for(int i=0;i<9;i++)//待机订单消息
        VM_Reply.Ordernumber[i]=0x00;//待机订单号全0
    }
  }
  else{//没开心跳开关则正在处理相关操作
    VM_Reply.ReplyState=VM_InnerCtrl.InnerState;
    VM_Reply.ReplyOPState=VM_InnerCtrl.InnerOPState;
    if(VM_InnerCtrl.ErrorCode==NoGoods)
      VM_Reply.Aisle=VM_InnerCtrl.NoGoodsAisle;
    else
      VM_Reply.Aisle=VM_InnerCtrl.Aisle;
    VM_Reply.ErrorCode=VM_InnerCtrl.ErrorCode;
    VM_Reply.BatteryCapacity=VM_InnerCtrl.BatteryCapacity;
    for(int i=0;i<9;i++)//待机订单消息
      VM_Reply.Ordernumber[i]=VM_InnerCtrl.Ordernumber[i];//订单号
  }
  //下面是赋值给currentDataPoint.valueMsg_Reply
  currentDataPoint.valueMsg_Reply[0]=VM_Reply.ReplyState*16+VM_Reply.ReplyOPState;
  currentDataPoint.valueMsg_Reply[1]=VM_Reply.Aisle*16+VM_Reply.ErrorCode;
  currentDataPoint.valueMsg_Reply[2]=VM_Reply.BatteryCapacity;
  for(int i=0;i<9;i++)//订单消息
    currentDataPoint.valueMsg_Reply[3+i]=VM_Reply.Ordernumber[i];
  currentDataPoint.valueMsg_Reply[12]=0;//开始校验
  for(int i=0;i<12;i++)//订单消息校验
    currentDataPoint.valueMsg_Reply[12]+=(currentDataPoint.valueMsg_Reply[i]/16+currentDataPoint.valueMsg_Reply[i]%16);
  }
}

/**
* Data point initialization function

* In the function to complete the initial user-related data
* @param none
* @return none
* @note The developer can add a data point state initialization value within this function
*/
void userInit(void)
{
    memset((uint8_t*)&currentDataPoint, 0, sizeof(dataPoint_t));
    
    /** Warning !!! DataPoint Variables Init , Must Within The Data Range **/ 
    /*
    */

}


/**
* @brief Millisecond timing maintenance function, milliseconds increment, overflow to zero

* @param none
* @return none
*/
void gizTimerMs(void)
{
    timerMsCount++;
}

/**
* @brief Read millisecond count

* @param none
* @return millisecond count
*/
uint32_t gizGetTimerCount(void)
{
    return timerMsCount;
}

/**
* @brief MCU reset function

* @param none
* @return none
*/
void mcuRestart(void)
{
    __set_FAULTMASK(1);
    HAL_NVIC_SystemReset();
}

/**@} */

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
 
  return ch;
}

/**
  * @brief  Period elapsed callback in non blocking mode 
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim==&htim2)
    {
      if(VM_Count.HeartSwitch==ON){
        VM_Count.Heartcount++;
      }
      if(VM_Count.LedSwitch==ON){
        VM_Count.Ledcount++;
      }
      if(VM_Count.OpenDoorSwitch==ON){//开门计时
        VM_Count.OpenDoorcount++;
      }
      if(VM_Count.MotoSwitch==ON){//电机计时
        VM_Count.Motocount++;
      }
      if(VM_Count.OpenShipmentDoorSwitch==ON){
        VM_Count.OpenShipmentDoorcount++;
      }
      if(VM_Count.CloseShipmentDoorSwitch==ON){
        VM_Count.CloseShipmentDoorcount++;
      }
      if(VM_Count.OpPassSwitch==ON){
        VM_Count.OpPasscount++;
      }
        // keyHandle((keysTypedef_t *)&keys);
        gizTimerMs();
    }
}

/**
* @brief Timer TIM3 init function

* @param none
* @return none
*/
void timerInit(void)
{
	HAL_TIM_Base_Start_IT(&htim2);
}

/**
  * @brief  This function handles USART IDLE interrupt.
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef*UartHandle)  
{  
    if(UartHandle->Instance == USART2)  
    {  
        gizPutData((uint8_t *)&aRxBuffer, 1);
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBuffer, 1);//开启下一次接收中断  
    }  
}  

/**
* @brief USART init function

* Serial communication between WiFi modules and device MCU
* @param none
* @return none
*/
void uartInit(void)
{
	HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBuffer, 1);//开启下一次接收中断  
}

/**
* @brief Serial port write operation, send data to WiFi module
*
* @param buf      : buf address
* @param len      : buf length
*
* @return : Return effective data length;-1，return failure
*/
int32_t uartWrite(uint8_t *buf, uint32_t len)
{
    uint8_t crc[1] = {0x55};
    uint32_t i = 0;
	
    if(NULL == buf)
    {
        return -1;
    }

    for(i=0; i<len; i++)
    {
        HAL_UART_Transmit_IT(&huart2, (uint8_t *)&buf[i], 1);
        while (huart2.gState != HAL_UART_STATE_READY);//Loop until the end of transmission

        if(i >=2 && buf[i] == 0xFF)
        {
          HAL_UART_Transmit_IT(&huart2, (uint8_t *)&crc, 1);
          while (huart2.gState != HAL_UART_STATE_READY);//Loop until the end of transmission
        }
    }

#ifdef PROTOCOL_DEBUG
    GIZWITS_LOG("MCU2WiFi[%4d:%4d]: ", gizGetTimerCount(), len);
    for(i=0; i<len; i++)
    {
        GIZWITS_LOG("%02x ", buf[i]);

        if(i >=2 && buf[i] == 0xFF)
        {
            GIZWITS_LOG("%02x ", 0x55);
        }
    }
    GIZWITS_LOG("\n");
#endif
    return len;
}  
