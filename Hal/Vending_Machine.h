/**
************************************************************
* @file         gizwits_product.h
* @brief        Corresponding gizwits_product.c header file (including product hardware and software version definition)
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
/*
2019.03.15更改，----后为更改内容，适配于新设备，新添注释必在----后，新添代码已注明
*/

#ifndef _Vending_Machine_H
#define _Vending_Machine_H

#include "common.h"

#define ON  1
#define OFF 0

#define _2GAutoPowerOn 1

#define OpenSecDoorCount 120
#define CloseSecDoorCount 160

typedef enum{
  stand_by=0x0f,//待机
  stand_by1=0x0e,//待机
  selfcheck=1,//自检
  shipment=2,//出货
  opendoor=3,//开门
  shutdown=4,//关机
  clearallstate=5//清除所有状态
}Vending_Machine_State;//设备状态

typedef enum{
  welcome=0,//欢迎使用
  In_shipment,//正在出货中
  Please_take_the_goods,//请取货
  Please_close_the_door,//请关好箱门
  Thank_you,//谢谢
  Sorry,//抱歉
  In_selfcheck,//正在自检
  selfcheck_complete,//自检完成
  Error_pleace_check,//出错
}Vending_Machine_Speakaword;//设备状态

typedef enum{
  OPsuccess=1,//操作成功
  OPrunning,//正在操作中
  OPfailed//操作失败
}Vending_Machine_OPState;//操作状态

typedef enum{
  NoAisle=0,//
  Aisle1,//
  Aisle2,//
  Aisle3,//
  Aisle12,//
  Aisle13,//
  Aisle23,//
  Aisle123,//
}Vending_Machine_Aisle;//操作状态

typedef enum{
  NoError=0,//无异常
  NoGoods,//无货
  MotoError,//电机异常
  ShieldError,//防盗板异常
  ShipmentError,//出货异常
  UnShipedError,//未取货
  ShipDoorError,//取货门异常
}Vending_Machine_ErrorCode;//错误码

typedef struct{//执行完必须置0
    uint8_t HeartSwitch;//
    long Heartcount;//

    uint8_t LedSwitch;//
    int Ledcount;//

    uint8_t OpenDoorSwitch;//
    int OpenDoorcount;//

    uint8_t MotoSwitch;//
    int Motocount;//

    uint8_t OpenShipmentDoorSwitch;//
    long OpenShipmentDoorcount;//

    uint8_t CloseShipmentDoorSwitch;//
    long CloseShipmentDoorcount;//
    uint8_t UnCloseShipmentDoorcount;//

    uint8_t CloseShipmentDoorComplete;//

    uint8_t OpPassSwitch;//自检通过开关
    int OpPasscount;//自检通过开关
    
}Vending_Machine_Count;//各种计时器开关

typedef enum{
  PerLazerCheckStage=0,//第一步：预先检查保证有货
  OpenStepMotoStage,//第二步：打开防盗板锁并开启防盗板----检查x轴电机初始位置
  MotoStage,//第三步：打开出货电机----开启x轴电机，在预定货道停止
  CloseStepMotoStage,//第四步：打开防盗板锁并关闭防盗板----开启y轴电机取货
  DropLazerCheckStage,//第五步：检测物品是否掉落----检测取货成功，x轴电机回到初始位置
  WaitShipmentTakeOffStage,//第六步:等待货物被取走----释放货物
  WaitShipmentDoorCloseStage,//第七步：等待取货门关闭----无
  WaitForStandBy//第八步：检测货道是否有货并上报无货状态并进入待机
}Vending_Machine_StepStage;//步骤码

typedef struct{
    Vending_Machine_State InnerState;//
    Vending_Machine_OPState InnerOPState;//
    Vending_Machine_Aisle Aisle;//货道
    Vending_Machine_Aisle SelfCheckAisle;//货道
    Vending_Machine_Aisle NoGoodsAisle;//货道
    Vending_Machine_StepStage CurrentStage;//当前步骤
    uint8_t ErrorCode;//错误码
    uint8_t BatteryCapacity;//电池电量
    uint8_t Ordernumber[9];//

    uint8_t DoSelfCheck;//控制SelfCheck代码执行的开关
    uint8_t DoShipment;//控制Shipment代码执行的开关
    uint8_t DoOpenDoor;//控制OpenDoor代码执行的开关
    uint8_t DoShutdown;//控制Shutdown代码执行的开关
    uint8_t DoZhuce;
    uint8_t SelfCheckPass;//自检通过开关
    uint8_t ReplySwitch;//回复开关
    //操作步骤开关
    uint8_t LazerCheckState;//第一步：激光检测----红外检测初始位置
    uint8_t LazerCheckFinish;//第一步：激光检测----红外检测初始位置
    uint8_t StartCheckState;//第一步：开卖检测
    uint8_t StartCheckFinish;//第一步：开卖检测
    uint8_t OpenStepMotoState;//第二步：开步进电机----启动x轴步进电机
    uint8_t OpenStepMotoFinish;//第二步：开步进电机----启动x轴步进电机
    uint8_t MotoState;//第三步：开电机
    uint8_t MotoFinish;//第三步：电机转完
    uint8_t CloseStepMotoState;//第四步：关步进电机
    uint8_t CloseStepMotoFinish;//第四步：关步进电机
    uint8_t DoorLazerCheckState;//第五步：掉落检测(注意报错)
    uint8_t DoorLazerCheckFinish;//第五步：掉落检测(注意报错)
    uint8_t ShipmentDoorState;//第五步：取货门检测(注意报错)
    uint8_t ShipmentDoorFinish;//第五步：取货门检测(注意报错)
    uint8_t ShipmentFinish;//第六步：取货(注意延时报错)
    uint8_t ShipmentFinishLazerCheck;//第七步：取货(注意延时报错)
} Vending_Machine_InnerCtrl;//内部状态
  
typedef struct{
    Vending_Machine_State ControlState;//
    Vending_Machine_Aisle Aisle;//
    uint8_t Ordernumber[9];//
    uint8_t CkeckSum;//
} Vending_Machine_Control;//设备接收的控制信息

typedef struct{
    Vending_Machine_State ReplyState;//
    Vending_Machine_OPState ReplyOPState;//
    Vending_Machine_Aisle Aisle;//
    uint8_t ErrorCode;//
    uint8_t BatteryCapacity;//
    uint8_t Ordernumber[9];//
}Vending_Machine_Reply;//设备回复信息

extern Vending_Machine_Count  VM_Count;//设备计时的状态

extern Vending_Machine_InnerCtrl  VM_InnerCtrl;//设备的状态
//VM_Control从平台接收
extern Vending_Machine_Control  VM_Control;//设备心跳的状态
//VM_Reply发送回平台
extern Vending_Machine_Reply  VM_Reply;//设备回复信息

extern uint8_t V_Mcountdownms;//倒计时时间
extern uint8_t V_Mcountdownmsstate;//倒计时开关

void Vending_MachineInit();//开机代码
void do_selfcheck();//自检
void do_shipment();//出货
void do_opendoor();//开门
void do_shutdown();//关机
void do_zhuce();//注册
void stepmoto(uint8_t dir,uint8_t count);//步进电机
void Speakaword(Vending_Machine_Speakaword Speakaword);//播放声音

#endif /*_Vending_Machine_H_*/