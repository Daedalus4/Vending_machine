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
#include "Vending_Machine.h"
#include "main.h"

Vending_Machine_Count  VM_Count;//设备计时的状态

Vending_Machine_InnerCtrl  VM_InnerCtrl;//设备心跳的状态
//VM_Control从平台接收
Vending_Machine_Control  VM_Control;//设备心跳的状态
//VM_Reply发送回平台
Vending_Machine_Reply  VM_Reply;//设备回复信息

void Vending_MachineInit(){
  VM_Count.LedSwitch=ON;
  VM_Count.HeartSwitch=ON;//开启心跳
  VM_InnerCtrl.ReplySwitch=ON;
  VM_InnerCtrl.SelfCheckPass=ON;//强制自检//0121测试修改为ON
  VM_InnerCtrl.InnerState=stand_by;
  VM_InnerCtrl.InnerOPState=OPsuccess;
  VM_InnerCtrl.Aisle=NoAisle;//默认1货道//0121修改
  VM_InnerCtrl.ErrorCode=NoError;
  //加一句语音
  //下面的打开激光可以关掉
  HAL_GPIO_WritePin(CTRL_AOLA_GPIO_Port, CTRL_AOLA_Pin, GPIO_PIN_SET);//打开5个激光
  HAL_GPIO_WritePin(CTRL_AO_DOOR_DE_GPIO_Port, CTRL_AO_DOOR_DE_Pin, GPIO_PIN_SET);//打开取货门激光
  HAL_GPIO_WritePin(CTRL_LIGHT_GPIO_Port, CTRL_LIGHT_Pin, GPIO_PIN_SET);//打开广告光
}

void do_selfcheck(){//自检过程中必须全程无错才能执行下一步
      Speakaword(In_selfcheck);
      
      HAL_GPIO_WritePin(MOTO1_GPIO_Port, MOTO1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(MOTO2_GPIO_Port, MOTO2_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(MOTO3_GPIO_Port, MOTO3_Pin, GPIO_PIN_SET);
      HAL_Delay(1000);
      HAL_GPIO_WritePin(MOTO1_GPIO_Port, MOTO1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MOTO2_GPIO_Port, MOTO2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MOTO3_GPIO_Port, MOTO3_Pin, GPIO_PIN_RESET);
      
  if(VM_InnerCtrl.DoSelfCheck==ON){
    VM_InnerCtrl.SelfCheckPass=ON;
    //第一步：有货检测(注意报错)在循环中只执行一次
    VM_InnerCtrl.LazerCheckState=ON;
    VM_InnerCtrl.LazerCheckFinish=ON;//0121测试修改为ON

    if(VM_InnerCtrl.LazerCheckState==OFF&&VM_InnerCtrl.SelfCheckPass==OFF){
      VM_InnerCtrl.InnerOPState=OPrunning;//开始了就报OPrunning
      Speakaword(In_selfcheck);
      VM_Count.HeartSwitch=OFF;//开始自检后关闭心跳
      VM_InnerCtrl.LazerCheckState=ON;
      HAL_GPIO_WritePin(CTRL_AOLA_GPIO_Port, CTRL_AOLA_Pin, GPIO_PIN_SET);//打开5个激光
      HAL_Delay(10);
      if(HAL_GPIO_ReadPin(LAST_DE_1_IN_GPIO_Port,LAST_DE_1_IN_Pin)==GPIO_PIN_SET){
        //1货道无货
        VM_InnerCtrl.InnerOPState=OPfailed;
        Speakaword(Error_pleace_check);
        if(VM_InnerCtrl.NoGoodsAisle==NoAisle)
          VM_InnerCtrl.NoGoodsAisle=Aisle1;
        else{
          switch (VM_InnerCtrl.NoGoodsAisle){
            case Aisle1:VM_InnerCtrl.NoGoodsAisle=Aisle1;break;
            case Aisle2:VM_InnerCtrl.NoGoodsAisle=Aisle12;break;
            case Aisle3:VM_InnerCtrl.NoGoodsAisle=Aisle13;break;
            case Aisle12:VM_InnerCtrl.NoGoodsAisle=Aisle12;break;
            case Aisle13:VM_InnerCtrl.NoGoodsAisle=Aisle13;break;
            case Aisle23:VM_InnerCtrl.NoGoodsAisle=Aisle123;break;
          }
        }
        VM_InnerCtrl.ErrorCode=NoGoods;
        //在这里检测一下电池
      }
      if(HAL_GPIO_ReadPin(LAST_DE_2_IN_GPIO_Port,LAST_DE_2_IN_Pin)==GPIO_PIN_SET){
        //2货道无货
        VM_InnerCtrl.InnerOPState=OPfailed;
        Speakaword(Error_pleace_check);
        if(VM_InnerCtrl.NoGoodsAisle==NoAisle)
          VM_InnerCtrl.NoGoodsAisle=Aisle2;
        else{
          switch (VM_InnerCtrl.NoGoodsAisle){
            case Aisle1:VM_InnerCtrl.NoGoodsAisle=Aisle12;break;
            case Aisle2:VM_InnerCtrl.NoGoodsAisle=Aisle2;break;
            case Aisle3:VM_InnerCtrl.NoGoodsAisle=Aisle23;break;
            case Aisle12:VM_InnerCtrl.NoGoodsAisle=Aisle12;break;
            case Aisle13:VM_InnerCtrl.NoGoodsAisle=Aisle123;break;
            case Aisle23:VM_InnerCtrl.NoGoodsAisle=Aisle23;break;
          }
        }
        VM_InnerCtrl.ErrorCode=NoGoods;
        //在这里检测一下电池
      }
      if(HAL_GPIO_ReadPin(LAST_DE_3_IN_GPIO_Port,LAST_DE_3_IN_Pin)==GPIO_PIN_SET){
        //3货道无货
        VM_InnerCtrl.InnerOPState=OPfailed;
        Speakaword(Error_pleace_check);
        if(VM_InnerCtrl.NoGoodsAisle==NoAisle)
          VM_InnerCtrl.NoGoodsAisle=Aisle3;
        else{
          switch (VM_InnerCtrl.NoGoodsAisle){
            case Aisle1:VM_InnerCtrl.NoGoodsAisle=Aisle13;break;
            case Aisle2:VM_InnerCtrl.NoGoodsAisle=Aisle23;break;
            case Aisle3:VM_InnerCtrl.NoGoodsAisle=Aisle3;break;
            case Aisle12:VM_InnerCtrl.NoGoodsAisle=Aisle123;break;
            case Aisle13:VM_InnerCtrl.NoGoodsAisle=Aisle13;break;
            case Aisle23:VM_InnerCtrl.NoGoodsAisle=Aisle23;break;
          }
        }
        VM_InnerCtrl.ErrorCode=NoGoods;
        //在这里检测一下电池
      }
      if(VM_InnerCtrl.ErrorCode==NoError)//没有报错则进行下一步
        VM_InnerCtrl.LazerCheckFinish=ON;
      HAL_GPIO_WritePin(CTRL_AOLA_GPIO_Port, CTRL_AOLA_Pin, GPIO_PIN_RESET);//关闭5个激光
    }
     VM_InnerCtrl.LazerCheckFinish=ON;
    VM_InnerCtrl.OpenStepMotoFinish=ON;
    //第三步：开电机，只在循环中只执行一次
    if(VM_InnerCtrl.MotoState==OFF&&VM_InnerCtrl.OpenStepMotoFinish==ON){
      VM_InnerCtrl.MotoState=ON;
      VM_Count.MotoSwitch=ON;
      
      switch(VM_InnerCtrl.SelfCheckAisle){//开启电机
        case NoAisle:HAL_GPIO_WritePin(MOTO1_GPIO_Port, MOTO1_Pin, GPIO_PIN_SET);break;
        case Aisle1:HAL_GPIO_WritePin(MOTO2_GPIO_Port, MOTO2_Pin, GPIO_PIN_SET);break;
        case Aisle12:HAL_GPIO_WritePin(MOTO3_GPIO_Port, MOTO3_Pin, GPIO_PIN_SET);break;
      }
    }
    if(VM_Count.Motocount>1800){//开始检测并停止电机，在循环中执行多次
      uint8_t MotoSensorState=0;
      switch(VM_InnerCtrl.SelfCheckAisle){
        case NoAisle:MotoSensorState=HAL_GPIO_ReadPin(MOTO1_SENSOR_GPIO_Port,MOTO1_SENSOR_Pin);break;
        case Aisle1:MotoSensorState=HAL_GPIO_ReadPin(MOTO1_SENSOR_GPIO_Port,MOTO1_SENSOR_Pin);break;
        case Aisle12:MotoSensorState=HAL_GPIO_ReadPin(MOTO1_SENSOR_GPIO_Port,MOTO1_SENSOR_Pin);break;
      }
      if(MotoSensorState==GPIO_PIN_SET){//旋转到位了吗，在循环中执行多次
        VM_Count.MotoSwitch=OFF;
        VM_Count.Motocount=0;
        
        switch(VM_InnerCtrl.SelfCheckAisle){//关机
          case NoAisle:HAL_GPIO_WritePin(MOTO1_GPIO_Port, MOTO1_Pin, GPIO_PIN_RESET);break;
          case Aisle1:HAL_GPIO_WritePin(MOTO2_GPIO_Port, MOTO2_Pin, GPIO_PIN_RESET);break;
          case Aisle12:HAL_GPIO_WritePin(MOTO3_GPIO_Port, MOTO3_Pin, GPIO_PIN_RESET);break;
        }
        if(VM_InnerCtrl.ErrorCode==NoError)//没有报错则进行下一步
          VM_InnerCtrl.MotoFinish=ON;//电机旋转一圈执行完成
      }
      else if(VM_Count.Motocount>3000){//电机坏了
        HAL_GPIO_WritePin(MOTO1_GPIO_Port, MOTO1_Pin, GPIO_PIN_RESET);//关闭电机
        VM_Count.MotoSwitch=OFF;
        VM_Count.Motocount=0;
        VM_InnerCtrl.InnerOPState=OPfailed;
        Speakaword(Error_pleace_check);
        if(VM_InnerCtrl.ErrorCode==NoError)//没有报错则进行下一步
          VM_InnerCtrl.ErrorCode=MotoError;
        //在这里检测一下电池
      }
    }
         VM_InnerCtrl.CloseStepMotoState=ON;
         VM_InnerCtrl.CloseStepMotoFinish=ON;
         VM_InnerCtrl.DoorLazerCheckState=ON;
         VM_InnerCtrl.DoorLazerCheckFinish=ON;
    //第六步：开取货门(延时报错)，只在循环中只执行一次
      VM_Count.OpenShipmentDoorSwitch=ON;
      VM_InnerCtrl.ShipmentDoorState=ON;
      VM_InnerCtrl.ShipmentDoorFinish=ON;
      
      
    if(VM_InnerCtrl.ShipmentFinish==OFF&&VM_InnerCtrl.ShipmentDoorFinish==ON){//取完货了要判断取货门是否关好，执行多次
      VM_Count.CloseShipmentDoorSwitch=ON;
      if(1){//对准10次以上        
        VM_InnerCtrl.InnerOPState=OPsuccess;   
        VM_Count.CloseShipmentDoorComplete=0;
        //先把这些个复位了先
        VM_Count.CloseShipmentDoorSwitch=OFF;
        VM_Count.CloseShipmentDoorcount=0;
        VM_InnerCtrl.LazerCheckState=OFF;
        VM_InnerCtrl.LazerCheckFinish=OFF;
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
      }
    }
          VM_Count.CloseShipmentDoorSwitch=ON;
            VM_InnerCtrl.InnerOPState=OPsuccess;
    
    
    if(1){//自检过后两秒
      VM_InnerCtrl.DoSelfCheck=OFF;//关闭自检开关不再执行DoselfCheck函数
      VM_InnerCtrl.InnerState=stand_by;
      VM_InnerCtrl.Aisle=NoAisle;
      VM_InnerCtrl.ErrorCode=NoError;
      // VM_InnerCtrl.BatteryCapacity=0x00;//这里检测一下电池电量
      VM_Count.HeartSwitch=ON;//自检完毕开启心跳。
      HAL_GPIO_WritePin(MOTO1_GPIO_Port, MOTO1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MOTO2_GPIO_Port, MOTO2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MOTO3_GPIO_Port, MOTO3_Pin, GPIO_PIN_RESET);
    }
  }
  
  
}
void do_shipment(){
  Speakaword(In_shipment);
  //if(VM_InnerCtrl.DoShipment==ON&&VM_InnerCtrl.SelfCheckPass==ON){//出货和自检都通过才行
    VM_InnerCtrl.StartCheckFinish=ON;
    VM_InnerCtrl.StartCheckState=ON;//0121测试修改为ON
    VM_Count.HeartSwitch=OFF;

    //第一步：开始检测，门锁激光、门锁开关、防盗板开关
    if(1){
      VM_InnerCtrl.InnerOPState=OPrunning;//开始了就报OPrunning
      VM_Count.HeartSwitch=OFF;//开始自检后关闭心跳
      VM_InnerCtrl.StartCheckState=ON;
      VM_InnerCtrl.StartCheckFinish=ON;
    VM_InnerCtrl.OpenStepMotoState=ON;
    VM_InnerCtrl.OpenStepMotoFinish=ON;
    //第三步：开电机，只在循环中只执行一次
    if(VM_InnerCtrl.MotoState==OFF&&VM_InnerCtrl.OpenStepMotoFinish==ON){
      VM_InnerCtrl.MotoState=ON;
      VM_Count.MotoSwitch=ON;
      switch(VM_InnerCtrl.Aisle){//开启对应电机
        case Aisle1:HAL_GPIO_WritePin(MOTO1_GPIO_Port, MOTO1_Pin, GPIO_PIN_SET);break;
        case Aisle2:HAL_GPIO_WritePin(MOTO2_GPIO_Port, MOTO2_Pin, GPIO_PIN_SET);break;
        case Aisle3:HAL_GPIO_WritePin(MOTO3_GPIO_Port, MOTO3_Pin, GPIO_PIN_SET);break;
      }
      Speakaword(Please_take_the_goods);
      HAL_Delay(1000);
      HAL_GPIO_WritePin(MOTO1_GPIO_Port, MOTO1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MOTO2_GPIO_Port, MOTO2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MOTO3_GPIO_Port, MOTO3_Pin, GPIO_PIN_RESET);
        stepmoto(OFF,1000);
        HAL_Delay(500);
        stepmoto(ON,1000);
        HAL_Delay(500);
    }
    if(VM_Count.Motocount>500){//开始检测并停止电机，在循环中执行多次
      uint8_t MotoSensorState=0;
      switch(VM_InnerCtrl.Aisle){
        case Aisle1:MotoSensorState=GPIO_PIN_SET;break;
        case Aisle2:MotoSensorState=GPIO_PIN_SET;break;
        case Aisle3:MotoSensorState=GPIO_PIN_SET;break;
      }
      
      if(VM_Count.Motocount>3000){//电机坏了
        HAL_GPIO_WritePin(MOTO1_GPIO_Port, MOTO1_Pin, GPIO_PIN_RESET);//关闭电机
        HAL_GPIO_WritePin(MOTO2_GPIO_Port, MOTO2_Pin, GPIO_PIN_RESET);//关闭电机
        HAL_GPIO_WritePin(MOTO3_GPIO_Port, MOTO3_Pin, GPIO_PIN_RESET);//关闭电机
        VM_Count.MotoSwitch=OFF;
        VM_Count.Motocount=0;
        VM_InnerCtrl.InnerOPState=OPfailed;
        Speakaword(Sorry);
        if(VM_InnerCtrl.ErrorCode==NoError)//没有报错则进行下一步
          VM_InnerCtrl.ErrorCode=MotoError;
        //在这里检测一下电池
      }
      if(MotoSensorState==GPIO_PIN_SET){//旋转到位了吗，在循环中执行多次
        switch(VM_InnerCtrl.Aisle){//关机
          case Aisle1:HAL_GPIO_WritePin(MOTO1_GPIO_Port, MOTO1_Pin, GPIO_PIN_RESET);break;
          case Aisle2:HAL_GPIO_WritePin(MOTO2_GPIO_Port, MOTO2_Pin, GPIO_PIN_RESET);break;
          case Aisle3:HAL_GPIO_WritePin(MOTO3_GPIO_Port, MOTO3_Pin, GPIO_PIN_RESET);break;
        }
        VM_Count.MotoSwitch=OFF;
        VM_Count.Motocount=0;
        VM_InnerCtrl.ErrorCode=NoError;
        if(VM_InnerCtrl.ErrorCode==NoError)//没有报错则进行下一步
          VM_InnerCtrl.MotoFinish=ON;//电机旋转一圈执行完成
      }
    }
    VM_InnerCtrl.CloseStepMotoState=ON;
     VM_InnerCtrl.CloseStepMotoFinish=ON;
     VM_InnerCtrl.DoorLazerCheckState=ON;
    VM_InnerCtrl.DoorLazerCheckFinish=ON;
    //第六步：开取货门(延时报错)，只在循环中只执行一次
    if(VM_InnerCtrl.ShipmentDoorState==OFF&&VM_InnerCtrl.DoorLazerCheckFinish==ON){
      VM_Count.OpenShipmentDoorSwitch=ON;
      VM_InnerCtrl.ShipmentDoorState=ON;
      //HAL_GPIO_WritePin(E_LOCK3_GPIO_Port, E_LOCK3_Pin, GPIO_PIN_SET);//打开取货门电磁锁
      //Speakaword(Please_take_the_goods);
      //HAL_GPIO_WritePin(CTRL_AOLA_GPIO_Port, CTRL_AOLA_Pin, GPIO_PIN_SET);//打开5个激光
    }
    VM_InnerCtrl.ShipmentDoorFinish=ON;
    if(VM_InnerCtrl.ShipmentFinish==OFF&&VM_InnerCtrl.ShipmentDoorFinish==ON){//取完货了要判断取货门是否关好，执行多次
      VM_Count.CloseShipmentDoorSwitch=ON;

      if(1){//对准10次以上表示关好门了且出货成功
        VM_Count.CloseShipmentDoorComplete=0;
        VM_InnerCtrl.SelfCheckAisle=NoAisle;
        VM_InnerCtrl.InnerOPState=OPsuccess;//出货操作成功
        VM_InnerCtrl.DoShipment=OFF;//关闭出货开关
        //Speakaword(Thank_you);
        //语音：出货成功
        //2S之后进入待机模式
        VM_Count.OpPassSwitch=ON;//保证不用再进自检模式了
        //先把这些个复位了先
        VM_Count.CloseShipmentDoorSwitch=OFF;
        VM_Count.CloseShipmentDoorcount=0;

//        VM_InnerCtrl.LazerCheckState=OFF;
//        VM_InnerCtrl.LazerCheckFinish=OFF;
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
      }
    }
  }
}

void do_opendoor(){//只要开了门则必定要自检，需考虑一下
  Speakaword(Please_close_the_door);
  if(VM_InnerCtrl.DoOpenDoor==ON){
    if(VM_Count.OpenDoorSwitch==OFF){//第一步，开锁，只执行一次
      VM_Count.OpenDoorSwitch=ON;
      HAL_GPIO_WritePin(E_LOCK1_GPIO_Port, E_LOCK1_Pin, GPIO_PIN_SET);
    }
    if(VM_Count.OpenDoorcount>50){//第二步，开锁时间大于3S，关锁，只执行一次
      // VM_InnerCtrl.InnerState=stand_by;//0108可疑注释掉
      HAL_GPIO_WritePin(E_LOCK1_GPIO_Port, E_LOCK1_Pin, GPIO_PIN_RESET);
      VM_Count.OpenDoorSwitch=OFF;//关闭计时与计数
      VM_Count.OpenDoorcount=0;
      //写回复语句
      VM_InnerCtrl.InnerState=opendoor;
      VM_InnerCtrl.InnerOPState=OPsuccess;
      VM_InnerCtrl.Aisle=NoAisle;
      VM_InnerCtrl.ErrorCode=NoError; 
      //加一句检测电池
      for(int i=0;i<9;i++)
        VM_InnerCtrl.Ordernumber[i]=VM_Control.Ordernumber[i];
      VM_InnerCtrl.DoOpenDoor=OFF;//DoOpenDoor执行完毕
    }
    VM_InnerCtrl.InnerState=stand_by;
  }
}
void do_shutdown(){
         Speakaword(Thank_you);
         VM_InnerCtrl.InnerState=stand_by;
  //if(VM_InnerCtrl.DoOpenDoor==ON){
   // ;//暂时什么也没写
  //}
}
void do_zhuce(){
  Speakaword(selfcheck_complete);
}
#define STEPTIME 6
void stepmoto(uint8_t dir,uint8_t count){
  if(dir==ON){
    //这里是正转的代码
    for(int i=0;i<count;i++){
      MOTO1_D(GPIO_PIN_RESET);
      MOTO1_A(GPIO_PIN_SET);//A
      HAL_Delay(STEPTIME);
      MOTO1_A(GPIO_PIN_RESET);
      MOTO1_B(GPIO_PIN_SET);//B
      HAL_Delay(STEPTIME);
      MOTO1_B(GPIO_PIN_RESET);
      MOTO1_C(GPIO_PIN_SET);//C
      HAL_Delay(STEPTIME);
      MOTO1_C(GPIO_PIN_RESET);
      MOTO1_D(GPIO_PIN_SET);//D
      HAL_Delay(STEPTIME);
     }
  }
  else{
    //这里是反转的代码
    for(int i=0;i<count;i++){
      MOTO1_A(GPIO_PIN_RESET);
      MOTO1_D(GPIO_PIN_SET);//D
      HAL_Delay(STEPTIME);
      MOTO1_D(GPIO_PIN_RESET);
      MOTO1_C(GPIO_PIN_SET);//C
      HAL_Delay(STEPTIME);
      MOTO1_C(GPIO_PIN_RESET);
      MOTO1_B(GPIO_PIN_SET);//B
      HAL_Delay(STEPTIME);
      MOTO1_B(GPIO_PIN_RESET);
      MOTO1_A(GPIO_PIN_SET);//A
      HAL_Delay(STEPTIME);
     }
  }

  MOTO1_A(GPIO_PIN_RESET);
  MOTO1_B(GPIO_PIN_RESET);
  MOTO1_C(GPIO_PIN_RESET);
  MOTO1_D(GPIO_PIN_RESET);
}
void Speakaword(Vending_Machine_Speakaword Speakaword){
  uint8_t speakmp3[6]={0xaa,0x07,0x02,0x00,0x01,0xb4};
  speakmp3[4]+=Speakaword;
  speakmp3[5]+=Speakaword;
  HAL_UART_Transmit(&huart3,&speakmp3[0],1,999);
  HAL_UART_Transmit(&huart3,&speakmp3[1],1,999);
  HAL_UART_Transmit(&huart3,&speakmp3[2],1,999);
  HAL_UART_Transmit(&huart3,&speakmp3[3],1,999);
  HAL_UART_Transmit(&huart3,&speakmp3[4],1,999);
  HAL_UART_Transmit(&huart3,&speakmp3[5],1,999);
}
