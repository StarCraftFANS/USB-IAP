#include "main.h"
//72000 1ms
//WIFI-DMX.COM
#define ABS(x,y) ((x>y)?(x-y):(y-x))

Memory_TypeDef Memory_Struct=
{	
        0,0,127,127,127,127,127,127,127,127,127,127,0,1,0,0,_SlaveMode1,0,10,200,0,_2channel,1,1,0,0,0,1,0,0,_English,_Normal,_Auto_On,  
};

const uint8_t RxMark[MARK_NUM]="WIFI-DMX.COM IAP-START S1";
uint8_t RxBuf[255];

/**********************************************************************
* 名    称：IWDG_Configuration()
* 功    能：看门狗配置
* 入口参数： 
* 出口参数：
***********************************************************************/
void IWDG_Configuration(void)
{
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); 	//访问之前要首先使能寄存器写
  IWDG_SetPrescaler(IWDG_Prescaler_64);             //内部低速时钟16分频,即频率为：32.768K / 64 =  0.512K，所以一个周期为：1.9ms
  IWDG_SetReload(4000);							    //2000*1.9ms = 3.8S
  IWDG_ReloadCounter();								//喂狗程序。软件必须以一定的间隔写入0xAAAA，否则，当计数器为0时，看门狗会产生复位
  IWDG_Enable(); 									//使能
}


void Check_Flash(void)
 {
     FlagStatus status = RESET; 
        status = FLASH_GetReadOutProtectionStatus();
         if(status != SET)
         {
                 FLASH_Unlock();  /* Flash 解锁 */
                 /* ENABLE the ReadOut Protection */ 
                 FLASH_ReadOutProtection(ENABLE);          //读保护使能
                 //FLASH_EnableWriteProtection(FLASH_WRProt_AllPages);  //写保护使能
         }
 }

/*
 * 函数名：main
 * 描述  ：主函数
 * 输入  ：无
 * 输出  ：无
 */
int main(void)
{
//Check_Flash();
  SystemInit();  
  IAP_Wait();
}

/******************* (C) COPYRIGHT 2012 *****END OF FILE****/
