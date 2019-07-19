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
* ��    �ƣ�IWDG_Configuration()
* ��    �ܣ����Ź�����
* ��ڲ����� 
* ���ڲ�����
***********************************************************************/
void IWDG_Configuration(void)
{
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); 	//����֮ǰҪ����ʹ�ܼĴ���д
  IWDG_SetPrescaler(IWDG_Prescaler_64);             //�ڲ�����ʱ��16��Ƶ,��Ƶ��Ϊ��32.768K / 64 =  0.512K������һ������Ϊ��1.9ms
  IWDG_SetReload(4000);							    //2000*1.9ms = 3.8S
  IWDG_ReloadCounter();								//ι���������������һ���ļ��д��0xAAAA�����򣬵�������Ϊ0ʱ�����Ź��������λ
  IWDG_Enable(); 									//ʹ��
}


void Check_Flash(void)
 {
     FlagStatus status = RESET; 
        status = FLASH_GetReadOutProtectionStatus();
         if(status != SET)
         {
                 FLASH_Unlock();  /* Flash ���� */
                 /* ENABLE the ReadOut Protection */ 
                 FLASH_ReadOutProtection(ENABLE);          //������ʹ��
                 //FLASH_EnableWriteProtection(FLASH_WRProt_AllPages);  //д����ʹ��
         }
 }

/*
 * ��������main
 * ����  ��������
 * ����  ����
 * ���  ����
 */
int main(void)
{
//Check_Flash();
  SystemInit();  
  IAP_Wait();
}

/******************* (C) COPYRIGHT 2012 *****END OF FILE****/
