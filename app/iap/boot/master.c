#include "main.h"
#include "stm32f10x.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
/* Private variables ---------------------------------------------------------*/
pFunction Jump_To_Application;
uint32_t JumpAddress;
/* Private variables ---------------------------------------------------------*/  
static uint32_t Address = 0x00,StartAddr = 0x00,EndAddr = 0x00;
uint32_t WRPR_Value = 0xFFFFFFFF, ProtectedPages = 0x0,BlockNbr = 0,UserMemoryMask = 0;
uint32_t FlashDestination = ApplicationAddress; /* Flash user program offset */
static volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;
static volatile TestStatus MemoryProgramStatus = PASSED;

uint8_t Number_Char[]={"0123456789 .C"};

__IO uint32_t CRCValue=0xffffffff;

void IAP_Delayus(uint32_t us) 
{ 
    uint32_t  i, j; 
    for( i = us; i > 0; i-- ) 
    { 
        for( j = 72 ; j > 0 ; j-- ) 
        {} 
    } 
}

unsigned int CRC32(unsigned int Data)
{
	char i=0;
	for(i=0;i<32;i++)
	{
	   if((CRCValue^Data)&0x80000000)
	   {
	      CRCValue=0x04c11db7^(CRCValue<<1);
	   }
	   else
	   {
	      CRCValue<<=1;
	   }
       Data<<=1;
	}
return CRCValue;
}

void MASTER_USART_Init(void)
{          
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE|RCC_APB2Periph_AFIO, ENABLE);
	      /* Enable CRC clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);    
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE); 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = MENU_ESC_Pin;
	GPIO_Init(MENU_ESC_Port, &GPIO_InitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);	    

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = DMX_CS_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(DMX_CS_Port, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = USART_BPS;
    USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART2, &USART_InitStructure);  

    USART_Cmd(USART2, ENABLE);
	DMX512_Master;
}

void DMX_SendData(unsigned short UDR)
{
#if DMX_Enable
	while (USART_GetFlagStatus(DMX_UART,USART_FLAG_TXE) == RESET){}
	//S-CMD_S S1-CMD_S1 S2-CMD_S2;
	USART_SendData(DMX_UART, UDR);
#endif
}

void IAP_MASTER(void)
{
	uint8_t state=0;
	MASTER_USART_Init();
#if LCD_Enable
	LCM_12864_Init();
	LCM_Clr();
	LCM_Prints(0,0,IAP_Version);
	LCM_Prints(0,1,"  IAP...");
#endif
	
	while (1)
	{

	switch(state){
		case 0:
			DMX_SendData(CMD_IAP);
			IAP_Delayus(50);
			DMX_SendData(((uint8_t)(~CMD_IAP))|0x100);
			IAP_Delayus(50);
#if (S103_S1_S2_Enable||S107_S1_S2_Enable) 
			if(!GPIO_ReadInputDataBit(MENU_ESC_Port,MENU_ESC_Pin))state=1;
#else
			if(!GPIO_ReadInputDataBit(MENU_ESC_Port,MENU_ESC_Pin))state=2;
#endif
		break;
		case 1:
			DMX_SendData(CMD_S2);		
			CRCValue=0xffffffff;	
#if LCD_Enable
			LCM_Prints(10,1,"  ");
			LCM_Prints(0,2,IAP_Version_S2);
#endif	
#if (S103_S1_S2_Enable||S107_S1_S2_Enable)
			Address = BANK1_IAP_START_ADDR;
			StartAddr= BANK1_IAP_START_ADDR;
			EndAddr = BANK1_IAP_END_ADDR;
#endif
			state++;
		break;
		case 2:
			DMX_SendData(CMD_S1);
			CRCValue=0xffffffff;
#if LCD_Enable	
			LCM_Prints(10,1,"  ");
			LCM_Prints(0,2,IAP_Version_S1);
#endif	
			Address = BANK2_IAP_START_ADDR;
			StartAddr= BANK2_IAP_START_ADDR;
			EndAddr = BANK2_IAP_END_ADDR;
			state++;
		break;
		case 3:
			DMX_SendData(CMD_S);
			CRCValue=0xffffffff;
#if LCD_Enable	
			LCM_Prints(10,1,"  ");
			LCM_Prints(0,2,IAP_Version_S);
#endif	
			Address = BANK3_IAP_START_ADDR;
			StartAddr= BANK3_IAP_START_ADDR;
			EndAddr = BANK3_IAP_END_ADDR;
			state++;
		break;
		default:break;
	}
	if(state){
		while(Address < EndAddr)
		{
			DMX_SendData((uint8_t)((*(__IO uint32_t*) Address)>>0));
			DMX_SendData((uint8_t)((*(__IO uint32_t*) Address)>>8));
			DMX_SendData((uint8_t)((*(__IO uint32_t*) Address)>>16));
			DMX_SendData((uint8_t)((*(__IO uint32_t*) Address)>>24));
#if LCD_Enable
			if(Address%FLASH_PAGE_SIZE==0)LCM_Frame(((Address-StartAddr)*100)/(EndAddr-StartAddr));
#endif
			CRC32((*(__IO uint32_t*) Address));
			Address = Address + 4;
		}
	
		DMX_SendData((uint8_t)(CRCValue>>0));
		DMX_SendData((uint8_t)(CRCValue>>8));
		DMX_SendData((uint8_t)(CRCValue>>16));
		DMX_SendData((uint8_t)(CRCValue>>24));
		if(state>3){
			state=0;
#if LCD_Enable
			LCM_Frame(100);	
			LCM_Prints(10,1,"OK");
#endif		
		}
	}
	}
}


