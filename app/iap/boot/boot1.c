#include "boot1.h"
#include "fifo.h"
#include "main.h"
#if M0518_Enable
#include "M0518.h"
#include "ledpwm.h"	

//WIFI-DMX.COM

typedef  void (*pFunction)(void);
pFunction Jump_To_Application;
uint32_t JumpAddress;
#define ApplicationAddress    (FMC_APROM_BASE+START_ADDR)
volatile unsigned int CRCValue=0xffffffff,uwCRCValue1=0,Data=0,Address=0,END_ADDR=0;
const uint8_t Number_Char[]={"0123456789 .C"};

__asm void __set_MS(uint32_t mainStackPointer)
{
  msr msp, r0
  bx lr
}

static int SetIAPBoot(void)
{
    uint32_t  au32Config[2];
    uint32_t u32CBS;

    /* Read current boot mode */
    u32CBS = (FMC->ISPSTA & FMC_ISPSTA_CBS_Msk) >> FMC_ISPSTA_CBS_Pos;
    if(u32CBS & 1)
    {
        /* Modify User Configuration when it is not in IAP mode */

        FMC_ReadConfig(au32Config, 2);
        if(au32Config[0] & 0x40)
        {
            FMC_EnableConfigUpdate();
            au32Config[0] &= ~0x40;
            FMC_Erase(FMC_CONFIG_BASE);
            FMC_WriteConfig(au32Config, 2);

            // Perform chip reset to make new User Config take effect
            SYS_ResetChip();
        }
    }
    return 0;
}

void M0_PwmDisp(void)
{
	//IO口功能选择
	/*
	*  PWM0_CH0---PA12 PWM1_CH0---PA2	BPWM0_CH0---PC0	  BPWM1_CH0---PD7
	*  PWM0_CH1---PA13 PWM1_CH1---PA3	BPWM0_CH1---PC1	  BPWM1_CH1---PD6
	*  PWM0_CH2---PA14 PWM1_CH2---PA10	BPWM0_CH2---PC2	  BPWM1_CH2---PB8
	*  PWM0_CH3---PA15 PWM1_CH3---PA11	BPWM0_CH3---PC3	  BPWM1_CH3---PB12
	*  PWM0_CH4---PA0  PWM1_CH4---PF4	BPWM0_CH4---PD15  BPWM1_CH4---PF8
	*  PWM0_CH5---PA1  PWM1_CH5---PF5	BPWM0_CH5---PD14  BPWM1_CH5---PB15
	*/
	
//	SYS->GPD_MFP |= ( SYS_GPD_MFP_PD15_BPWM0_CH4| SYS_GPD_MFP_PD14_BPWM0_CH5);

	SYS->GPC_MFP |= ( SYS_GPC_MFP_PC0_BPWM0_CH0| SYS_GPC_MFP_PC1_BPWM0_CH1| SYS_GPC_MFP_PC2_BPWM0_CH2| SYS_GPC_MFP_PC3_BPWM0_CH3 );	
			  	 
	SYS->ALT_MFP|= ( 
		SYS_ALT_MFP_PC0_BPWM0_CH0 |SYS_ALT_MFP_PC1_BPWM0_CH1 |SYS_ALT_MFP_PC2_BPWM0_CH2 | 				 //BPWM0
		SYS_ALT_MFP_PC3_BPWM0_CH3 );	  
	
	SYS->ALT_MFP2|= (
		SYS_ALT_MFP2_PC0_BPWM0_CH0 |SYS_ALT_MFP2_PC1_BPWM0_CH1 |SYS_ALT_MFP2_PC2_BPWM0_CH2 | 				 //BPWM0
		SYS_ALT_MFP2_PC3_BPWM0_CH3 );	  
	
	SYS->ALT_MFP3|= ( 
		SYS_ALT_MFP3_PC0_BPWM0_CH0 |SYS_ALT_MFP3_PC1_BPWM0_CH1 |SYS_ALT_MFP3_PC2_BPWM0_CH2 | 				 //BPWM0
		SYS_ALT_MFP3_PC3_BPWM0_CH3 );	  
	
	SYS->ALT_MFP4|= ( 
		SYS_ALT_MFP4_PC0_BPWM0_CH0 |SYS_ALT_MFP4_PC1_BPWM0_CH1 |SYS_ALT_MFP4_PC2_BPWM0_CH2 | 				 //BPWM0
		SYS_ALT_MFP4_PC3_BPWM0_CH3 );	  	
	 
	 //BPWM0 Group( 0, 1, 2, 3, 4, 5 )
     CLK_EnableModuleClock(BPWM0_MODULE);
//   CLK_SetModuleClock(BPWM0_MODULE, CLK_CLKSEL3_BPWM0_S_PLL, 0);	 //Select PWM module clock source
     CLK->CLKSEL3 &= ~CLK_CLKSEL3_BPWM0_S_PCLK;
     SYS_ResetModule(BPWM0_RST); 									  //Reset BPWM0                                    

     BPWM_SetOutputChannel(BPWM0,0,XPWM_CNR_VAL_SET,XPWM_INIT_SET,SWITCH_SET,0);
	 BPWM_SetOutputChannel(BPWM0,1,XPWM_CNR_VAL_SET,XPWM_INIT_SET,SWITCH_SET,0);
	 BPWM_SetOutputChannel(BPWM0,2,XPWM_CNR_VAL_SET,XPWM_INIT_SET,SWITCH_SET,0);
	 BPWM_SetOutputChannel(BPWM0,3,XPWM_CNR_VAL_SET,XPWM_INIT_SET,SWITCH_SET,0);

	 BPWM_EnableOutput( BPWM0, BPWM_CH_0_MASK|BPWM_CH_1_MASK | BPWM_CH_2_MASK | BPWM_CH_3_MASK );
	 
     BPWM_Start( BPWM0, BPWM_CH_0_MASK| BPWM_CH_1_MASK | BPWM_CH_2_MASK | BPWM_CH_3_MASK );	 

}

void SW300P_PwmDisp(void)
{
	//IO口功能选择
	/*
	*  PWM0_CH0---PA12 PWM1_CH0---PA2	BPWM0_CH0---PC0	  BPWM1_CH0---PD7
	*  PWM0_CH1---PA13 PWM1_CH1---PA3	BPWM0_CH1---PC1	  BPWM1_CH1---PD6
	*  PWM0_CH2---PA14 PWM1_CH2---PA10	BPWM0_CH2---PC2	  BPWM1_CH2---PB8
	*  PWM0_CH3---PA15 PWM1_CH3---PA11	BPWM0_CH3---PC3	  BPWM1_CH3---PB12
	*  PWM0_CH4---PA0  PWM1_CH4---PF4	BPWM0_CH4---PD15  BPWM1_CH4---PF8
	*  PWM0_CH5---PA1  PWM1_CH5---PF5	BPWM0_CH5---PD14  BPWM1_CH5---PB15
	*/
	
	SYS->GPA_MFP |= (SYS_GPA_MFP_PA12_PWM0_CH0| SYS_GPA_MFP_PA13_PWM0_CH1| SYS_GPA_MFP_PA14_PWM0_CH2| SYS_GPA_MFP_PA15_PWM0_CH3| 
		SYS_GPA_MFP_PA10_PWM1_CH2| SYS_GPA_MFP_PA11_PWM1_CH3);
	SYS->GPF_MFP |= ( SYS_GPF_MFP_PF4_PWM1_CH4| SYS_GPF_MFP_PF5_PWM1_CH5);
			  	 
	SYS->ALT_MFP|= ( SYS_ALT_MFP_PA12_PWM0_CH0| SYS_ALT_MFP_PA13_PWM0_CH1| SYS_ALT_MFP_PA14_PWM0_CH2|	 //PWM0
		SYS_ALT_MFP_PA15_PWM0_CH3 |   
		SYS_ALT_MFP_PA10_PWM1_CH2 |				 //PWM1
		SYS_ALT_MFP_PA11_PWM1_CH3 |SYS_ALT_MFP_PF4_PWM1_CH4  |SYS_ALT_MFP_PF5_PWM1_CH5  );	  
	
	SYS->ALT_MFP2|= ( SYS_ALT_MFP2_PA12_PWM0_CH0| SYS_ALT_MFP2_PA13_PWM0_CH1| SYS_ALT_MFP2_PA14_PWM0_CH2|	 //PWM0
		SYS_ALT_MFP2_PA15_PWM0_CH3 |   
		SYS_ALT_MFP2_PA10_PWM1_CH2 |				 //PWM1
		SYS_ALT_MFP2_PA11_PWM1_CH3 |SYS_ALT_MFP2_PF4_PWM1_CH4  |SYS_ALT_MFP2_PF5_PWM1_CH5 );	  
	
	SYS->ALT_MFP3|= ( SYS_ALT_MFP3_PA12_PWM0_CH0| SYS_ALT_MFP3_PA13_PWM0_CH1| SYS_ALT_MFP3_PA14_PWM0_CH2|	 //PWM0
		SYS_ALT_MFP3_PA15_PWM0_CH3 |   
		SYS_ALT_MFP3_PA10_PWM1_CH2 |				 //PWM1
		SYS_ALT_MFP3_PA11_PWM1_CH3 |SYS_ALT_MFP3_PF4_PWM1_CH4  |SYS_ALT_MFP3_PF5_PWM1_CH5  );	  
	
	SYS->ALT_MFP4|= ( SYS_ALT_MFP4_PA12_PWM0_CH0| SYS_ALT_MFP4_PA13_PWM0_CH1| SYS_ALT_MFP4_PA14_PWM0_CH2|	 //PWM0
		SYS_ALT_MFP4_PA15_PWM0_CH3 |   
		SYS_ALT_MFP4_PA10_PWM1_CH2 |				 //PWM1
		SYS_ALT_MFP4_PA11_PWM1_CH3 |SYS_ALT_MFP4_PF4_PWM1_CH4  |SYS_ALT_MFP4_PF5_PWM1_CH5  );	  	  	
	 
	 // PWM0 Group ( 0, 1, 2, 3, 4 5 )
	 CLK_EnableModuleClock(PWM0_MODULE);
 //    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL3_PWM0_S_PLL, 0);	 //Select PWM module clock source

	 CLK->CLKSEL3 &= ~CLK_CLKSEL3_PWM0_S_PCLK;
     SYS_ResetModule(PWM0_RST);     //Reset PWM0 

	 PWM_SetOutputChannel(PWM0,0,XPWM_CNR_VAL_SET,XPWM_INIT_SET,SWITCH_SET,0);
	 PWM_SetOutputChannel(PWM0,1,XPWM_CNR_VAL_SET,XPWM_INIT_SET,SWITCH_SET,0);
	 PWM_SetOutputChannel(PWM0,2,XPWM_CNR_VAL_SET,XPWM_INIT_SET,SWITCH_SET,0);
	 PWM_SetOutputChannel(PWM0,3,XPWM_CNR_VAL_SET,XPWM_INIT_SET,SWITCH_SET,0);

	 PWM_EnableOutput( PWM0, BPWM_CH_0_MASK| BPWM_CH_1_MASK | BPWM_CH_2_MASK | BPWM_CH_3_MASK );

	 //PWM1 Group ( 0, 1, 2, 3, 4, 5)
	 CLK_EnableModuleClock(PWM1_MODULE);
 //    CLK_SetModuleClock(PWM1_MODULE, CLK_CLKSEL3_PWM1_S_PLL , 0);	 //Select PWM module clock source
     CLK->CLKSEL3 &= ~CLK_CLKSEL3_PWM1_S_PCLK;
     SYS_ResetModule(PWM1_RST);     //Reset PWM1 

	 PWM_SetOutputChannel(PWM1,2,MOTOR_CNR_VAL_SET,MOTOR_INIT_SET,MOTOR_SWITCH_SET,0);
	 PWM_SetOutputChannel(PWM1,3,MOTOR_CNR_VAL_SET,MOTOR_INIT_SET,MOTOR_SWITCH_SET,0);
	 PWM_SetOutputChannel(PWM1,4,MOTOR_CNR_VAL_SET,MOTOR_INIT_SET,MOTOR_SWITCH_SET,0);
	 PWM_SetOutputChannel(PWM1,5,MOTOR_CNR_VAL_SET,MOTOR_INIT_SET,MOTOR_SWITCH_SET,0);

	 PWM_EnableOutput( PWM1, BPWM_CH_2_MASK | BPWM_CH_3_MASK |BPWM_CH_4_MASK | BPWM_CH_5_MASK);

	 PWM_Start( PWM0, PWM_CH_0_MASK| PWM_CH_1_MASK | PWM_CH_2_MASK | PWM_CH_3_MASK ); 
     PWM_Start( PWM1, PWM_CH_2_MASK | PWM_CH_3_MASK |PWM_CH_4_MASK | PWM_CH_5_MASK); 

}

void SYS_Init(void)
{
    DMX512_Slave;
	SYS_UnlockReg();
	/*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);
#if PWM_ENABLE
	LedPwmInit();
#endif
#if PWM_M0_DISP
	M0_PwmDisp();
#endif
#if PWM_M0_SW300P
	SW300P_PwmDisp();
#endif
#if UART0_Enable
    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP |= SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;
#endif
#if UART1_Enable

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART1_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

	SYS->GPB_MFP |= (SYS_GPB_MFP_PB4_UART1_RXD | SYS_GPB_MFP_PB5_UART1_TXD );

#if DMX_Enable
	UART_EnableInt(DMX_UART, (UART_IER_RDA_IEN_Msk));
#endif

#endif

#if LED_Enable
	GPIO_SetMode(LED_PORT, LED_Pin, GPIO_PMD_OUTPUT);
	LED_DMX=0;
#endif

#if DMX_Enable
	GPIO_SetMode(RS_DMX_PORT, RS_DMX_Pin, GPIO_PMD_OUTPUT);
    DMX512_Slave;
	UART_Open(DMX_UART, USART_BPS);	
#endif

#if D485_Enable
	GPIO_SetMode(RS_485_PORT, RS_485_Pin, GPIO_PMD_OUTPUT);
	D485_Master;
	UART_Open(D485_UART,USART_BPS);
#endif

	/* Enable FMC ISP function */
    FMC_Open();
	if(SetIAPBoot() < 0)
    {
        //SerialPutString("Failed to set IAP boot mode!\n");
        //goto lexit;
    }
}


void Jump_To_App(void)
{
//	/* Mask all interrupt before changing VECMAP to avoid wrong interrupt handler fetched */
//	__disable_irq();
//	__set_PRIMASK(1);
//	
//	/* Set VECMAP to LDROM for booting from LDROM */
//	FMC_SetVectorPageAddr(FMC_APROM_BASE+START_ADDR);
//	
//	/* Software reset to boot to LDROM */
//	NVIC_SystemReset();

	//reset all IPS
	outpw(&SYS->IPRSTC2,0xFFFFFFFF);
	outpw(&SYS->IPRSTC2,0);
	__disable_irq();
	/* Mask all interrupt before changing VECMAP to avoid wrong interrupt handler fetched */
	__set_PRIMASK(1);
	
	/* Set VECMAP to LDROM for booting from LDROM */
	FMC_SetVectorPageAddr(ApplicationAddress);
		
	/* Jump to user application */
	JumpAddress = *(__IO uint32_t*) (ApplicationAddress + 4);
	Jump_To_Application = (pFunction) JumpAddress;
	/* Initialize user application's Stack Pointer */
	__set_MS(*(__IO uint32_t*) ApplicationAddress);
	Jump_To_Application();
}

void LCM_Init(void)
{
#if LCD_Enable
	OLED_Init();
#endif
}

void FLASH_EraseAll(void)
{
	uint32_t PAGE=0;
	FMC_EnableAPUpdate();
	// Program Flash Bank1 
	Address = BANK1_WRITE_START_ADDR;	
	while(PAGE < (BANK1_WRITE_END_ADDR-BANK1_WRITE_START_ADDR))
	{			
		FMC_Erase(BANK1_WRITE_START_ADDR+PAGE);
		PAGE = PAGE +FMC_FLASH_PAGE_SIZE;	
	}
}

void D485_SendData(unsigned short UDR)
{
#if D485_Enable	
static uint8_t i=0;
	//S-CMD_S S1-CMD_S1 S2-CMD_S2;
	if((UDR&0x100)==0){
		if(i==1){
			while ( ( D485_UART->FSR & UART_FSR_TX_FULL_Msk ) ) ;
			UART_WRITE(D485_UART, (unsigned char)UDR);		
		}
		else{
			while ( !( D485_UART->FSR & UART_FSR_TE_FLAG_Msk ) ) ; 
		    D485_UART->LCR |= UART_LCR_SPE_Msk;			   // Enable Stick
		    D485_UART->LCR |= UART_LCR_PBE_Msk;	           // Set PBE		
			D485_UART->LCR |= UART_LCR_EPE_Msk;			   // Clr EPE
			UART_WRITE(D485_UART, (unsigned char)UDR);
			i=1;		 
		}							
	}
	else
	{
		if(i==2){
			while ( ( D485_UART->FSR & UART_FSR_TX_FULL_Msk ) ) ;
			UART_WRITE(D485_UART, (unsigned char)UDR); 	    	
		}
		else{
			while ( !( D485_UART->FSR & UART_FSR_TE_FLAG_Msk ) ) ; 
			D485_UART->LCR &= ~UART_LCR_PBE_Msk;   // 不发送校验位
			UART_WRITE(D485_UART, (unsigned char)UDR);
			i=2;		 
		}	
	}
	//Wait for UART free	
#endif
}

void DMX_SendData(unsigned short UDR)
{
static uint8_t i=0;
#if DMX_Enable	
	//S-CMD_S S1-CMD_S1 S2-CMD_S2;
	if((UDR&0x100)==0){
		if(i==1){
			while ( ( DMX_UART->FSR & UART_FSR_TX_FULL_Msk ) ) ;
			UART_WRITE(DMX_UART, (unsigned char)UDR);		
		}
		else{
			while ( !( DMX_UART->FSR & UART_FSR_TE_FLAG_Msk ) ) ; 
		    DMX_UART->LCR |= UART_LCR_SPE_Msk;			   // Enable Stick
		    DMX_UART->LCR |= UART_LCR_PBE_Msk;	           // Set PBE		
			DMX_UART->LCR |= UART_LCR_EPE_Msk;			   // Clr EPE
			UART_WRITE(DMX_UART, (unsigned char)UDR);
			i=1;		 
		}							
	}
	else{
		if(i==2){
			while ( ( DMX_UART->FSR & UART_FSR_TX_FULL_Msk ) ) ;
			UART_WRITE(DMX_UART, (unsigned char)UDR); 	    	
		}
		else{
			while ( !( DMX_UART->FSR & UART_FSR_TE_FLAG_Msk ) ) ; 
			DMX_UART->LCR &= ~UART_LCR_PBE_Msk;   // 不发送校验位
			UART_WRITE(DMX_UART, (unsigned char)UDR);	
			i=2;		 
		}	
	}
#endif
}

void FLASH_ProgramOneWord(unsigned int Address, unsigned int Data)
{
	FMC_Write(Address, Data);
}

void FLASH_ProgramUnLock(void)
{
	FMC_EnableAPUpdate();	 
}

void FLASH_ProgramLock(void)
{
	FMC_DisableAPUpdate();	 
}

#if DMX_Enable
void DMX_UARTIRQHandler(void)
{
	uint16_t UDR;
		
	if(DMX_UART->ISR & UART_ISR_RDA_IF_Msk){
		if ( ( DMX_UART->FSR & UART_FSR_FEF_Msk ) ) {
			UDR = DMX_UART->RBR;
		}
		else{
			UDR = DMX_UART->RBR|0x0100;
		}
		FifoIn(&MyFifo,UDR);
	}
			
	if(DMX_UART->ISR & UART_ISR_THRE_IF_Msk)
	{
		DMX_UART->IER &= ~UART_IER_THRE_IEN_Msk;      // 禁止发送寄存器空中断 	
	}
}
#endif

#endif

#if STM32_Enable
#include "stm32f10x.h"
#if LCD_Enable
#include "12864i_016.h"
#endif
#if S1_Enable
#include "Pwm_T2.h"
#include "Pwm_T3.h"
#endif	
/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
/* Private variables ---------------------------------------------------------*/
pFunction Jump_To_Application;
uint32_t JumpAddress;
/* Private variables ---------------------------------------------------------*/  
static uint32_t EraseCounter = 0x00, Address = 0x00,END_ADDR=0;
static uint32_t Data = 0x3210ABCD;
static uint32_t NbrOfPage = 0x00;
uint32_t WRPR_Value = 0xFFFFFFFF, ProtectedPages = 0x0,BlockNbr = 0,UserMemoryMask = 0;
uint32_t FlashDestination = ApplicationAddress; /* Flash user program offset */
static volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;
static volatile TestStatus MemoryProgramStatus = PASSED;

const uint8_t Number_Char[]={"0123456789 .C"};

__IO uint32_t CRCValue=0xffffffff,uwCRCValue1 = 0,uwCRCValue2 = 0,uwCRCValue3 = 0;


/**
  * @brief  Disable the write protection of desired pages
  * @param  None
  * @retval None
  */
void FLASH_DisableWriteProtectionPages(void)
{
  uint32_t useroptionbyte = 0, WRPR = 0;
  uint16_t var1 = OB_IWDG_SW, var2 = OB_STOP_NoRST, var3 = OB_STDBY_NoRST;
  FLASH_Status status = FLASH_BUSY;

  WRPR = FLASH_GetWriteProtectionOptionByte();

  /* Test if user memory is write protected */
  if ((WRPR & UserMemoryMask) != UserMemoryMask)
  {
    useroptionbyte = FLASH_GetUserOptionByte();

    UserMemoryMask |= WRPR;

    status = FLASH_EraseOptionBytes();

    if (UserMemoryMask != 0xFFFFFFFF)
    {
      status = FLASH_EnableWriteProtection((uint32_t)~UserMemoryMask);
    }

    /* Test if user Option Bytes are programmed */
    if ((useroptionbyte & 0x07) != 0x07)
    { 
      /* Restore user Option Bytes */
      if ((useroptionbyte & 0x01) == 0x0)
      {
        var1 = OB_IWDG_HW;
      }
      if ((useroptionbyte & 0x02) == 0x0)
      {
        var2 = OB_STOP_RST;
      }
      if ((useroptionbyte & 0x04) == 0x0)
      {
        var3 = OB_STDBY_RST;
      }

      FLASH_UserOptionByteConfig(var1, var2, var3);
    }

    if (status == FLASH_COMPLETE)
    {
      /* Generate System Reset to load the new option byte values */
      NVIC_SystemReset();
    }
    else{
    }
  }
  else{
  }
}

void USART1_IAP_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
		
	USART_InitStructure.USART_BaudRate = USART_BPS;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure); 
	USART_Cmd(USART1, ENABLE);
}

void USART_DMX_Init(void)
{    
  
#if S_107_Enable
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
  
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 15;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

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

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    USART_Cmd(USART2, ENABLE);
#endif

#if SUPPORT_USART3
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
  
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 15;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); 
#if Remap_USART3
	GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);
     // Configure USART3 Tx (PB.10) as alternate function push-pull 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);	    
    // Configure USART3 Rx (PB.11) as input floating 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
#else
     //Configure USART3 Tx (PB.10) as alternate function push-pull 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);	    
    // Configure USART3 Rx (PB.11) as input floating 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
#endif


#endif

#if SUPPORT_USART1
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
  
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 15;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 

     // Configure USART3 Tx (PB.10) as alternate function push-pull 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// Configure USART1 Rx (PA.10) as input floating 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

#endif

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	// Configure USART3 DMX-CON (PA.05) as function push-pull
    GPIO_InitStructure.GPIO_Pin = DMX_CS_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(DMX_CS_Port, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = USART_BPS;
    USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
     // Configure DMX_UART 
    USART_Init(DMX_UART, &USART_InitStructure); 
	
	USART_ITConfig(DMX_UART, USART_IT_RXNE, ENABLE); 

    // Enable the DMX_UART 
    USART_Cmd(DMX_UART, ENABLE);



}

void USART_D485_Init(void)
{

#if D485_Enable

#if S_107_Enable
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	  // Enable UART4 clock 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4 , ENABLE);
	//GPIO_PinRemapConfig(GPIO_FullRemap_UART4, ENABLE);
    	// USART2 GPIO config 
     // Configure UART4 Tx (PB.10) as alternate function push-pull 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);	    
    // Configure UART4 Rx (PB.11) as input floating 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
	// Configure UART4 DMX-CON (PA.05) as function push-pull
    GPIO_InitStructure.GPIO_Pin = D485_CS_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(D485_CS_Port, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = USART_BPS;
    USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
     // Configure UART4 
    USART_Init(UART4, &USART_InitStructure);  
    //USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
    // Enable the USART1 
    USART_Cmd(UART4, ENABLE);
#endif
#if S_103_Enable
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); 

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);	    

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = D485_CS_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(D485_CS_Port, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = USART_BPS;
    USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART2, &USART_InitStructure);  

    USART_Cmd(USART2, ENABLE);
#endif

#endif

}


void SYS_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE|RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2|RCC_APB1Periph_TIM3|RCC_APB1Periph_TIM4, ENABLE); 
	      /* Enable CRC clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE); 

#if S1_Enable	
    TIM2_PWM_Init();//电机X
	TIM3_PWM_Init();//电机Y
#else
	
	#if S_103_Enable
		#if PJZ_SYKZ_Enable
		#else
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	    GPIO_Init(GPIOB, &GPIO_InitStructure);
		GPIO_SetBits(GPIOB,GPIO_Pin_1);
		#endif
	#else
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_6);
	#endif

	#if S2_Enable
	TIM2_PWM_Init();//电机
	TIM3_PWM_Init();
	TIM4_PWM_Init();
	#endif

#endif	
	GPIO_InitStructure.GPIO_Pin = DMX_CS_Pin;       
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(DMX_CS_Port, &GPIO_InitStructure);
	DMX512_Slave;
#if D485_Enable
	GPIO_InitStructure.GPIO_Pin = D485_CS_Pin;       
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(D485_CS_Port, &GPIO_InitStructure);
	D485_Master;
#endif
#if LED_Enable
	GPIO_InitStructure.GPIO_Pin = LED_DMX_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(LED_DMX_Port, &GPIO_InitStructure);
#endif
#if D485_Enable
	USART_D485_Init();
#endif
#if DMX_Enable
	USART_DMX_Init();
#endif
	#if My9221_Enable
	MY9221_Init();
	MY9221_LED();
	MY9221_Enable;
	#endif

	#if PZJ_XYZ_Enable
	TIM2_PWM_Init();//电机
	TIM3_PWM_Init();
	TIM4_PWM_Init();
	#endif

	#if PJZ_SYKZ_Enable
	TIM2_PWM_Init();//电机
	TIM3_PWM_Init();
	TIM4_PWM_Init();

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_InitStructure.GPIO_Pin = SEG_1_Pin;
    GPIO_Init(SEG_1_Port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = SEG_2_Pin;
    GPIO_Init(SEG_2_Port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = SEG_3_Pin;
    GPIO_Init(SEG_3_Port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = SEG_4_Pin;
    GPIO_Init(SEG_4_Port, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SEG_A_Pin;
    GPIO_Init(SEG_A_Port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = SEG_B_Pin;
    GPIO_Init(SEG_B_Port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = SEG_C_Pin;
    GPIO_Init(SEG_C_Port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = SEG_D_Pin;
    GPIO_Init(SEG_D_Port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = SEG_E_Pin;
    GPIO_Init(SEG_E_Port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = SEG_F_Pin;
    GPIO_Init(SEG_F_Port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = SEG_G_Pin;
    GPIO_Init(SEG_G_Port, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = SEG_H_Pin;
    GPIO_Init(SEG_H_Port, &GPIO_InitStructure);

	SEG_1_H();SEG_2_H();SEG_3_H();SEG_4_H();
	SEG_A_H();SEG_B_H();SEG_C_H();SEG_D_H();
	SEG_E_H();SEG_F_H();SEG_G_H();SEG_H_H();
	#endif
}

void Jump_To_App(void)
{
    __disable_irq();
	USART_Cmd(DMX_UART, DISABLE);
#if D485_Enable
	USART_Cmd(D485_UART, DISABLE);
#endif
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, DISABLE);
	if (((*(__IO uint32_t*)ApplicationAddress) & 0x2FFE0000 ) == 0x20000000)
    { 
      /* Jump to user application */
      JumpAddress = *(__IO uint32_t*) (ApplicationAddress + 4);
      Jump_To_Application = (pFunction) JumpAddress;
      /* Initialize user application's Stack Pointer */
      __set_MSP(*(__IO uint32_t*) ApplicationAddress);
      Jump_To_Application();
    }

}

void LCM_Init(void)
{
#if LCD_Enable
	LCM_12864_Init();
#endif
#if LCD1602_Enable		
	LCM_1602_init();
#endif
}

void FLASH_EraseAll(void)
{
	/* Porgram FLASH Bank1 ********************************************************/       
	// Unlock the Flash Bank1 Program Erase controller 
	FLASH_Unlock();

	/* Get the number of block (4 or 2 pages) from where the user program will be loaded */
	BlockNbr = (FlashDestination - 0x08000000) >> 12;
	
	/* Compute the mask to test if the Flash memory, where the user program will be
	loaded, is write protected */
	#if defined (STM32F10X_MD) || defined (STM32F10X_MD_VL)
	UserMemoryMask = ((uint32_t)~((1 << BlockNbr) - 1));
	#else /* USE_STM3210E_EVAL */
	if (BlockNbr < 62)
	{
		UserMemoryMask = ((uint32_t)~((1 << BlockNbr) - 1));
	}
	else
	{
		UserMemoryMask = ((uint32_t)0x80000000);
	}
	#endif /* (STM32F10X_MD) || (STM32F10X_MD_VL) */
	if ((FLASH_GetWriteProtectionOptionByte() & UserMemoryMask) != UserMemoryMask)
	{
		FLASH_DisableWriteProtectionPages();
	}	

	// Define the number of page to be erased 
	NbrOfPage = (BANK1_WRITE_END_ADDR - BANK1_WRITE_START_ADDR) / FLASH_PAGE_SIZE;
	
	// Clear All pending flags 
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	
	
	// Erase the FLASH pages 
	for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
	{
		FLASHStatus = FLASH_ErasePage(BANK1_WRITE_START_ADDR + (FLASH_PAGE_SIZE * EraseCounter));
	}
}

void D485_SendData(unsigned short UDR)
{
#if D485_Enable
	while (USART_GetFlagStatus(D485_UART,USART_FLAG_TXE) == RESET){}
	//S-CMD_S S1-CMD_S1 S2-CMD_S2;
	USART_SendData(D485_UART, UDR);
#endif
}

void DMX_SendData(unsigned short UDR)
{
#if DMX_Enable
	while (USART_GetFlagStatus(DMX_UART,USART_FLAG_TXE) == RESET){}
	//S-CMD_S S1-CMD_S1 S2-CMD_S2;
	USART_SendData(DMX_UART, UDR);
#endif
}

void FLASH_ProgramOneWord(unsigned int Address, unsigned int Data)
{
	FLASH_ProgramWord(Address, Data);
}

void FLASH_ProgramUnLock(void)
{
	FLASH_Unlock();	 
}	

void FLASH_ProgramLock(void)
{
	FLASH_Lock();	 
}

void USART1_IRQHandler(void)
{	
	uint16_t UDR;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		UDR = (uint16_t)USART1->DR;
		FifoIn(&MyFifo,UDR);
	}
	if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
	{ 
		USART1->CR1&=~(1<<7);
	}
	UDR = (uint16_t)USART1->DR;
}

void USART2_IRQHandler(void)
{	
	uint16_t UDR;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		UDR = (uint16_t)USART2->DR;
		FifoIn(&MyFifo,UDR);
	}
	if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
	{ 
		USART2->CR1&=~(1<<7);
	}
	UDR = (uint16_t)USART2->DR;
}

void USART3_IRQHandler(void)
{	
	uint16_t UDR;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		UDR = (uint16_t)USART3->DR;
		FifoIn(&MyFifo,UDR);
	}
	if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
	{ 
		USART3->CR1&=~(1<<7);
	}
	UDR = (uint16_t)USART3->DR;
}

void UART4_IRQHandler(void)
{	
	uint16_t UDR;
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
	{
		UDR = (uint16_t)UART4->DR;
		FifoIn(&MyFifo,UDR);
	}
	if(USART_GetITStatus(UART4, USART_IT_TXE) != RESET)
	{ 
		UART4->CR1&=~(1<<7);
	}
	UDR = (uint16_t)UART4->DR;
}


void IAP_Delayus(uint32_t us) 
{ 
    uint32_t  i, j; 
    for( i = us; i > 0; i-- ) 
    { 
        for( j = 72 ; j > 0 ; j-- ) 
        {} 
    } 
}

void MASTER_USART_Init(void)
{      
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
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

void IAP_MASTER(void)
{
#if LCD_Enable
	LCM_12864_Init();
	LCM_Clr();
	LCM_Prints(0,0,IAP_Version);
	LCM_Prints(0,1,"  IAP...");
#endif
	MASTER_USART_Init();
	
	while (1)
	{
	while(GPIO_ReadInputDataBit(MENU_ESC_Port,MENU_ESC_Pin))
	{
		USART_SendData(USART2,CMD_IAP);
		IAP_Delayus(50);
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET){}
	}
#if (S103_S1_S2_Enable||S107_S1_S2_Enable) 
	USART_SendData(USART2,CMD_S2);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET){}	
#if LCD_Enable
	CRC_ResetDR();
	LCM_Prints(10,1,"  ");
	LCM_Prints(0,2,IAP_Version_S2);
#endif	
	Address = BANK1_IAP_START_ADDR;

	while((Address < BANK1_IAP_END_ADDR) && (FLASHStatus == FLASH_COMPLETE))
	{
		USART_SendData(USART2,(uint8_t)(*(__IO uint32_t*) Address));
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET){}
		USART_SendData(USART2,(uint8_t)((*(__IO uint32_t*) Address)>>8));
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET){}
		USART_SendData(USART2,(uint8_t)((*(__IO uint32_t*) Address)>>16));
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET){}
		USART_SendData(USART2,(uint8_t)((*(__IO uint32_t*) Address)>>24));
		while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET){}
#if LCD_Enable
		if(Address%FLASH_PAGE_SIZE==0)LCM_Frame(((Address-BANK1_IAP_START_ADDR)*100)/(BANK1_IAP_END_ADDR-BANK1_IAP_START_ADDR));
#endif
		CRC->DR = (*(__IO uint32_t*) Address);
		Address = Address + 4;
	}

	uwCRCValue1=CRC->DR;
	USART_SendData(USART2,(uint8_t)(uwCRCValue1));
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET){}
	USART_SendData(USART2,(uint8_t)(uwCRCValue1>>8));
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET){}
	USART_SendData(USART2,(uint8_t)(uwCRCValue1>>16));
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET){}
	USART_SendData(USART2,(uint8_t)(uwCRCValue1>>24));
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET){}
#endif
	
	USART_SendData(USART2,CMD_S1);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET){}	
#if LCD_Enable
	CRC_ResetDR();
	LCM_Prints(10,1,"  ");
	LCM_Prints(0,2,IAP_Version_S1);
#endif	
	Address = BANK2_IAP_START_ADDR;

	while((Address < BANK2_IAP_END_ADDR) && (FLASHStatus == FLASH_COMPLETE))
	{
		USART_SendData(USART2,(uint8_t)(*(__IO uint32_t*) Address));
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET){}
		USART_SendData(USART2,(uint8_t)((*(__IO uint32_t*) Address)>>8));
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET){}
		USART_SendData(USART2,(uint8_t)((*(__IO uint32_t*) Address)>>16));
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET){}
		USART_SendData(USART2,(uint8_t)((*(__IO uint32_t*) Address)>>24));
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET){}
#if LCD_Enable
		if(Address%FLASH_PAGE_SIZE==0)LCM_Frame(((Address-BANK2_IAP_START_ADDR)*100)/(BANK2_IAP_END_ADDR-BANK2_IAP_START_ADDR));
#endif
		CRC->DR = (*(__IO uint32_t*) Address);
		Address = Address + 4;
	}
	uwCRCValue2=CRC->DR;
	USART_SendData(USART2,(uint8_t)(uwCRCValue2));
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET){}
	USART_SendData(USART2,(uint8_t)(uwCRCValue2>>8));
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET){}
	USART_SendData(USART2,(uint8_t)(uwCRCValue2>>16));
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET){}
	USART_SendData(USART2,(uint8_t)(uwCRCValue2>>24));
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET){}
  
	USART_SendData(USART2,CMD_S);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET){}	
#if LCD_Enable
	CRC_ResetDR();
	LCM_Prints(10,1,"  ");
	LCM_Prints(0,2,IAP_Version_S);
#endif	
	Address = BANK3_IAP_START_ADDR;

	while((Address < BANK3_IAP_END_ADDR) && (FLASHStatus == FLASH_COMPLETE))
	{
		USART_SendData(USART2,(uint8_t)(*(__IO uint32_t*) Address));
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET){}
		USART_SendData(USART2,(uint8_t)((*(__IO uint32_t*) Address)>>8));
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET){}
		USART_SendData(USART2,(uint8_t)((*(__IO uint32_t*) Address)>>16));
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET){}
		USART_SendData(USART2,(uint8_t)((*(__IO uint32_t*) Address)>>24));
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET){}
#if LCD_Enable
		if(Address%FLASH_PAGE_SIZE==0)LCM_Frame(((Address-BANK3_IAP_START_ADDR)*100)/(BANK3_IAP_END_ADDR-BANK3_IAP_START_ADDR));
#endif
		CRC->DR = (*(__IO uint32_t*) Address);
		Address = Address + 4;
	}
	uwCRCValue3=CRC->DR;
	USART_SendData(USART2,(uint8_t)(uwCRCValue3));
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET){}
	USART_SendData(USART2,(uint8_t)(uwCRCValue3>>8));
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET){}
	USART_SendData(USART2,(uint8_t)(uwCRCValue3>>16));
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET){}
	USART_SendData(USART2,(uint8_t)(uwCRCValue3>>24));
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET){}
#if LCD_Enable
	LCM_Frame(100);
	
	LCM_Prints(10,1,"OK");
#endif
	}
}

#endif


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

void IAP_Wait(void)
{
uint8_t RxCn=0;
uint16_t UDR;
uint32_t i=0,j=0;

	SYS_Init();
#if LCD_Enable
	LCM_Init();
	LCM_Clr();
	LCM_Prints(4,1,"Init...");
#endif
#if LCD1602_Enable		
	LCM_1602_init();
	LCM_Write_string(0,0,"Init...");
#endif
	while(1)
	{
		if(MyFifo.count){
			if(FifoOut(&MyFifo,&UDR)==FifoNormal){
				RxBuf[RxCn++]=UDR& 0x00ff;

#if IAP1_Enable
		    	boot_task(UDR);//IAP1
#else
#if D485_Enable
			    D485_SendData(UDR);
#endif
#endif
			}
			if(RxBuf[0]!=RxMark[0])RxCn=0;
		}	
		else if(RxCn >= MARK_NUM-1){
		   i=0;
		   for(i=0;i<MARK_NUM;i++){
		   	   if(RxMark[i]!=RxBuf[i]){break;}
		   }
		   if(i>=MARK_NUM-1){
		   	break;
		   }
		   RxCn=0;
		}

		j=j+1;
#if LCD_Enable
	#if M0518_Enable
			if(j%200==0)LCM_PrintsTemp(12,1,(unsigned char*)&Number_Char[((100000-j)*10)/(100000)]);
			if(j>=100000)Jump_To_App();
	#else
			if(j%200==0)LCM_PrintsTemp(12,1,(unsigned char*)&Number_Char[((200000-j)*10)/(200000)]);
			if(j>=200000)Jump_To_App();
	#endif
#else
	#if (S_107_Enable|S_103_Enable) 
		#if LCD1602_Enable
			#if M0518_Enable
				if(j%200==0)LCM_Write_Dat(4,1,(unsigned char)Number_Char[((120000-j)*10)/(120000)]);
				if(j>=120000)Jump_To_App();
			#else
				if(j%200==0)LCM_Write_Dat(4,1,(unsigned char)Number_Char[((60000-j)*10)/(60000)]);
				if(j>=60000)Jump_To_App();
			#endif
		#else
			if(j>=2200000)Jump_To_App();
		#endif
	#else
			if(j>=4200000)Jump_To_App();
	
	#endif
#endif	
	
	}

again:
	CRCValue=0xffffffff;

	FLASH_EraseAll();
#if LCD_Enable
	LCM_Prints(2,2,"M0518 IAP");
#endif
#if LCD1602_Enable		
	LCM_Write_string(0,1,"M0518 IAP");
#endif
#if PJZ_SYKZ_Enable
	SEG_1_L();SEG_2_L();SEG_3_L();SEG_4_L();

	SEG_A_H();SEG_B_H();SEG_C_H();SEG_D_H();
	SEG_E_H();SEG_F_H();SEG_G_L();SEG_H_H();
#endif
	while(1)
	{
	if(MyFifo.count){
		if(FifoOut(&MyFifo,&UDR)==FifoNormal){
#if D485_Enable
			D485_SendData(UDR);
#endif
#if S_107_Enable
			if((UDR & 0x01ff) == CMD_IAP)break;
#endif
#if S_103_Enable
			if((UDR & 0x01ff) == CMD_IAP)break;
#endif
#if S1_Enable
			if((UDR & 0x01ff) == CMD_IAP)break;
#endif
#if S2_Enable
			if((UDR & 0x01ff) == CMD_IAP)break;
#endif
			i++;
#if LED_Enable
			if(512==i){LED_DMX_ON;}
			else if(1024<=i){LED_DMX_OFF;i=0;}
#endif	  
		}
	}
	}

	while(MyFifo.count==0){}FifoOut(&MyFifo,&UDR);Data=(uint8_t)(UDR);
	while(MyFifo.count==0){}FifoOut(&MyFifo,&UDR);Data|=(uint8_t)UDR<<8;
	while(MyFifo.count==0){}FifoOut(&MyFifo,&UDR);
	// Program Flash Bank1 
	Address = BANK1_WRITE_START_ADDR;
	END_ADDR=BANK1_WRITE_START_ADDR+Data;
#if LCD_Enable
	#if STM32_Enable
	LCM_Prints(2,3,"S1");
	#endif	
	#if M0518_Enable
	LCM_Prints(2,3,"S1");
	#endif	
#endif	
	while(Address < END_ADDR)
	{
		while(MyFifo.count==0){}FifoOut(&MyFifo,&UDR);Data=(uint8_t)(UDR);
		while(MyFifo.count==0){}FifoOut(&MyFifo,&UDR);Data|=(uint8_t)UDR<<8;
		while(MyFifo.count==0){}FifoOut(&MyFifo,&UDR);Data|=(uint8_t)UDR<<16;
		while(MyFifo.count==0){}FifoOut(&MyFifo,&UDR);Data|=(uint8_t)UDR<<24;
		FLASH_ProgramOneWord(Address,Data);
		CRC32(Data);
		Address = Address + 4;
		i++;
#if LED_Enable
		if(512==i){LED_DMX_ON;}
		else if(1024<=i){LED_DMX_OFF;i=0;}
#endif
#if LCD_Enable
//		if(i%1000==0)LCM_Frame(((Address-BANK1_WRITE_START_ADDR)*100)/(END_ADDR-BANK1_WRITE_START_ADDR));
#endif
	}
	
	while(MyFifo.count==0){}FifoOut(&MyFifo,&UDR);uwCRCValue1=(uint8_t)UDR;
	while(MyFifo.count==0){}FifoOut(&MyFifo,&UDR);uwCRCValue1|=(uint8_t)UDR<<8;
	while(MyFifo.count==0){}FifoOut(&MyFifo,&UDR);uwCRCValue1|=(uint8_t)UDR<<16;
	while(MyFifo.count==0){}FifoOut(&MyFifo,&UDR);uwCRCValue1|=(uint8_t)UDR<<24;

	FLASH_ProgramLock();

	if(uwCRCValue1==CRCValue)
	{
#if LCD_Enable
		LCM_Prints(10,3,"OK");
#endif
	//	while(1);
#if (S_107_Enable|S_103_Enable) 
		Jump_To_App();
#else
		while(1)
		{
			while(MyFifo.count==0){}FifoOut(&MyFifo,&UDR);
			if((UDR & 0x01ff) == CMD_S)Jump_To_App();
		}
#endif
	}
#if LCD_Enable
	LCM_Prints(10,3,"again");
#endif
	goto again;
}













