#ifndef __BOOTCONFIG_H__
#define __BOOTCONFIG_H__

#define STM32_Enable   1
//#define M0518_Enable   1

//#define S_107_Enable   1
#define S_103_Enable   1
//#define S1_Enable  1
//#define S1_Enable  1
//#define S2_XY_100S_Enable  1


#define START_ADDR (0x3000)

#define BPS_115200	 (0)
#define BPS_250000	 (1)


#if (S_107_Enable||S_103_Enable)
	#define DMX_Enable  1
	#define D485_Enable 1
	#define LED_Enable  1
	#define FAN_Enable  1
	#define LCD_Enable  1
#define UPGRADE_TRANSMIT	1 //��ʾ��ת������
#define UPGRADE_RESPONSE	0 //�������Ӧ��Ӧ
#else
	#define DMX_Enable  1
	#define D485_Enable 0
	#define LED_Enable  0
	#define FAN_Enable  0
	#define LCD_Enable  0
#define UPGRADE_TRANSMIT	0 //��ʾ��ת������
#define UPGRADE_RESPONSE	1 //�������Ӧ��Ӧ
#endif

/* ****************************************************������Ϣ������********************************************************************* */
/*
 ����˵��
 �����Ƿ�����ʾ�壬����DISPLAY_ENABLE���������ʾ������ʵ��Display_Task()������Ĺ��ܺ����������������ʾ�����ͺͳߴ����
 �жϰ����ǵƾ�Ψһ�İ��ӣ�������UPGRADE_TRANSMITת����UPGRADE_RESPONSE��Ӧ��Ϊ0
 �жϰ�������ʾ�壬���ҵƾ߻�������壬������UPGRADE_TRANSMITת��Ϊ1��UPGRADE_RESPONSE��ӦΪ0
 �жϰ���������壬������UPGRADE_RESPONSE��ӦΪ1��UPGRADE_TRANSMITת��Ϊ0
 �жϰ��ӽ�������ʹ�õĴ��ڣ�����UART_RP���������485���������ö�Ӧ��UART_RP_USE485
 ���������Ҫת�����ݣ���ʾ�壩������UART_TP���������485���������ö�Ӧ��UART_TP_USE485
 ������ڲ���485�������ݣ�����һ���շ��������ţ����ö�Ӧ�����ţ�������485оƬ���������շ���Ӧ�ĵ�ƽ
*/
#define DISPLAY_ENABLE		0	//=1��ʾ����ʾ��

//���մ�������
#define UART_RP			DMX_UART	//�������յĴ���
#define UART_RP_USE485  1		    //���մ����Ƿ�ʹ��485
#if UART_RP_USE485

#define RS485_TX_EN		DMX512_Master
#define RS485_RX_EN		DMX512_Slave
#endif
/*
 ֻ��һ����ӵĵƾ߲���Ҫת������Ӧ
 ��ʾ����Ҫ����ת��
 �������Ҫ������Ӧ
*/
//#define UPGRADE_TRANSMIT	1 //��ʾ��ת������
//#define UPGRADE_RESPONSE	0 //�������Ӧ��Ӧ
//#define UPGRADE_TRANSMIT	0 //��ʾ��ת������
//#define UPGRADE_RESPONSE	1 //�������Ӧ��Ӧ
#if UPGRADE_TRANSMIT
	//ת����������
	#define UART_TP			D485_UART	//����ת���Ĵ���
	#define UART_TP_USE485		1		//ת������ʹ��485
	#if UART_TP_USE485
		#define RS485_TX_EN_TP		D485_Master
		#define RS485_RX_EN_TP		D485_Slave
	#endif
#endif

/* ****************************************************������****************************************************************************** */

#define INFO_DeviceCode    	'D','S','0','0','8','0','0','1'	//�豸��
#define INFO_MCUTYPE      	CODE_STM32F103C8T6				//оƬ���
#define INFO_PCBID			2								//���ڵ�PCB�����
#define INFO_MCUID			1								//PCB���϶��MCU�����
#define INFO_VERSION      	0x10//BOOTLoader�İ汾����ʹ�ô�汾0x10��0x20��0x30��0x40

#define DS_Info_Chip_IC     'S','T','M','3','2','F','1','0','3','C','B','T','6',' ',' ',' '


#define DS_Info_Chip_ADDR     0x08000800
#define DS_Info_Anthor_ADDR   0x08000810
#define DS_Info_Device_ADDR   0x08000820
#define DS_Info_Function_ADDR 0x08000830



#if M0518_Enable
	#define PLLCON_SETTING      CLK_PLLCON_50MHz_HXT
	#define PLL_CLOCK           50000000
	
	#if DMX_Enable
	#define RS_DMX_DE_PORT      PA4
	#define RS_DMX_Pin      BIT4
	#define RS_DMX_PORT     PA
	#define DMX512_Master   RS_DMX_DE_PORT=1
	#define DMX512_Slave    RS_DMX_DE_PORT=0
	#define UART1_Enable  1 
	#define DMX_UART    UART1
	#endif
	
	#if D485_Enable
	#define RS_485_DE_PORT      PB3
	#define RS_485_Pin      BIT3
	#define RS_485_PORT     PB
	#define D485_Master   RS_485_DE_PORT=1
	#define D485_Slave    RS_485_DE_PORT=0
	#define UART0_Enable  1 
	#define D485_UART   UART0
	#endif
	
	#if LED_Enable
	#define LED_Pin      BIT5
	#define LED_PORT     PA
	#define LED_DMX      PA5
	#define LED_DMX_OFF     LED_DMX=~LED_DMX;
	#define LED_DMX_ON      LED_DMX=~LED_DMX;
	#endif
	
	#define BANK1_WRITE_START_ADDR  ((unsigned int)0x00000000+START_ADDR)
	#define BANK1_WRITE_END_ADDR    ((unsigned int)0x00010000)
	#define FLASH_PAGE_SIZE         0x200           /*!< Flash Page Size (512 Bytes) */

#endif

#if STM32_Enable

#if DMX_Enable
	#if S_107_Enable
	#define DMX_CS_Pin		GPIO_Pin_4
	#define DMX_CS_Port		GPIOD
	
	#define DMX512_Master   GPIO_SetBits(DMX_CS_Port,DMX_CS_Pin)
	#define DMX512_Slave    GPIO_ResetBits(DMX_CS_Port,DMX_CS_Pin)
	/*	 
	#define DMX512_Master   GPIO_ResetBits(DMX_CS_Port,DMX_CS_Pin) //��
	#define DMX512_Slave    GPIO_SetBits(DMX_CS_Port,DMX_CS_Pin)
	*/
	#define DMX_UART    USART2 
	#endif
	#if S_103_Enable
	#define DMX_CS_Pin		GPIO_Pin_9
	#define DMX_CS_Port		GPIOB
	#define DMX512_Master   GPIO_SetBits(DMX_CS_Port,DMX_CS_Pin)
	#define DMX512_Slave    GPIO_ResetBits(DMX_CS_Port,DMX_CS_Pin)
	#define DMX_UART    USART3 
	#endif
	#if S1_Enable
	#define DMX_CS_Pin	     GPIO_Pin_5
	#define DMX_CS_Port      GPIOB
	#define DMX512_Master    GPIO_SetBits(DMX_CS_Port,DMX_CS_Pin)
	#define DMX512_Slave     GPIO_ResetBits(DMX_CS_Port,DMX_CS_Pin)
	#define DMX_UART    USART3 
	#endif	
	#if S2_Enable
#if S2_XY_100S_Enable
	#define DMX_CS_Pin	     GPIO_Pin_12
	#define DMX_CS_Port      GPIOC
	#define Remap_USART3   (1)
#else
	#define DMX_CS_Pin	     GPIO_Pin_8
	#define DMX_CS_Port      GPIOA
#endif
	#define DMX512_Master    GPIO_SetBits(DMX_CS_Port,DMX_CS_Pin)
	#define DMX512_Slave     GPIO_ResetBits(DMX_CS_Port,DMX_CS_Pin)
	#define DMX_UART    USART3 
	#endif	
#endif
#if D485_Enable
	#if S_107_Enable
	#define D485_CS_Pin		GPIO_Pin_12
	#define D485_CS_Port    GPIOC
	#define D485_Master     GPIO_SetBits(D485_CS_Port,D485_CS_Pin)
	#define D485_Slave      GPIO_ResetBits(D485_CS_Port,D485_CS_Pin)
	
	#define D485_UART   UART4
	#endif 
	#if S_103_Enable
	#define D485_CS_Pin		GPIO_Pin_6
	#define D485_CS_Port    GPIOB
	#define D485_Master     GPIO_SetBits(D485_CS_Port,D485_CS_Pin)
	#define D485_Slave      GPIO_ResetBits(D485_CS_Port,D485_CS_Pin)
	
	#define D485_UART   USART2

	#endif
#endif

#if LED_Enable
	#if S_107_Enable
	#define LED_DMX_Pin     GPIO_Pin_3
	#define LED_DMX_Port    GPIOD
	#define LED_DMX_OFF     GPIO_SetBits(LED_DMX_Port,LED_DMX_Pin)
	#define LED_DMX_ON      GPIO_ResetBits(LED_DMX_Port,LED_DMX_Pin)
	#else
	#define LED_DMX_Pin     GPIO_Pin_15
	#define LED_DMX_Port    GPIOB
	#define LED_DMX_OFF     GPIO_SetBits(LED_DMX_Port,LED_DMX_Pin)
	#define LED_DMX_ON      GPIO_ResetBits(LED_DMX_Port,LED_DMX_Pin)
	#endif
#endif

/* Exported types ------------------------------------------------------------*/
typedef  void (*pFunction)(void);

#define ApplicationAddress    (0x8000000+START_ADDR)

/* Define the STM32F10x FLASH Page Size depending on the used STM32 device */
#if defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || defined (STM32F10X_CL) || defined (STM32F10X_XL)
	#define FLASH_PAGE_SIZE    ((unsigned short)0x800)
#else
	#define FLASH_PAGE_SIZE    ((unsigned short)0x400)
#endif

#define BANK1_WRITE_START_ADDR  ((unsigned int)(0x08000000+START_ADDR))

	#if S_107_Enable
	#define BANK1_WRITE_END_ADDR    ((unsigned int)0x08019000)
	#else
	#define BANK1_WRITE_END_ADDR    ((unsigned int)0x08010000)
	#endif

#endif

#endif
