#ifndef __FLASH_H__
#define	__FLASH_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/

/* ���Ͷ��� ------------------------------------------------------------------*/
#define STM32_FLASH_SIZE        1024  // ��ѡSTM32��FLASH������С(��λΪK)
#define STM_SECTOR_SIZE	 FLASH_PAGE_SIZE

#define APP_START_ADDR       (0x08000000+START_ADDR)  	//Ӧ�ó�����ʼ��ַ(�����FLASH)

/* �������� ------------------------------------------------------------------*/
void FLASH_WritePage (unsigned int WriteAddr, unsigned int * pBuffer);

#endif /* __FLASH_H__ */
