#ifndef __FLASH_H__
#define	__FLASH_H__

/* 包含头文件 ----------------------------------------------------------------*/

/* 类型定义 ------------------------------------------------------------------*/
#define STM32_FLASH_SIZE        1024  // 所选STM32的FLASH容量大小(单位为K)
#define STM_SECTOR_SIZE	 FLASH_PAGE_SIZE

#define APP_START_ADDR       (0x08000000+START_ADDR)  	//应用程序起始地址(存放在FLASH)

/* 函数声明 ------------------------------------------------------------------*/
void FLASH_WritePage (unsigned int WriteAddr, unsigned int * pBuffer);

#endif /* __FLASH_H__ */
