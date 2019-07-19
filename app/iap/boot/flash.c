#include "flash.h"
#include "boot1.h"

//WIFI-DMX.COM


void FLASH_WritePage ( unsigned int WriteAddr, unsigned int * pBuffer)	
{
	unsigned int i=0;	
	while(i<FLASH_PAGE_SIZE)
	{
	   FLASH_ProgramOneWord(WriteAddr+i, *(pBuffer+i/4));
	   i=i+4;
	} 
}

