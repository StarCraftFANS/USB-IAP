#ifndef __BOOT1_H__
#define __BOOT1_H__

typedef enum
{
 CMD_IAP  =((unsigned short)0x01F1),
 CMD_S	  =((unsigned short)0x0101),
 CMD_S1	  =((unsigned short)0x0111),
 CMD_S2	  =((unsigned short)0x0121),
 CMD_S3	  =((unsigned short)0x0131),
 CMD_S4	  =((unsigned short)0x0141),
}CMD_Typedef;

typedef  void (*pFunction)(void);

void IAP_MASTER(void);

#endif


