#ifndef __CONFIG_H
#define __CONFIG_H

#define Ver_No1 1
#define Ver_No2 0
#define Ver_No3 0

#define MENU_TH         1
//#define MENU_CH         1
//#define MENU_HL         1
//#define SUPPORT_CHINESE 1
					
#define Ver_UP        "WASH 19"
#define Ver_DOWM      " V1.00"
#define Ver_Child     "WASH 19 V1.00"

#define Chan_1	 "1:12Ch          "
#define Chan_2	 "2:15Ch          "
#define Chan_3	 "3:16Ch          "
#define ShowChan_1    "12Ch"
#define ShowChan_2    "15Ch"
#define ShowChan_3    "16Ch"

//#define USE_FULL_NULL    1
#define USE_FULL_ZOOM    1
#define USE_FULL_ROTA    1

#define	BackLightTime	8000//20MS*300=6S

#ifdef  USE_FULL_ZOOM
#define Manu_Show2Z     "Zoom    "
#define Manu_Show1Z     "[Zoom]  "
#endif
#ifdef  USE_FULL_ROTA
#define Manu_Show2R     "Rota    "  
#define Manu_Show1R     "[Rota]  " 
#endif


typedef enum
{
 _1channel	  =((uint8_t)12),
 _2channel	  =((uint8_t)15),
 _3channel	  =((uint8_t)16),
 _4channel	  =((uint8_t)46),
}Channel_Type;




#endif /* __CONFIG_H */
