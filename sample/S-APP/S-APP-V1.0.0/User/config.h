#ifndef __CONFIG_H
#define __CONFIG_H

#define Ver_No1 1
#define Ver_No2 0
#define Ver_No3 0

					
#define Ver_UP        "SW-100P"
#define Ver_DOWM      " V1.00"
#define Ver_Child     "SW-100P"

#define Chan_1	 "1:6Ch           "
#define Chan_2	 "2:7Ch           "
#define Chan_3	 "3:42Ch          "
#define ShowChan_1    "6Ch"
#define ShowChan_2    "7Ch"
#define ShowChan_3    "42Ch"

//#define USE_FULL_NULL    1
#define USE_FULL_ZOOM    1
#define USE_FULL_ROTA    1

#define	BackLightTime	900//20MS*1250=25S

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
 _1channel	  =((uint8_t)6),
 _2channel	  =((uint8_t)7),
 _3channel	  =((uint8_t)42),
 _4channel	  =((uint8_t)43),
}Channel_Type;




#endif /* __CONFIG_H */
