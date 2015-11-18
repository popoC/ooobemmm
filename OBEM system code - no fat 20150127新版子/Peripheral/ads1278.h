#ifndef ADS1278_H
#define ADS1278_H
#include "msp430x54xA.h"
#include "stdbool.h"

//- P4.1   CLK
//- P1.1   SCLK
//- P1.2   DRDY

//- P2.0   D1
//- P2.1   D2
//- P2.2   D3
//- P2.3   D4
//- P1.4   D5
//- P1.4   D6
//- P1.4   D7
//- P1.4   D8


#define SCLK   BIT1  //P1
#define DRDY_1 BIT2  //P1
#define SYNC   BIT3  //P1 

#define DOUT_1 BIT0  //P2
#define DOUT_2 BIT1  //P2
#define DOUT_3 BIT2  //P2
#define DOUT_4 BIT3  //P2
#define DOUT_5 BIT4  //P1
#define DOUT_6 BIT5  //P1
#define DOUT_7 BIT6  //P1
#define DOUT_8 BIT7  //P1


#define RESET BIT6


#define SCLK_H  P1OUT|=SCLK
#define SCLK_L  P1OUT&=~SCLK
#define SYNC_H  P1OUT|=SYNC
#define SYNC_L  P1OUT&=~SYNC
#define DOUT_1_V (P2IN&DOUT_1)==DOUT_1
#define DOUT_2_V (P2IN&DOUT_2)==DOUT_2
#define DOUT_3_V (P2IN&DOUT_3)==DOUT_3
#define DOUT_4_V (P2IN&DOUT_4)==DOUT_4
#define DOUT_5_V (P1IN&DOUT_5)==DOUT_5
#define DOUT_6_V (P1IN&DOUT_6)==DOUT_6
#define DOUT_7_V (P1IN&DOUT_7)==DOUT_7
#define DOUT_8_V (P1IN&DOUT_8)==DOUT_8

void ADS1278_Init();
int ADS1278_Get_Data(unsigned long *data);
int get_ads1278_databuffer(unsigned char *data);


#endif