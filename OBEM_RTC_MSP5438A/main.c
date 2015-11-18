//--- 2015/0306 增加I2C timeout

//-- 2015/0420 修改電路定義腳位 OBEM20150127 新版子

//
//  I2C PIN --------
//  P3.1
//  P3.2
//          GND--- 
//  GPS PIN --------
//  P5.6 -> Rx
//  P5.7 <- Tx
//          GND---
//  P2.2 -> PPS IN
//******************
//  Seascan PIN --------
//  P2.0 -> PPS IN
//
//  P1.0 -> LED1
//  P1.1 -> LED2
//*********************

#include  <msp430x54xA.h>
#include <stdbool.h>
#include <stdio.h>

#define I2CTIMEOUT 100


#define GPS_PPS  BIT2
#define SEA_PPS  BIT0    //-- 手動黏板 這裡要注意~~~~ 看腳位定義~~~


void Crystal_Init(); 
void Timer_Init();

long int CloCk_tick = 0;

char string[100];
volatile unsigned char command_mode = 0;
//-----------
//--
//
//command_mode = 1; 要即時資料
//command_mode = 2; 要時間差
//
//
//--
//--------------------

unsigned char *PTxData;                     // Pointer to TX data
unsigned char TxData2[] =              // Table of data to transmit
{  0x01,  0x02,  0x03,  0x04,  0x05,  0x06,  0x07,  0x08,};

unsigned char GPS_timestr_BUF[8];


typedef struct {
	unsigned char RTC_Serial[8];    // int year month day hour minute sec ms1 ms2
                                        // 
        int	gps_flag;		// GPS 訊號是否進來
   long	int 	diff_sec;		// 時間差_ GPS與RTC內部時間   秒
   long int	diff_msec;		// 時間差_ GPS與RTC內部時間   豪秒
        int	rtc_mode;		// RTC目前狀態 ----
        bool    is_leapyear;            //判斷閏年
                                        //             1.未與GPS對時過 
                                        //             2.完成對時
}SEASCAN_RTC;

SEASCAN_RTC senscan_Rtc;




void I2C_Init();
void uart_GPS_init();

void Get_Diff_Init();


//---------------------------------------------------------------------------
void main( void )
{
  // Stop watchdog timer to prevent time out reset
  WDTCTL = WDTPW + WDTHOLD;
  P1DIR |= 0x01;
  
  Crystal_Init();
  //UART_Init(COM1);
  Timer_Init();

  I2C_Init();              //-- 啟動I2C 
  uart_GPS_init();         //-- 啟動GPS
  Get_Diff_Init();         //-- 啟動IO中斷
  
  
  
  P1DIR |= (BIT0+BIT1+BIT2);   // LED1 2
//  P1OUT |= (BIT0+BIT1);   // LED1 2
  P1OUT &= ~(BIT0+BIT1+BIT2);   // LED1 2
  
  
  senscan_Rtc.is_leapyear=false;
  senscan_Rtc.rtc_mode = 1;
  senscan_Rtc.RTC_Serial[0]=10;
  senscan_Rtc.RTC_Serial[1]=20;
  senscan_Rtc.RTC_Serial[2]=11;
  senscan_Rtc.RTC_Serial[3]=15;
  senscan_Rtc.RTC_Serial[4]=12;
  senscan_Rtc.RTC_Serial[5]=11;
  senscan_Rtc.RTC_Serial[6]=22;
  
  senscan_Rtc.gps_flag = 0;
  while(1){
  
    __bis_SR_register(LPM0_bits + GIE);     // LPM0, ADC12_ISR will force exit 
    
    
    if(senscan_Rtc.gps_flag == 1){
      P1OUT|=BIT0;
    }
    else{
      P1OUT&= ~BIT0;
    }
    
    
   
  }
  
  //return 0;
}
//------------------------------------------------------------------------------
void Get_Diff_Init(){
     //--  P2.1 2.2 GPS Seascan 1pps in
     //--  P2.1 GPS Lo/Hi
     //--  P2.2 Seascan Hi/Lo
  
  P2SEL = 0 ;
  P2REN |= (GPS_PPS+SEA_PPS);                            // Enable DRDY internal resistance  P2!!!!
  P2OUT |= (GPS_PPS+SEA_PPS);                            // Set DRDY as pull-Up resistance
  P2IES &= ~GPS_PPS;                                  //  Lo/Hi edge
  P2IES |= SEA_PPS;                                   //  Hi/Lo edge  

  P2IE |= (GPS_PPS+SEA_PPS);                             //  interrupt enabled
  P2IFG &= ~(GPS_PPS+SEA_PPS);                           //  IFG cleared

  

}
// Port 2 interrupt service routine
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
  if((P2IFG & SEA_PPS) ==SEA_PPS){ 
  
      CloCk_tick = 0;     //  ms = 0;
    
    
      senscan_Rtc.RTC_Serial[5]++;
  
      if(senscan_Rtc.RTC_Serial[5]>=60){senscan_Rtc.RTC_Serial[5]=0;senscan_Rtc.RTC_Serial[4]++;}
      if(senscan_Rtc.RTC_Serial[4]>=60){senscan_Rtc.RTC_Serial[4]=0;senscan_Rtc.RTC_Serial[3]++;}
      if(senscan_Rtc.RTC_Serial[3]>=24){senscan_Rtc.RTC_Serial[3]=0;senscan_Rtc.RTC_Serial[2]++;}
   
      if(senscan_Rtc.is_leapyear){
        if(senscan_Rtc.RTC_Serial[2]>29){  if(senscan_Rtc.RTC_Serial[1]==2 ){senscan_Rtc.RTC_Serial[2]=1;senscan_Rtc.RTC_Serial[1]++;}   }   
        if(senscan_Rtc.RTC_Serial[2]>30){  if((senscan_Rtc.RTC_Serial[1]==4)||(senscan_Rtc.RTC_Serial[1]==6)||(senscan_Rtc.RTC_Serial[1]==9)||(senscan_Rtc.RTC_Serial[1]==11)){senscan_Rtc.RTC_Serial[2]=1;senscan_Rtc.RTC_Serial[1]++;}   }
        if(senscan_Rtc.RTC_Serial[2]>31){  senscan_Rtc.RTC_Serial[2]=1;senscan_Rtc.RTC_Serial[1]++;   }
      }
      else{
        if(senscan_Rtc.RTC_Serial[2]>28){  if(senscan_Rtc.RTC_Serial[1]==2 ){senscan_Rtc.RTC_Serial[2]=1;senscan_Rtc.RTC_Serial[1]++;}   }   
        if(senscan_Rtc.RTC_Serial[2]>30){  if((senscan_Rtc.RTC_Serial[1]==4)||(senscan_Rtc.RTC_Serial[1]==6)||(senscan_Rtc.RTC_Serial[1]==9)||(senscan_Rtc.RTC_Serial[1]==11)){senscan_Rtc.RTC_Serial[2]=1;senscan_Rtc.RTC_Serial[1]++;}   }
        if(senscan_Rtc.RTC_Serial[2]>31){  senscan_Rtc.RTC_Serial[2]=1;senscan_Rtc.RTC_Serial[1]++;   }    
      }
      if(senscan_Rtc.RTC_Serial[1]>12){senscan_Rtc.RTC_Serial[1]=1;senscan_Rtc.RTC_Serial[0]++;
         if( ((senscan_Rtc.RTC_Serial[0]%4)==0) &&((senscan_Rtc.RTC_Serial[0]%100)!=0) || (senscan_Rtc.RTC_Serial[0]%400)==0){  //--//-閏年   2月有29天
                    senscan_Rtc.is_leapyear = true;      }
         else{      senscan_Rtc.is_leapyear = false;  } 
      }
    
     
      
  //  __bic_SR_register_on_exit(LPM0_bits); // Exit LPM0
  //  return;
  }
  if((P2IFG & GPS_PPS) == GPS_PPS){
  
       senscan_Rtc.diff_msec = CloCk_tick;
       TxData2[0] = senscan_Rtc.diff_msec/10000;
       TxData2[1] = (senscan_Rtc.diff_msec-(TxData2[0]*10000))/1000;
       TxData2[2] = (senscan_Rtc.diff_msec-(TxData2[0]*10000)-(TxData2[1]*1000))/100;
       TxData2[3] = (senscan_Rtc.diff_msec-(TxData2[0]*10000)-(TxData2[1]*1000)-(TxData2[2]*100))/10;
       TxData2[4] = (senscan_Rtc.diff_msec-(TxData2[0]*10000)-(TxData2[1]*1000)-(TxData2[2]*100)-(TxData2[3]*10));
       TxData2[5] = senscan_Rtc.diff_sec;

              if((senscan_Rtc.gps_flag==1) && (senscan_Rtc.rtc_mode==1)){ 
                senscan_Rtc.rtc_mode = 2;  //--- GPS 字串更新完成  -- 僅做一次(上電後收到GPS做)
                senscan_Rtc.gps_flag=0;    //
                

                senscan_Rtc.RTC_Serial[0]=GPS_timestr_BUF[0];
                senscan_Rtc.RTC_Serial[1]=GPS_timestr_BUF[1];
                senscan_Rtc.RTC_Serial[2]=GPS_timestr_BUF[2];
                
                senscan_Rtc.RTC_Serial[3]=GPS_timestr_BUF[3];
                senscan_Rtc.RTC_Serial[4]=GPS_timestr_BUF[4];
                senscan_Rtc.RTC_Serial[5]=GPS_timestr_BUF[5];
                
                   if( ((senscan_Rtc.RTC_Serial[0]%4)==0) &&((senscan_Rtc.RTC_Serial[0]%100)!=0) || (senscan_Rtc.RTC_Serial[0]%400)==0){  //--//-閏年   2月有29天
                    senscan_Rtc.is_leapyear = true;      }
                    else{      senscan_Rtc.is_leapyear = false;  } 
              }
              else if((senscan_Rtc.gps_flag==1) && (senscan_Rtc.rtc_mode==2)){
                
                    
                    senscan_Rtc.diff_sec = 
                    (((long int)senscan_Rtc.RTC_Serial[2]*24*60*60)+((long int)senscan_Rtc.RTC_Serial[3]*60*60)+((long int)senscan_Rtc.RTC_Serial[4]*60)+senscan_Rtc.RTC_Serial[5])-
                    (((long int)GPS_timestr_BUF[2]*24*60*60)+((long int)GPS_timestr_BUF[3]*60*60)+((long int)GPS_timestr_BUF[4]*60)+GPS_timestr_BUF[5]);
                    senscan_Rtc.gps_flag=0;
              }
       
       
         
  }
  

  P2IFG &= ~(GPS_PPS+SEA_PPS);                           //  IFG cleared
  __bic_SR_register_on_exit(LPM0_bits); // Exit LPM0
      //return;
}
//------------------------------------------------------------------------------

void Timer_Init(){

     TA0CCTL0 = CCIE;                          // CCR0 interrupt enabled
     TA0CCR0 = 1200;                           //10kHz  0.1ms
     TA0CTL = TASSEL_2 + MC_1 + TACLR;         // SMCLK, upmode, clear TAR

}
void Crystal_Init(){
    //--啟動 12MHZ 與32767HZ
    P5SEL |= 0x0C;                             // Port select XT2
    P7SEL |= 0x03;                             // Port select XT1
    UCSCTL6 &= ~(XT1OFF + XT2OFF);            // Set XT1 & XT2 On
    UCSCTL6 |= XCAP_3;                        // Internal load cap

    do    // Loop until XT1,XT2 & DCO stabilizes
    {
      UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + XT1HFOFFG + DCOFFG);
                                              // Clear XT2,XT1,DCO fault flags
      SFRIFG1 &= ~OFIFG;                      // Clear fault flags
    }while (SFRIFG1&OFIFG);                   // Test oscillator fault flag

    UCSCTL6 &= ~XT2DRIVE0;                    // Decrease XT2 Drive according to expected frequency
    UCSCTL4 |= SELA_0 + SELS_5 + SELM_5;      // Select SMCLK, ACLK source and DCO source
   

  
}
//--------------------------------------------------------------------------------

#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
{
  CloCk_tick++;
   //P1OUT ^= 0x01;
  if(CloCk_tick>=10000){
    CloCk_tick=0;      
    P1OUT ^= 0x01;
    
     /*if(senscan_Rtc.gps_flag == 1){
       senscan_Rtc.gps_flag = 0;
        P1OUT ^=BIT1;
     }
    */
     /*
     string[0] = '!';
     string[1] = 'x';
     string[2] = '0';
     string[3] = 13;
     string[4] = 10;
     string[5] = 0;
     UART_SendStr(string,COM1);
     */
   //__bic_SR_register_on_exit(LPM0_bits); // Exit LPM0
    
    
    
  }
}
//----------------------------------------------------------------------------
void I2C_Init(){

  P3SEL |= 0x06;                            // Assign I2C pins to USCI_B0
  UCB0CTL1 |= UCSWRST;                      // Enable SW reset
  UCB0CTL0 = UCMODE_3 + UCSYNC;             // I2C Slave, synchronous mode
  UCB0I2COA = 0x48;                         // Own Address is 048h
  UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
  UCB0IE |= UCTXIE + UCSTPIE + UCSTTIE;     // Enable STT and STP interrupt
                                            // Enable TX interrupt
  UCB0IE |= UCRXIE;                         // Enable RX interrupt

}


//------------------------------------------------------------------------------
#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
{
  switch(__even_in_range(UCB0IV,12))
  {
  case  0: break;                           // Vector  0: No interrupts
  case  2: break;                           // Vector  2: ALIFG
  case  4: break;                           // Vector  4: NACKIFG
  case  6:                                  // Vector  6: STTIFG
    UCB0IFG &= ~UCSTTIFG;                   // Clear start condition int flag
    break;
  case  8:                                  // Vector  8: STPIFG
    UCB0IFG &= ~UCSTPIFG;                   // Clear stop condition int flag
    //__bic_SR_register_on_exit(LPM0_bits);   // Exit LPM0 if data was transmitted
    break;
  case 10:                            // Vector 10: RXIFG
    command_mode = UCB0RXBUF;                     // Get RX data
    if(command_mode==1){
      
     //P1OUT |=BIT2; 
      
      senscan_Rtc.RTC_Serial[6] =(unsigned char)(CloCk_tick >> 8);
      senscan_Rtc.RTC_Serial[7] =(unsigned char) CloCk_tick;
      
      PTxData = (unsigned char *)senscan_Rtc.RTC_Serial ;      // Start of TX buffer
    }
    if(command_mode==2){
     P1OUT ^=BIT2; 
      PTxData = (unsigned char *)TxData2;      // Start of TX buffer
    }
    break;
    
  case 12:                                  // Vector 12: TXIFG  
    UCB0TXBUF = *PTxData++;                 // Transmit data at address PTxData
    break;
  default:
   
   __no_operation();
    break;
  }
}

void uart_GPS_init(){

    P5SEL |= 0xc0;                             // P5.6,7 = USCI_A0 TXD/RXD
      UCA1CTL1 |= UCSWRST;                      // **Put state machine in reset**

      UCA1CTL1 |= UCSSEL_2;                     // CLK = SCLK
      UCA1BR0 = 0xe2; 
      UCA1BR1 = 0x04;                           //
      UCA1MCTL = UCBRS_0+UCBRF_0;               // Modulation UCBRSx=3, UCBRFx=0

      UCA1CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
      UCA1IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt

}
//---------------- 接收來自GPS的字串

int get_GPS_str_counter = 0;

char get_GPS_str_BUF[128];


//----------------
#pragma vector=USCI_A1_VECTOR 
__interrupt void USCI_A1_ISR(void)
{
  
  //---命令格式    GPS
  switch(__even_in_range(UCA1IV,4))
  {
  case 0:break;                             // Vector 0 - no interrupt
  case 2:                                   // Vector 2 - RXIFG
//    while (!(UCA1IFG&UCTXIFG));           // USCI_A1 TX buffer ready?
//    UCA1TXBUF = UCA1RXBUF;                // TX -> RXed character
    get_GPS_str_BUF[get_GPS_str_counter] = UCA1RXBUF;
    get_GPS_str_counter++;
    
    if(get_GPS_str_counter>=120){
      get_GPS_str_counter=0;   
    }
    else{
       if(get_GPS_str_BUF[get_GPS_str_counter-1]=='\n'){
         if(get_GPS_str_counter>65){
           if((get_GPS_str_BUF[0]=='$')&&(get_GPS_str_BUF[1]=='G')&&(get_GPS_str_BUF[2]=='P')&&(get_GPS_str_BUF[3]=='R')&&(get_GPS_str_BUF[4]=='M')&&(get_GPS_str_BUF[5]=='C')&&(get_GPS_str_BUF[17]=='A')){
                senscan_Rtc.gps_flag = 1;
           
                GPS_timestr_BUF[0]=(get_GPS_str_BUF[57]-48)*10+(get_GPS_str_BUF[58]-48);
                GPS_timestr_BUF[1]=(get_GPS_str_BUF[55]-48)*10+(get_GPS_str_BUF[56]-48);
                GPS_timestr_BUF[2]=(get_GPS_str_BUF[53]-48)*10+(get_GPS_str_BUF[54]-48);
                
                GPS_timestr_BUF[3]=(get_GPS_str_BUF[7]-48)*10+(get_GPS_str_BUF[8]-48);
                GPS_timestr_BUF[4]=(get_GPS_str_BUF[9]-48)*10+(get_GPS_str_BUF[10]-48);
                GPS_timestr_BUF[5]=(get_GPS_str_BUF[11]-48)*10+(get_GPS_str_BUF[12]-48);
                
                
                
           }
         }
         
         get_GPS_str_counter=0;   
       }
    }
    
   
      
    break;
  case 4:break;                             // Vector 4 - TXIFG
  default: break;  
  }
 //   __bic_SR_register_on_exit(LPM3_bits); // Exit LPM0
}
//----------------------------------------------------------------------------
