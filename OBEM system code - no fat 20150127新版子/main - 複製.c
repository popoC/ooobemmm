//-------------P6---------------------------------------------------------------
//       P1.5
//         |
//      ___↓_                    ________________             _____
//     |      |P1.3  ->  P6.1    |                |<---  Rx   |     |
//     |      |P1.4  ->  P6.2    |                |--->  Tx   |  PC |
//     |      |P1.1  <-  P6.3    |                |--->  Gnd  |_____|
//     | 2013 |P1.2  <-  P6.4    |    430f5436     |
//     |      |                  |                |
//     |      |                  |                |
//     |______|P1.7  <-  P6.7    |________________|
//------------------------------------------------------------------------------
// 2015 0129 OBEM SYSTEM BOARD !! V1.0

#include  <msp430x54xA.h>
#include <stdio.h>
#include "main.h"

#include "uart.h"
#include "ads1278.h" 
#include "ff.h"


bool StartRecord = false;
long int wait_for_start = 0;

int OUTPUT_MODE = AD_DATA;
//-----------------------

void Crystal_Init();   
//----------for seascan---------------------------------------------------------
#define TimeOut 5000
   void getTime_init();
   void get_time();    // ----- 2.56ms about~
   void IO_Send_Byte(int SendBuf);
    int get_byte();
                  
  unsigned char RTC_Serial[8]; // int ms,sec,minute,hour,day,month,year;  2012 1224 fix
  bool Set_RTC_Flag = false;                  
                    
//----------for rs232-----------------------------------------------------------
char COM2_Command[COM_BUF_Size];
char string[150];
void delay_ms(int del);

//----------for sd card---------------------------------------------------------
 
//--全域變數----控制


 unsigned char Save_flag = 1;
 unsigned long save_data_block_counter = 0;
 unsigned long file_sn=0;
 
#define save_block_MAX 300*1*60   //--- 大約 30分鐘

 bool file_is_create = false;
 
 
//------------------------------------------------------------------------------
  
  int out_timer_counter = 0;             //-- 不要讓輸出時間字串的頻率太高~
  
    
//------------------------------------------------------------------------------
    static unsigned long ad_data[8];

//--------
 
 DWORD get_fattime (void)
{
  DWORD testtime;
 // get_time();   //-- 跟MSP430F2013 要時間     
       testtime = 0;    
  testtime = ((DWORD)(RTC_Serial[0]+20)<<25)
             |((DWORD)(RTC_Serial[1]) << 21)
             |((DWORD)(RTC_Serial[2]) << 16)
             |((DWORD)(RTC_Serial[3]) << 11)
             |((DWORD)(RTC_Serial[4]) << 5)
             |((DWORD)(RTC_Serial[5]) >> 1);
  
  //testtime = ((DWORD)(2012 - 1980) << 25)|((DWORD)3 << 21)|((DWORD)22 << 16)|((DWORD)0 << 11)|((DWORD)16 << 5)|((DWORD)0 >> 1);
  return testtime;
}
    
    
void main(void)
{

   WDTCTL = WDTPW + WDTHOLD;                    // Stop WDT
   P1DIR |= BIT0 +BIT1 ;
   P1OUT |= BIT0;                           //   爺亨版
   
   //----- system init ------
   Crystal_Init();                          // 震盪器初始化

   UART_Init(COM2);                         // Rs232初始化    
   ADS1278_Init();

   getTime_init();                          // MSP430F2013 connect
   
   
   
   
  
   //------------------
   
   
  while(!StartRecord){
          __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0, enable interrupts 此處中斷大約100Hz
           wait_for_start++;
           if(wait_for_start > 180000)StartRecord = true;   //  自動啟動裝置 大約三十分鐘
               
                if(Set_RTC_Flag){
                 Set_RTC_Flag = false;
                   P6OUT|=BIT7 ;
                   //set time to msp430f2013
                   IO_Send_Byte(RTC_Serial[0]);
                   P6OUT&= ~BIT7;
                   for(int i=1;i<7;i++) IO_Send_Byte(RTC_Serial[i]); 
                }   
                
         switch(OUTPUT_MODE){
           case LOGGER_TIME:
             out_timer_counter++;
             if(out_timer_counter>=10){
               out_timer_counter = 0;
                     string[0] = 't';                  //資料型態 fix 2012 1224
                     string[9] = 0;  string[10] = 1;  
              get_time();      memcpy(&(string[1]),RTC_Serial,8);
              RS232_Send_Char(string,11,COM2);
             }
           break;      
           case AD_DATA:
                if(ADS1278_Get_Data(ad_data)==1){
                  P1OUT |= 0x01;
                  for(int m=0;m<8;m++)  //--- 這理輸出時間會比較慢~ 跟真實的採樣率不同 會卡到
                  {
                   string[0] = 'd';                   //資料型態
                   string[1] = (ad_data[m] >> 16 ) & 0xff ;
                   string[2] = (ad_data[m] >> 8 ) & 0xff;
                   string[3] = (ad_data[m]) & 0xff;
                   string[4] =  m;  string[5] =  '\r';
                   RS232_Send_Char(string,7,COM2);  
                  }
                }
          break;
          case RE_SET:
            OUTPUT_MODE = 0;  WDTCTL = WDT_ARST_1000;  //--開狗    
           break;    
         }    
      
  }
  //-------------------------------- start record-----------------------------
    //----- ----------- ------
 /*  
      unsigned int bw;
      FATFS fs;
      FIL file1;
  
   unsigned char memory[512];
   f_mount(&fs,"",0);
   //f_open(&file1,"test00.txt",FA_OPEN_EXISTING|FA_WRITE|FA_READ); //-- 讀取已經存在的檔案
   f_open(&file1,"test00.txt",FA_CREATE_ALWAYS|FA_WRITE|FA_READ);   //-- 總是新建一個檔案
   
   static DWORD sizeofdata = 0;
   sizeofdata = file1.fsize ;
   
   
   
   f_lseek(&file1,sizeofdata); //-- 移動到檔案最後面~ 如果檔案存在的話
   
   
   for(int i=0;i<10;i++){
      for(int j=0;j<512;j++)memory[j]=i+49;
      f_write(&file1,memory,sizeof(memory),&bw);
   
   }
   
  f_close(&file1);
   
   */
  //-----------------------------
     unsigned int bw;
      FATFS fs;
      FIL file1;
      f_mount(&fs,"",0);   

      char file_name[50];
      unsigned char data_memory[512];

     static FRESULT res1,res2,res3;
      
  while(StartRecord){
    __bis_SR_register(LPM0_bits + GIE); 
     
    
    if(save_data_block_counter == 0){
       get_time();
       file_sn = 0;
      // for(int i=0;i<50;i++)file_name[i]=0;
       memset(file_name,0,50);
       file_sn = ((DWORD)(RTC_Serial[2]) << 16)
             |((DWORD)(RTC_Serial[3]) << 11)|((DWORD)(RTC_Serial[4]) << 5)|((DWORD)(RTC_Serial[5]) >> 1);

       sprintf(file_name,"%05ld.dat",file_sn); 
      
       res1 = f_open(&file1,file_name,FA_CREATE_ALWAYS|FA_WRITE|FA_READ);
       if(res1==FR_OK){   //-- 總是新建一個檔案
       
         file_is_create = true;
       }
    }
   
     P1OUT &= ~BIT0; 
    get_time();
    if((get_ads1278_databuffer(data_memory))  && (file_is_create)){
   //  file_is_create = false;
     data_memory[0] = (save_data_block_counter>>8) & 0xff;;
     data_memory[1] = save_data_block_counter & 0xff;
     
     data_memory[512-1] = RTC_Serial[7];
     data_memory[512-2] = RTC_Serial[6];
     data_memory[512-3] = RTC_Serial[5];
      
     res2 =  f_write(&file1,data_memory,sizeof(data_memory),&bw);
     
     
     
     
     if(res2 != FR_OK){
        while(1);
     }
      save_data_block_counter++;
    }
    P1OUT |= BIT0;  
    
    
/*
    
    if(ads1278_data_counter==data_in_buffer){
      
      P1OUT |= BIT0;   
      
     if(Save_flag == 2){
       f_write(&file1,data_memory_1,sizeof(data_memory_1),&bw);
       for(int i=0;i<512;i++)data_memory_1[i]=0;
     }
     else{
       f_write(&file1,data_memory_2,sizeof(data_memory_2),&bw);
       for(int i=0;i<512;i++)data_memory_2[i]=0;
     }
      
      save_data_block_counter++; 

     P1OUT &= ~BIT0;     
    }
  */  
    
    
    if(save_data_block_counter==save_block_MAX){
        res3 = f_close(&file1);
          file_is_create = false;
        if(res3==FR_OK){   
         save_data_block_counter = 0;
         
       }
        else{
         save_data_block_counter = 0;
        }
        
    }
  
    
  
  
  }
  
  
 /* 
  get_time();
  
  P1OUT &= ~0x01;
  
  if(!SD_INIT()){return;}                 // sd card 初始化
  if(read_SD_format()){return;}           // read SD card format 
  
  
  
  while(StartRecord){
  
        __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0, enable interrupts
               
    if(ADS1222_Interrupt_Flag){
    
           ADS1222_Interrupt_Flag = false;

           Update_Cont++;


           if((Update_Cont%6) == 0){
            
             P1OUT |= 0x01;
              get_time();  
              memcpy(&(string[1]),RTC_Serial,8);
             RS232_Send_Char(string,11,COM2);
              P1OUT &= ~0x01;
           
           }           

           if(Update_Cont == Update_Per_Block){
             Update_Cont = 0;
             
             update_table();

            
           }

           if(check_SD_card)WDTCTL = WDT_ARST_1000;  //--開狗
         
           do_storage_2();           //

    }

    if(OUTPUT_MODE==RE_SET){
        OUTPUT_MODE = 0; 
       WDTCTL = WDT_ARST_1000;  //--開狗 RESET
    }      
    
    
  }
  */
   __no_operation();                          // For debugger                   
}
//----------------------------------------------------------------------------
void Crystal_Init(){
    //--啟動 16MHZ 與32767HZ
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
 
  P4SEL = 0x02;                               // P4 option select 
  P4DIR = 0x02;                               // P4 outputs 100kHz...min....
  TBCCR0 = 200-1;                               // PWM Period  100 K
  TBCCR1 = 100;                                 // CCR1 PWM Duty Cycle	
  TBCCTL1 = OUTMOD_2;                         // PWM toggle/reset 

  TBCTL = TBSSEL_2 + MC_1 + TBCLR;            // SMCLK, upmode, clear TBR    
  
}
//--------------------------------------------------------------------------------



//----------------------------------------------------------------------------
void delay_ms(int del){
int dt;
  for(int delAy=0;delAy<del;delAy++)    //--delay 
    for(dt=0;dt<4000;dt++);
}
//----------------------------------------------------------------------------

void getTime_init(){
  P6DIR |= (BIT3+BIT4+BIT7);      //out   BIT7 很重要
  P6DIR &= ~(BIT1+BIT2+BIT6);     //input
  P6OUT &= ~(BIT4+BIT3+BIT7);
}
void get_time(){
  memset(RTC_Serial,0,8);
   P6OUT |= BIT3;
   RTC_Serial[0] = get_byte();
   P6OUT &= ~BIT3;
 for(int i=1;i<8;i++) RTC_Serial[i] = get_byte();
   
}

int get_byte(){
int buf=0;  int time = 0;
 for(int i=0;i<8;i++){
  while(((P6IN&BIT2)!=BIT2)&&(time < TimeOut)){time++;}
   time = 0;
   P6OUT |= BIT4;
  while(((P6IN&BIT2)==BIT2)&&(time < TimeOut)){time++;}
   time = 0;
   if(P6IN & BIT1) buf+=0x01 <<i;
   P6OUT &= ~BIT4;
 }
 return(buf);
}
//------------------------------------------------------------------------------
void IO_Send_Byte(int Send_Buf){
   int SendN = 0;
   int time = 0;
     for(SendN=0;SendN<8;SendN++){
        P6OUT|=BIT4;
        while(((P6IN&BIT2)!=BIT2)&&(time < TimeOut) ){time++;}   //代表資料可以送
        time = 0;
        if((Send_Buf&0x01)==0x01){
          P6OUT|=BIT3;
        }
        else{
          P6OUT&= ~BIT3;
        }
        Send_Buf >>=1 ;
        P6OUT&=~BIT4;
        while(((P6IN&BIT2)==BIT2)&&((time < TimeOut))){time++;}   //代表資料收走了
        time = 0;
     }
}
//------------------------------------------------------------------------------

