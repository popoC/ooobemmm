// 2015 0129 OBEM SYSTEM BOARD !! V1.0
#include  <msp430x54xA.h>
#include "stdio.h"
#include "main.h"
#include "uart.h"
#include "spi.h"
#include "sdhc.h" 

#define save_block_MAX 300*1*2   //--- 大約 ?分鐘


//---------- 記憶體寫入法

#define Update_Per_Block 30
int Update_Cont = 0;
 unsigned char Save_flag = 1;
 int total;
u32 startblock, endblock;
u32 blockaddress, modemaddress = 0;
 
 
int check_SD_card = 1;
int SD_INIT();
int do_sd_initialize (sd_context_t *sdc);
int read_SD_format();
void do_storage_2();
void update_table(void);



//------------------------------------------------------------------------------
bool StartRecord = false;           // 模式切換 記錄或設定?
int OUTPUT_MODE = AD_DATA;          // 模式選擇-- 透過RS232變更
long int wait_for_start = 0;        // 自動重啟 計數
int out_timer_counter = 0;          //-- 不要讓輸出時間字串的頻率太高~
//----------DATA FLOW ----------------------------------------------------------
 unsigned char data_memory_1[Max_Data_Buffer][512];       //資料暫存器
 bool data_buffer_state[Max_Data_Buffer];                 //資料暫存器狀態
 BYTE write_data[512];                                    //寫入區

 //--全域變數----控制
unsigned long save_data_block_counter = 0;

//------------------------------------------------------------------------------
  
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
char string[150];
//------------------------------------------------------------------------------
BYTE view_ad_data[8*3];

int si,sj;
int sQs;
BYTE databyte[8][3];
BYTE data_value[8]={128,64,32,16,8,4,2,1};

bool adc_busy = false;
 
 unsigned int  ads1278_data_counter = 0;
 unsigned int data_in_buffer = 480; //-- 20筆資料   24*20
 int save_flag = 0;
 int buffer_counter = 0;
 bool data_buffer_is_full = false;
 static bool save_fq = 0;   //-- 198Hz -> 99Hz 用
 
 
 unsigned long ads1278_data[8];
 unsigned long view_ads1278_data[8];
 int check_counter = 0;



//------------------------------------------------------------------------------
          FATFS fs2; 
          FIL file12;
          
void main(void)
{
   WDTCTL = WDTPW + WDTHOLD;                    // Stop WDT
 
   
   P1DIR = 0;
   P1DIR |= BIT0;                            //LED
   P1OUT |= BIT0;                           //   爺亨版
   //----- system init ------
   Crystal_Init();                          // 震盪器初始化
   UART_Init(COM2);                         // Rs232初始化    
   ADS1278_Init();
   getTime_init();                          // MSP430F2013 connect
  
    _EINT();  //啟動全域中斷  
    
    
  //   __no_operation();                          // For debugger 
   //----- sd 讀寫測試
     StartRecord = 1; 
  
      

      f_mount(&fs2,"",0);   //-- 檢查SD卡   
    //  f_mkfs(0,0,0);
      
      int file_sn2=0;
      BYTE gg_data_memory[512];
      
      for(int ii=0;ii<50;ii++){
        
       file_sn2= ii;
       sprintf(file_name,"%05d.txt",file_sn2);
       res1= f_open(&file12,file_name,FA_CREATE_ALWAYS|FA_WRITE);
     if(res1 !=FR_OK){
           __no_operation();
          }
       for(int i=0;i<300;i++){
         P1OUT ^= 0x01;
            memset(gg_data_memory,0,512); 
            memset(gg_data_memory,'G',i%512); 
            
          res2=  f_write(&file12,gg_data_memory,512,&bw);
          if(res2 !=FR_OK){
           __no_operation();
          }
       
          
       }
     
  //   P1OUT &= ~0x01;
   //  P1OUT |= 0x01;
     

        
       res3 =f_close(&file12);
        if(res3 !=FR_OK){
           __no_operation();
          }
        if(ii==10){
        __no_operation();                          // For debugger 
        }
     
        __no_operation();                          // For debugger 
      }
   __no_operation();                          // For debugger 
   while(1);
        
   StartRecord = false;
   
   
   //----------
   
  while(!StartRecord){
          __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0, enable interrupts 此處中斷大約 200Hz
           wait_for_start++;
           if(wait_for_start > 360000){
             
             for(int ig=0;ig<Max_Data_Buffer;ig++)data_buffer_state[ig]=false;  

             StartRecord = true;   //  自動啟動裝置 大約三十分鐘
               
           }
               
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
                if(ADS1278_Get_Data2(view_ad_data)==1){
            //      P1OUT |= 0x01;
                  for(int m=0;m<8;m++)  //--- 這理輸出時間會比較慢~ 跟真實的採樣率不同 會卡到
                  {
                   string[0] = 'd';                   //資料型態
                   string[1] = view_ad_data[m*3+0];
                   string[2] = view_ad_data[m*3+1];
                   string[3] = view_ad_data[m*3+2];
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
  //-----------------------------
     

	  if(!SD_INIT()){return;}                 // sd card 初始化
  if(read_SD_format()){return;}           // read SD card format
	 
	while(StartRecord){
             __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0, enable interrupts
               
	   for(int y = 0;y<Max_Data_Buffer;y++){
          if(data_buffer_state[y]==true){              //需要寫資料
               get_time();   // ? 測試後再看要不要放
           P1OUT |= BIT0; 
             data_buffer_state[y] = false;

		   Update_Cont++;
           if((Update_Cont%6) == 0){
              P1OUT |= 0x01;
             string[0] = 't';
              memcpy(&(string[1]),RTC_Serial,7);
              RS232_Send_Char(string,11,COM2);
              P1OUT &= ~0x01;
           }           
           if(Update_Cont == Update_Per_Block){
             Update_Cont = 0;
             update_table();
           }
			 
              for(int m=0;m<512;m++)write_data[m]=0;
               write_data[0] = (save_data_block_counter>>8) & 0xff;;
               write_data[1] = save_data_block_counter & 0xff;
               write_data[512-1] = RTC_Serial[7];   write_data[512-2] = RTC_Serial[6];
               write_data[512-3] = RTC_Serial[5];   write_data[512-4] = RTC_Serial[4];
               write_data[512-5] = RTC_Serial[3];   write_data[512-6] = RTC_Serial[2];
               for(int p=2;p<512-3;p++)write_data[p] = data_memory_1[y][p];
			   
            check_SD_card = sd_write_block(&sdc, blockaddress, sd_buffer1);
            sd_wait_notbusy (&sdc);
              for(int p=0;p<512;p++)data_memory_1[y][p]=0;              
                if( blockaddress >= 63078300)
                  {
                    //-do something for protect data
                    return ;
                  }
                  else{
                    blockaddress++;
					save_data_block_counter++;
					if(save_data_block_counter==save_block_MAX)save_data_block_counter=0;
                  }           
          P1OUT &= ~BIT0;  
        }
        
      }	   
			   
		   
			   
  
    if(OUTPUT_MODE==RE_SET){
        OUTPUT_MODE = 0; 
       WDTCTL = WDT_ARST_1000;  //--開狗 RESET
    }      
    
    
  }
	 
	 
	 
	 
	 
	 //-----------------------


    
    while(StartRecord){
      
        //  P1OUT ^= BIT0;   
    if(save_data_block_counter == 0 ){
       get_time();
       
       memset(file_name,0,50);
       
       file_name[0] = RTC_Serial[1]+65;       file_name[1] = RTC_Serial[2]/10+48;
       file_name[2] = RTC_Serial[2]%10+48;    file_name[3] = RTC_Serial[3]+65;
       file_name[4] = RTC_Serial[4]/10+48;    file_name[5] = RTC_Serial[4]%10+48;
       file_name[6] = '.';                    file_name[7] = 't';
       file_name[8] = 'x';                    file_name[9] = 't';     
       file_name[10] = 0;

//       res1 = f_open(&file1,file_name,FA_CREATE_ALWAYS|FA_WRITE);
       res1 = f_open(&file1,file_name,FA_CREATE_ALWAYS|FA_WRITE|FA_READ);
       if(res1==FR_OK){                                          //-- 新建一個檔案
         file_is_create = true;
         save_data_block_counter++;
       }
       
       
       
    }
    if(file_is_create){
      
      for(int y = 0;y<Max_Data_Buffer;y++){
        if(data_buffer_state[y]==true){
          get_time();   // ? 測試後再看要不要放
           P1OUT |= BIT0; 
           data_buffer_state[y] = false;
           
               //memcpy(write_data,data_memory_1[y],512);
               //memset(data_memory_1[y],0,512);  

           for(int m=0;m<512;m++)write_data[m]=0;
           
               write_data[0] = (save_data_block_counter>>8) & 0xff;;
               write_data[1] = save_data_block_counter & 0xff;
               write_data[512-1] = RTC_Serial[7];
               write_data[512-2] = RTC_Serial[6];
               write_data[512-3] = RTC_Serial[5];
               
               for(int p=2;p<512-3;p++){
                 write_data[p] = data_memory_1[y][p];
               }
                            
            res2 =  f_write(&file1,write_data,512,&bw);
                       
               
           save_data_block_counter++;
  
          P1OUT &= ~BIT0;  
        }
        
      }
    }
    
    
    
    
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
  
   __no_operation();                          // For debugger                   
}
//----------------------------------------------------------------------------
void Crystal_Init(){
    //--啟動 20MHZ 與32767HZ
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
 
       if(RTC_Serial[1]>12)RTC_Serial[1]=12;
       if(RTC_Serial[2]>31)RTC_Serial[2]=31;
       if(RTC_Serial[3]>24)RTC_Serial[3]=24;
       if(RTC_Serial[4]>60)RTC_Serial[4]=0;
       if(RTC_Serial[5]>60)RTC_Serial[5]=0;
       
 
 
   
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
//------------------------------------------------------------------------------
 
 DWORD get_fattime (void)
{
  DWORD testtime;
  get_time();   //-- 跟MSP430F2013 要時間     
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
    

void ADS1278_Init(){
  P2SEL = 0;
  P1DIR |= (SCLK+SYNC);                       // Set SCLK SYNC be OutPut
  P2REN |= DRDY_1;                            // Enable DRDY internal resistance  P2!!!!
  P2OUT |= DRDY_1;                            // Set DRDY as pull-Up resistance
  P2IE |= DRDY_1;                             // DRDY interrupt enabled
  P2IES |= DRDY_1;                            // DRDY Hi/Lo edge//  P1IES &= ~DRDY_1;   Hi/Lo edge
  P2IFG &= ~DRDY_1;                           // DRDY IFG cleared

  P2DIR &= ~(BIT0+BIT1+BIT2+BIT3);            //set Data InPut P2.0~2.3 
  P1DIR &= ~(BIT4+BIT5+BIT6+BIT7);            //set Data InPut P1.4~1.7 
  
  P4SEL = 0x02;                               // P4 option select 
  P4DIR = 0x02;                               // P4 outputs 100kHz...min....
  TBCCR0 = 200-1;                               // PWM Period  200 K
  TBCCR1 = 100;                                 // CCR1 PWM Duty Cycle	
  TBCCTL1 = OUTMOD_2;                         // PWM toggle/reset 

  TBCTL = TBSSEL_2 + MC_1 + TBCLR;            // SMCLK, upmode, clear TBR    
  for(long int i=0;i<159600;i++){}  


}
int ADS1278_Get_Data2(BYTE *data){
  while(adc_busy);

  for(int k=0;k<8;k++){
   data[k*3+0] = databyte[k][0];
   data[k*3+1] = databyte[k][1];
   data[k*3+2] = databyte[k][2];
   
  }
  
    adc_busy = true;
  
    return(1);
 
}

int ADS1278_Get_Data(unsigned long *data){
  
  
  while(adc_busy);
  
    
   memcpy(&(data[0]),view_ads1278_data,4*8);
      
  
    
  
    adc_busy = true;
  
    return(1);
 
}

// Port 2 interrupt service routine
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
  if((P2IFG & DRDY_1) !=DRDY_1){ 
    P2IFG = 0;                                //  all IFG cleared
    __bic_SR_register_on_exit(LPM0_bits); // Exit LPM0
    return;
  }
  else{
    P2IFG &= ~DRDY_1;                          // P1.2 IFG cleared
  }
  
  save_fq^=0x01;   //-- 195Hz / 2 == 97Hz
  if(save_fq){
         __bic_SR_register_on_exit(LPM0_bits); // Exit LPM0
         return; 
 }
    for(sQs=0;sQs<8;sQs++){
        databyte[sQs][0] = 0;
        databyte[sQs][1] = 0;
        databyte[sQs][2] = 0;
    }
    __delay_cycles(200);   
   //__delay_cycles(400);   
  
  for(sj=0;sj<3;sj++){
    for(si=0;si<8;si++){
        SCLK_H;
       
      if((P2IN&BIT0)==BIT0)databyte[0][sj] = databyte[0][sj]+ data_value[si];
      if((P2IN&BIT1)==BIT1)databyte[1][sj] = databyte[1][sj]+ data_value[si];
      if((P2IN&BIT2)==BIT2)databyte[2][sj] = databyte[2][sj]+ data_value[si];
      if((P2IN&BIT3)==BIT3)databyte[3][sj] = databyte[3][sj]+ data_value[si];
      if((P1IN&BIT4)==BIT4)databyte[4][sj] = databyte[4][sj]+ data_value[si];
      if((P1IN&BIT5)==BIT4)databyte[5][sj] = databyte[5][sj]+ data_value[si];      
      if((P1IN&BIT6)==BIT4)databyte[6][sj] = databyte[6][sj]+ data_value[si];
      if((P1IN&BIT7)==BIT4)databyte[7][sj] = databyte[7][sj]+ data_value[si];
      
        __delay_cycles(150);
        SCLK_L;
        __delay_cycles(200);
    }
  }
  
       
  
  
  if(!StartRecord){
      check_counter++;
      if(check_counter==2){
        for(si=0;si<8;si++){
           view_ads1278_data[si] = ads1278_data[si];
        }
          check_counter = 0;   
          adc_busy = false;     
      } 
  }  
  
    if(StartRecord){
           
            data_memory_1[buffer_counter][(ads1278_data_counter)+2] = databyte[0][0];
            data_memory_1[buffer_counter][(ads1278_data_counter)+3] = databyte[0][1];
            data_memory_1[buffer_counter][(ads1278_data_counter)+4] = databyte[0][2];
            
            data_memory_1[buffer_counter][(ads1278_data_counter)+5] = databyte[1][0];
            data_memory_1[buffer_counter][(ads1278_data_counter)+6] = databyte[1][1];
            data_memory_1[buffer_counter][(ads1278_data_counter)+7] = databyte[1][2];
            
            data_memory_1[buffer_counter][(ads1278_data_counter)+8] = databyte[2][0];
            data_memory_1[buffer_counter][(ads1278_data_counter)+9] =  databyte[2][1];
            data_memory_1[buffer_counter][(ads1278_data_counter)+10] = databyte[2][2];
            
            data_memory_1[buffer_counter][(ads1278_data_counter)+11] = databyte[3][0];
            data_memory_1[buffer_counter][(ads1278_data_counter)+12] = databyte[3][1];
            data_memory_1[buffer_counter][(ads1278_data_counter)+13] = databyte[3][2];
            
            data_memory_1[buffer_counter][(ads1278_data_counter)+14] = databyte[4][0];
            data_memory_1[buffer_counter][(ads1278_data_counter)+15] = databyte[4][1];
            data_memory_1[buffer_counter][(ads1278_data_counter)+16] = databyte[4][2];
            
            data_memory_1[buffer_counter][(ads1278_data_counter)+17] = databyte[5][0];
            data_memory_1[buffer_counter][(ads1278_data_counter)+18] = databyte[5][1];
            data_memory_1[buffer_counter][(ads1278_data_counter)+19] = databyte[5][2];
            
            data_memory_1[buffer_counter][(ads1278_data_counter)+20] = databyte[6][0];
            data_memory_1[buffer_counter][(ads1278_data_counter)+21] = databyte[6][1];
            data_memory_1[buffer_counter][(ads1278_data_counter)+22] = databyte[6][2];
            
            data_memory_1[buffer_counter][(ads1278_data_counter)+23] = databyte[7][0];
            data_memory_1[buffer_counter][(ads1278_data_counter)+24] = databyte[7][1];
            data_memory_1[buffer_counter][(ads1278_data_counter)+25] = databyte[7][2];
          
   
          ads1278_data_counter= ads1278_data_counter+24;
          
          if(ads1278_data_counter==data_in_buffer){       //-- block 滿了~
            
              data_buffer_state[buffer_counter] = true;   
              buffer_counter++;
              if(buffer_counter==Max_Data_Buffer)buffer_counter=0;
            
              ads1278_data_counter = 0;
              data_buffer_is_full = true;
          
          }
    }
     
    __bic_SR_register_on_exit(LPM0_bits); // Exit LPM0
      //return;
}
//-------------------------------------------------------------------------------------
//------------------------------------------------------------------------------
void do_storage_2()
{
            if(Save_flag == 2){
             check_SD_card = sd_write_block(&sdc, blockaddress, sd_buffer1);
                   sd_wait_notbusy (&sdc);
              for(int i=0;i<512;i++)sd_buffer1[i]=0;
            }
            else{
            check_SD_card =  sd_write_block (&sdc, blockaddress, sd_buffer2);
                  sd_wait_notbusy (&sdc);  
              for(int i=0;i<512;i++)sd_buffer2[i]=0;
            }
                    // 8G SD card limit -> 15660032 ~~ 15660000
                    // 32G               ->63078400 ~~
                                      
                  if( blockaddress >= 63078300)
                  {
                    //-do something for protect data
                    return ;
                  }
                  else{
                    blockaddress++;
                  }
}
//------------------------------------------------------------------------------

void update_table()
{
        endblock = blockaddress-1;
        sd_format_table[16*(total)+4] = (unsigned char)(endblock >> 24);
        sd_format_table[16*(total)+5] = (unsigned char)(endblock >> 16);
        sd_format_table[16*(total)+6] = (unsigned char)(endblock >> 8);
        sd_format_table[16*(total)+7] = (unsigned char)(endblock);

        sd_write_block (&sdc, 0, sd_format_table);
        sd_wait_notbusy (&sdc);
}

int do_sd_initialize (sd_context_t *sdc)
{
	/* Initialize the SPI controller */
	spi_initialize();
	/* Set the maximum SPI clock rate possible */
	spi_set_divisor(PERIPH_CLOCKRATE/400000);
	/* Initialization OK? */
	if (sd_initialize(sdc) != 1)
		return 0;
        spi_set_divisor(2);   //---- 2011 0905 spi full speed
	return 1;
}

int SD_INIT()
{
        int j, ok = 0;
        
  	/* Set some reasonable values for the timeouts. */
        sdc.timeout_write = PERIPH_CLOCKRATE/8;
        sdc.timeout_read = PERIPH_CLOCKRATE/20;
        sdc.busyflag = 0;    

        for (j=0; j<SD_INIT_TRY && ok != 1; j++)
	{
	    ok = do_sd_initialize(&sdc);
	}
        
        return ok;
}

int read_SD_format(){
        // 讀取 format table
        sd_read_block(&sdc, 0, sd_format_table);
        sd_wait_notbusy (&sdc);
        // 讀取資料筆數
        total = sd_format_table[0];
        total = (total << 8) + sd_format_table[1];
        // 讀取最後一筆的 endblock
        startblock = sd_format_table[16*(total)+4];
        startblock = (startblock << 8 ) + sd_format_table[16*(total)+5];
        startblock = (startblock << 8 ) + sd_format_table[16*(total)+6];
        startblock = (startblock << 8 ) + sd_format_table[16*(total)+7];
        if(startblock<=100)startblock=100;
        // 寫入資料筆數
        total ++;
        sd_format_table[0] = total >> 8;
        sd_format_table[1] = total & 0xFF;  //  0x0F-> 0xFF------ 2010 10/06..... 錯誤修正
        // 設定目前的起始與結尾block
        startblock ++;
        blockaddress = startblock;
        endblock = startblock;
        sd_format_table[16*(total)+0] = (unsigned char)(startblock >> 24);
        sd_format_table[16*(total)+1] = (unsigned char)(startblock >> 16);
        sd_format_table[16*(total)+2] = (unsigned char)(startblock >> 8);
        sd_format_table[16*(total)+3] = (unsigned char)(startblock);
        sd_format_table[16*(total)+4] = (unsigned char)(endblock >> 24);
        sd_format_table[16*(total)+5] = (unsigned char)(endblock >> 16);
        sd_format_table[16*(total)+6] = (unsigned char)(endblock >> 8);
        sd_format_table[16*(total)+7] = (unsigned char)(endblock);

//       check_SD_card = 
        sd_write_block (&sdc, 0, sd_format_table);
        sd_wait_notbusy (&sdc);
        return 0;
}
