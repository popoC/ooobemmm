//2015 0305 ¼W¥[I2C timeout
// 2015 0303 RTC §ó§ï¬°I2C¨ú­È OBEM SYSTEM BOARD  V2.0
// 2015 0129 OBEM SYSTEM BOARD !! V1.0
#include  <msp430x54xA.h>
#include "stdio.h"
#include "main.h"
#include "uart.h"




#define I2CTIMEOUT 1000

#define save_block_MAX 300*1*2   //--- ¤j¬ù ?¤ÀÄÁ

//-----------------I2C Åª¨ú®É¶¡
void I2C_Init();
int I2C_ReadMulti(unsigned char slave_addr,unsigned char reg,unsigned char *data,unsigned char len);
int i2c_err_no = 0;
unsigned char RTC_TIMER_Drift_[8];       // Allocate 20 byte of RAM

//---------- °O¾ÐÅé¼g¤Jªk

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

void update_table(void);
unsigned char sd_format_table[512];


//------------------------------------------------------------------------------
bool StartRecord = false;           // ¼Ò¦¡¤Á´« °O¿ý©Î³]©w?
int OUTPUT_MODE = AD_DATA;          // ¼Ò¦¡¿ï¾Ü-- ³z¹LRS232ÅÜ§ó
long int wait_for_start = 0;        // ¦Û°Ê­«±Ò ­p¼Æ
int out_timer_counter = 0;          //-- ¤£­nÅý¿é¥X®É¶¡¦r¦êªºÀW²v¤Ó°ª~
//----------DATA FLOW ----------------------------------------------------------
 unsigned char data_memory_1[Max_Data_Buffer][512];       //¸ê®Æ¼È¦s¾¹
 bool data_buffer_state[Max_Data_Buffer];                 //¸ê®Æ¼È¦s¾¹ª¬ºA
 BYTE write_data[512];                                    //¼g¤J°Ï

 //--¥þ°ìÅÜ¼Æ----±±¨î
unsigned long save_data_block_counter = 0;
bool DO_AD_Sample1 = false;



//------------------------------------------------------------------------------
  
void Crystal_Init();   
//----------for seascan---------------------------------------------------------
//#define TimeOut 5000
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
 unsigned int data_in_buffer = 480; //-- 20µ§¸ê®Æ   24*20
 int save_flag = 0;
 int buffer_counter = 0;
 bool data_buffer_is_full = false;
 static bool save_fq = 0;   //-- 198Hz -> 99Hz ¥Î
 
 
 unsigned long ads1278_data[8];
 unsigned long view_ads1278_data[8];
 int check_counter = 0;



//------------------------------------------------------------------------------
          
void main(void)
{
   WDTCTL = WDTPW + WDTHOLD;                    // Stop WDT
 
   
   P1DIR = 0xff;
   P1OUT = 0;
   P1DIR |= BIT0;                            //LED
   P1OUT |= BIT0;                           //   ·Ý¦ëª©
   //----- system init ------
   Crystal_Init();                          // ¾_Àú¾¹ªì©l¤Æ
   UART_Init(COM2);                         // Rs232ªì©l¤Æ    
   ADS1278_Init();
   I2C_Init();
   
   
   P1OUT |= SYNC;
    _EINT();  //±Ò°Ê¥þ°ì¤¤Â_  
    
    
  //   __no_operation();                          // For debugger 
 
   StartRecord = false;
   
   
   //----------
   
  while(!StartRecord){
          __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0, enable interrupts ¦¹³B¤¤Â_¤j¬ù 100Hz
           wait_for_start++;
           if(wait_for_start > 60000){
             
             for(int ig=0;ig<Max_Data_Buffer;ig++)data_buffer_state[ig]=false;  

             StartRecord = true;   //  ¦Û°Ê±Ò°Ê¸Ë¸m ¤j¬ù¤T¤Q¤ÀÄÁ
               
           }
               
                if(Set_RTC_Flag){
                 /*Set_RTC_Flag = false;
                   P6OUT|=BIT7 ;
                   //set time to msp430f2013
                   IO_Send_Byte(RTC_Serial[0]);
                   P6OUT&= ~BIT7;
                   for(int i=1;i<7;i++) IO_Send_Byte(RTC_Serial[i]); 
                  */
                }   
                
         switch(OUTPUT_MODE){
           case LOGGER_TIME:
             out_timer_counter++;
             if(out_timer_counter>=10){
               out_timer_counter = 0;
                     string[0] = 't';                  //¸ê®Æ«¬ºA fix 2012 1224
                     string[9] = 0;  string[10] = 1;  
              get_time();      memcpy(&(string[1]),RTC_Serial,8);
              RS232_Send_Char(string,11,COM2);
              
              
              
              memset(RTC_TIMER_Drift_,0,8);
              i2c_err_no = 0;
              i2c_err_no = I2C_ReadMulti(0x48,2,RTC_TIMER_Drift_,8);
              RTC_TIMER_Drift_[6] = i2c_err_no;
                   string[0] = 'u';               
                   string[9] = 0;  string[10] = 1;  
                   memcpy(&(string[1]),RTC_TIMER_Drift_,8);
              RS232_Send_Char(string,11,COM2);
  
              
              
             }
           break;      
           case AD_DATA:
                if(ADS1278_Get_Data2(view_ad_data)==1){
            //      P1OUT |= 0x01;
                  for(int m=0;m<8;m++)  //--- ³o²z¿é¥X®É¶¡·|¤ñ¸ûºC~ ¸ò¯u¹êªº±Ä¼Ë²v¤£¦P ·|¥d¨ì
                  {
                   string[0] = 'd';                   //¸ê®Æ«¬ºA
                   string[1] = view_ad_data[m*3+0];
                   string[2] = view_ad_data[m*3+1];
                   string[3] = view_ad_data[m*3+2];
                   string[4] =  m;  string[5] =  '\r';
                   RS232_Send_Char(string,7,COM2);  
                  }
                }
          break;
          case RE_SET:
            OUTPUT_MODE = 0;  WDTCTL = WDT_ARST_1000;  //--¶}ª¯    
           break;    
         }    
      
  }
  //-------------------------------- start record-----------------------------
  //-----------------------------
     

	  if(!SD_INIT()){return;}                 // sd card ªì©l¤Æ
  if(read_SD_format()){return;}           // read SD card format
	
  while(StartRecord){
             __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0, enable interrupts
               /*
             if(ads1278_data_counter==24){  //--- °µ§¹²Ä¤@µ§AD®É¶¡AD1
               get_time();   // ? ´ú¸Õ«á¦A¬Ý­n¤£­n©ñ  
               
                data_memory_1[buffer_counter][512-1] = RTC_Serial[7];
                data_memory_1[buffer_counter][512-2] = RTC_Serial[6];
                data_memory_1[buffer_counter][512-3] = RTC_Serial[5];
                data_memory_1[buffer_counter][512-4] = RTC_Serial[4];
                data_memory_1[buffer_counter][512-5] = RTC_Serial[3];
                data_memory_1[buffer_counter][512-6] = RTC_Serial[2];
                data_memory_1[buffer_counter][512-7] = RTC_Serial[1];
             }       
             */
	   for(int y = 0;y<Max_Data_Buffer;y++){
                if(data_buffer_state[y]==true){              //»Ý­n¼g¸ê®Æ
             
                      
                      data_buffer_state[y] = false;
                for(int m=0;m<512;m++)write_data[m]=0;
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
                          
                          memset(RTC_TIMER_Drift_,0,8);
                          I2C_ReadMulti(0x48,2,RTC_TIMER_Drift_,8);
                  }

               write_data[0] = (save_data_block_counter>>8) & 0xff;;
               write_data[1] = save_data_block_counter & 0xff;
/*               write_data[512-1] = RTC_Serial[7];   write_data[512-2] = RTC_Serial[6];
               write_data[512-3] = RTC_Serial[5];   write_data[512-4] = RTC_Serial[4];
               write_data[512-5] = RTC_Serial[3];   write_data[512-6] = RTC_Serial[2];
               write_data[512-7] = RTC_Serial[1];
*/
               write_data[512-8] = RTC_TIMER_Drift_[7];
               write_data[512-9] = RTC_TIMER_Drift_[6];
               write_data[512-10] = RTC_TIMER_Drift_[5];
               write_data[512-11] = RTC_TIMER_Drift_[4];
               write_data[512-12] = RTC_TIMER_Drift_[3];
               write_data[512-13] = RTC_TIMER_Drift_[2];               
               write_data[512-14] = RTC_TIMER_Drift_[1];               
               write_data[512-15] = RTC_TIMER_Drift_[0];               
               
               for(int p=2;p<512-15;p++)write_data[p] = data_memory_1[y][p];
	       
               write_data[511]= data_memory_1[y][511];
               write_data[510]= data_memory_1[y][510];               
               write_data[509]= data_memory_1[y][509];
               write_data[508]= data_memory_1[y][508];               
               write_data[507]= data_memory_1[y][507];               
               write_data[506]= data_memory_1[y][506];               
               write_data[505]= data_memory_1[y][505];                         
               
            check_SD_card = sd_write_block(&sdc, blockaddress, write_data);
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
       WDTCTL = WDT_ARST_1000;  //--¶}ª¯ RESET
    }      
    
    
  }
	 
	 
	 
	 
	 
	 //-----------------------


    
  
   __no_operation();                          // For debugger                   
}
//----------------------------------------------------------------------------
void Crystal_Init(){
    //--±Ò°Ê 20MHZ »P32767HZ
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

void get_time(){
  memset(RTC_Serial,0,8);

   I2C_ReadMulti(0x48,1,RTC_Serial,8);
       if(RTC_Serial[1]>12)RTC_Serial[1]=12;
       if(RTC_Serial[2]>31)RTC_Serial[2]=31;
       if(RTC_Serial[3]>24)RTC_Serial[3]=24;
       if(RTC_Serial[4]>60)RTC_Serial[4]=0;
       if(RTC_Serial[5]>60)RTC_Serial[5]=0;
   
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
 
 DWORD get_fattime (void)
{
  DWORD testtime;
  get_time();   //-- ¸òMSP430F2013 ­n®É¶¡     
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
  P1SEL = 0;
  P1DIR |= (SCLK+SYNC);                       // Set SCLK SYNC be OutPut
  P1DIR &= ~(DRDY_1);
  P1REN |= DRDY_1;                            // Enable DRDY internal resistance  P1!!!!
  P1OUT |= DRDY_1;                            // Set DRDY as pull-Up resistance
  P1IE |= DRDY_1;                             // DRDY interrupt enabled
  P1IES |= DRDY_1;                            // DRDY Hi/Lo edge//  P1IES &= ~DRDY_1;   Hi/Lo edge
  P1IFG &= ~DRDY_1;                           // DRDY IFG cleared

  P2DIR &= ~(BIT0+BIT1+BIT2+BIT3);            //set Data InPut P2.0~2.3 
  P2DIR &= ~(BIT4+BIT5+BIT6+BIT7);            //set Data InPut P2.4~1.7 
  
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
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
  if((P1IFG & DRDY_1) !=DRDY_1){ 
    P1IFG = 0;                                //  all IFG cleared
    __bic_SR_register_on_exit(LPM0_bits); // Exit LPM0
    return;
  }
  else{
    P1IFG &= ~DRDY_1;                          // P1.2 IFG cleared
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
      if((P2IN&BIT4)==BIT4)databyte[4][sj] = databyte[4][sj]+ data_value[si];
      if((P2IN&BIT5)==BIT5)databyte[5][sj] = databyte[5][sj]+ data_value[si];      
      if((P2IN&BIT6)==BIT6)databyte[6][sj] = databyte[6][sj]+ data_value[si];
      if((P2IN&BIT7)==BIT7)databyte[7][sj] = databyte[7][sj]+ data_value[si];
      
        __delay_cycles(150);
        SCLK_L;
        __delay_cycles(200);
    }
  }
  
       
  
  
  if(!StartRecord){
        for(si=0;si<8;si++){
           view_ads1278_data[si] = ads1278_data[si];
        }
          check_counter = 0;   
          adc_busy = false;     
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
          
          
          
            if(ads1278_data_counter==24){  //--- °µ§¹²Ä¤@µ§AD®É¶¡AD1  --- Â\¦b³o´ú¸Õª¬ºA~~~ ¤¤Â_¤¤©I¥s¤¤Â_
               get_time();   // ? ´ú¸Õ«á¦A¬Ý­n¤£­n©ñ  
               
                data_memory_1[buffer_counter][512-1] = RTC_Serial[7];
                data_memory_1[buffer_counter][512-2] = RTC_Serial[6];
                data_memory_1[buffer_counter][512-3] = RTC_Serial[5];
                data_memory_1[buffer_counter][512-4] = RTC_Serial[4];
                data_memory_1[buffer_counter][512-5] = RTC_Serial[3];
                data_memory_1[buffer_counter][512-6] = RTC_Serial[2];
                data_memory_1[buffer_counter][512-7] = RTC_Serial[1];
             }       

          
          
          
          
          
          
          
          if(ads1278_data_counter==data_in_buffer){       //-- block º¡¤F~
            
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
        // Åª¨ú format table
        sd_read_block(&sdc, 0, sd_format_table);
        sd_wait_notbusy (&sdc);
        // Åª¨ú¸ê®Æµ§¼Æ
        total = sd_format_table[0];
        total = (total << 8) + sd_format_table[1];
        // Åª¨ú³Ì«á¤@µ§ªº endblock
        startblock = sd_format_table[16*(total)+4];
        startblock = (startblock << 8 ) + sd_format_table[16*(total)+5];
        startblock = (startblock << 8 ) + sd_format_table[16*(total)+6];
        startblock = (startblock << 8 ) + sd_format_table[16*(total)+7];
        if(startblock<=100)startblock=100;
        // ¼g¤J¸ê®Æµ§¼Æ
        total ++;
        sd_format_table[0] = total >> 8;
        sd_format_table[1] = total & 0xFF;  //  0x0F-> 0xFF------ 2010 10/06..... ¿ù»~­×¥¿
        // ³]©w¥Ø«eªº°_©l»Pµ²§Àblock
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
//------------------------------------------------------------------------------

int I2C_ReadMulti(unsigned char slave_addr,unsigned char reg,unsigned char *data,unsigned char len){
    int i;
    //-- 2015 0306 §ï¼g ¥[¤JTIMEOUT
   // unsigned short int TimeOut = 0;
    unsigned  int TimeOut = 0;
    
    if (UCB3STAT & UCBBUSY)
      {   
          UCB3CTL1 |= UCSWRST;
          
          UCB3STAT &=~UCSTTIFG;
          
          UCB3CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
          
          return 7;
      } 
   
    
    TimeOut = I2CTIMEOUT;
     while( (UCB3CTL1 &  UCTXSTP)&&(TimeOut--));              // úü STOP úd
     if(!TimeOut)return 1;
     
     
    //--- «ü©w slave addr     
      UCB3CTL1 |= UCSWRST;                     // •xUCSWRST 
      UCB3I2CSA = slave_addr;
      UCB3CTL1 &= ~UCSWRST;                     //  UCSWRST 
     
     //--- UCTXSTT=²£¥Í°_©l«H¸¹  UCTR=1¼g¼Ò¦¡
      UCB3CTL1 |= UCTXSTT+UCTR;                 // 0x3C master -> slave
      
      UCB3TXBUF  = (reg );                      //•ã•x”æ 
      TimeOut = I2CTIMEOUT;
      while((!(UCB3IFG & UCTXIFG))&&(TimeOut--));              //
      if(!TimeOut)return 2;
      
      UCB3CTL1 |= UCTXSTP;                      // I2C stop condition ?
    
      UCB3CTL1 &= ~UCTR;                        // 0x3D Åª¼Ò¦¡
      TimeOut = I2CTIMEOUT;
      while( (UCB3CTL1 &  UCTXSTP)&&(TimeOut--));              // úü STOP úd
      if(!TimeOut)return 3;
      
      UCB3CTL1 |= UCTXSTT;                    //start~reading~
      
      TimeOut = I2CTIMEOUT;
      while((UCB3CTL1& UCTXSTT)&&(TimeOut--));               // ?úüUCTXSTT=0  
      if(!TimeOut)return 4;
      
      for(i=0;i<len-1;i++){
         TimeOut = I2CTIMEOUT;
         while( (!(UCB3IFG & UCRXIFG))&&(TimeOut--));
         if(!TimeOut)return 5;
         data[i]= UCB3RXBUF;   
      }
      UCB3CTL1 |=UCTXSTP;
         TimeOut = I2CTIMEOUT;
         while( (!(UCB3IFG & UCRXIFG))&&(TimeOut--));
         if(!TimeOut)return 6;
        data[len-1]= UCB3RXBUF;

        
        return 0;
}
/*void I2C_ReadMulti(unsigned char slave_addr,unsigned char reg,unsigned char *data,unsigned char len){
    int i;

     while( UCB3CTL1 &  UCTXSTP);              // úü STOP úd
    //--- «ü©w slave addr     
      UCB3CTL1 |= UCSWRST;                     // •xUCSWRST 
      UCB3I2CSA = slave_addr;
      UCB3CTL1 &= ~UCSWRST;                     //  UCSWRST 
     
     //--- UCTXSTT=²£¥Í°_©l«H¸¹  UCTR=1¼g¼Ò¦¡
      UCB3CTL1 |= UCTXSTT+UCTR;                 // 0x3C master -> slave
      
      UCB3TXBUF  = (reg );                      //•ã•x”æ 
      while(!(UCB3IFG & UCTXIFG));              //
      
      UCB3CTL1 |= UCTXSTP;                      // I2C stop condition ?
    
      UCB3CTL1 &= ~UCTR;                        // 0x3D Åª¼Ò¦¡
      while( UCB3CTL1 &  UCTXSTP);              // úü STOP úd
      UCB3CTL1 |= UCTXSTT;                    //start~reading~
      while(UCB3CTL1& UCTXSTT);               // ?úüUCTXSTT=0  
      
      
      for(i=0;i<len-1;i++){
         while( !(UCB3IFG & UCRXIFG));
         data[i]= UCB3RXBUF;   
      }
      UCB3CTL1 |=UCTXSTP;
         while( !(UCB3IFG & UCRXIFG));
        data[len-1]= UCB3RXBUF;

}*/
//-------------------------------------------------------------------------------
void I2C_Init(){
  P10SEL |= 0x06;                            // Assign I2C pins to USCI_B3 P10.1 10.2
  UCB3CTL1 |= UCSWRST;                      // Enable SW reset
  UCB3CTL0 = UCMST + UCMODE_3 + UCSYNC;     // I2C Master, synchronous mode
  UCB3CTL1 = UCSSEL_2 + UCSWRST;            // Use SMCLK, keep SW reset
  UCB3BR0 = 20;                             // fSCL = SMCLK/20 = ~100kHz
  UCB3BR1 = 0;
  UCB3I2CSA = 0x48;                         // Slave Address is 048h
  UCB3CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
  //UCB3IE |= UCRXIE;                         // Enable RX interrupt
  //UCB3IE |= UCTXIE;                         // Enable TX interrupt

}
