#include "uart.h"
#include "stdbool.h"


   int control_mode;
   int UART_COM1_RX_count;
   unsigned char UART_COM1_RX_BUF[COM_BUF_Size];
   int UART_COM2_RX_count;
   unsigned char UART_COM2_RX_BUF[COM_BUF_Size];
   int UART_COM3_RX_count;
   unsigned char UART_COM3_RX_BUF[COM_BUF_Size];
   int UART_COM4_RX_count;
   unsigned char UART_COM4_RX_BUF[COM_BUF_Size];

   bool Check_Command_flag; 
//  extern char string[150];
//  extern bool StartRecord;
//  extern bool Set_RTC_Flag;
//extern unsigned char RTC_Serial[8];
 
//------------------------------------------------------------------------------
void UART_Init(unsigned char com){
   //���n���Ұ� XT2 12MHZ
  if( (com&COM1) == COM1 ){
          
     P3SEL = 0x30;                             // P3.4,5 = USCI_A0 TXD/RXD
      UCA0CTL1 |= UCSWRST;                      // **Put state machine in reset**

      UCA0CTL1 |= UCSSEL_2;                     // CLK = SCLK = 12MHZ 
      UCA0BR0 = 0xe2;                           //   (see User's Guide)  12M/9600 
      UCA0BR1 = 0x04;                           //
      //UCA0MCTL = 0;
      UCA0MCTL = UCBRS_0+UCBRF_0;               // Modulation 

      UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
      UCA0IE |= UCRXIE;  
  }
  if( (com&COM2) == COM2 ){

      P5SEL |= 0xc0;                             // P5.6,7 = USCI_A0 TXD/RXD
      UCA1CTL1 |= UCSWRST;                      // **Put state machine in reset**

      UCA1CTL1 |= UCSSEL_2;                     // CLK = SCLK
      UCA1BR0 = 0xAD; 
      UCA1BR1 = 0x00;                           //
      UCA1MCTL = UCBRS_3+UCBRF_0;               // Modulation UCBRSx=3, UCBRFx=0

      UCA1CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
      UCA1IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
  }

   if( (com&COM3) == COM3 ){

      P9SEL |= 0x30;                             // P9.4,5 = USCI_A0 TXD/RXD
      UCA2CTL1 |= UCSWRST;                      // **Put state machine in reset**

      UCA2CTL1 |= UCSSEL_2;                     // CLK = sCLK
      UCA2BR0 = 0x8D;                           // 20Mhz/2400=8333 (see User's Guide) for seascan
      UCA2BR1 = 0x20;                           //
      UCA2MCTL = UCBRS_3+UCBRF_0;               // Modulation UCBRSx=3, UCBRFx=0

      UCA2CTL0 = 0;
      UCA2CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
      UCA2IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
  }
  UART_COM1_RX_count = 0;
  UART_COM2_RX_count = 0;
  UART_COM3_RX_count = 0;
  
}
//----------------------------------------------------------------------------
#pragma vector=USCI_A0_VECTOR  //COM1
__interrupt void USCI_A0_ISR(void)
{
  switch(__even_in_range(UCA0IV,4))
  {
  case 0:break;                             // Vector 0 - no interrupt
  case 2:                                   // Vector 2 - RXIFG
//    while (!(UCA0IFG&UCTXIFG));             // USCI_A0 TX buffer ready?
        UART_COM1_RX_BUF[UART_COM1_RX_count] = UCA0RXBUF;
        UART_COM1_RX_count++;
    if(UART_COM1_RX_count>=COM_BUF_Size)UART_COM1_RX_count=0;
    break;
  case 4:break;                             // Vector 4 - TXIFG
  default: break;  
  }
    __bic_SR_register_on_exit(LPM0_bits); // Exit LPM0
}
//----------------------------------------------------------------------------
#pragma vector=USCI_A1_VECTOR //COM2
__interrupt void USCI_A1_ISR(void)
{
  
  //---�R�O�榡    @xm..........$
  //---            @ �}�l�r��
  //---            x �O�R�r��
  //---            m �R�O�Ҧ�
  //---            .....  �R�O
  //---            $ �����r��   
  
  switch(__even_in_range(UCA1IV,4))
  {
  case 0:break;                             // Vector 0 - no interrupt
  case 2:                                   // Vector 2 - RXIFG
//    while (!(UCA1IFG&UCTXIFG));           // USCI_A1 TX buffer ready?
//    UCA1TXBUF = UCA1RXBUF;                // TX -> RXed character
    UART_COM2_RX_BUF[UART_COM2_RX_count] = UCA1RXBUF;
    UART_COM2_RX_count++;
    if(UART_COM2_RX_count>=COM_BUF_Size)UART_COM2_RX_count=0;   
    
    if( (UART_COM2_RX_BUF[0]=='@')){        //- �R�O���
       if((UART_COM2_RX_count>0)&&(UART_COM2_RX_count==UART_COM2_RX_BUF[1])&&(UART_COM2_RX_BUF[UART_COM2_RX_count-1]=='$')){

         switch(UART_COM2_RX_BUF[2]){
         case 'T':
           break;
         case 'A':
           break;
         case 'S':
           break;
         case 'R':
           break;   
         case 't':
            
           break; 
           
           
         }
       UART_COM2_RX_count = 0;  
       }
     }
     else{
       UART_COM2_RX_count = 0;
     }

     
     
    if(UART_COM2_RX_count>0){
     if(UART_COM2_RX_BUF[UART_COM2_RX_count-1]=='$')UART_COM2_RX_count = 0;
    }

    

      
    break;
  case 4:break;                             // Vector 4 - TXIFG
  default: break;  
  }
 //   __bic_SR_register_on_exit(LPM3_bits); // Exit LPM0
}
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
#pragma vector=USCI_A2_VECTOR  //COM3
__interrupt void USCI_A2_ISR(void)
{
  switch(__even_in_range(UCA2IV,4))
  {
  case 0:break;                             // Vector 0 - no interrupt
  case 2:                                   // Vector 2 - RXIFG
    break;
  case 4:break;                             // Vector 4 - TXIFG
  default: break;  
  }
 //   __bic_SR_register_on_exit(LPM3_bits); // Exit LPM0
}
//----------------------------------------------------------------------------
void UART_SendByte(unsigned char data,unsigned char com)
{
  if((com&COM1) == COM1){
    while (!(UCA0IFG&UCTXIFG));             // USCI_A1 TX buffer ready?
    UCA0TXBUF = data;
  }
  if((com&COM2) == COM2){
    while (!(UCA1IFG&UCTXIFG));             // USCI_A1 TX buffer ready?
    UCA1TXBUF = data;
  }
  if((com&COM3) == COM3){
    while (!(UCA2IFG&UCTXIFG));             // USCI_A1 TX buffer ready?
    UCA2TXBUF = data;
  }
  
  
}
void UART_SendStr(char *str,unsigned char com)
{
        while (*str != '\0')
        {
            UART_SendByte(*str++,com);	          // �o�e�ƾ�
        }
 //       UART_COM1_RX_count=0;
 //       UART_COM2_RX_count=0;                     // 20110310�ק� �o�e�R�O��M��BUFF�����
 //       UART_COM3_RX_count=0;   
        
}
void RS232_Send_Char(char *str,int count,unsigned char com)
{
        for(int i=0;i<count;i++)
        {
            UART_SendByte(*str++,com);	          // �o�e�ƾ�
        }
}
//-------------------------------------------------------------------------------
