
/********** Oregon Scientific RF protocol decoder *********
** v. 1.00a
** by Alexander Yerezeyev 
** http://alyer.frihost.net
** compiled with IAR 5.30 for target MCU Atmega8 @ 16MHz
*
* Program reads and decode low-level RF data from 
* Oregon Scientific v1.0, v2.1 and v3.0 sensors and put the messages to COM port
* for high-level software (weather station, datalogers, etc)
* Creation Date: 19/Jun/2010
* Copyright: (C)2010 by Alexander Yerezeyev
* License: GNU GPL v3 (see http://www.gnu.org/licenses/gpl-3.0.txt)
**********************************************************/

#include <ioavr.h>
#include "compiler.h"
#include "stdafx.h"

#define SET_ICP_RISING  sbi (TCCR1B, ICES1); // set ICP as rising edge sensitive
#define SET_ICP_FALLING cbi (TCCR1B, ICES1); // set ICP as falling edge sensitive
#define ICP_IS_RISING   ((TCCR1B&(1<<ICES1))!=0) // check that ICP is rising edge sensitive
#define ICP_IS_FALLING  ((TCCR1B&(1<<ICES1))==0) // check that ICP is falling edge sensitive

#define InRange(TVAL, TSETUP, dT) ((TVAL<=(TSETUP+dT))&&(TVAL>=(TSETUP-dT)))
#define SetTimer_us(us) (us*XTALL/8)  // return timer counter value for microseconds

#define LED     PD5   // LED pin
#define LEDPORT PORTD // LED port
#define LEDDDR  DDRD  // LED ddr
#define LED_ON  sbi (LEDPORT, LED)  // turn on LED
#define LED_OFF cbi (LEDPORT, LED)  // turn off LED

//#define us1000  SetTimer_us(973) // Timer Value for 1ms pulse width
//#define us500   SetTimer_us(491) // Timer Value for 0.5ms pulse width
#define us1000  SetTimer_us(1000) // Timer Value for 1ms pulse width
#define us500   SetTimer_us(500) // Timer Value for 0.5ms pulse wid

#define start_bit_led   // set if you want to view start of data pulse @ LED pin
#define preamlbe_led    // set if you want to view preamble pulses @ LED pin
#define bit_led         // set if you want to view decoded bits @ LED pin
#define DEBUG_PRINT1  // set to enable debug messages for OS1 protocol
#define DEBUG_PRINT2_1  // set to enable debug messages for OS2.1 protocol
#define DEBUG_PRINT3    // set to enable debug messages for OS3 protocol


enum STATES_OS1
{
  STATE_OS1_IDLE=0,    // OS1 decoder in iddle state
  STATE_OS1_PREAMBULE, // OS1 decoder starts to receive the preamble
  STATE_OS1_SYNC1,     // OS1 decoder waits SYNC1 pulse
  STATE_OS1_SYNC2,     // OS1 decoder waits SYNC2 pulse
  STATE_OS1_SYNC3,     // OS1 decoder waits SYNC3 pulse
  STATE_OS1_DATA,      // OS1 decoder receiving DATA
};

enum STATES_OS2
{
  STATE_OS2_IDLE=0,     // OS2.1 decoder in iddle state
  STATE_OS2_PREAMBULE,
  STATE_OS2_SYNC,       // OS2.1 decoder is synchronised with preamble pulses
  STATE_OS2_DATA        // start-bit is received and OS2.1 decoder is redy to decode data
};    

enum STATES_OS3
{
  STATE_OS3_IDLE=0,     // OS3 decoder in iddle state
  STATE_OS3_PREAMBULE,
  STATE_OS3_SYNC,       // OS3 decoder is synchronised with preamble pulses
  STATE_OS3_DATA        // start-bit is received and OS3 decoder is redy to decode data
};    


enum STATES_DECODER
{
  STATE_DECODER_IDLE=0, // OS decoder is in iddle state
  STATE_DECODER_OS1,    // OS decoder in OS1 mode
  STATE_DECODER_OS2,    // OS decoder in OS2 mode
  STATE_DECODER_OS3     // OS decoder in OS3 mode
};


typedef struct
{
  UINT16 oldICP;
//  UINT16 newICP;
} ICP_VARIABLES_STRUCTURE;

typedef struct
{
//  UINT8 Stream[16];
  UINT8 BitPosition;
  UINT8 SyncCnt;
  UINT8 State;
  UINT8 LastBit;
} OS1_VARIABLES_STRUCTURE;

typedef struct
{
//  UINT8 Stream[32];
  UINT8 BitPosition;
  UINT8 SyncCnt;
  UINT8 State;
  UINT8 LastBit;
} OS2_VARIABLES_STRUCTURE;

typedef struct
{
//  UINT8 Stream[16];
  UINT8 BitPosition;
  UINT8 SyncCnt;
  UINT8 State;
  UINT8 LastBit;
} OS3_VARIABLES_STRUCTURE;

__no_init UINT8 Stream[32];
__no_init ICP_VARIABLES_STRUCTURE ICP_VARS;
__no_init OS1_VARIABLES_STRUCTURE OS1_VARS;
__no_init OS2_VARIABLES_STRUCTURE OS2_VARS;
__no_init OS3_VARIABLES_STRUCTURE OS3_VARS;
__no_init char STATE_DECODER;

inline void UART_TX(char var)
{
   while ( !( UCSRA & (1<<UDRE)));
   UDR = var;
};

//Print 0x0A,0x0B
inline void print_rn (void)
{ 
  UART_TX(0x0A);
  UART_TX(0x0D);
}

__no_init char debug_buff[32];
unsigned char debug_index=0;

void uart_init(void)
{
/* UART0 initialisation */
/* desired baud rate: 115200 */
/* actual baud rate: 115200 (0.0%) */
/* char size: 8 bit */
/* parity: Disabled */
UCSRB = 0x00; /* disable while setting baud rate */
UCSRA = 0x00;
UCSRC = 0x86; // 8 bit data
UBRRL = 8; // set baud rate lo (38400 @ 16 MHz)
UBRRH = 0x00; /* set baud rate hi */
UCSRB = 0x08; //TX are enabled
}

/***********************************************
Fast i2a routine for AVR 8bit platform
by RST7/CBSIE
***********************************************/
__z void i2a(char *s, UINT16 v) 
{
asm (" \n"  
"//   14 {\n"
"//   15   UINT8 m0; //R16\n"
"//   16   UINT8 m1; //R17\n"
"//R18-R20 - 24bit fmul result\n"
"//R21 - c,b,a ->06 8D B9\n"
"//R22 - zero reg\n"
"	CLR	R22\n"
"	LDI	R21,0x06\n"
"//  v=__multiply_unsigned(m0,0x06)+3;\n"
"	MUL	R16,R21\n"
"	MOVW	R19:R18,R1:R0\n"
"	SUBI	R18,0xFD\n"
"	SBCI	R19,0xFF\n"
"//  v+=__multiply_unsigned(m1,0x06)<<8;\n"
"	MUL	R17,R21\n"
"	MOV	R20,R1\n"
"	ADD	R19,R0\n"
"	ADC	R20,R22\n"
"//  v+=__multiply_unsigned(m1,0x8D);\n"
"       LDI     R21, 0x8D\n"
"       MUL     R17, R21\n"
"       ADD     R18, R0\n"
"       ADC     R19, R1\n"
"	ADC	R20, R22\n"
"//  v+=__multiply_unsigned(m0,0x8D)>>8;\n"
"       MUL     R16, R21\n"
"       ADD     R18, R1\n"
"       ADC     R19, R22\n"
"	ADC	R20, R22\n"
"//  v+=__multiply_unsigned(m1,0xB9)>>8;\n"
"	LDI	R16,0x10    ; Counter & flags\n"
"	LDI	R21,0xB9\n"
"       MUL     R17, R21\n"
"       LDI     R21, 10    ; Next multiplicand\n"
"       ADD     R18, R1\n"
"       ADC     R19, R22\n"
"	ADC	R20, R22\n"
"	BREQ	L_i2a_0\n"
"	SUBI	R20,208\n"
"	ST	Z+,R20\n"
"	INC	R16\n"
"L_i2a_0:\n"
"//   39     UINT16 hv;\n"
"//   40     UINT8 bv;\n"
"//   41     bv=v>>8;\n"
"       MOV     R17, R19\n"
"//   42     v=__multiply_unsigned(v,10);\n"
"       MUL     R18, R21\n"
"       MOVW    R19:R18, R1:R0\n"
"//   43     hv=__multiply_unsigned(bv,10);\n"
"       MUL     R17, R21\n"
"//   44     v+=(hv&0xFF)<<8;\n"
"       ADD     R19, R0\n"
"//   45     if (SREG_Bit0) hv+=0x100;\n"
"	ADC	R1, R22\n"
"//   46     bv=hv>>8;\n"
"       MOV     R17, R1\n"
"//   47     if ((i|bv)&0x8F)\n"
"       MOV     R20, R1\n"
"       OR      R20, R16\n"
"       ANDI    R20, 0x8F\n"
"       BREQ    L_i2a_1\n"
"//   48     {\n"
"//   49       *s++=bv+'0';\n"
"	SUBI	R17,208\n"
"	ST	Z+,R17\n"
"//   50       i|=1;\n"
"//     ORI     R18, 0x01\n"
"L_i2a_1:\n"
"//   51     }\n"
"//   52     i<<=1;\n"
"	ROL	R16\n"
"//   54   while(!SREG_Bit0);\n"
"       BRBC    0, L_i2a_0\n"
"//   55   *s=0;\n"
"       ST      Z, R22\n"
"//   56 }\n");
//      RET
}

/***********************************************
integer to HEX routine
***********************************************/
void i2hex(UINT8 val, char* dest, int len)
{
	char* cp;
	UINT8 n;
	char x;        
	n = val;
	cp = &dest[len];        
        *cp='\0';
	while (cp > dest)
	{
		x = n & 0xF;
		n >>= 4;
		*--cp = x + ((x > 9) ? 'A' - 10 : '0');
	}
	return;
}

#pragma inline = forced
print_hexword (unsigned int word)
{
  __no_init char h2a_locbuf[5];
  char* ph2a;
  ph2a=&h2a_locbuf[0];
  i2hex((UINT8)(word>>8), ph2a, 2);      
//  ph2a=&h2a_locbuf[0];
  while (*ph2a) UART_TX(*ph2a++);  
  ph2a=&h2a_locbuf[0];
  i2hex((UINT8)(word), ph2a, 2);      
//  ph2a=&h2a_locbuf[0];
  while (*ph2a) UART_TX(*ph2a++); 
  UART_TX(','); 
}


void print_fstr (const char __flash * s)
{
      while (*s) UART_TX (*s++);
}

void print_decnum ( const char __flash * s, UINT16 Num)
{
  char* pi2a;
  __no_init char i2a_locbuf[6];
  pi2a=&i2a_locbuf[0];
  i2a(pi2a, Num);
  while (*s) UART_TX (*s++);
//  UART_TX(':'); 
  while (*pi2a) UART_TX(*pi2a++);  
  UART_TX(',');
  UART_TX(' '); 
}

void print_hexnum ( const char __flash * s, UINT16 Num)
{
  char* pi2a;
  __no_init char i2a_locbuf[6];
  pi2a=&i2a_locbuf[0];
  i2hex(Num, pi2a,2);
  while (*s) UART_TX (*s++);
//  UART_TX(':'); 
  while (*pi2a) UART_TX(*pi2a++);  
  UART_TX(',');
  UART_TX(' '); 
}

void print_stream (const char __flash * f, UINT8 *Num, UINT8 len)
{
  __no_init char i2a_locbuf[3];
  char *p;
  while (*f) UART_TX (*f++);    
  p= &i2a_locbuf[0];
//  print_rn();
  while (len--)
  {
    i2hex(*Num++, p,2);
    UART_TX('0'); UART_TX('x');
    while (*p) UART_TX (*p++);    
    if (len) UART_TX(','); UART_TX(' ');
  }  
}

#define Reset_OS1 {if (STATE_DECODER==STATE_DECODER_OS1) STATE_DECODER=STATE_DECODER_IDLE; OS1_VARS.State=STATE_OS1_IDLE; OS1_VARS.BitPosition=0; OS1_VARS.SyncCnt=0;}
#define Reset_OS2 {if (STATE_DECODER==STATE_DECODER_OS2) STATE_DECODER=STATE_DECODER_IDLE; OS2_VARS.State=STATE_OS2_IDLE; OS2_VARS.BitPosition=0; OS2_VARS.SyncCnt=0;}
#define Reset_OS3 {if (STATE_DECODER==STATE_DECODER_OS3) STATE_DECODER=STATE_DECODER_IDLE; OS3_VARS.State=STATE_OS3_IDLE; OS3_VARS.BitPosition=0; OS3_VARS.SyncCnt=0;}
#define Stream_Error {STATE_DECODER=STATE_DECODER_IDLE; Reset_OS1; Reset_OS2; Reset_OS3;LED_OFF;debug_index=0;}

#pragma inline = forced
static void ArrBitDefine (UINT8 BitNum, UINT8 BitVal, UINT8* Array)
{
  UINT8 ByteNumber;  
  ByteNumber=(BitNum>>3);
  UINT8* Addr = &Array[ByteNumber];
  UINT8 BN =(BitNum&0x07);
  if (BitVal)    sbi (*Addr, BN);
  else  cbi (*Addr, BN);
}


#pragma vector =  TIMER1_COMPA_vect
__interrupt void TIMER1_COMPA(void)
{
//  unsigned char  i;
  debug_index=0;
  if (OS1_VARS.State==STATE_OS1_DATA)
  {
    #ifdef DEBUG_PRINT1
      print_fstr("\r\nE1");
    #endif  
    // Try to process OS1 data
    __no_operation();
     print_stream("\r\nOS1.0, ", &Stream[0], OS1_VARS.BitPosition>>3);
  }
  if (OS2_VARS.State==STATE_OS2_DATA)
  {
    #ifdef DEBUG_PRINT2_1
      print_fstr("\r\nE2");
    #endif      
    // Try to process OS2 data
     __no_operation();     
     print_stream("\r\nOS2.1, ", &Stream[0], OS2_VARS.BitPosition>>4);
  }  
  if (OS3_VARS.State==STATE_OS3_DATA)
  {
    #ifdef DEBUG_PRINT3
      print_fstr("\r\nE3");
    #endif      
    // Try to process OS3 data
     __no_operation();
//     print_stream("\r\ndebug: ", &debug_buff[0], 31);
     print_stream("\r\nOS3.0, ", &Stream[0], OS3_VARS.BitPosition>>3);
     print_rn();
  } 
    LED_OFF;
    Reset_OS1;     Reset_OS2;     Reset_OS3;
    STATE_DECODER=STATE_DECODER_IDLE;
    TCNT1=0;
    SET_ICP_FALLING; 
    TIFR=(1<<ICF1)|(1<<OCF1A); // Reset old interrupts flags
}

#pragma vector =  TIMER1_CAPT_vect
__interrupt void ISR_TIMER1_CAPT(void)
{
  UINT16 newICP; //Local Copy of ICP register
  UINT16 period;
  newICP=ICR1;  // Read and copy current ICP value
  TCNT1=0; // Reset counter
  period = newICP;//-ICP_VARS.oldICP;  
    //1.5 ms pulse handler ()    
  if InRange(period, SetTimer_us(1630), 250)
    {      
      if (STATE_DECODER==STATE_DECODER_IDLE)
      {
        /*
        if ICP_IS_FALLING
        {
         // Else If OS1 decoder state is "Idle" then interpretate 1.5 ms high pulse as OS1 preamble
          Reset_OS2; Reset_OS3; // set OS2, OS3 state as idle        
          OS1_VARS.BitPosition++; // Increment Preamble bits counter                
          if (OS1_VARS.BitPosition>=6) 
          {
            OS1_VARS.State=STATE_OS1_SYNC1;
            OS1_VARS.BitPosition=0;
            STATE_DECODER=STATE_DECODER_OS1;
            #ifdef DEBUG_PRINT1
            print_fstr("\r\nS11");             
            #endif
          }
         }
        */
      }
      else if (STATE_DECODER==STATE_DECODER_OS1)
      {
        switch (OS1_VARS.State)
        {
          case STATE_OS1_IDLE:
            Reset_OS1;
            break;
          case STATE_OS1_SYNC1:
            Reset_OS1;
            break;            
          case STATE_OS1_SYNC2:
            Reset_OS1;
            break;            
          case STATE_OS1_SYNC3:
            Reset_OS1;
            break;                        
          case STATE_OS1_DATA:
            // If OS1 decoder state is "Data reception" then interpretate 1.5 ms high pulse as OS1 narrow high data pulse
            if ICP_IS_FALLING
            {
              if (OS1_VARS.LastBit == 1)
              {
                ArrBitDefine(OS1_VARS.BitPosition, 1, Stream);
                #ifdef DEBUG_PRINT1
//                  UART_TX('1');
                #endif  
                OS1_VARS.LastBit=1;
                if (OS1_VARS.BitPosition++>127) OS1_VARS.BitPosition=0;
              }
              else __no_operation(); // Just skip this transition              
            }
            else //ICP_IS_RISING
            {
              Reset_OS1;
            }
            break;           
        }                                
      }
      else Stream_Error;
    }
    else if ((InRange(period, SetTimer_us(1293), 250))&&(OS1_VARS.State==STATE_OS1_DATA))
    { 
           if ICP_IS_RISING
            {
              if (OS1_VARS.LastBit==0)
              {
                  ArrBitDefine(OS1_VARS.BitPosition, 0, Stream);
                  #ifdef DEBUG_PRINT1
//                    UART_TX('0');
                  #endif 
                   if (OS1_VARS.BitPosition++>127) OS1_VARS.BitPosition=0;
               }
             else  __no_operation(); // Just skip this pulse
            }
           else Reset_OS1;
    }
   //3.0 ms pulse handler
   else if InRange(period,  SetTimer_us(2921), 400)
    {  
     if (STATE_DECODER==STATE_DECODER_IDLE)
      {      
        Reset_OS1;
      }
     else if (STATE_DECODER==STATE_DECODER_OS1)
     {
       switch (OS1_VARS.State)
       {
       case STATE_OS1_SYNC1:
         Reset_OS1;
         break;
       case STATE_OS1_SYNC2:
         Reset_OS1;
         break;         
       case STATE_OS1_SYNC3:         
         Reset_OS1;
         break;         
       case STATE_OS1_DATA:         
       if ICP_IS_FALLING
       {
       // If OS1 decoder state is "Data reception" then interpretate 3 ms high pulse as OS1 wide high data pulse
        if (OS1_VARS.LastBit==0)
        {
          ArrBitDefine(OS1_VARS.BitPosition, 1, Stream);
          OS1_VARS.LastBit=1;
          #ifdef DEBUG_PRINT1
//            UART_TX('1');
          #endif
          if (OS1_VARS.BitPosition++>127) OS1_VARS.BitPosition=0;
        }
        else Reset_OS1; //Error in OS1 manchester stream         
       }
       else //ICP_IS_RISING
       {
        // If OS1 decoder state is "Data reception" then interpretate 3 ms low pulse as OS1 wide low data pulse
        if (OS1_VARS.LastBit==1)
        {
          ArrBitDefine(OS1_VARS.BitPosition, 0, Stream);
          OS1_VARS.LastBit=0;
          #ifdef DEBUG_PRINT1
//            UART_TX('0');
          #endif  
          if (OS1_VARS.BitPosition++>127) OS1_VARS.BitPosition=0;
        }
        else Reset_OS1; //Error in OS1 manchester stream 
       }         
        break;                  
       }       
     }
     else Stream_Error;       
    }
    //4.31 ms pulse handler (First sync pulse for OS1.0 protocol)
    else if InRange(period, SetTimer_us(4310), 350)
    {
      Reset_OS2;Reset_OS3;
      if (STATE_DECODER==STATE_DECODER_IDLE)
      {
        if ICP_IS_RISING
        {
          if (OS1_VARS.State==STATE_OS1_IDLE)
          {
            OS1_VARS.State=STATE_OS1_SYNC2;
            STATE_DECODER=STATE_DECODER_OS1;
            #ifdef DEBUG_PRINT1
//            print_fstr("\r\nS12");             
            #endif            
          }
          else Reset_OS1;
        }
        else
        {
          Reset_OS1;
        }
      }
      else Stream_Error;
    }
    //5.5 ms pulse handler (second sync pulse for OS1.0 protocol)
    else if InRange(period, SetTimer_us(5630), 250)
     {
        Reset_OS2;Reset_OS3;       
       if (STATE_DECODER==STATE_DECODER_OS1)
       {
         if ICP_IS_FALLING
         {
          if (OS1_VARS.State==STATE_OS1_SYNC2)
          {
            OS1_VARS.State=STATE_OS1_SYNC3;
            #ifdef DEBUG_PRINT1
 //           print_fstr("\r\nS13");             
            #endif            
          }
          else Reset_OS1;
         }
         else 
         {
           Reset_OS1;
         }
       }
       else Stream_Error
    }
    //5.31 ms pulse handler (Third short sync pulse for OS1.0 protocol)
    else if InRange (period, SetTimer_us(5310), 250)
    {
        Reset_OS2;Reset_OS3;      
      if (STATE_DECODER==STATE_DECODER_OS1)
      {
       if ICP_IS_RISING
       {
         if (OS1_VARS.State==STATE_OS1_SYNC3)
          {
            OS1_VARS.State=STATE_OS1_DATA;
            OS1_VARS.BitPosition=0; 
//            ArrBitDefine(OS1_VARS.BitPosition, 1, Stream);
            OS1_VARS.LastBit=1;
//            OS1_VARS.BitPosition++;   
            #ifdef DEBUG_PRINT1
            print_fstr("\r\nD1S");             
            #endif            
          }
         else Reset_OS1;
       }
       else Reset_OS1;
      }
    else  Stream_Error;    
    }
    //6.86 ms pulse handler (Third long sync pulse for OS1.0 protocol)
    else if InRange (period, SetTimer_us(6750), 350)      
    {
              Reset_OS2;Reset_OS3;
       if (STATE_DECODER==STATE_DECODER_OS1)
       {
        if ICP_IS_RISING
        {
          if (OS1_VARS.State==STATE_OS1_SYNC3)
            {
              OS1_VARS.State=STATE_OS1_DATA;          
              OS1_VARS.BitPosition=0; 
//              ArrBitDefine(OS1_VARS.BitPosition, 0, Stream);          
              OS1_VARS.LastBit=0;
//              OS1_VARS.BitPosition++;          
              #ifdef DEBUG_PRINT1
              print_fstr("\r\nD1L");             
              #endif                            
            }      
          else Reset_OS1;
        }
        else Reset_OS1;
       }
       else Stream_Error;
    }  
  /*1.0 ms pulse handler (OS2.1 preambule or wide data pulse, OS3 high wide pulse)*/
   else if InRange (period, us1000, 325)
    {
         Reset_OS1; // set OS1 state as idle
         if (STATE_DECODER==STATE_DECODER_IDLE)
         { // OS2.1 preamble high pulse _|--|_
           Reset_OS3;
           OS2_VARS.BitPosition++; // Increment OS2 Preamble bits counter
           if (OS2_VARS.BitPosition>=24) 
            {
               #ifdef DEBUG_PRINT2_1
                  print_fstr("\r\nS2");
               #endif  
               OS2_VARS.State=STATE_OS2_SYNC;
               OS2_VARS.BitPosition=0;
               STATE_DECODER=STATE_DECODER_OS2;
            }
         }
         else if (STATE_DECODER==STATE_DECODER_OS2)
         {
          switch (OS2_VARS.State)
          {
           case STATE_OS2_SYNC: 
             #ifdef  preamlbe_led 
             if ICP_IS_FALLING
             {
               LED_OFF;
             } else LED_ON;
             #endif
             break;
           case STATE_OS2_DATA:
              if ICP_IS_FALLING
               {
                 if (OS2_VARS.LastBit==0)
                 {
                    UREG BitPos=OS2_VARS.BitPosition;
                    if (BitPos&0x01) ArrBitDefine(BitPos>>1, 1, Stream);
    //              UART_TX('1');
                    OS2_VARS.LastBit=1;
                    if (BitPos++>254) BitPos=0;
                    OS2_VARS.BitPosition=BitPos;           
                    #ifdef bit_led
                       LED_ON;
                    #endif  
                 }
                 else Reset_OS2; //Error in 2.1 stream
               }
             else //ICP_IS_RISING
             {
               if (OS2_VARS.LastBit==1)
               {
                 UREG BitPos=OS2_VARS.BitPosition;           
                 if (BitPos&0x01) ArrBitDefine(BitPos>>1, 0, Stream);
 //                UART_TX('0');
                 OS2_VARS.LastBit=0;
                 if (BitPos++>254) BitPos=0;
                 OS2_VARS.BitPosition=BitPos;
                 #ifdef bit_led
                   LED_OFF;
                 #endif  
               }
               else Reset_OS2; //Error in OS2.1 Stream                             
             }
             break; 
            }
         }
         else if (STATE_DECODER==STATE_DECODER_OS3)
         {
           switch (OS3_VARS.State)
           {
             case STATE_OS3_SYNC:
               if ICP_IS_RISING
               {
                  Reset_OS2;
                  #ifdef start_bit_led 
                    LED_ON;
                  #endif
                  #ifdef DEBUG_PRINT3
                    print_fstr("\r\nD3");   
                  #endif  
  //              print_hexword(newICP);  
                  OS3_VARS.State=STATE_OS3_DATA;  // OS3 decoder in data mode
                  OS3_VARS.LastBit=1;
                  OS3_VARS.BitPosition=0;
               }
              case STATE_OS3_DATA:
                if ICP_IS_FALLING  //OS3 high wide data pulse _|--|_          
                {
                  if (OS3_VARS.LastBit==0) //_|--|_ (0,1)
                    {
                        ArrBitDefine(OS3_VARS.BitPosition, 1, Stream);              
                        #ifdef bit_led
                          LED_ON;
                        #endif  
                        OS3_VARS.LastBit=1;              
                        if (OS3_VARS.BitPosition++>127) OS3_VARS.BitPosition=0;
                    }
                  else Reset_OS3;  // Error in the OS3 manchester stream |-|_|--|_
                }
                else // ICP_IS_RISING //-|__|-
                {
                  if (OS3_VARS.LastBit==1)
                  {
                    ArrBitDefine(OS3_VARS.BitPosition, 0, Stream);
                    OS3_VARS.LastBit=0;
                    #ifdef bit_led
                       LED_OFF;
                    #endif  
    //              LED_OFF;
                    if (OS3_VARS.BitPosition++>127) OS3_VARS.BitPosition=0;
                  }
                  else  Reset_OS3; //Do not detect OS3 error for 1st data pulse                        
                }
              break;        
           }
         }                       
    }
    /*0.5 ms pulse handler (OS2 start bit or low width data pulse or OS3 preamble or OS3 high narrow bit*/
    else if InRange (period, us500, 250)      
    {
     if (STATE_DECODER==STATE_DECODER_IDLE)
     {
       // If OS3 decoder is in "iddle" state then interpretate 0.5ms high pulse as os3 preamble
       // Reset os1 and os2 decoders to iddle state
        Reset_OS1; Reset_OS2;
        OS3_VARS.BitPosition++; // Increment Preamble bits counter                
        if (OS3_VARS.BitPosition>=32) 
        {
          #ifdef DEBUG_PRINT3
            print_fstr("\r\nS3");
          #endif  
          OS3_VARS.State=STATE_OS3_SYNC;
          OS3_VARS.BitPosition=0;
          STATE_DECODER=STATE_DECODER_OS3;
        }        
     }
     else if (STATE_DECODER==STATE_DECODER_OS2)
     {
      switch (OS2_VARS.State)
         {
           case STATE_OS2_SYNC:
             if ICP_IS_RISING
             {
               OS2_VARS.State=STATE_OS2_DATA;
               OS2_VARS.LastBit=1;
               OS2_VARS.BitPosition=0;
               #ifdef DEBUG_PRINT2_1
                 print_fstr("\r\nD2");   
               #endif                 
             }               
             break;  
           case STATE_OS2_DATA:
            if ICP_IS_FALLING
            {
              // data high narrow
              if (OS2_VARS.LastBit==1)              
              {
                UREG BitPos=OS2_VARS.BitPosition;           
                if (BitPos&0x01) ArrBitDefine(BitPos>>1, 1, Stream);
                OS2_VARS.LastBit=1;
                if (BitPos++>254) BitPos=0;
                OS2_VARS.BitPosition=BitPos;
                #ifdef bit_led
                   LED_ON;
                #endif                
              }
              else {};   //              __no_operation(); // Do nothing - just wait next edge
            }
            else
            {
               if (OS2_VARS.LastBit==0)
                {                
                   UREG BitPos=OS2_VARS.BitPosition;           
                   if (BitPos&0x01) ArrBitDefine(BitPos>>1, 0, Stream);
   //              UART_TX('0');
                   OS2_VARS.LastBit=0;
                   if (BitPos++>254) BitPos=0;
                   OS2_VARS.BitPosition=BitPos;           
                   #ifdef bit_led
                     LED_OFF;
                   #endif  
                }
               else {};
            }
            break;             
         }      
     }
     else if (STATE_DECODER==STATE_DECODER_OS3)
      {
        switch (OS3_VARS.State)
         {
          case STATE_OS3_SYNC:            
            #ifdef  preamlbe_led 
              if ICP_IS_FALLING
              {
                LED_ON;
              } else LED_OFF;
            #endif
            break;          
          case STATE_OS3_DATA:
            if ICP_IS_FALLING
            {
              // Else If OS3 decoder is in "data" state then interpretate 0.5ms high pulse as narrow data pulse
              if (OS3_VARS.LastBit==1) //_|-|_|-|_ (1,1)
              {
                ArrBitDefine(OS3_VARS.BitPosition, 1, Stream);
                #ifdef bit_led
                   LED_ON;
                #endif  
  //            LED_ON;
                OS3_VARS.LastBit=1;
                if (OS3_VARS.BitPosition++>127) OS3_VARS.BitPosition=0; 
              }
              else {};//              __no_operation(); // Do nothing - just wait next edge
            }
            else
            { // OS3 low narrow            
              if (OS3_VARS.LastBit==0)
              {
                ArrBitDefine(OS3_VARS.BitPosition, 0, Stream);
                #ifdef bit_led
                   LED_OFF;
                #endif            
     //          LED_OFF;
                OS3_VARS.LastBit=0;
                if (OS3_VARS.BitPosition++>127) OS3_VARS.BitPosition=0;
              }
              else __no_operation(); // Do nothing - just wait next edge        
            }
            break;                    
          }
        }                       
      }
    else // Pulse with incorrect latency
    {
      if (STATE_DECODER==STATE_DECODER_IDLE) {Stream_Error;}  // Stream error during OS3 preamble capturing
      else if (STATE_DECODER==STATE_DECODER_OS1)
      {
        switch (OS1_VARS.State)
        {
           case STATE_OS1_IDLE:
             Reset_OS1;
             break;
           case STATE_OS1_SYNC1:
            Reset_OS1;          
             break;   
           case STATE_OS1_SYNC2:
            Reset_OS1;                          
             break;            
           case STATE_OS1_SYNC3:
            Reset_OS1;                          
             break;            
           case STATE_OS1_DATA:
            Reset_OS1;         
            #ifdef DEBUG_PRINT1
              if ICP_IS_RISING
              {
                 UART_TX('D'); UART_TX('-'); print_hexword(newICP);
               // Stream error during OS2 data capturing
              }
              else //ICP_IS_FALLING
              {
                 UART_TX('D'); UART_TX('+'); print_hexword(newICP);              
              }
            #endif               
             break;                         
        }
      }
      else if (STATE_DECODER==STATE_DECODER_OS2)
      {
        switch (OS2_VARS.State)
        {
          case STATE_OS2_IDLE:
            Reset_OS2;             // Stream error during OS2 preamble capturing
            break;
          case STATE_OS2_DATA:
            /*
            if ((ICP_IS_RISING)||(!(InRange (period, 150, 70))))
            {
              Reset_OS2;
              #ifdef DEBUG_PRINT2_1
              if ICP_IS_RISING
              {
                 UART_TX('D'); UART_TX('-'); print_hexword(newICP);
               // Stream error during OS2 data capturing
              }
              else //ICP_IS_FALLING
              {
                 UART_TX('D'); UART_TX('+'); print_hexword(newICP);              
              }
              #endif   
            }
            else if ((ICP_IS_FALLING) && (InRange (period, 150, 70)))
            {
              #ifdef DEBUG_PRINT2_1
                print_fstr("\r\nA2");
              #endif     
            }
            */
            if (OS2_VARS.BitPosition>=48)
            {
              #ifdef DEBUG_PRINT2_1
                print_fstr("\r\nA2");
              #endif     
            }
            else
            {
              Reset_OS2;
              #ifdef DEBUG_PRINT2_1
              if ICP_IS_RISING
              {
                 UART_TX('D'); UART_TX('-'); print_hexword(newICP);
               // Stream error during OS2 data capturing
              }
              else //ICP_IS_FALLING
              {
                 UART_TX('D'); UART_TX('+'); print_hexword(newICP);              
              }
              #endif                
            }
            break;
          case STATE_OS2_SYNC:
            Reset_OS2;
            break;
       }      
      }
      else if (STATE_DECODER==STATE_DECODER_OS3)
      {
         switch (OS3_VARS.State)
          {
            case STATE_OS3_DATA:
              
              if ((ICP_IS_RISING)||(!(InRange (period, 175, 50))))
              {
                Reset_OS3;
                #ifdef DEBUG_PRINT3
                if ICP_IS_RISING
                {
                   UART_TX('D'); UART_TX('-'); print_hexword(newICP);
                }
                else
                {
                   UART_TX('D'); UART_TX('+'); print_hexword(newICP);                
                }                   
                #endif
              }
              else if ((ICP_IS_FALLING) && (InRange (period, 175, 50)))
              {
                print_fstr("\r\nA3");
              }              
              break;
            case STATE_OS3_SYNC:
              Reset_OS3;
              break;
          }
      }                 
    }           
   if (ICP_IS_RISING) SET_ICP_FALLING else SET_ICP_RISING;  
};

//Let's start decoding

void main( void )
{
//  UREG i,j=0;
  cbi (LEDPORT, LED); 
  sbi (LEDDDR, LED);
  LED_ON;
  _delay_ms(1000);
  LED_OFF;
  uart_init();
  print_fstr("\r\nOregon Scientific Receiver \r\nFor v1.0, v2.1, v3.0 protocols\r\nBy A.Yerezeyev, (c)2010\r\n");
  _delay_ms(10);
  ICP_VARS.oldICP=0;
  Reset_OS1; Reset_OS2; Reset_OS3; STATE_DECODER=STATE_DECODER_IDLE;
  __disable_interrupt();
  OCR1A=16384; // Timeout for bitstream
  //Timer/Counter1, Input Capture Interrupt Enable
  TIMSK|=(1<<TICIE1)|(1<<OCIE1A); 
  // start Timer1 at CLK/8 with input capture negative mode and input filter enabled
  TCCR1B=(1<<CS11)|(1<<ICNC1);
  TIFR=(1<<ICF1)|(1<<OCF1A);
  __enable_interrupt();
  while (1) nop();
};
