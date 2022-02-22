#include "init_rx220.h"
#include "iodefine.h"
#include "init_rx220.h"
#include "iodefine.h"

void init_rx220(void){
	
	SYSTEM.PRCR.WORD 	= 0xA50B;		//Write protect ALL OFF
	SYSTEM.SCKCR.BIT.ICK 	= 0x0;			//ICLK divided by 1
	SYSTEM.SCKCR.BIT.PCKB 	= 0x0;			//PCLKB divided by 1
	SYSTEM.SCKCR.BIT.PCKD 	= 0x0;			//PCLKD divided by 1
	SYSTEM.SCKCR3.BIT.CKSEL = 0x1; 			//HOCO selected
	SYSTEM.HOCOCR.BIT.HCSTP	= 0x0;			//HOCO ON
	SYSTEM.HOCOCR2.BIT.HCFRQ	= 0x0;		//HOCO 32MHz
	SYSTEM.HOCOPCR.BIT.HOCOPCNT	= 0x0;		//HOCO power ON
	 
	//Standby OFF
	 MSTP(MTU) = 0;
	 MSTP(SCI1) = 0;
	 MSTP(S12AD) = 0;
	 
	//I/O Setting
	PORTE.PDR.BYTE = 0x3e;		//PORTE bit1,2,3,4,5: Output
	PORT5.PDR.BYTE = 0x30;		//PORT5 bit4,5: Output
	PORT1.PDR.BYTE = 0xf0;		//PORT1 bit4,5,6,7: Output
	PORT2.PDR.BYTE = 0x80;		//PORT2 bit7: Output
	PORT3.PDR.BYTE = 0x06;		//PORT3 bit1,2: Output
	PORTH.PDR.BYTE = 0x0f;		//PORTH bit1,2,3,4: Output
	PORTB.PDR.BYTE = 0xff;		//PORTB bit1-7: Outpout
	PORTA.PDR.BYTE = 0x00;		//PORTA bit1-7: Input
	
	PORTE.PODR.BIT.B3 = 1;		//PORTE bit3: High
	PORTE.PODR.BIT.B4 = 1;		//PORTE bit4: High
	PORTE.PODR.BIT.B5 = 1;		//PORTE bit5: High
	PORT5.PODR.BYTE = 0x00;		//PORT5 bit4,5: Low
	PORT1.PODR.BIT.B4 = 1;		//PORTB bit4: High(0v)7seg
	PORT1.PODR.BIT.B5 = 1;		//PORTB bit5: High
	PORT1.PODR.BIT.B6 = 1;		//PORTB bit6: High
	PORT1.PODR.BIT.B7 = 1;		//PORTB bit7: High
	PORT2.PODR.BIT.B7 = 1;		//PORT2 bit7: High
	PORT3.PODR.BIT.B1 = 1;		//PORT3 bit1: High
	PORT3.PODR.BIT.B2 = 1;		//PORT3 bit2: High
	PORTH.PODR.BYTE = 0x00;		//PORTH bit1,2,3,4: Low
	PORTB.PODR.BYTE = 0x00;		//PORTB bit1-7: low
	
	PORTE.PMR.BYTE = 0x00;		//PORTE bit3,4,5: I/O
	PORT5.PMR.BYTE = 0x00;		//PORT5 bit4,5: I/O
	PORT1.PMR.BYTE = 0x00;		//PORT1 bit4,5,6,7: I/O
	PORT2.PMR.BYTE = 0x00;		//PORT2 bit7: I/O
	PORT3.PMR.BYTE = 0x00;		//PORT3 bit1,2: I/O
	PORTH.PMR.BYTE = 0x00;		//PORTH bit1,2,3,4: I/O
	PORTB.PMR.BYTE = 0x00;		//PORTB bit1-7: I/O
	
	PORT2.PMR.BIT.B6 = 1; 		//PORT2 bit6: SCI TXD1
	PORTE.PMR.BIT.B1 = 1;		//PORTE bit1: MTIOC4C
	PORTE.PMR.BIT.B2 = 1; 		//PORTE bit2: MTIOC4A
	PORTA.PMR.BIT.B1 = 1;		//PORTA bit1: MTCLKC(p390)
	PORTA.PMR.BIT.B3 = 1;		//PORTA bit3: MTCLKD
	PORTA.PMR.BIT.B4 = 1;		//PORTA bit4: MTCLKA
	PORTA.PMR.BIT.B6 = 1;		//PORTA bit6: MTCLKB
	PORTB.PMR.BIT.B3 = 1;		//PORTB bit3: MTIOC0A(speaker)
	
	//MPC setting
	MPC.PWPR.BIT.B0WI = 0;
	MPC.PWPR.BIT.PFSWE = 1;
	MPC.P26PFS.BYTE = 0xa; 		//P26 TXD1
	MPC.PE1PFS.BYTE = 0x1;		//PE1 MTIOC4C
	MPC.PE2PFS.BYTE = 0x1;		//PE2 MTIOC4A
	MPC.PA1PFS.BYTE = 0x2;		//PA1 MTCLKC
	MPC.PA3PFS.BYTE = 0x2;		//PA3 MTCLKD
	MPC.PA4PFS.BYTE = 0x2;		//PA4 MTCLKA
	MPC.PA6PFS.BYTE = 0x2;		//PA6 MTCLKB
	MPC.PB3PFS.BYTE = 0x1;		//PB3 MTIOC0A
	
	//MTU2a Setting
	//Speaker PWM Setting
	MTU0.TCR.BIT.TPSC = 0x2;	//PCLK/16(2Mhz)
	MTU0.TCR.BIT.CCLR = 0x2;	//When TGRB compare match Clear TCNT
	MTU0.TMDR.BIT.MD = 0x2;		//PWM Mode1
	MTU0.TIORH.BYTE = 0x12;		//initial Low, TGRA compare match High TGRB compare match Low
	MTU0.TGRA = 500;
	MTU0.TGRB = 1000;		//2khz
	MTU0.TIER.BIT.TGIEA = 0;	//Disable interrupt from TGIA
	
	//RotallyEncoder Setting
	MTU1.TMDR.BIT.MD = 0x4;		//MTCLK Mode1(p417)
	MTU2.TMDR.BIT.MD = 0x4;		//MTCLK Mode1
	MTU2.TCNT = 10000;		//count init(R)
	MTU1.TCNT = 10000;		//count init(L)
	//Sensor Blinking
	MTU3.TCR.BIT.TPSC = 0x2;	//PCLK/16(2Mhz)
	MTU3.TCR.BIT.CCLR = 0x2;	//When TGRB compare match Clear TCNT
	MTU3.TGRA = (204-1);		//102us
	MTU3.TGRB = (250-1);		//125us interrupt period
	MTU3.TIER.BIT.TGIEA = 1;	//Enable interrupt from TGRA
	MTU3.TIER.BIT.TGIEB = 1;	//Enable interrupt from TGRB
	
	//PWM Motor
	MTU4.TCR.BIT.TPSC = 0x0;	//PCLK/1(32Mhz)
	MTU4.TCR.BIT.CCLR = 0x2;	//When TGRB compare match Clear TCNT
	MTU4.TMDR.BIT.MD = 0x02;	//PWM mode1
	MTU.TOER.BIT.OE4A = 1;		//MTIOC4A Output
	MTU.TOER.BIT.OE4C = 1;		//MTIOC4C Output
	MTU4.TIORH.BYTE = 0x12;		//initial Low, TGRA compare match High TGRB compare match Low
	MTU4.TIORL.BYTE = 0x12;		//initial Low, TGRC compare match High TGRD compare match Low
	MTU4.TGRA = (320-1);		//Duty MTIOC4A (left weel)
	MTU4.TGRB = (320-1);		//PWM period (left weel)
	MTU4.TGRC = (320-1);		//Duty MTIOC4C (right weel)
	MTU4.TGRD = (320-1);		//PWM period (right weel)
	MTU4.TIER.BIT.TGIEB = 1;	//Enable interrupt from TGRB
	
	
	//SCI
	SCI1.SMR.BYTE = 0x00;		//PCLK/1, stopbit=1, no parity bit 
	SCI1.SEMR.BIT.ABCS = 1;		//1bit = 8clock cicle
	SCI1.BRR = 51;			//baudrate 38462bps
	SCI1.SCR.BYTE = 0x20;		//TE=1
	
	//ADC
	S12AD.ADCER.BIT.ADRFMT = 0;	//ADC data right justified
	S12AD.ADCSR.BIT.EXTRG = 1;	//Asynchronous
	S12AD.ADCSR.BIT.TRGE = 1;	//Enable ADC start by Trigger
	S12AD.ADCSR.BIT.ADCS = 0x0;	//Single Mode
	
	//Interrupt Setting
	IEN(MTU3,TGIA3) = 1;		//Enable MTU3 TGIA interrupt
	IPR(MTU3,TGIA3) = 10;		//MTU3 Priority is 10
	IEN(MTU3,TGIB3) = 1;		//Enable MTU3 TGIB interrupt
	IPR(MTU3,TGIB3) = 10;		//MTU3 Priority is 10
	IEN(MTU4,TGIB4) = 1;		//Enable MTU4 TGIB interrupt
	IPR(MTU4,TGIB4) = 9;		//MTU3 Priority is 10
	
	MTU.TSTR.BIT.CST1 = 1;		//MTU1 Timer start
	MTU.TSTR.BIT.CST2 = 1;		//MTU2 Timer start
	MTU.TSTR.BIT.CST3 = 1;		//MTU3 Timer start
	MTU.TSTR.BIT.CST4 = 1;		//MTU4 Timer start
	MTU1.TCNT = 10000;			//encoder default value 10000
	MTU2.TCNT = 10000;			//encoder default value 10000
		
}



/*
void init_rx220(void){
	SYSTEM.PRCR.WORD 	= 0xA50B;		//Write protect ALL OFF
	SYSTEM.SCKCR.BIT.ICK 	= 0x0;			//ICLK divided by 1
	SYSTEM.SCKCR.BIT.PCKB 	= 0x0;			//PCLKB divided by 1
	SYSTEM.SCKCR.BIT.PCKD 	= 0x0;			//PCLKD divided by 1
	SYSTEM.SCKCR3.BIT.CKSEL = 0x1; 			//HOCO selected
	SYSTEM.HOCOCR.BIT.HCSTP	= 0x0;			//HOCO ON
	SYSTEM.HOCOCR2.BIT.HCFRQ	= 0x0;		//HOCO 32MHz
	SYSTEM.HOCOPCR.BIT.HOCOPCNT	= 0x0;		//HOCO power ON
	
	//Standby OFF
	 MSTP(MTU) = 0;
	 MSTP(SCI1) = 0;
	 MSTP(S12AD) = 0;
	
	//I/O Setting
	PORTE.PDR.BYTE = 0x36;		//PORTE bit1, 2, 4, 5: Output
	PORT5.PDR.BYTE = 0x30;		//PORT5 bit4, 5: Output
	PORT1.PDR.BYTE = 0xf0;		//PORT1 bit4, 5, 6, 7: Output
	PORT2.PDR.BYTE = 0x80;		//PORT2 bit7: Output
	PORT3.PDR.BYTE = 0x06;		//PORT3 bit1, 2: Output
	PORTH.PDR.BYTE = 0x0f;		//PORTH bit1,2,3,4: Output
	PORTB.PDR.BYTE = 0xff;		//PORTB bit1-7: Output
	PORTA.PDR.BYTE = 0x00;		//PORTB bit1-7: Input
	
	PORTE.PODR.BYTE = 0x00;		//PORTE bit1, 4, 5: Low
	PORT5.PODR.BYTE = 0x00;		//PORT5 bit4, 5: Low
	PORT1.PODR.BYTE = 0x00;		//PORT1 bit4, 5, 6, 7: Low
	PORT2.PODR.BYTE = 0x00;		//PORT2 bit7: Low
	PORT3.PODR.BYTE = 0x00;		//PORT3 bit1, 2: Low
	PORTH.PODR.BYTE = 0x00;		//PORTH bit1,2,3,4: Low
	PORTB.PODR.BYTE = 0x00;		//PORTB bit1-7: Low
	
	PORTE.PMR.BYTE = 0x00;		//PORTE bit1, 4, 5: I/O
	PORT5.PMR.BYTE = 0x00;		//PORT5 bit4, 5: I/O
	PORT1.PMR.BYTE = 0x00;		//PORT1 bit4, 5, 6, 7: I/O
	PORT2.PMR.BYTE = 0x00;		//PORT2 bit7: I/O
	PORT3.PMR.BYTE = 0x00;		//PORT3 bit1, 2: I/O
	PORTH.PMR.BYTE = 0x00;		//PORTH bit1,2,3,4: I/O
	PORTB.PMR.BYTE = 0x00;		//PORTB bit1-7: I/O
	
	PORT2.PMR.BIT.B6 = 1;		//PORT2 bit6: SCI TXD1
	PORTE.PMR.BIT.B1 = 1; 		//PORTE bit1: MTIOC4C
	PORTE.PMR.BIT.B2 = 1; 		//PORTE bit2: MTIOC4A
	PORTA.PMR.BIT.B1 = 1; 		//PORTE bit1: MTIOC4C
	PORTA.PMR.BIT.B3 = 1; 		//PORTE bit3: MTIOC4D
	PORTA.PMR.BIT.B4 = 1; 		//PORTE bit4: MTIOC4A
	PORTA.PMR.BIT.B6 = 1; 		//PORTE bit6: MTIOC4B
	
	//MPC setting
	MPC.PWPR.BIT.B0WI = 0;
	MPC.PWPR.BIT.PFSWE = 1;
	MPC.P26PFS.BYTE = 0xa;		//P26 TXD1
	MPC.PE1PFS.BYTE = 0x1; 		//PE1 MTIOC4C
	MPC.PE2PFS.BYTE = 0x1; 		//PE2 MTIOC4A
	MPC.PA1PFS.BYTE = 0x2;		//MTCLK
	MPC.PA3PFS.BYTE = 0x2;		//MTCLK
	MPC.PA4PFS.BYTE = 0x2;		//MTCLK
	MPC.PA6PFS.BYTE = 0x2;		//MTCLK
	 
	//MTU2a Setting
	MTU1.TMDR.BIT.MD = 0x4;		//MTCLK mode1
	MTU2.TMDR.BIT.MD = 0x4;		//MTCLK mode1
	
	MTU3.TCR.BIT.TPSC = 0x2;	//PCLK/16
	MTU3.TCR.BIT.CCLR = 0x2;	//Clear TGRB compare match
	MTU3.TGRA = (204-1);		//102us
	MTU3.TGRB = (250-1);		//125us interrupt period
	MTU3.TIER.BIT.TGIEA = 1;	//Enable interrupt from TGIA
	MTU3.TIER.BIT.TGIEB = 1;	//Enable interrupt from TGIB
	
	MTU4.TCR.BIT.TPSC = 0x0;	//PCLK/1
	MTU4.TCR.BIT.CCLR = 0x2;	//Clear TGRB compare match
	MTU4.TMDR.BIT.MD = 0x02;	//PWM mode1
	MTU.TOER.BIT.OE4A = 1;		//MTIOC4A Output
	MTU.TOER.BIT.OE4C = 1;		//MTIOC4C Output
	MTU4.TIORH.BYTE = 0x12;		//initial Low, compare match High
	MTU4.TIORL.BYTE = 0x12;		//initial Low, compare match High 
	MTU4.TGRA = (320-1);		//Duty MTIOC4A(left)
	MTU4.TGRB = (320-1);		//PWM period
	MTU4.TGRC = (320-1);		//Duty MTIOC4C(right)
	MTU4.TGRD = (320-1);		//PWM period
	MTU4.TIER.BIT.TGIEB = 1;	//Enable interrupt from TGIB
	 
	//SCI
	SCI1.SMR.BYTE = 0x00;			//PCLK/1, stopbit=1, no parity bit
	SCI1.SEMR.BIT.ABCS = 1;			//1bit =8clock cycle
	SCI1.BRR = 51;				//baudrate 38462bps
	SCI1.SCR.BYTE = 0x20;			//TE = 1
	
	//ADC
	S12AD.ADCER.BIT.ADRFMT = 0;		//ADC data right justified
	S12AD.ADCSR.BIT.EXTRG = 1;		//Asynchronous
	S12AD.ADCSR.BIT.TRGE = 1;		//Enable ADC start by Trigger
	 
	//Interrupt Setting
	IEN(MTU3,TGIA3) = 1;			//Enable MTU3 TGIA interrupt
	IPR(MTU3,TGIA3) = 10;			//MTU3 Priority is 10
	IEN(MTU3,TGIB3) = 1;			//Enable MTU3 TGIB interrupt
	IPR(MTU3,TGIB3) = 10;			//MTU3 Priority is 10
	IEN(MTU4,TGIB4) = 1;			//Enable MTU4 TGIB interrupt
	IPR(MTU4,TGIB4) = 9;			//MTU4 Priority is 10
	
	MTU.TSTR.BIT.CST1 = 1;			//MTU1 Timer start
	MTU.TSTR.BIT.CST2 = 1;			//MTU2 Timer start
	MTU.TSTR.BIT.CST3 = 1;			//MTU3 Timer Start
	MTU.TSTR.BIT.CST4 = 1;			//MTU4 Timer start
	
	MTU1.TCNT = 10000;			//encoder default value 10000
	MTU2.TCNT = 10000;			//encoder default value 10000
	
}
*/