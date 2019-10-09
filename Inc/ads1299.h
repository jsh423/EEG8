#ifndef __ADS1299_H
#define __ADS1299_H
#include "gpio.h"
//#include "openbci.h"
//////////////////////////////////////////////////////////////////////////////////	 

//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//////////////////////////////////////////////////////////////////////////////////////////////////////////


#define	ADS1299_CS0 		PBout(12)  		//ADS1299的片选信号
//#define ADS1299_CS1			PDout(12)
//#define ADS1299_CS2		PDout(13)
//#define ADS1299_CS3			PDout(14)

//#define SW_IMP PBout(14)
//#define PWM_LED PAout(0)
//#define ADS1299_RST2		PAout(15)
//#define ADS1299_RST3		PBout(10)

//#define ADS1299_RST1			PBout(12)
//#define ADS1299_PWDN1			PBout(1)
//#define ADS1299_PWDN2			PBout(9)
//#define ADS1299_PWDN3			PBout(8)

//#define ADS1299_RST0		PBout(7)
//#define ADS1299_START   PDout(12)
//#define ADS1299_PWDN0	PCout(6)
//#define LED0 PBout(1)   //LED0
#define ADS1299_ID 0X3E		

#define openvibeflag 0  //如果采用openvibe协议置一，否则为零

//void NRF24L01_Init(void);//初始化
void ADS1299_Init(void);
void ADS1299_WREG(uint8_t address,uint8_t value);
uint8_t ADS1299_PREG(uint8_t reg);
uint8_t ADS1299_PREGS(void);
void ADS1299_RDATA(uint8_t *p);
void ADS1299_Command(uint8_t Command);
void ADS1299_IT(void);
uint8_t ADS1299_Check(void);
void ADS1299_CHANGE_CHANEL(uint8_t n,uint8_t sw);
void Recev_Data(void);
void Set_Sps(uint8_t i);
void ImpTest_Start(void);
void ImpTest_Stop(void);
//void led_task(void *pdata);
//extern uint8_t adc_buf2[20][27];
//extern void *Sem_Task_ads1299;
//extern uint8_t adc_buf2[28];
extern uint8_t parameter[10];

extern uint8_t stat[4];

extern uint8_t sps;


//SPI Command Definition Byte Assignments (Datasheet, p35)
#define _WAKEUP 0x02 // Wake-up from standby mode
#define _STANDBY 0x04 // Enter Standby mode
#define _RESET 0x06 // Reset the device registers to default
#define _START 0x08 // Start and restart (synchronize) conversions
#define _STOP 0x0A // Stop conversion
#define _RDATAC 0x10 // Enable Read Data Continuous mode (default mode at power-up)
#define _SDATAC 0x11 // Stop Read Data Continuous mode
#define _RDATA 0x12 // Read data by command; supports multiple read back


//Register Addresses
#define ID 0x00
#define CONFIG1 0x01
#define CONFIG2 0x02
#define CONFIG3 0x03
#define LOFF 0x04
#define CH1SET 0x05
#define CH2SET 0x06
#define CH3SET 0x07
#define CH4SET 0x08
#define CH5SET 0x09
#define CH6SET 0x0A
#define CH7SET 0x0B
#define CH8SET 0x0C
#define BIAS_SENSP 0x0D
#define BIAS_SENSN 0x0E
#define LOFF_SENSP 0x0F
#define LOFF_SENSN 0x10
#define LOFF_FLIP 0x11
#define LOFF_STATP 0x12
#define LOFF_STATN 0x13
#define GPIO 0x14
#define MISC1 0x15
#define MISC2 0x16
#define CONFIG4 0x17

#define MWSPT_NSEC 3

#define OPENBCI_NCHAN (8)  // number of EEG channels
// CHANNEL SETTINGS 
#define POWER_DOWN      (0)
#define GAIN_SET        (1)
#define INPUT_TYPE_SET  (2)
#define BIAS_SET        (3)
#define SRB2_SET        (4)
#define SRB1_SET        (5)
#define YES      	(0x01)
#define NO      	(0x00)

//gainCode choices
#define ADS_GAIN01 (0b00000000)	// 0x00
#define ADS_GAIN02 (0b00010000)	// 0x10
#define ADS_GAIN04 (0b00100000)	// 0x20
#define ADS_GAIN06 (0b00110000)	// 0x30
#define ADS_GAIN08 (0b01000000)	// 0x40
#define ADS_GAIN12 (0b01010000)	// 0x50
#define ADS_GAIN24 (0b01100000)	// 0x60

//inputType choices
#define ADSINPUT_NORMAL (0b00000000)
#define ADSINPUT_SHORTED (0b00000001)
#define ADSINPUT_BIAS_MEAS (0b00000010)
#define ADSINPUT_MVDD (0b00000011)
#define ADSINPUT_TEMP (0b00000100)
#define ADSINPUT_TESTSIG (0b00000101)
#define ADSINPUT_BIAS_DRP (0b00000110)
#define ADSINPUT_BIAS_DRN (0b00000111)

//test signal choices...ADS1299 datasheet page 41
#define ADSTESTSIG_AMP_1X (0b00000000)
#define ADSTESTSIG_AMP_2X (0b00000100)
#define ADSTESTSIG_PULSE_SLOW (0b00000000)
#define ADSTESTSIG_PULSE_FAST (0b00000001)
#define ADSTESTSIG_DCSIG (0b00000011)
#define ADSTESTSIG_NOCHANGE (0b11111111)

//Lead-off signal choices
#define LOFF_MAG_6NA (0b00000000)
#define LOFF_MAG_24NA (0b00000100)
#define LOFF_MAG_6UA (0b00001000)
#define LOFF_MAG_24UA (0b00001100)
#define LOFF_FREQ_DC (0b00000000)
#define LOFF_FREQ_7p8HZ (0b00000001)
#define LOFF_FREQ_31p2HZ (0b00000010)
#define LOFF_FREQ_FS_4 (0b00000011)
#define PCHAN (0)
#define NCHAN (1)
#define OFF (0)
#define ON (1)

// used for channel settings
#define ACTIVATE_SHORTED (2)
#define ACTIVATE (1)
#define DEACTIVATE (0)

#define PCKT_START 0xA0	// prefix for data packet error checking
#define PCKT_END 0xC0	// postfix for data packet error checking




























#endif
