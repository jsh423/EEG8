#include "ads1299.h"
#include "spi.h"
#include "stm32f1xx_hal_gpio.h"

//#include "openbci.h""
//#include "arm_math.h"
//#include "dma.h"
//////////////////////////////////////////////////////////////////////////////////	 
									  
////////////////////////////////////////////////////////////////////////////////// 	
uint8_t check=0;
uint8_t TxData0[28]={0x12,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //���͵�ַ
//uint8_t TxData1[24]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//uint8_t RxData[4][4]; //���ܵ�ַ
uint8_t sps=0x06;
uint8_t gain=0x05;//06:24,05:12,04:8,03:6,02:4,01:2,00:1
uint8_t parameter[10];
uint8_t Flag_adc;
//u32 stat;
uint8_t index1=0;
uint8_t res3;
uint8_t adc_buf2[105];
uint8_t stat[4];

void ADS1299_SDATAC(void)

{
	uint8_t i=0;
	//for(i=0;i<4;i++)
	//{
	ADS1299_CHANGE_CHANEL(i,0);
	//ADS1299_CS0=0;
	SPI2_ReadWriteByte(0X11);
	//ADS1299_CS0=1;
		ADS1299_CHANGE_CHANEL(i,1);
		//delay_us(2);
	//}
}
void ADS1299_RDATAC(void)
{
	uint8_t i=0;
	//for(i=0;i<4;i++)
	//{
	ADS1299_CHANGE_CHANEL(i,0);
	SPI2_ReadWriteByte(0X10);
	ADS1299_CHANGE_CHANEL(i,1);
	//}
}
void ADS1299_Command(uint8_t Command)
{
	uint8_t i=0;
//	for(i=0;i<4;i++)
//	{
	ADS1299_CHANGE_CHANEL(i,0);
	SPI2_ReadWriteByte(Command);
	ADS1299_CHANGE_CHANEL(i,1);
	//}
}

//��ȡ�Ĵ�������
uint8_t ADS1299_PREG(uint8_t reg)
{
	  uint8_t Byte;	
   	//ADS1299_CS0=0;                 //ʹ��SPI����
	ADS1299_CHANGE_CHANEL(0,0);
	//delay_us(200);
  	SPI2_ReadWriteByte(0X00|0X20);//���ͼĴ�����	
  	SPI2_ReadWriteByte(0X00);      //д��Ĵ�����ֵ
	Byte=SPI2_ReadWriteByte(0);
	ADS1299_CHANGE_CHANEL(0,1);
  	//ADS1299_CS0=1;                 //��ֹSPI����	   
  	return(Byte);       		    //����״ֵ̬
}
//д��ADS1299�Ĵ�������
void ADS1299_WREG(uint8_t address,uint8_t value)
{
	uint8_t i=0;

	ADS1299_CHANGE_CHANEL(i,0);
	//delay_us(3);
	SPI2_ReadWriteByte(0X40|address);
	SPI2_ReadWriteByte(00);
	SPI2_ReadWriteByte(value);
	//delay_us(2);
	ADS1299_CHANGE_CHANEL(i,1);
	
}
void ADS1299_WREG_Single(uint8_t n,uint8_t address,uint8_t value)
{
	//uint8_t i;
	
	ADS1299_CHANGE_CHANEL(n,0);
	//delay_us(3);
	SPI2_ReadWriteByte(0X40|address);
	SPI2_ReadWriteByte(00);
	SPI2_ReadWriteByte(value);
	//delay_us(2);
	ADS1299_CHANGE_CHANEL(n,1);
	
}
	
#if 1
void ADS1299_CHANGE_CHANEL(uint8_t n,uint8_t sw)
{
	switch(n)
	{
		case 0:
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,sw);
			//ADS1299_CS0=sw;
			break;
//		case 1:
//			ADS1299_CS1=sw;
//			break;
//		case 2:
//			ADS1299_CS2=sw;
//			break;
//		case 3:
//			ADS1299_CS3=sw;
//			break;
		default:
			break;
	}
}
#endif
//��ʼ��24L01��IO��
void ADS1299_Init(void)
{

	ADS1299_CHANGE_CHANEL(0,1);; //���ε�һƬADS1299,���Ե�оƬ
	ADS1299_Command(_WAKEUP);
	ADS1299_Command(_RESET);
	
	HAL_Delay(20);
	parameter[0]=0xa0;//֡ͷ
	parameter[1]=8;//֡��
	parameter[9]=0xc0;//֡β
	parameter[2]=0x21;
	parameter[3]=0x00;//�豸����Ϊ00�Ե�
	parameter[4]=32;//������Ϊ32
	parameter[5]=gain;		//�Ŵ���Ϊ24��
	parameter[6]=sps;	//�����ʣ�06Ϊ250��05Ϊ500sps,04Ϊ1Ksps,03Ϊ2Ksps
	parameter[7]=0;		//Ŀǰ�˲��ŵ���λ��
	parameter[8]=0;
	
	HAL_Delay(20);
	ADS1299_SDATAC();//�˳���������ģʽ���Ա���мĴ���������
	ADS1299_WREG(CONFIG3,0xe0);//�����ڲ���׼

	//ADS1299_WREG(CONFIG1,(0Xd0|sps));	//���ö�ض�ģʽ��ͨ������250SPS 1k	 
	HAL_Delay(20);
	ADS1299_WREG_Single(0,CONFIG1,(0Xf0|sps));
	HAL_Delay(20);
	ADS1299_WREG(CONFIG2,0XD0);//�����ź����ڲ�����
	ADS1299_WREG(CONFIG3,0XFc);//�����ڲ���׼
	//ADS1299_WREG_Single(0,CONFIG3,0XFc);
	//delay_ms(100);
	HAL_Delay(20);
	
	while(check!=0x3e)
	{
	check=ADS1299_PREG(ID);
	ADS1299_WREG(LOFF,0x00);//DC-Lead-off���
	HAL_Delay(20);
//	ADS1299_WREG(CH1SET,(0x05|(gain<<4)));//��һͨ�����ö�·������ϵͳ����
//	ADS1299_WREG(CH2SET,(0x05|(gain<<4)));
//	ADS1299_WREG(CH3SET,(0x05|(gain<<4)));
//	ADS1299_WREG(CH4SET,(0X05|(gain<<4)));//��һͨ��������ͨ����
//	ADS1299_WREG(CH5SET,(0X05|(gain<<4)));
//	ADS1299_WREG(CH6SET,(0X00|(gain<<4)));
//	ADS1299_WREG(CH7SET,(0X00|(gain<<4)));
//	ADS1299_WREG(CH8SET,(0X00|(gain<<4)));
//	ADS1299_WREG(BIAS_SENSP,0Xff);//����ֱ��ƫ��
//	ADS1299_WREG(BIAS_SENSN,0Xff);
//	ADS1299_WREG(LOFF,0x00);//DC-Lead-off���	
//	ADS1299_WREG_Single(0,MISC1,0X20);
////	ADS1299_WREG_Single(1,MISC1,0X20);
////	ADS1299_WREG_Single(2,MISC1,0X20);
//	//ADS1299_WREG_Single(3,MISC1,0X20);
//	ADS1299_Command(_RDATA);//�����ȡ����ģʽ
	}
}
		
	
void ADS1299_IT(void)
{
		  GPIO_InitTypeDef GPIO_InitStruct;
    __HAL_RCC_GPIOB_CLK_ENABLE();			//����GPIOIʱ��
	
	GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 4);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	
//	GPIO_Initure.Pin=GPIO_PIN_10;               //PB12
//    GPIO_Initure.Mode=GPIO_MODE_IT_FALLING;
//	//GPIO_MODE_IT_FALLING;     //�½��ش���
//    GPIO_Initure.Pull=GPIO_PULLUP;
//    HAL_GPIO_Init(GPIOD,&GPIO_Initure);
//	
//	 //�ж���13-PD10
//    HAL_NVIC_SetPriority(EXTI15_10_IRQn,1,1);   //��ռ���ȼ�Ϊ2�������ȼ�Ϊ1
//    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);         //ʹ���ж���13 
	//if(openvibeflag) 
		ADS1299_Command(_START);
	///else ADS1299_Command(_STOP);
	//delay_ms(10);
}
uint8_t p[18];
uint8_t ADS1299_PREGS(void)
{
	uint8_t i=0;
	ADS1299_CS0=0;
	SPI2_ReadWriteByte(0x00|0x20);
	SPI2_ReadWriteByte(0x12);
	//for(i=0;i<3;i++)
	{
		p[i]=SPI2_ReadWriteByte(0X00);
	}
	ADS1299_CS0=1;
return p[0];
}
//���ADS1299�Ƿ����
//����ֵ:0���ɹ�;1��ʧ��	
uint8_t ADS1299_Check(void)
{
	
	check=ADS1299_PREG(ID);

	//if(check==0x3e) 
	{		
		ADS1299_WREG(CONFIG2,0XD0);//�����ź����ڲ�����
		ADS1299_WREG(CONFIG3,0XFc);//�����ڲ���׼
		//ADS1299_WREG_Single(0,CONFIG3,0XFc);
		//delay_ms(100);
		HAL_Delay(20);
		ADS1299_WREG(LOFF,0x00);//DC-Lead-off���
		ADS1299_WREG(CH1SET,(0x05|(gain<<4)));//��һͨ�����ö�·������ϵͳ����
		ADS1299_WREG(CH2SET,(0x05|(gain<<4)));
		ADS1299_WREG(CH3SET,(0x05|(gain<<4)));
		ADS1299_WREG(CH4SET,(0X05|(gain<<4)));//��һͨ��������ͨ����
		ADS1299_WREG(CH5SET,(0X05|(gain<<4)));
		ADS1299_WREG(CH6SET,(0X00|(gain<<4)));
		ADS1299_WREG(CH7SET,(0X00|(gain<<4)));
		ADS1299_WREG(CH8SET,(0X00|(gain<<4)));
		ADS1299_WREG(BIAS_SENSP,0Xff);//����ֱ��ƫ��
		ADS1299_WREG(BIAS_SENSN,0Xff);
		ADS1299_WREG(LOFF,0x00);//DC-Lead-off���	
		ADS1299_WREG_Single(0,MISC1,0X20);
		ADS1299_WREG_Single(1,MISC1,0X20);
		ADS1299_WREG_Single(2,MISC1,0X20);
		//ADS1299_WREG_Single(3,MISC1,0X20);

		ADS1299_Command(_RDATA);//�����ȡ����ģʽ
		return 0;
	}
		
	
	//else return 1;

}	 
//void ImpTest_Start(void)
//{
//	//ADS1299_SDATAC();//�˳���������ģʽ���Ա���мĴ���������//��������ģʽ
//	ADS1299_WREG(LOFF,0x0a);//AC-Lead-off���

//	//ADS1299_WREG(BIAS_SENSP,0Xff);//����ֱ��ƫ��
//	//ADS1299_WREG(BIAS_SENSN,0Xff);
//	//ADS1299_WREG(LOFF_SENSP,0xff);//�����迹����
//	//ADS1299_WREG(LOFF_SENSN,0xff);
//}

//void ImpTest_Stop(void)
//{
//	//ADS1299_SDATAC();//�˳���������ģʽ���Ա���мĴ���������//��������ģʽ
//	ADS1299_WREG(LOFF,0x00);//AC-Lead-off���ر�
//	SW_IMP=1;
//	PWM_LED=1;
//	//ADS1299_WREG(BIAS_SENSP,0Xff);//����ֱ��ƫ��
//	//ADS1299_WREG(BIAS_SENSN,0Xff);
//	//ADS1299_WREG(LOFF_SENSP,0x00);//�ر��迹����
//	//ADS1299_WREG(LOFF_SENSN,0x00);
//}
	
void Set_Sps(uint8_t i)
{
				//ADS1299_START=0;//�Ƚ�ADS1299���ݲ������ص�
//				ADS1299_SDATAC();//�˳���������ģʽ���Ա���мĴ���������//��������ģʽ
//				//ADS1299_WREG(CONFIG1,(0Xd0|sps));	//���ö�ض�ģʽ��ͨ������250SPS 1k	 
				ADS1299_WREG_Single(0,CONFIG1,(0Xf0|i));
				
//				ADS1299_WREG(CONFIG1,(0Xd0|i));	//���ö�ض�ģʽ��ͨ������250SPS 1k	 
//				ADS1299_WREG_Single(0,CONFIG1,(0Xf0|i));
				//ADS1299_WREG(0X01,(0X90|sps));	//���ö�ض�ģʽ��ͨ������250SPS 1k//���ò�����
				//ADS1299_Command(0x12);//�����ȡ����ģʽ//�˳�����ģʽ
}

//uint8_t regdata[18];

				  


					  
//void Recev_Data(void)
//{
//	uint8_t k;
//	uint8_t buf3[28]={0};
//	//uint8_t res1,res2,res3;
//	//uint8_t inbyte[100];
//	//uint8_t Txda=0xff;
//	//u32 stat1;
//	//u32 Adcres[32];
//	//float  tempdata_f;
//	//arm_fir_instance_f32 S;
//	//index1=0;
//	//arm_fir_instance_f32 S;
//	// float32_t *input1,*output1;
////	 input1=&input[0];
//	// output1=&output[0];
//	 //��ʼ��
//	// arm_fir_init_f32(&S,29,(float32_t*)&firCoeffs32BS[0],&res[0],20);
//	//adc_buf2=0��
//	if(res3<0xff) 
//	{
//		res3++;
//	}
//	else res3=0;
//	adc_buf2[0]=0xa0;
//	if(openvibeflag) 
//	{
//		adc_buf2[1]=res3;
//		adc_buf2[102]=0xc0;
//	}
//	else
//	{
//	adc_buf2[1]=sizeof(adc_buf2)-2;
//	adc_buf2[104]=0xc0;
//	adc_buf2[2]=0x01;
////	adc_buf2[26]=0xc0;
//	
//	adc_buf2[3]=res3;
//	}
//	//tcp_server_sendbuf=buf3;'
//	for(k=0;k<4;k++)
//	{
//		ADS1299_CHANGE_CHANEL(k,0);
//		//ADS1299_DMA_Start();
//		//ADS1299_DMA_Start();
////		HAL_SPI_TransmitReceive_DMA(&SPI2_Handler,TxData0,adc_buf2,28);
////		while(1)
////	{
////		if(__HAL_DMA_GET_FLAG(&DMASPIRx_Handler,DMA_FLAG_TCIF3_7))//�ȴ�DMA2_Steam7�������
////         {
////                    __HAL_DMA_CLEAR_FLAG(&DMASPIRx_Handler,DMA_FLAG_TCIF3_7);//���DMA2_Steam7������ɱ�־
////                    HAL_SPI_DMAStop(&SPI2_Handler);      //��������Ժ�رմ���DMA
////			 break;
////		 }
////	 }
//		//HAL_SPI_TransmitReceive(&SPI2_Handler,TxData0,RxData,4, 1000); 
//		HAL_SPI_TransmitReceive(&SPI2_Handler,TxData0,buf3,28, 1200); 
////		for(n=0;n<24;n++)
////		{
////			buf3[n]=n;
////		}
//		if(openvibeflag) memcpy(&adc_buf2[k*24+2],&buf3[4],24);
//		else
//		{
//		stat[k]=(buf3[1]<<4)|(buf3[2]>>4);
//		memcpy(&adc_buf2[k*24+8],&buf3[4],24);
//		//memcpy(&adc_buf2[4],&buf3[4],24);
//	//	memcpy(&stat[k*3],&buf3[1],3);
//		//stat[k]=buf3[0]
//		}
////		SPI2_ReadWriteByte(0x12);
////		SPI2_ReadWriteByte(0x00);
////		SPI2_ReadWriteByte(0x00);
////		SPI2_ReadWriteByte(0x00);
//			
//				//Adcres[index1][n]=0;
////				for(j=0;j<24;j++)
////				{

////					adc_buf2[j
////						//adc_buf2[n++]=SPI2_ReadWriteByte(0x00);
////					inbyte[k*24+3*i+j]=SPI2_ReadWriteByte(0x00);
////					Adcres[index1][k*8+i]=(Adcres[index1][k*8+i]<<8)| inbyte[k*24+3*i+j];
////					
////				}
//				//n++;
//				//Adcres[i+1]=(adc_buf2[n-2]<<16)+(adc_buf2[n-1]<<8)+adc_buf2[n];
//				//Adcres[i]=(buf1[0]<<16)+(buf1[1]<<8)+buf1[2];
//			//}
//////////			
//			
//			ADS1299_CHANGE_CHANEL(k,1);
////			for(i=0;i<8;i++)
////			{
////				
////				//adc_buf2[0]=SPI2_ReadWriteByte(0x00);
////				Adcres[k*8+i]=buf3[3*i+4]<<24;
////				Adcres[k*8+i]|=buf3[i*3+5]<<16;
////				Adcres[k*8+i]|=buf3[i*3+6]<<8;
////				Adcres[k*4+i]/=80;
////				
////			}
////			
//			//delay_us(2);
//			//ADS1299_CS3=1;
//			
//		//}
//			
//		}
//			//if(openvibeflag) 
//			memcpy(&adc_buf2[4],stat,4);
//			Num=netcam_fifo_write(&adc_buf2[0]);
//			if((Num>0)&&(~(tcp_server_flag&1<<7)))
//			{
//				tcp_server_flag|=(1<<7);//������Ҫ����
//				//udp_demo_flag|=(1<<7);
//				
////				//Num=index1;
//////				//index1=0;
//			}
//}
 
//void EXTI4_IRQHandler(void)
//void EXTI15_10_IRQHandler(void)
//{
////	if(_HAL_GPIO_EXTI_GET_IT(GPIO_PIN_13)!=RESET)
////	{
////		_HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);
//		//OSIntEnter();
//		//OSSemPost(Sem_Task_ads1299); // �����ź���,�����������������ϵͳ���ȣ������жϷ�����һ��Ҫ��ࡣ
//		//EXTI_ClearITPendingBit(EXTI_Line13); // �����־λ
//		Recev_Data();
//		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_10);
//         //HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);//�����жϴ����ú���
//		//OSIntExit();
//	//}
//}

//uint8_t err;
//led����
//void ads1299_task(void *pdata)
//{
//	
////	Sem_Task_LED2=OSSemCreate(0);
////	OSSemPend(Sem_Task_LED2,0,&errno);
////	OSSemPost(Sem_Task_LED2);
//	(void) pdata;
//	Sem_Task_ads1299 = OSSemCreate(0);
//	//ADS1299_CS0=0;
//	while(1)
//	{
//		//OSSemPend(Sem_Task_ads1299,0,&err);  // �ȴ��ź���
//		//Recev_Data();
//		
//			
//		OSTimeDlyHMSM(0,0,0,20);  //��ʱ500ms
// 	}
//}
//�жϷ����������Ҫ��������
//��HAL�������е��ⲿ�жϷ�����������ô˺���
//��ADS1299�����ݽ��д���
//uint8_t t;
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
////uint8_t inbyte,i,j,k,n=2;
////adc_buf2[0]=0xa0;
//	//adc_buf2[26]=0xc0;
//	//u32 byteCounter=0,channelData[8];
//	//delay_us(2);//��һ��С��ʱ���Է�ֹ����
//	if(GPIO_Pin==GPIO_PIN_13)
//	{
//		//ads1299_data
//		//ads1299_data_flag=1;
//		Recev_Data();

//		
//			
//	
//}
//}


























