#include "ads1299.h"
#include "spi.h"
#include "stm32f1xx_hal_gpio.h"

//#include "openbci.h""
//#include "arm_math.h"
//#include "dma.h"
//////////////////////////////////////////////////////////////////////////////////	 
									  
////////////////////////////////////////////////////////////////////////////////// 	
uint8_t check=0;
uint8_t TxData0[28]={0x12,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //发送地址
//uint8_t TxData1[24]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//uint8_t RxData[4][4]; //接受地址
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

//读取寄存器数据
uint8_t ADS1299_PREG(uint8_t reg)
{
	  uint8_t Byte;	
   	//ADS1299_CS0=0;                 //使能SPI传输
	ADS1299_CHANGE_CHANEL(0,0);
	//delay_us(200);
  	SPI2_ReadWriteByte(0X00|0X20);//发送寄存器号	
  	SPI2_ReadWriteByte(0X00);      //写入寄存器的值
	Byte=SPI2_ReadWriteByte(0);
	ADS1299_CHANGE_CHANEL(0,1);
  	//ADS1299_CS0=1;                 //禁止SPI传输	   
  	return(Byte);       		    //返回状态值
}
//写入ADS1299寄存器数据
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
//初始化24L01的IO口
void ADS1299_Init(void)
{

	ADS1299_CHANGE_CHANEL(0,1);; //屏蔽第一片ADS1299,测试单芯片
	ADS1299_Command(_WAKEUP);
	ADS1299_Command(_RESET);
	
	HAL_Delay(20);
	parameter[0]=0xa0;//帧头
	parameter[1]=8;//帧长
	parameter[9]=0xc0;//帧尾
	parameter[2]=0x21;
	parameter[3]=0x00;//设备代码为00脑电
	parameter[4]=32;//导联数为32
	parameter[5]=gain;		//放大倍数为24倍
	parameter[6]=sps;	//采样率：06为250，05为500sps,04为1Ksps,03为2Ksps
	parameter[7]=0;		//目前滤波放到上位机
	parameter[8]=0;
	
	HAL_Delay(20);
	ADS1299_SDATAC();//退出连续读数模式，以便进行寄存器的设置
	ADS1299_WREG(CONFIG3,0xe0);//开启内部基准

	//ADS1299_WREG(CONFIG1,(0Xd0|sps));	//设置多回读模式，通信速率250SPS 1k	 
	HAL_Delay(20);
	ADS1299_WREG_Single(0,CONFIG1,(0Xf0|sps));
	HAL_Delay(20);
	ADS1299_WREG(CONFIG2,0XD0);//测试信号由内部产生
	ADS1299_WREG(CONFIG3,0XFc);//开启内部基准
	//ADS1299_WREG_Single(0,CONFIG3,0XFc);
	//delay_ms(100);
	HAL_Delay(20);
	
	while(check!=0x3e)
	{
	check=ADS1299_PREG(ID);
	ADS1299_WREG(LOFF,0x00);//DC-Lead-off检查
	HAL_Delay(20);
//	ADS1299_WREG(CH1SET,(0x05|(gain<<4)));//第一通道设置短路，测试系统噪声
//	ADS1299_WREG(CH2SET,(0x05|(gain<<4)));
//	ADS1299_WREG(CH3SET,(0x05|(gain<<4)));
//	ADS1299_WREG(CH4SET,(0X05|(gain<<4)));//第一通道设置普通输入
//	ADS1299_WREG(CH5SET,(0X05|(gain<<4)));
//	ADS1299_WREG(CH6SET,(0X00|(gain<<4)));
//	ADS1299_WREG(CH7SET,(0X00|(gain<<4)));
//	ADS1299_WREG(CH8SET,(0X00|(gain<<4)));
//	ADS1299_WREG(BIAS_SENSP,0Xff);//开启直流偏置
//	ADS1299_WREG(BIAS_SENSN,0Xff);
//	ADS1299_WREG(LOFF,0x00);//DC-Lead-off检查	
//	ADS1299_WREG_Single(0,MISC1,0X20);
////	ADS1299_WREG_Single(1,MISC1,0X20);
////	ADS1299_WREG_Single(2,MISC1,0X20);
//	//ADS1299_WREG_Single(3,MISC1,0X20);
//	ADS1299_Command(_RDATA);//命令读取数据模式
	}
}
		
	
void ADS1299_IT(void)
{
		  GPIO_InitTypeDef GPIO_InitStruct;
    __HAL_RCC_GPIOB_CLK_ENABLE();			//开启GPIOI时钟
	
	GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 4);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	
//	GPIO_Initure.Pin=GPIO_PIN_10;               //PB12
//    GPIO_Initure.Mode=GPIO_MODE_IT_FALLING;
//	//GPIO_MODE_IT_FALLING;     //下降沿触发
//    GPIO_Initure.Pull=GPIO_PULLUP;
//    HAL_GPIO_Init(GPIOD,&GPIO_Initure);
//	
//	 //中断线13-PD10
//    HAL_NVIC_SetPriority(EXTI15_10_IRQn,1,1);   //抢占优先级为2，子优先级为1
//    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);         //使能中断线13 
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
//检测ADS1299是否存在
//返回值:0，成功;1，失败	
uint8_t ADS1299_Check(void)
{
	
	check=ADS1299_PREG(ID);

	//if(check==0x3e) 
	{		
		ADS1299_WREG(CONFIG2,0XD0);//测试信号由内部产生
		ADS1299_WREG(CONFIG3,0XFc);//开启内部基准
		//ADS1299_WREG_Single(0,CONFIG3,0XFc);
		//delay_ms(100);
		HAL_Delay(20);
		ADS1299_WREG(LOFF,0x00);//DC-Lead-off检查
		ADS1299_WREG(CH1SET,(0x05|(gain<<4)));//第一通道设置短路，测试系统噪声
		ADS1299_WREG(CH2SET,(0x05|(gain<<4)));
		ADS1299_WREG(CH3SET,(0x05|(gain<<4)));
		ADS1299_WREG(CH4SET,(0X05|(gain<<4)));//第一通道设置普通输入
		ADS1299_WREG(CH5SET,(0X05|(gain<<4)));
		ADS1299_WREG(CH6SET,(0X00|(gain<<4)));
		ADS1299_WREG(CH7SET,(0X00|(gain<<4)));
		ADS1299_WREG(CH8SET,(0X00|(gain<<4)));
		ADS1299_WREG(BIAS_SENSP,0Xff);//开启直流偏置
		ADS1299_WREG(BIAS_SENSN,0Xff);
		ADS1299_WREG(LOFF,0x00);//DC-Lead-off检查	
		ADS1299_WREG_Single(0,MISC1,0X20);
		ADS1299_WREG_Single(1,MISC1,0X20);
		ADS1299_WREG_Single(2,MISC1,0X20);
		//ADS1299_WREG_Single(3,MISC1,0X20);

		ADS1299_Command(_RDATA);//命令读取数据模式
		return 0;
	}
		
	
	//else return 1;

}	 
//void ImpTest_Start(void)
//{
//	//ADS1299_SDATAC();//退出连续读数模式，以便进行寄存器的设置//进入设置模式
//	ADS1299_WREG(LOFF,0x0a);//AC-Lead-off检查

//	//ADS1299_WREG(BIAS_SENSP,0Xff);//开启直流偏置
//	//ADS1299_WREG(BIAS_SENSN,0Xff);
//	//ADS1299_WREG(LOFF_SENSP,0xff);//开启阻抗测试
//	//ADS1299_WREG(LOFF_SENSN,0xff);
//}

//void ImpTest_Stop(void)
//{
//	//ADS1299_SDATAC();//退出连续读数模式，以便进行寄存器的设置//进入设置模式
//	ADS1299_WREG(LOFF,0x00);//AC-Lead-off检查关闭
//	SW_IMP=1;
//	PWM_LED=1;
//	//ADS1299_WREG(BIAS_SENSP,0Xff);//开启直流偏置
//	//ADS1299_WREG(BIAS_SENSN,0Xff);
//	//ADS1299_WREG(LOFF_SENSP,0x00);//关闭阻抗测试
//	//ADS1299_WREG(LOFF_SENSN,0x00);
//}
	
void Set_Sps(uint8_t i)
{
				//ADS1299_START=0;//先将ADS1299数据采样给关掉
//				ADS1299_SDATAC();//退出连续读数模式，以便进行寄存器的设置//进入设置模式
//				//ADS1299_WREG(CONFIG1,(0Xd0|sps));	//设置多回读模式，通信速率250SPS 1k	 
				ADS1299_WREG_Single(0,CONFIG1,(0Xf0|i));
				
//				ADS1299_WREG(CONFIG1,(0Xd0|i));	//设置多回读模式，通信速率250SPS 1k	 
//				ADS1299_WREG_Single(0,CONFIG1,(0Xf0|i));
				//ADS1299_WREG(0X01,(0X90|sps));	//设置多回读模式，通信速率250SPS 1k//设置采样率
				//ADS1299_Command(0x12);//命令读取数据模式//退出设置模式
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
//	 //初始化
//	// arm_fir_init_f32(&S,29,(float32_t*)&firCoeffs32BS[0],&res[0],20);
//	//adc_buf2=0；
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
////		if(__HAL_DMA_GET_FLAG(&DMASPIRx_Handler,DMA_FLAG_TCIF3_7))//等待DMA2_Steam7传输完成
////         {
////                    __HAL_DMA_CLEAR_FLAG(&DMASPIRx_Handler,DMA_FLAG_TCIF3_7);//清除DMA2_Steam7传输完成标志
////                    HAL_SPI_DMAStop(&SPI2_Handler);      //传输完成以后关闭串口DMA
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
//				tcp_server_flag|=(1<<7);//有数据要发送
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
//		//OSSemPost(Sem_Task_ads1299); // 发送信号量,这个函数并不会引起系统调度，所以中断服务函数一定要简洁。
//		//EXTI_ClearITPendingBit(EXTI_Line13); // 清除标志位
//		Recev_Data();
//		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_10);
//         //HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);//调用中断处理公用函数
//		//OSIntExit();
//	//}
//}

//uint8_t err;
//led任务
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
//		//OSSemPend(Sem_Task_ads1299,0,&err);  // 等待信号量
//		//Recev_Data();
//		
//			
//		OSTimeDlyHMSM(0,0,0,20);  //延时500ms
// 	}
//}
//中断服务程序中需要做的事情
//在HAL库中所有的外部中断服务函数都会调用此函数
//对ADS1299的数据进行处理
//uint8_t t;
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
////uint8_t inbyte,i,j,k,n=2;
////adc_buf2[0]=0xa0;
//	//adc_buf2[26]=0xc0;
//	//u32 byteCounter=0,channelData[8];
//	//delay_us(2);//做一个小延时，以防止干扰
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


























