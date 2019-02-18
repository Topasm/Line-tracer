#include <stm32f10x_gpio.h> // GPIO 제어
#include <stm32f10x_rcc.h> // RCC 제어
#include <stm32f10x.h> // 인터럽트 벡터 테이블
#include <misc.h> // NVIC 설정 관련
#include <stm32f10x_dma.h>
#include <stm32f10x_adc.h>
#include <stdio.h>

float Kp=15,Ki=5,Kd=15;
float error=0, P=0, I=0, D=0, PID_value=0;
float previous_error=0, previous_I=0;
unsigned int Channel3Pulse=0;
unsigned int Channel4Pulse=0;

uint16_t sensor[6]={0,0,0,0,0,0};//ADC 저장소
void RCC_Configuration(void)
{
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//내가 추가함 타이머에 클럭인가
//DMA1 RCC
RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
RCC_ADCCLKConfig(RCC_PCLK2_Div6); //clock for ADC (max 14MHz, 72/6=12MHz)
RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //enable ADC clock
}
void GPIO_Configuration()
{
//모터 드라이브 GPIO설정+led 커고 ㄱ끄기6
GPIO_InitTypeDef GPIO_InitStructure;
GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
GPIO_Init(GPIOB, &GPIO_InitStructure);
//수발광 센서용
GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;
GPIO_Init(GPIOA, &GPIO_InitStructure);
//usart gpio
GPIO_InitTypeDef GPIO_SET; //GPIO_Initialize 구조체 선언
GPIO_SET.GPIO_Pin = GPIO_Pin_11; //tim
GPIO_SET.GPIO_Mode = GPIO_Mode_AF_PP; //Alternative Function push-pull
GPIO_SET.GPIO_Speed = GPIO_Speed_50MHz; //속도
GPIO_Init(GPIOB,&GPIO_SET); //제출
GPIO_SET.GPIO_Pin = GPIO_Pin_10; //tim
GPIO_SET.GPIO_Mode = GPIO_Mode_AF_PP; //Alternative Function push-pull
GPIO_SET.GPIO_Speed = GPIO_Speed_50MHz; //속도
GPIO_Init(GPIOB,&GPIO_SET); //제출
/*Configure peripheral I/O remapping */
GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);
//타이머채널 3,4 사용 풀리리맵시 각각pb10,11
}
void DMA_Configuration()
{
DMA_InitTypeDef DMA_InitStructure;
DMA_DeInit(DMA1_Channel1);
DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)sensor;
DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
DMA_InitStructure.DMA_BufferSize = 6;
DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
DMA_InitStructure.DMA_Priority = DMA_Priority_High;
DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel1, ENABLE); //Enable the DMA1 - Channel1
}


void adc_config()
{
//configure ADC1 parameters
ADC_InitTypeDef myADC;
myADC.ADC_Mode = ADC_Mode_Independent;
myADC.ADC_ScanConvMode = ENABLE;
myADC.ADC_ContinuousConvMode = ENABLE;
myADC.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
myADC.ADC_DataAlign = ADC_DataAlign_Right;
myADC.ADC_NbrOfChannel = 6;
ADC_Init(ADC1, &myADC);
ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5 );
ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_55Cycles5 ); 
ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_55Cycles5 ); 
ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_55Cycles5 ); 
ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_55Cycles5 ); 
ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_55Cycles5 ); 
ADC_DMACmd(ADC1, ENABLE);
//enable
ADC_Cmd(ADC1, ENABLE);
ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
//pwm 을 위한 타이머 2개
void TIM_Configuration(void){
//타이머설정
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_TimeBaseStructure.TIM_Period=100;
TIM_TimeBaseStructure.TIM_Prescaler=1000;
TIM_TimeBaseStructure.TIM_ClockDivision=0;
TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
TIM_Cmd(TIM2, ENABLE);}
void pwm(unsigned int  Channel3Pulse, unsigned int Channel4Pulse){
//pwm 설정
TIM_OCInitTypeDef TIM_OCInitStructure;
TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_Low;
TIM_OCInitStructure.TIM_Pulse=Channel3Pulse;
TIM_OC3Init(TIM2, &TIM_OCInitStructure);//오른쪽
TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
TIM_OCInitStructure.TIM_Pulse=Channel4Pulse;
TIM_OC4Init(TIM2, &TIM_OCInitStructure);//왼쪽
TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
TIM_ARRPreloadConfig(TIM2, ENABLE);
}void ReadSensorLed()
{
if(sensor[0] < 150&&sensor[3] >150)
{ GPIO_ResetBits(GPIOB, GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8);//문제가 된다
GPIO_SetBits(GPIOB, GPIO_Pin_8);
error=3;
}
else if(sensor[5] < 150&&sensor[2]>150 )
{ GPIO_ResetBits(GPIOB, GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8);
GPIO_SetBits(GPIOB, GPIO_Pin_3);
error= -3;
}

else if(sensor[1] < 150&&sensor[4]> 150)
{ GPIO_ResetBits(GPIOB, GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8);
GPIO_SetBits(GPIOB, GPIO_Pin_7);
error=2;
}
else if(sensor[4] < 150&&sensor[1] >150)
{ GPIO_ResetBits(GPIOB, GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8);
GPIO_SetBits(GPIOB, GPIO_Pin_4);
error=-2;
}

else if(sensor[2] <150&&sensor[5]> 150)
{ GPIO_ResetBits(GPIOB, GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8);
GPIO_SetBits(GPIOB, GPIO_Pin_6);
error=1;
}
else if(sensor[3] < 150&&sensor[0] >150)
{ GPIO_ResetBits(GPIOB, GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8);
GPIO_SetBits(GPIOB, GPIO_Pin_5);
error=-1;
}

else
{ GPIO_ResetBits(GPIOB, GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8);
if(error>1){
error=4;
}
else if(error<-1){
error=-4;
}
}
}
void calculate_pid()
{
P = error;
I = I + previous_I;
D = error-previous_error;
PID_value = (Kp*P) + (Ki*I) + (Kd*D);
previous_I=I;
previous_error=error;
}
void motor_control()
{
//모터제어 시작
GPIO_ResetBits(GPIOB, GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
//pb13 14 하면 앞으로 감
//pb12 15 하면 뒤로?
GPIO_SetBits(GPIOB, GPIO_Pin_13);
GPIO_SetBits(GPIOB, GPIO_Pin_14);
Channel4Pulse=50+PID_value;
Channel3Pulse=50-PID_value;
if(Channel4Pulse>=100)
{Channel4Pulse=100;
}
else if(Channel4Pulse<=0)
{Channel4Pulse=0;
}
else if(Channel3Pulse>=100)
{Channel4Pulse=100;
}
else if(Channel3Pulse<=0)
{Channel4Pulse=0;
}}
void main(void)
{
RCC_Configuration();
GPIO_Configuration();
adc_config(); //configure ADC
DMA_Configuration();
TIM_Configuration();
while(1){
ReadSensorLed();
calculate_pid();
motor_control();
pwm(Channel3Pulse, Channel4Pulse);
}
}